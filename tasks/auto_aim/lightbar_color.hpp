#ifndef AUTO_AIM__LIGHTBAR_COLOR_HPP
#define AUTO_AIM__LIGHTBAR_COLOR_HPP

#include <algorithm>
#include <cmath>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace auto_aim
{
// Image evidence only. Network class/confidence and color history cannot vote.
class LightbarColorGate
{
public:
  struct Config
  {
    int min_brightness = 60, min_pixels = 10, min_channel_delta = 20;
    int min_green_delta = 10;
    double min_delta = 15.0, single_bar_delta = 20.0;
    double min_coverage = 0.10, min_agreement = 0.80;
    bool require_both = true;
  };
  struct Bar
  {
    double score = 0.0, agreement = 0.0, coverage = 0.0;
    int count = 0;
  };
  struct Result
  {
    bool valid = false;
    double score = 0.0;
    Bar left, right;
  };

  void set_config(const Config & config) { config_ = config; }

  Result evaluate(const cv::Mat & rgb, const std::vector<cv::Point2f> & points) const
  {
    Result result;
    if (rgb.empty() || rgb.type() != CV_8UC3 || points.size() != 4) return result;
    for (const auto & p : points)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
          p.x >= rgb.cols || p.y >= rgb.rows) return result;
    const double length = std::max(cv::norm(points[3] - points[0]),
                                   cv::norm(points[2] - points[1]));
    const double width = std::max(cv::norm(points[1] - points[0]),
                                  cv::norm(points[2] - points[3]));
    if (length < 1.0 || width < 1.0) return result;
    const int thickness = std::max(5, static_cast<int>(std::lround(
      std::min(0.12 * width, 0.45 * length))));
    result.left = sample(rgb, points[0], points[3], thickness);
    result.right = sample(rgb, points[1], points[2], thickness);
    const bool left = reliable(result.left, config_.min_delta);
    const bool right = reliable(result.right, config_.min_delta);
    if (left && right) {
      // A red/blue conflict is rejected even if one bar has more pixels.
      if ((result.left.score > 0) != (result.right.score > 0)) return result;
      result.score = (result.left.score * result.left.count +
        result.right.score * result.right.count) / (result.left.count + result.right.count);
      result.valid = true;
    } else if (!config_.require_both && left && reliable(result.left, config_.single_bar_delta)) {
      result.score = result.left.score;
      result.valid = true;
    } else if (!config_.require_both && right && reliable(result.right, config_.single_bar_delta)) {
      result.score = result.right.score;
      result.valid = true;
    }
    return result;
  }

private:
  Config config_;

  bool reliable(const Bar & bar, double delta) const
  {
    return bar.count >= config_.min_pixels && bar.coverage >= config_.min_coverage &&
      bar.agreement >= config_.min_agreement && std::abs(bar.score) >= delta;
  }

  Bar sample(const cv::Mat & rgb, cv::Point2f p0, cv::Point2f p1, int thickness) const
  {
    Bar stats;
    cv::Rect roi = cv::boundingRect(std::vector<cv::Point2f>{p0, p1});
    roi.x -= thickness; roi.y -= thickness;
    roi.width += 2 * thickness; roi.height += 2 * thickness;
    roi &= cv::Rect(0, 0, rgb.cols, rgb.rows);
    if (roi.empty()) return stats;
    cv::Mat mask(roi.size(), CV_8UC1, cv::Scalar(0));
    const cv::Point offset(roi.x, roi.y);
    cv::line(mask, cv::Point(cvRound(p0.x), cvRound(p0.y)) - offset,
      cv::Point(cvRound(p1.x), cvRound(p1.y)) - offset,
      cv::Scalar(255), thickness, cv::LINE_8);
    std::vector<double> samples;
    int mask_pixels = 0;
    for (int y = 0; y < roi.height; ++y) {
      const auto * row = rgb.ptr<cv::Vec3b>(y + roi.y);
      const auto * m = mask.ptr<unsigned char>(y);
      for (int x = 0; x < roi.width; ++x) {
        if (!m[x]) continue;
        ++mask_pixels;
        const auto & pixel = row[x + roi.x];
        const int red = pixel[0], green = pixel[1], blue = pixel[2];
        const int peak = std::max(red, blue), chroma = red - blue;
        if (peak < config_.min_brightness || std::abs(chroma) < config_.min_channel_delta ||
            peak - green < config_.min_green_delta) continue;
        samples.push_back(100.0 * chroma / std::max(1, red + blue));
      }
    }
    if (samples.empty() || mask_pixels == 0) return stats;
    auto middle = samples.begin() + samples.size() / 2;
    std::nth_element(samples.begin(), middle, samples.end());
    const double median = *middle;
    double sum = 0.0;
    int agree = 0;
    for (const double value : samples) {
      if (std::abs(value - median) > 22.0) continue;
      sum += value;
      ++stats.count;
      if ((value > 0) == (median > 0)) ++agree;
    }
    if (!stats.count) return stats;
    stats.score = sum / stats.count;
    // Keep conflicting pixels in the denominator: median filtering must not
    // turn a half-red/half-blue lightbar into a unanimous color vote.
    stats.agreement = static_cast<double>(agree) / samples.size();
    stats.coverage = static_cast<double>(stats.count) / mask_pixels;
    return stats;
  }
};
}  // namespace auto_aim
#endif
