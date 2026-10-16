#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <list>
#include <memory>
#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace auto_aim
{
struct YoloDetectionDiagnostics
{
  bool available = false;
  int raw_rows = 0;
  int candidates = 0;
  int after_nms = 0;
  int rejected_not_armor = 0;
  int rejected_confidence = 0;
  int rejected_type = 0;
  int accepted = 0;
  int color_direct = 0;
  int color_memory = 0;
  int color_unconfirmed = 0;
  float max_confidence = 0.0f;
  int max_class = -1;
};

class YOLOBase
{
public:
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count, bool)
  {
    return detect(img, frame_count);
  }
  virtual YoloDetectionDiagnostics diagnostics() const { return {}; }

  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1, bool full_frame = false);
  YoloDetectionDiagnostics diagnostics() const;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
