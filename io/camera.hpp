#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{

// Read-thread metadata only. SDK frame IDs remain separate from the legacy
// hardware-trigger/count API, whose behaviour must not change for other modes.
struct CameraFrameInfo
{
  bool latest_supported = false;
  bool latest_selected = false;
  uint64_t capture_sequence = 0;
  uint64_t sdk_frame_id = 0;
};

class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  virtual void read_latest(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
  {
    read(img, timestamp);
  }
  virtual CameraFrameInfo frame_info() const { return {}; }
};

class Camera
{
public:
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  void read_latest(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  CameraFrameInfo frame_info() const;

  // 🆕 获取最后读取的相机帧ID
  uint64_t get_last_frame_id() const;

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP
