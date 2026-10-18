// GameIO backend: DXGI desktop-duplication capture of the UE game window.
// Replaces io::Camera for the gestalt UE-simulator bridge. The captured frame
// carries the DXGI LastPresentTime (QPC) as its timestamp — the closest thing
// to a hardware camera timestamp this pipeline has; feed it to tracker.track()
// so the EKF timestamps measurements at PRESENT time, not process time.
#ifndef IO_GESTALT_WIN_CAPTURE_HPP
#define IO_GESTALT_WIN_CAPTURE_HPP

#include <chrono>
#include <array>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <string>

namespace io::gestalt
{
struct CapturedFrame
{
  cv::Mat img;              // BGR, cropped to game client rect minus footer
  double t_present = 0.0;   // seconds, steady/QPC clock, at DXGI present
  uint64_t frame_id = 0;
  uint32_t writer_process_id = 0;
  std::array<double, 3> camera_position_ue_cm{};
  std::array<double, 4> camera_quaternion_ue_xyzw{};
  double fov_degrees = 0.0;
  double camera_arm_length_cm = 0.0;
  uint32_t view_actor_unique_id = 0;
  uint32_t takeover_target_unique_id = 0;
  int32_t takeover_player_id = -1;
  int32_t takeover_attribute_map_id = -1;
  uint32_t takeover_epoch = 0;
  uint32_t identity_flags = 0;
  bool has_camera_pose = false;
  bool ok = false;
};

class WinCapture
{
public:
  // window_bottom_crop: the native version footer strip (28 px) draws even
  // with -hudhidden=1; crop it exactly like the python bridge did, and pass
  // full client height/2 as cy to the pinhole K (principal point stays at the
  // UNCROPPED centre - see aim_solver.py camera_matrix note).
  explicit WinCapture(int window_bottom_crop = 28);
  ~WinCapture();

  WinCapture(const WinCapture &) = delete;
  WinCapture & operator=(const WinCapture &) = delete;

  // Locates the UE game window (unrealeditor*/gestalt*/robotbridge* process,
  // client >= 800x500) and (re)initialises duplication on its monitor.
  bool init();

  // Blocks up to timeout_ms for a NEW presented frame. Returns ok=false on
  // timeout (no new present) or device loss (call init() again on repeated
  // failure). Window rect is re-read every frame so window moves are safe.
  CapturedFrame grab(int timeout_ms = 100);

  int full_client_height() const { return full_client_h_; }  // pre-crop, for cy
  int width() const { return width_; }
  int height() const { return height_; }
  std::string window_title() const { return title_; }

  // Pin the game window topmost (dxcam-style): duplication reads the SCREEN,
  // so an occluding window would silently replace the pixels.
  void raise_window();

private:
  struct Impl;
  Impl * impl_;
  int crop_bottom_;
  int width_ = 0, height_ = 0, full_client_h_ = 0;
  std::string title_;
};

}  // namespace io::gestalt

#endif  // IO_GESTALT_WIN_CAPTURE_HPP
