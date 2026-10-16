#include <fmt/core.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>  // for setenv
#include <filesystem>
#include <iterator>
#include <memory>
#include <optional>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

// ROS2 headers (仅在 ROS2 可用时编译，用于可视化)
#ifdef AMENT_CMAKE_FOUND
#include <rclcpp/rclcpp.hpp>
#include "tools/ros2_visualizer.hpp"
#endif

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/xuc.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/gyro_face_selector.hpp"
#include "tasks/auto_aim/gyro_phase_scheduler.hpp"
#include "tasks/auto_aim/robust_range_filter.hpp"
#include "tasks/auto_aim/rotation_model_manager.hpp"
#include "tasks/auto_aim/shot_delay_adapter.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

using namespace std::chrono;

namespace
{
// Observation history only: never stores a pending fire request. The sender's
// hold/rearm cycle must not interrupt consecutive prediction qualification.
struct RotationFireWindow
{
  int frames = 0;
  int target_name = -1, target_type = -1, armor_count = 0, face_id = -1;
  bool event_face = false;
  double phase_rad = NAN, error_rad = NAN, next_error_rad = NAN;
  std::chrono::steady_clock::time_point timestamp{};
  bool two_frames_ready = false, current_frame_is_better = false;

  void update(
    bool valid, bool rebuilt, int name, int type, int count, int face,
    bool uses_event_face, double phase, double error,
    std::chrono::steady_clock::time_point time, int required_frames,
    double max_phase_step, double tolerance)
  {
    two_frames_ready = false;
    current_frame_is_better = false;
    next_error_rad = NAN;
    if (!valid || face < 0 || !std::isfinite(phase) || !std::isfinite(error)) {
      frames = 0;
      return;
    }
    const bool continuous = frames > 0 && !rebuilt && time > timestamp &&
      name == target_name && type == target_type && count == armor_count &&
      face == face_id && uses_event_face == event_face &&
      std::abs(std::remainder(phase - phase_rad, 2.0 * M_PI)) <= max_phase_step;
    const int needed = std::max(2, required_frames);
    frames = continuous ? std::min(frames + 1, needed) : 1;
    if (continuous) {
      // One observed frame step forward; no future sample or fitted model.
      next_error_rad = std::remainder(
        error + std::remainder(error - error_rad, 2.0 * M_PI), 2.0 * M_PI);
      current_frame_is_better = std::abs(error) <= std::abs(next_error_rad) ||
        std::abs(next_error_rad) > tolerance;
    }
    two_frames_ready = frames >= needed;
    target_name = name;
    target_type = type;
    armor_count = count;
    face_id = face;
    event_face = uses_event_face;
    phase_rad = phase;
    error_rad = error;
    timestamp = time;
  }
};

// Allocated once only when the complete debug path is enabled. These are
// observations of existing calculations, never another prediction or gate.
struct ShotPredictionDebug
{
  auto_aim::AimPredictionTrace native;
  int64_t aimer_begin_ns = 0, aimer_end_ns = 0, latest_rx_ns = 0;
  double feedback_speed_mps = NAN, aimer_input_mps = NAN;
  const char * input_source = "unknown";
  const char * gate_branch = "native_shooter";
  double event_flight_time_s = NAN, event_delay_s = NAN;
  double event_prediction_horizon_s = NAN;
  std::array<double, 4> event_xyza{NAN, NAN, NAN, NAN};
};

struct PredictionSessionDebug
{
  uint64_t track_epoch = 0;
  bool had_target = false;
  bool calibration_recorded = false;
  int name = -1, type = -1, armor_count = 0;
};
}  // namespace

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 🚀 关键性能优化：全局线程控制
  // vtune分析发现88个线程抢16个核心，导致严重性能下降
  // - cv::cvtColor的Bayer转RGB触发TBB并行：61.660s (10.9%)
  // - 线程调度开销（__sched_yield）：94.800s (16.7%)
  // - Spin Time浪费：99.970s

  // 1. 强制OpenCV单线程（避免cvtColor创建大量线程）
  cv::setNumThreads(1);

  // 2. 限制TBB全局线程池（OpenVINO和OpenCV共用）
  setenv("TBB_NUM_THREADS", "4", 1);  // NUC有16核，4线程足够

  // 3. 限制OpenMP线程数（某些OpenCV编译可能使用）
  setenv("OMP_NUM_THREADS", "4", 1);

  tools::logger()->info("[Performance] Thread limits: OpenCV=1, TBB=4, OMP=4");

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;
  cv::CommandLineParser cli(argc, argv, keys);

  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  const auto integration_yaml = YAML::LoadFile(config_path);
  const bool competition_headless =
    integration_yaml["competition_headless"] &&
    integration_yaml["competition_headless"].as<bool>();
  const bool infantry_diagnostics_enabled =
    integration_yaml["infantry_diagnostics_enabled"] &&
    integration_yaml["infantry_diagnostics_enabled"].as<bool>();
  const double xuc_imu_alignment_ms = integration_yaml["xuc_imu_alignment_ms"]
    ? std::clamp(integration_yaml["xuc_imu_alignment_ms"].as<double>(), -30.0, 30.0)
    : 0.0;
  const auto xuc_imu_alignment =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double, std::milli>(xuc_imu_alignment_ms));
  const auto diagnostic_enemy_color_name = integration_yaml["enemy_color"].as<std::string>();
  const auto diagnostic_enemy_color = diagnostic_enemy_color_name == "red"
    ? auto_aim::Color::red : auto_aim::Color::blue;
  const auto diagnostics_run_id = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const std::filesystem::path diagnostics_image_dir = "data/infantry_diagnostics";
  bool diagnostics_images_available = false;
  if (infantry_diagnostics_enabled) {
    std::error_code error;
    std::filesystem::create_directories(diagnostics_image_dir, error);
    diagnostics_images_available = !error;
    tools::logger()->info(
      "[INFANTRY_DIAG] enabled run={} log_limit=3000 image_limit=20 image_dir={} image_ready={}",
      diagnostics_run_id, diagnostics_image_dir.string(), diagnostics_images_available);
    if (error) tools::logger()->warn("[INFANTRY_DIAG] image directory error: {}", error.message());
  }
  const bool direct_yaw_guard_enabled =
    integration_yaml["xuc_direct_yaw_guard"] &&
    integration_yaml["xuc_direct_yaw_guard"].as<bool>();
  const bool prediction_lead_limit_configured =
    integration_yaml["xuc_max_prediction_lead_rad"].IsDefined();
  const double max_prediction_lead_rad =
    prediction_lead_limit_configured
      ? std::max(0.0, integration_yaml["xuc_max_prediction_lead_rad"].as<double>())
      : 0.0;
  const double direct_yaw_filter_tau_s =
    integration_yaml["xuc_direct_yaw_filter_tau_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_filter_tau_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_yaw_hold_s =
    integration_yaml["xuc_direct_yaw_hold_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_hold_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_yaw_max_rate_rad_s =
    integration_yaml["xuc_direct_yaw_max_rate_dps"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_max_rate_dps"].as<double>()) * M_PI / 180.0
      : 0.0;
  const double final_yaw_command_max_rate_rad_s =
    integration_yaml["xuc_final_yaw_command_max_rate_dps"]
      ? std::max(
          0.0, integration_yaml["xuc_final_yaw_command_max_rate_dps"].as<double>()) *
          M_PI / 180.0
      : 0.0;
  // Keep the fixed camera/gimbal boresight zero separate from the empirical
  // impact correction.  The former centers the visual line of sight; the latter
  // deliberately leaves the armor off-centre so the bore, not the camera, points
  // at the desired impact point.
  const double direct_yaw_boresight_correction_rad =
    integration_yaml["xuc_direct_yaw_boresight_correction_deg"]
      ? std::clamp(
          integration_yaml["xuc_direct_yaw_boresight_correction_deg"].as<double>(),
          -5.0,
          5.0) * M_PI / 180.0
      : 0.0;
  const double direct_yaw_impact_correction_rad =
    integration_yaml["xuc_direct_yaw_impact_correction_deg"]
      ? std::clamp(
          integration_yaml["xuc_direct_yaw_impact_correction_deg"].as<double>(),
          -10.0,
          10.0) * M_PI / 180.0
      : 0.0;
  const double direct_yaw_command_correction_rad = std::clamp(
    direct_yaw_boresight_correction_rad + direct_yaw_impact_correction_rad,
    -10.0 * M_PI / 180.0,
    10.0 * M_PI / 180.0);
  // The lower yaw loop can retain a static error even though the absolute target
  // is received correctly (field logs showed about 1.2 deg).  Close that gap in
  // the one coordinate both packet directions actually share: raw lower-board
  // yaw.  This layer sits after target prediction, so it removes actuator lag and
  // bias without replacing the ESKF/phase lead used for moving targets.
  const bool yaw_tracking_pi_enabled =
    integration_yaml["xuc_yaw_tracking_pi_enabled"] &&
    integration_yaml["xuc_yaw_tracking_pi_enabled"].as<bool>();
  const double yaw_tracking_kp = integration_yaml["xuc_yaw_tracking_kp"]
    ? std::clamp(integration_yaml["xuc_yaw_tracking_kp"].as<double>(), 0.0, 2.0)
    : 0.0;
  const double yaw_tracking_ki_per_s = integration_yaml["xuc_yaw_tracking_ki_per_s"]
    ? std::clamp(integration_yaml["xuc_yaw_tracking_ki_per_s"].as<double>(), 0.0, 5.0)
    : 0.0;
  const double yaw_tracking_integral_limit_rad =
    (integration_yaml["xuc_yaw_tracking_integral_limit_deg"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_integral_limit_deg"].as<double>(),
          0.0,
          5.0)
      : 0.0) * M_PI / 180.0;
  const double yaw_tracking_output_limit_rad =
    (integration_yaml["xuc_yaw_tracking_output_limit_deg"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_output_limit_deg"].as<double>(),
          0.0,
          8.0)
      : 0.0) * M_PI / 180.0;
  const double yaw_tracking_integrate_error_limit_rad =
    (integration_yaml["xuc_yaw_tracking_integrate_error_limit_deg"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_integrate_error_limit_deg"].as<double>(),
          0.1,
          10.0)
      : 3.0) * M_PI / 180.0;
  const double yaw_tracking_arm_dwell_s =
    (integration_yaml["xuc_yaw_tracking_arm_dwell_ms"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_arm_dwell_ms"].as<double>(),
          0.0,
          2000.0)
      : 500.0) / 1000.0;
  if (yaw_tracking_pi_enabled) {
    tools::logger()->info(
      "[XUC][YAW-PI] enabled kp={:.2f} ki={:.2f}/s "
      "integral_limit={:.2f}deg output_limit={:.2f}deg "
      "integrate_error_limit={:.2f}deg arm_dwell={:.0f}ms",
      yaw_tracking_kp, yaw_tracking_ki_per_s,
      yaw_tracking_integral_limit_rad * 180.0 / M_PI,
      yaw_tracking_output_limit_rad * 180.0 / M_PI,
      yaw_tracking_integrate_error_limit_rad * 180.0 / M_PI,
      yaw_tracking_arm_dwell_s * 1000.0);
  }
  const bool direct_pitch_guard_enabled =
    integration_yaml["xuc_direct_pitch_guard"] &&
    integration_yaml["xuc_direct_pitch_guard"].as<bool>();
  const double direct_pitch_filter_tau_s =
    integration_yaml["xuc_direct_pitch_filter_tau_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_filter_tau_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_pitch_hold_s =
    integration_yaml["xuc_direct_pitch_hold_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_hold_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_pitch_max_rate_rad_s =
    integration_yaml["xuc_direct_pitch_max_rate_dps"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_max_rate_dps"].as<double>()) * M_PI / 180.0
      : 0.0;
  const double direct_pitch_error_gain =
    integration_yaml["xuc_direct_pitch_error_gain"]
      ? std::clamp(integration_yaml["xuc_direct_pitch_error_gain"].as<double>(), 0.0, 2.0)
      : 1.0;
  // Use this bounded value until the referee link reports a plausible measured
  // projectile speed; omni-code exposes that value in bullet_speed_TJ.
  const double xuc_projectile_speed_mps = integration_yaml["xuc_projectile_speed_mps"]
    ? std::clamp(integration_yaml["xuc_projectile_speed_mps"].as<double>(), 1.0, 50.0)
    : 23.0;
  // The real-shot reference was recorded at a specific friction-wheel speed.
  // Keep that calibration speed independent from the current launch speed;
  // otherwise changing wheel RPM would silently re-fit the fixed boresight bias
  // and cancel most of the intended trajectory change.
  const double xuc_ballistic_reference_projectile_speed_mps =
    integration_yaml["xuc_ballistic_reference_projectile_speed_mps"]
      ? std::clamp(
          integration_yaml["xuc_ballistic_reference_projectile_speed_mps"].as<double>(),
          1.0,
          50.0)
      : xuc_projectile_speed_mps;
  const bool xuc_ballistic_pitch_enabled = integration_yaml["xuc_ballistic_pitch_enabled"] &&
    integration_yaml["xuc_ballistic_pitch_enabled"].as<bool>();
  const double xuc_camera_forward_of_muzzle_m =
    integration_yaml["xuc_camera_forward_of_muzzle_m"]
      ? std::max(0.0, integration_yaml["xuc_camera_forward_of_muzzle_m"].as<double>())
      : 0.0;
  const double xuc_camera_above_muzzle_m = integration_yaml["xuc_camera_above_muzzle_m"]
    ? std::max(0.0, integration_yaml["xuc_camera_above_muzzle_m"].as<double>())
    : 0.0;
  const double xuc_ballistic_reference_camera_range_m =
    integration_yaml["xuc_ballistic_reference_camera_range_m"]
      ? std::max(0.10, integration_yaml["xuc_ballistic_reference_camera_range_m"].as<double>())
      : 1.0;
  const double xuc_ballistic_reference_total_drop_m =
    integration_yaml["xuc_ballistic_reference_total_drop_m"]
      ? std::max(0.0, integration_yaml["xuc_ballistic_reference_total_drop_m"].as<double>())
      : 0.0;
  const double xuc_ballistic_max_compensation_rad =
    (integration_yaml["xuc_ballistic_max_compensation_deg"]
       ? std::clamp(
           integration_yaml["xuc_ballistic_max_compensation_deg"].as<double>(), 0.0, 25.0)
       : 12.0) * M_PI / 180.0;
  const double xuc_ballistic_pitch_trim_rad =
    (integration_yaml["xuc_ballistic_pitch_trim_deg"]
       ? std::clamp(
           integration_yaml["xuc_ballistic_pitch_trim_deg"].as<double>(), -5.0, 5.0)
       : 0.0) * M_PI / 180.0;
  auto_aim::RobustRangeFilter::Config robust_range_config;
  robust_range_config.time_constant_s =
    integration_yaml["xuc_range_filter_tau_ms"]
      ? std::clamp(
          integration_yaml["xuc_range_filter_tau_ms"].as<double>(), 0.0, 500.0) /
          1000.0
      : 0.08;
  robust_range_config.max_rate_mps =
    integration_yaml["xuc_range_filter_max_rate_mps"]
      ? std::clamp(
          integration_yaml["xuc_range_filter_max_rate_mps"].as<double>(), 0.1, 20.0)
      : 6.0;
  robust_range_config.raw_blend =
    integration_yaml["xuc_range_filter_raw_blend"]
      ? std::clamp(
          integration_yaml["xuc_range_filter_raw_blend"].as<double>(), 0.0, 0.5)
      : 0.05;
  robust_range_config.max_raw_model_residual_m =
    integration_yaml["xuc_range_filter_max_raw_residual_m"]
      ? std::clamp(
          integration_yaml["xuc_range_filter_max_raw_residual_m"].as<double>(),
          0.05,
          2.0)
      : 0.40;
  const double direct_pitch_focal_y_px =
    integration_yaml["camera_matrix"] && integration_yaml["camera_matrix"].size() >= 5
      ? integration_yaml["camera_matrix"][4].as<double>()
      : 0.0;
  const double direct_yaw_focal_x_px =
    integration_yaml["camera_matrix"] && integration_yaml["camera_matrix"].size() >= 1
      ? integration_yaml["camera_matrix"][0].as<double>()
      : 0.0;
  const double xuc_pitch_sign_for_log = integration_yaml["xuc_pitch_sign"]
    ? integration_yaml["xuc_pitch_sign"].as<double>()
    : -1.0;
  const bool xuc_fire_guard_enabled = integration_yaml["xuc_fire_guard_enabled"] &&
    integration_yaml["xuc_fire_guard_enabled"].as<bool>();
  const double xuc_fire_stable_s = integration_yaml["xuc_fire_stable_ms"]
    ? std::max(0.0, integration_yaml["xuc_fire_stable_ms"].as<double>()) / 1000.0
    : 0.5;
  const double xuc_fire_yaw_tolerance_rad = integration_yaml["xuc_fire_yaw_tolerance_deg"]
    ? std::max(0.0, integration_yaml["xuc_fire_yaw_tolerance_deg"].as<double>()) * M_PI / 180.0
    : 1.0 * M_PI / 180.0;
  const double xuc_fire_pitch_tolerance_rad = integration_yaml["xuc_fire_pitch_tolerance_deg"]
    ? std::max(0.0, integration_yaml["xuc_fire_pitch_tolerance_deg"].as<double>()) * M_PI / 180.0
    : 1.0 * M_PI / 180.0;
  const bool xuc_gyro_center_follow_enabled =
    integration_yaml["xuc_gyro_center_follow_enabled"] &&
    integration_yaml["xuc_gyro_center_follow_enabled"].as<bool>();
  const double xuc_gyro_omega_threshold_rad_s =
    integration_yaml["xuc_gyro_omega_threshold_rad_s"]
      ? std::max(0.0, integration_yaml["xuc_gyro_omega_threshold_rad_s"].as<double>())
      : 1.0;
  const double xuc_vehicle_model_enter_rad_s =
    integration_yaml["xuc_vehicle_model_enter_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_vehicle_model_enter_rad_s"].as<double>(), 0.0, 10.0)
      : xuc_gyro_omega_threshold_rad_s;
  const double xuc_vehicle_model_exit_rad_s =
    integration_yaml["xuc_vehicle_model_exit_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_vehicle_model_exit_rad_s"].as<double>(), 0.0,
          xuc_vehicle_model_enter_rad_s)
      : std::min(0.5, xuc_vehicle_model_enter_rad_s);
  const double xuc_high_speed_enter_rad_s =
    integration_yaml["xuc_high_speed_enter_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_high_speed_enter_rad_s"].as<double>(), 0.1, 20.0)
      : 3.2;
  const double xuc_high_speed_exit_rad_s =
    integration_yaml["xuc_high_speed_exit_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_high_speed_exit_rad_s"].as<double>(), 0.0,
          xuc_high_speed_enter_rad_s)
      : std::min(2.5, xuc_high_speed_enter_rad_s);
  const double xuc_rotation_mode_dwell_s =
    integration_yaml["xuc_rotation_mode_dwell_ms"]
      ? std::clamp(
          integration_yaml["xuc_rotation_mode_dwell_ms"].as<double>(), 0.0, 1000.0) /
          1000.0
      : 0.15;
  const double xuc_rotation_tracker_dropout_hold_s =
    integration_yaml["xuc_rotation_tracker_dropout_hold_ms"]
      ? std::clamp(
          integration_yaml["xuc_rotation_tracker_dropout_hold_ms"].as<double>(),
          0.0, 1000.0) / 1000.0
      : 0.0;
  const double xuc_rotation_rate_filter_tau_s =
    integration_yaml["xuc_rotation_rate_filter_tau_ms"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_filter_tau_ms"].as<double>(),
          0.0, 1000.0) / 1000.0
      : 0.0;
  const double xuc_rotation_rate_disagreement_rad_s =
    integration_yaml["xuc_rotation_rate_disagreement_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_disagreement_rad_s"].as<double>(),
          0.1, 10.0)
      : 1.2;
  const double xuc_rotation_speed_step_reset_rad_s =
    integration_yaml["xuc_rotation_speed_step_reset_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_rotation_speed_step_reset_rad_s"].as<double>(),
          0.1, 10.0)
      : 1.2;
  const int xuc_rotation_rate_agreement_updates =
    integration_yaml["xuc_rotation_rate_agreement_updates"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_agreement_updates"].as<int>(), 1, 60)
      : 2;
  const int xuc_rotation_rate_disagreement_updates =
    integration_yaml["xuc_rotation_rate_disagreement_updates"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_disagreement_updates"].as<int>(), 2, 120)
      : 3;
  const double xuc_gyro_center_hold_s =
    integration_yaml["xuc_gyro_center_hold_ms"]
      ? std::max(0.0, integration_yaml["xuc_gyro_center_hold_ms"].as<double>()) / 1000.0
      : 0.6;
  const bool xuc_fire_require_tracker_converged =
    integration_yaml["xuc_fire_require_tracker_converged"] &&
    integration_yaml["xuc_fire_require_tracker_converged"].as<bool>();
  const bool xuc_one_face_once_fire =
    integration_yaml["xuc_one_face_once_fire"] &&
    integration_yaml["xuc_one_face_once_fire"].as<bool>();
  const double xuc_one_face_limit_rad =
    (integration_yaml["xuc_one_face_limit_deg"]
       ? std::clamp(integration_yaml["xuc_one_face_limit_deg"].as<double>(), 1.0, 45.0)
       : 12.0) * M_PI / 180.0;
  const double xuc_one_face_rearm_rad =
    (integration_yaml["xuc_one_face_rearm_deg"]
       ? std::clamp(integration_yaml["xuc_one_face_rearm_deg"].as<double>(), 5.0, 90.0)
       : 34.0) * M_PI / 180.0;
  const double xuc_gyro_fire_stable_s =
    integration_yaml["xuc_gyro_fire_stable_ms"]
      ? std::max(0.0, integration_yaml["xuc_gyro_fire_stable_ms"].as<double>()) / 1000.0
      : 0.03;
  // A stationary target is still qualified by the proven image-space gate.
  // In gyro mode, however, the visible plate is moving while the gimbal follows
  // the vehicle centre. Qualify the armor selected by Aimer at projectile impact
  // time instead of reusing the current-frame pixel error as the precise trigger.
  const bool xuc_gyro_predicted_fire_enabled =
    integration_yaml["xuc_gyro_predicted_fire_enabled"] &&
    integration_yaml["xuc_gyro_predicted_fire_enabled"].as<bool>();
  // A native yaw-error fallback was useful while bringing up the event clock,
  // but it has no phase/cycle information.  Let field configs fail closed while
  // the PLL is unlocked, without changing legacy configs that omit this key.
  const bool xuc_gyro_require_event_lock_for_fire =
    integration_yaml["xuc_gyro_require_event_lock_for_fire"] &&
    integration_yaml["xuc_gyro_require_event_lock_for_fire"].as<bool>();
  const double xuc_gyro_predicted_yaw_tolerance_rad =
    (integration_yaml["xuc_gyro_predicted_yaw_tolerance_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_predicted_yaw_tolerance_deg"].as<double>(),
           0.1,
           10.0)
       : 1.0) * M_PI / 180.0;
  const double xuc_gyro_predicted_yaw_trim_rad =
    (integration_yaml["xuc_gyro_predicted_yaw_trim_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_predicted_yaw_trim_deg"].as<double>(), -5.0, 5.0)
       : 0.0) * M_PI / 180.0;
  const double xuc_gyro_visibility_yaw_tolerance_rad =
    (integration_yaml["xuc_gyro_visibility_yaw_tolerance_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_visibility_yaw_tolerance_deg"].as<double>(),
           0.5,
           20.0)
       : 6.0) * M_PI / 180.0;
  const int xuc_gyro_fire_min_consecutive_frames =
    integration_yaml["xuc_gyro_fire_min_consecutive_frames"]
      ? std::clamp(
          integration_yaml["xuc_gyro_fire_min_consecutive_frames"].as<int>(), 1, 10)
      : 2;
  // Event-clock fire timing. The calibrated nominal remains the fallback;
  // the current lower-board protocol does not provide feeder timing feedback.
  const double xuc_event_fire_delay_s =
    integration_yaml["xuc_event_fire_delay_s"]
      ? std::clamp(
          integration_yaml["xuc_event_fire_delay_s"].as<double>(), 0.0, 0.5)
      : 0.10;
  const double xuc_event_fire_delay_min_s =
    integration_yaml["xuc_event_fire_delay_min_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_fire_delay_min_ms"].as<double>(), 0.0, 500.0) /
          1000.0
      : std::max(0.0, xuc_event_fire_delay_s - 0.020);
  const double xuc_event_fire_delay_max_s =
    integration_yaml["xuc_event_fire_delay_max_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_fire_delay_max_ms"].as<double>(), 0.0, 500.0) /
          1000.0
      : std::min(0.5, xuc_event_fire_delay_s + 0.020);
  auto_aim::ShotDelayAdapter::Config shot_delay_config;
  shot_delay_config.enabled =
    integration_yaml["xuc_shot_delay_adaptation_enabled"] &&
    integration_yaml["xuc_shot_delay_adaptation_enabled"].as<bool>();
  shot_delay_config.nominal_delay_s = xuc_event_fire_delay_s;
  shot_delay_config.min_delay_s = std::min(
    xuc_event_fire_delay_min_s, xuc_event_fire_delay_max_s);
  shot_delay_config.max_delay_s = std::max(
    xuc_event_fire_delay_min_s, xuc_event_fire_delay_max_s);
  shot_delay_config.reference_feedback_s =
    integration_yaml["xuc_shot_delay_reference_ms"]
      ? std::max(0.0, integration_yaml["xuc_shot_delay_reference_ms"].as<double>()) /
          1000.0
      : 0.0;
  shot_delay_config.sample_min_s =
    (integration_yaml["xuc_shot_delay_sample_min_ms"]
       ? std::clamp(
           integration_yaml["xuc_shot_delay_sample_min_ms"].as<double>(), 0.0, 200.0)
       : 5.0) / 1000.0;
  shot_delay_config.sample_max_s =
    (integration_yaml["xuc_shot_delay_sample_max_ms"]
       ? std::clamp(
           integration_yaml["xuc_shot_delay_sample_max_ms"].as<double>(), 5.0, 500.0)
       : 100.0) / 1000.0;
  shot_delay_config.window_size = static_cast<std::size_t>(
    integration_yaml["xuc_shot_delay_window"]
      ? std::clamp(integration_yaml["xuc_shot_delay_window"].as<int>(), 3, 31)
      : 9);
  shot_delay_config.min_samples = static_cast<std::size_t>(
    integration_yaml["xuc_shot_delay_min_samples"]
      ? std::clamp(integration_yaml["xuc_shot_delay_min_samples"].as<int>(), 3, 31)
      : 5);
  shot_delay_config.ewma_alpha =
    integration_yaml["xuc_shot_delay_ewma_alpha"]
      ? std::clamp(
          integration_yaml["xuc_shot_delay_ewma_alpha"].as<double>(), 0.01, 1.0)
      : 0.35;
  shot_delay_config.adaptation_gain =
    integration_yaml["xuc_shot_delay_adaptation_gain"]
      ? std::clamp(
          integration_yaml["xuc_shot_delay_adaptation_gain"].as<double>(), 0.0, 1.0)
      : 1.0;
  shot_delay_config.max_step_s =
    (integration_yaml["xuc_shot_delay_max_step_ms"]
       ? std::clamp(
           integration_yaml["xuc_shot_delay_max_step_ms"].as<double>(), 0.0, 10.0)
       : 1.5) / 1000.0;
  shot_delay_config.outlier_sigma =
    integration_yaml["xuc_shot_delay_outlier_sigma"]
      ? std::clamp(
          integration_yaml["xuc_shot_delay_outlier_sigma"].as<double>(), 1.0, 10.0)
      : 3.5;
  shot_delay_config.outlier_floor_s =
    (integration_yaml["xuc_shot_delay_outlier_floor_ms"]
       ? std::clamp(
           integration_yaml["xuc_shot_delay_outlier_floor_ms"].as<double>(), 0.0, 50.0)
       : 4.0) / 1000.0;
  shot_delay_config.min_prediction_half_width_s =
    (integration_yaml["xuc_shot_delay_95pi_min_half_ms"]
       ? std::clamp(
           integration_yaml["xuc_shot_delay_95pi_min_half_ms"].as<double>(), 1.0, 30.0)
       : 8.0) / 1000.0;
  const double xuc_event_flight_time_uncertainty_s =
    integration_yaml["xuc_event_flight_time_uncertainty_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_flight_time_uncertainty_ms"].as<double>(),
          0.0,
          100.0) /
          1000.0
      : 0.005;
  // If the precise due time lies shortly after this frame, wait until that
  // steady-clock instant and send the one-frame request there.  This removes the
  // 16 ms quantization of a ~60 FPS vision loop without delaying ordinary
  // tracking frames.
  const double xuc_event_trigger_window_s =
    integration_yaml["xuc_event_trigger_window_ms"]
      ? std::max(0.0, integration_yaml["xuc_event_trigger_window_ms"].as<double>()) /
          1000.0
      : 0.025;
  const double xuc_event_send_late_tolerance_s =
    integration_yaml["xuc_event_send_late_tolerance_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_send_late_tolerance_ms"].as<double>(),
          0.0,
          10.0) /
          1000.0
      : 0.002;
  const double xuc_event_crossing_capture_rad =
    (integration_yaml["xuc_event_crossing_capture_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_crossing_capture_deg"].as<double>(), 5.0, 45.0)
       : 20.0) * M_PI / 180.0;
  const double xuc_event_crossing_rearm_rad =
    (integration_yaml["xuc_event_crossing_rearm_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_crossing_rearm_deg"].as<double>(), 2.0, 35.0)
       : 12.0) * M_PI / 180.0;
  const double xuc_event_min_crossing_separation_ratio =
    integration_yaml["xuc_event_min_crossing_separation_ratio"]
      ? std::clamp(
          integration_yaml["xuc_event_min_crossing_separation_ratio"].as<double>(),
          0.20,
          0.90)
      : 0.55;
  const double xuc_event_period_min_s =
    integration_yaml["xuc_event_period_min_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_period_min_ms"].as<double>(), 30.0, 2000.0) /
          1000.0
      : 0.12;
  const double xuc_event_period_max_s =
    integration_yaml["xuc_event_period_max_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_period_max_ms"].as<double>(), 50.0, 5000.0) /
          1000.0
      : 1.20;
  const double xuc_event_period_ema_alpha =
    integration_yaml["xuc_event_period_ema_alpha"]
      ? std::clamp(
          integration_yaml["xuc_event_period_ema_alpha"].as<double>(), 0.01, 1.0)
      : 0.25;
  const double xuc_event_max_period_relative_error =
    integration_yaml["xuc_event_max_period_relative_error"]
      ? std::clamp(
          integration_yaml["xuc_event_max_period_relative_error"].as<double>(),
          0.05,
          0.50)
      : 0.22;
  const double xuc_event_max_period_step_ratio =
    integration_yaml["xuc_event_max_period_step_ratio"]
      ? std::clamp(
          integration_yaml["xuc_event_max_period_step_ratio"].as<double>(),
          0.001,
          0.20)
      : 0.025;
  const double xuc_event_omega_period_weight =
    integration_yaml["xuc_event_omega_period_weight"]
      ? std::clamp(
          integration_yaml["xuc_event_omega_period_weight"].as<double>(),
          0.0,
          1.0)
      : 0.15;
  const double xuc_event_period_residual_alpha =
    integration_yaml["xuc_event_period_residual_alpha"]
      ? std::clamp(
          integration_yaml["xuc_event_period_residual_alpha"].as<double>(),
          0.0,
          0.5)
      : 0.08;
  const double xuc_event_phase_correction_alpha =
    integration_yaml["xuc_event_phase_correction_alpha"]
      ? std::clamp(
          integration_yaml["xuc_event_phase_correction_alpha"].as<double>(),
          0.0,
          1.0)
      : 0.45;
  const double xuc_event_max_phase_correction_s =
    integration_yaml["xuc_event_max_phase_correction_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_max_phase_correction_ms"].as<double>(),
          0.0,
          20.0) /
          1000.0
      : 0.015;
  const double xuc_event_max_phase_residual_s =
    integration_yaml["xuc_event_max_phase_residual_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_max_phase_residual_ms"].as<double>(),
          5.0,
          150.0) /
          1000.0
      : 0.090;
  const int xuc_event_period_window_size =
    integration_yaml["xuc_event_period_window_size"]
      ? std::clamp(
          integration_yaml["xuc_event_period_window_size"].as<int>(), 3, 15)
      : 7;
  const double xuc_event_phase_uncertainty_floor_s =
    integration_yaml["xuc_event_phase_uncertainty_floor_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_phase_uncertainty_floor_ms"].as<double>(),
          0.0,
          50.0) /
          1000.0
      : 0.002;
  const int xuc_event_phase_residual_window_size =
    integration_yaml["xuc_event_phase_residual_window_size"]
      ? std::clamp(
          integration_yaml["xuc_event_phase_residual_window_size"].as<int>(), 3, 15)
      : 5;
  const int xuc_event_min_lock_updates =
    integration_yaml["xuc_event_min_lock_updates"]
      ? std::clamp(
          integration_yaml["xuc_event_min_lock_updates"].as<int>(), 2, 12)
      : 4;
  const int xuc_event_relock_rejections =
    integration_yaml["xuc_event_relock_rejections"]
      ? std::clamp(
          integration_yaml["xuc_event_relock_rejections"].as<int>(), 2, 12)
      : 3;
  const double xuc_event_armor_width_m =
    integration_yaml["xuc_event_armor_width_m"]
      ? std::clamp(
          integration_yaml["xuc_event_armor_width_m"].as<double>(), 0.05, 0.40)
      : 0.135;
  const double xuc_event_armor_edge_margin_m =
    integration_yaml["xuc_event_armor_edge_margin_m"]
      ? std::clamp(
          integration_yaml["xuc_event_armor_edge_margin_m"].as<double>(), 0.0, 0.10)
      : 0.005;
  const double xuc_event_model_phase_uncertainty_rad =
    (integration_yaml["xuc_event_model_phase_uncertainty_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_model_phase_uncertainty_deg"].as<double>(),
           0.0,
           20.0)
       : 1.0) * M_PI / 180.0;
  // A small, explicit grace angle relaxes the conservative worst-case budget
  // without bypassing phase lock or gimbal tracking.
  const double xuc_event_phase_grace_rad =
    (integration_yaml["xuc_event_phase_grace_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_phase_grace_deg"].as<double>(), 0.0, 4.0)
       : 0.0) * M_PI / 180.0;
  // The crossing PLL observes the physical face-center event directly.  The
  // EKF future-face prediction is useful telemetry, but its phase ripples at
  // armor hand-over can be made a soft check for higher fire availability.
  const bool xuc_event_require_face_consistency =
    !integration_yaml["xuc_event_require_face_consistency"] ||
    integration_yaml["xuc_event_require_face_consistency"].as<bool>();
  const double xuc_event_face_max_alignment_rad =
    (integration_yaml["xuc_event_face_max_alignment_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_face_max_alignment_deg"].as<double>(),
           1.0,
           45.0)
       : 15.0) * M_PI / 180.0;
  const double xuc_event_gimbal_tracking_tolerance_rad =
    (integration_yaml["xuc_event_gimbal_tracking_tolerance_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_gimbal_tracking_tolerance_deg"].as<double>(),
           0.2,
           10.0)
       : 2.5) * M_PI / 180.0;
  const double xuc_shot_rearm_s =
    integration_yaml["xuc_shot_rearm_ms"]
      ? std::clamp(
          integration_yaml["xuc_shot_rearm_ms"].as<double>(), 0.0, 2000.0) /
          1000.0
      : 0.70;
  // A face switch is approximately 90 degrees on a four-armor target. The
  // same face moves only about five degrees per frame at 60 FPS and 5 rad/s.
  // Reject a two-frame qualification that silently crosses a face switch.
  const double xuc_gyro_candidate_max_phase_step_rad =
    (integration_yaml["xuc_gyro_candidate_max_phase_step_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_candidate_max_phase_step_deg"].as<double>(),
           5.0,
           45.0)
       : 20.0) * M_PI / 180.0;

#ifdef AMENT_CMAKE_FOUND
  // 初始化ROS2可视化
  rclcpp::init(argc, argv);
  auto visualizer = std::make_shared<tools::ROS2Visualizer>("standard_node", "standard_markers");
  tools::logger()->info("[ROS2] Visualizer initialized");
#endif

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  io::XucSender xuc(config_path);  // 新版XUC：接收IMU，发送目标角

  auto_aim::YOLO detector(config_path, false);  // 启用调试，显示检测窗口
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  const bool roi_reacquisition_enabled =
    integration_yaml["use_roi"].as<bool>() &&
    integration_yaml["yolo_name"].as<std::string>() == "yolo26";
  const cv::Rect configured_detection_roi(
    integration_yaml["roi"]["x"].as<int>(), integration_yaml["roi"]["y"].as<int>(),
    integration_yaml["roi"]["width"].as<int>(), integration_yaml["roi"]["height"].as<int>());
  bool full_frame_search = true;
  auto_aim::Aimer aimer(config_path);
  auto_aim::ShotDelayAdapter shot_delay_adapter(shot_delay_config);
  auto_aim::Shooter shooter(config_path);
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter, true);
  auto_aim::GyroPhaseScheduler::Config gyro_phase_config;
  gyro_phase_config.crossing_capture_rad = xuc_event_crossing_capture_rad;
  gyro_phase_config.crossing_rearm_rad = xuc_event_crossing_rearm_rad;
  gyro_phase_config.min_crossing_separation_ratio =
    xuc_event_min_crossing_separation_ratio;
  gyro_phase_config.min_face_period_s = std::min(
    xuc_event_period_min_s, xuc_event_period_max_s);
  gyro_phase_config.max_face_period_s = std::max(
    xuc_event_period_min_s, xuc_event_period_max_s);
  gyro_phase_config.period_ema_alpha = xuc_event_period_ema_alpha;
  gyro_phase_config.max_period_relative_error =
    xuc_event_max_period_relative_error;
  gyro_phase_config.max_period_step_ratio = xuc_event_max_period_step_ratio;
  gyro_phase_config.omega_period_weight = xuc_event_omega_period_weight;
  gyro_phase_config.omega_alias_guard_relative_error = 0.35;
  gyro_phase_config.period_residual_alpha = xuc_event_period_residual_alpha;
  gyro_phase_config.phase_correction_alpha = xuc_event_phase_correction_alpha;
  gyro_phase_config.max_phase_correction_s = xuc_event_max_phase_correction_s;
  gyro_phase_config.max_phase_residual_s = xuc_event_max_phase_residual_s;
  gyro_phase_config.phase_uncertainty_floor_s = xuc_event_phase_uncertainty_floor_s;
  gyro_phase_config.period_window_size = xuc_event_period_window_size;
  gyro_phase_config.phase_residual_window_size = xuc_event_phase_residual_window_size;
  gyro_phase_config.min_lock_updates = xuc_event_min_lock_updates;
  gyro_phase_config.relock_after_rejections = xuc_event_relock_rejections;
  auto_aim::GyroPhaseScheduler gyro_phase_scheduler(gyro_phase_config);
  auto_aim::GyroFaceSelector gyro_face_selector(xuc_event_face_max_alignment_rad);
  auto_aim::RobustRangeFilter robust_range_filter(robust_range_config);
  auto_aim::RotationModelManager::Config rotation_model_config;
  rotation_model_config.vehicle_enter_rad_s = xuc_vehicle_model_enter_rad_s;
  rotation_model_config.vehicle_exit_rad_s = xuc_vehicle_model_exit_rad_s;
  rotation_model_config.high_speed_phase_enabled =
    integration_yaml["xuc_high_speed_phase_enabled"]
      ? integration_yaml["xuc_high_speed_phase_enabled"].as<bool>() : true;
  rotation_model_config.high_speed_enter_rad_s = xuc_high_speed_enter_rad_s;
  rotation_model_config.high_speed_exit_rad_s = xuc_high_speed_exit_rad_s;
  rotation_model_config.mode_dwell_s = xuc_rotation_mode_dwell_s;
  rotation_model_config.tracker_dropout_hold_s =
    xuc_rotation_tracker_dropout_hold_s;
  rotation_model_config.rate_filter_tau_s = xuc_rotation_rate_filter_tau_s;
  rotation_model_config.max_rate_disagreement_rad_s =
    xuc_rotation_rate_disagreement_rad_s;
  rotation_model_config.speed_step_reset_rad_s =
    xuc_rotation_speed_step_reset_rad_s;
  rotation_model_config.rate_agreement_updates =
    xuc_rotation_rate_agreement_updates;
  rotation_model_config.rate_disagreement_updates =
    xuc_rotation_rate_disagreement_updates;
  auto_aim::RotationModelManager rotation_model_manager(rotation_model_config);
  // 🎯 所有模块初始化完成，启动相机触发
  tools::logger()->info("=== All modules initialized ===");

#ifdef AMENT_CMAKE_FOUND
  // 发布静态TF: gimbal -> camera（使用标定参数）
  visualizer->publish_static_tf("gimbal", "camera",
    solver.R_camera2gimbal(),
    solver.t_camera2gimbal());
  tools::logger()->info("[ROS2] Published static TF: gimbal -> camera");
#endif

  tools::logger()->info("=== Entering main loop ===");

  cv::Mat img;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  // 性能分析计时器
  std::chrono::steady_clock::time_point t_start, t_end;

  // 🆕 同步匹配相关变量（需要在循环外声明，以便后续日志使用）
  uint64_t frame_id = 0;
  uint64_t frame_id_last =0;
  int64_t trigger_imu_count = 0;

  // 📊 相机帧率测量变量
  std::chrono::steady_clock::time_point camera_last_frame_time;
  std::chrono::steady_clock::time_point camera_current_frame_time;
  double camera_fps_instant = 0.0;     // 瞬时帧率
  double camera_fps_avg = 0.0;         // 平均帧率
  int camera_frame_count = 0;          // 帧计数

  // Bounded yaw conditioner.  Detector measurements remain authoritative while
  // a small, validated native prediction lead helps fast lateral tracking.
  // Long visual loss drops control without destroying reacquisition continuity.
  bool direct_yaw_initialized = false;
  bool direct_yaw_control_confirmed = false;
  double direct_yaw_filtered = 0.0;
  bool direct_yaw_long_loss = false;
  std::chrono::steady_clock::time_point direct_yaw_last_update;
  std::chrono::steady_clock::time_point direct_yaw_last_measurement;
  // Limit the complete yaw reference after model selection and prediction.
  // The measurement conditioner alone cannot bound an abruptly changing
  // native prediction lead or a vehicle-center/armor-model hand-over.
  bool final_yaw_command_initialized = false;
  bool final_yaw_lower_armed = false;
  double final_yaw_command_filtered = 0.0;
  std::chrono::steady_clock::time_point final_yaw_command_last_update;
  double yaw_tracking_integral_raw_rad = 0.0;
  std::chrono::steady_clock::time_point yaw_tracking_last_update;
  std::chrono::steady_clock::time_point yaw_tracking_armed_since;
  bool yaw_tracking_initialized = false;
  bool yaw_tracking_lower_armed = false;
  bool direct_pitch_initialized = false;
  double direct_pitch_filtered = 0.0;
  bool direct_pitch_long_loss = false;
  std::chrono::steady_clock::time_point direct_pitch_last_update;
  std::chrono::steady_clock::time_point direct_pitch_last_measurement;
  bool xuc_fire_candidate_active = false;
  RotationFireWindow rotation_fire_window;
  bool xuc_fire_ready_logged = false;
  int xuc_fire_candidate_frames = 0;
  std::chrono::steady_clock::time_point xuc_fire_candidate_since;
  bool xuc_one_face_armed = true;
  bool xuc_gyro_center_follow_logged = false;
  bool xuc_gyro_center_memory_valid = false;
  double xuc_gyro_center_memory_yaw = 0.0;
  std::chrono::steady_clock::time_point xuc_gyro_center_last_fresh;
  bool xuc_gyro_cadence_hold_logged = false;
  std::int64_t event_last_fired_cycle = -1;
  uint64_t xuc_next_shot_id = 1;
  uint64_t xuc_pending_shot_id = 0;
  bool xuc_pending_shot = false;
  bool xuc_pending_shot_tx_ok = false;
  bool xuc_pending_slot_estimate_logged = false;
  std::chrono::steady_clock::time_point xuc_pending_shot_time;
  double xuc_pending_feeder_turns_at_request = NAN;
  std::chrono::steady_clock::time_point fps_measure_start; // 平均帧率测量开始时间
  bool fps_measure_started = false;
  std::size_t diagnostics_records = 0;
  std::size_t diagnostics_images = 0;
  std::chrono::steady_clock::time_point diagnostics_last_record{};
  std::chrono::steady_clock::time_point diagnostics_last_image{};
  bool diagnostics_previous_armor = false;
  bool diagnostics_previous_target = false;
  bool diagnostics_previous_control = false;
  std::unique_ptr<ShotPredictionDebug> shot_prediction_debug;
  std::unique_ptr<PredictionSessionDebug> prediction_session_debug;
  if (infantry_diagnostics_enabled && xuc.shot_debug_enabled()) {
    try {
      shot_prediction_debug = std::make_unique<ShotPredictionDebug>();
      prediction_session_debug = std::make_unique<PredictionSessionDebug>();
    } catch (const std::exception & e) {
      tools::logger()->warn("[SHOT_DEBUG] frame capture unavailable: {}", e.what());
    }
  }
  constexpr std::array<const char *, 14> fire_gate_names = {
    "detection", "target", "tracker_converged", "frame_imu", "auto_mode",
    "control", "yaw_observation", "pitch_observation", "yaw_error",
    "pitch_error", "face", "motion_transition", "command_rearm", "stable_window"};
  std::array<uint64_t, fire_gate_names.size()> fire_blocked_frames{};
  std::array<double, fire_gate_names.size()> fire_blocked_ms{};
  std::array<double, fire_gate_names.size()> fire_longest_episode_ms{};
  std::array<bool, fire_gate_names.size()> fire_gate_open{};
  bool fire_gate_snapshot_valid = false;
  int fire_first_blocker = -1;
  int fire_previous_blocker = -1;
  double fire_current_episode_ms = 0.0;
  std::chrono::steady_clock::time_point fire_last_gate_sample{};
  std::chrono::steady_clock::time_point fire_last_gate_summary{};

  // 初始化默认命令（第一帧使用）
  io::Command default_command;
  default_command.yaw = 0.0;
  default_command.pitch = 0.0;
  default_command.control = false;
  default_command.shoot = false;

  while (!exiter.exit()) {
    auto loop_start = std::chrono::steady_clock::now();  // 🔍 性能监控：循环开始

    // Send the initial neutral command before reading the continuously
    // acquiring HikRobot camera. Its exposure is not triggered by this packet.
    static bool first_loop = true;
    if (first_loop) {
      // 第一帧发送默认命令
      if (xuc.enabled()) {
        xuc.send(default_command);
      } else {
        cboard.send(default_command);
      }
      tools::logger()->info("[SYNC] 发送默认命令，等待相机触发");
      first_loop = false;
    }

    camera.read(img, t);
    if (infantry_diagnostics_enabled && shot_prediction_debug) {
      *shot_prediction_debug = ShotPredictionDebug{};
    }

    // 📊 相机帧率测量
    camera_current_frame_time = std::chrono::steady_clock::now();
    if (!fps_measure_started) {
      // 初始化帧率测量
      camera_last_frame_time = camera_current_frame_time;
      fps_measure_start = camera_current_frame_time;
      fps_measure_started = true;
      camera_frame_count = 0;
    } else {
      // 计算瞬时帧率（基于相邻两帧的时间间隔）
      auto frame_interval = std::chrono::duration<double>(camera_current_frame_time - camera_last_frame_time).count();
      if (frame_interval > 0) {
        camera_fps_instant = 1.0 / frame_interval;
      }

      // 计算平均帧率（基于总时间和总帧数）
      camera_frame_count++;
      auto total_time = std::chrono::duration<double>(camera_current_frame_time - fps_measure_start).count();
      if (total_time > 0) {
        camera_fps_avg = camera_frame_count / total_time;
      }

      // 更新上一帧时间
      camera_last_frame_time = camera_current_frame_time;

      // 每50帧打印一次帧率统计
      static int fps_log_counter = 0;
      if (++fps_log_counter % 50 == 0) {
        tools::logger()->info(
          "📊 [CAMERA FPS] 瞬时帧率={:.1f}fps, 平均帧率={:.1f}fps, 总帧数={}, 运行时间={:.1f}s",
          camera_fps_instant, camera_fps_avg, camera_frame_count, total_time);
      }
    }

      // ==================== 基于 count 硬同步（使用环形数组） ====================
      // 核心思想：相机由MCU硬触发,每来一帧图像对应一个IMU计数（0-15循环）
      // 映射关系：frame_id % 16 = imu_count
      // MCU发送的imu_count是4位二进制，范围0-15
      static const int64_t frame_id_to_imu_offset = 0;  // 🔧 手动调试参数（微调偏移）

      static bool first_frame = true;

      frame_id = camera.get_last_frame_id();

      // 新版硬件中IMU连接下位机，姿态由XUC上行包提供。
      bool xuc_imu_sync_valid = false;
      io::XucTimedImu frame_imu;
      std::chrono::steady_clock::time_point frame_imu_query_time{};
      std::chrono::steady_clock::time_point frame_imu_requested_time{};
      if (xuc.enabled()) {
        const auto feedback_now = std::chrono::steady_clock::now();
        if (infantry_diagnostics_enabled && xuc_pending_shot &&
            xuc_pending_shot_tx_ok && !xuc_pending_slot_estimate_logged) {
          const double feeder_turns_est =
            xuc.feeder_motor_turns_estimate() - xuc_pending_feeder_turns_at_request;
          if (feeder_turns_est >= 4.5) {
            const double elapsed_ms = std::chrono::duration<double, std::milli>(
              feedback_now - xuc_pending_shot_time).count();
            tools::logger()->info(
              "[SHOT_FEED_EST] id={} after_tx_ms={:.1f} "
              "feeder_motor_turns_est={:+.2f} projectile_exit=unknown",
              xuc_pending_shot_id, elapsed_ms, feeder_turns_est);
            xuc_pending_slot_estimate_logged = true;
          }
        }
        if (xuc_pending_shot &&
            std::chrono::duration<double>(feedback_now - xuc_pending_shot_time).count() >=
              xuc_shot_rearm_s &&
            // With no extra timer, complete the request only after wire release.
            (xuc_shot_rearm_s > 0.0 || xuc.shoot_request_ready())) {
          const double feeder_turns_est =
            xuc.feeder_motor_turns_estimate() - xuc_pending_feeder_turns_at_request;
          if (infantry_diagnostics_enabled) {
            tools::logger()->info(
              "[SHOT] id={} command_window_complete rearm_ms={:.0f} "
              "feeder_motor_turns_est={:+.2f} one_slot_motor_turns=4.50 "
              "one_slot_rotation_est={} projectile_exit=unknown",
              xuc_pending_shot_id, xuc_shot_rearm_s * 1000.0,
              feeder_turns_est, feeder_turns_est >= 4.5 ? 1 : 0);
          } else {
            tools::logger()->info(
              "[SHOT] id={} command_window_complete rearm_ms={:.0f} "
              "feeder_telemetry=unavailable",
              xuc_pending_shot_id, xuc_shot_rearm_s * 1000.0);
          }
          xuc_pending_shot = false;
          xuc_pending_shot_tx_ok = false;
          xuc_pending_shot_id = 0;
        }
      // 海康等不提供 frame_id 的相机使用 steady_clock 时间戳插值。
      } else if (frame_id == 0) {
        q = cboard.imu_at(t);
        solver.set_R_gimbal2world(q);

        static int timestamp_sync_log_counter = 0;
        if (++timestamp_sync_log_counter % 50 == 0) {
          auto sync_ypr = tools::eulers(q.toRotationMatrix(), 2, 1, 0);
          tools::logger()->info(
            "[SYNC][Timestamp] yaw={:.2f} deg pitch={:.2f} deg roll={:.2f} deg",
            sync_ypr[0] * 180.0 / M_PI,
            sync_ypr[1] * 180.0 / M_PI,
            sync_ypr[2] * 180.0 / M_PI);
        }

        mode = cboard.mode;
      } else if (frame_id - frame_id_last != 0) {
      // ✅ 修复：根据frame_id计算对应的IMU计数（0-15循环）
      trigger_imu_count = (frame_id + frame_id_to_imu_offset) % 16;
      if (trigger_imu_count < 0) trigger_imu_count += 16;

      // 🔍 检查frame_id是否跳帧
      if (frame_id - frame_id_last > 1) {
        tools::logger()->warn("[SYNC] ⚠️ 相机跳帧！frame_id: {} -> {} (跳过{}帧)",
          frame_id_last, frame_id, frame_id - frame_id_last - 1);
      }

      //使用环形数组O(1)查询IMU数据
      auto imu_result = cboard.get_imu_from_ring_buffer(trigger_imu_count);

      if (imu_result.valid) {
        // 环形数组查询成功
        q = imu_result.q;  // 四元数
        t = imu_result.timestamp;

        // 🔍 计算帧间隔和跳帧情况
        static auto last_t = t;
        static uint64_t last_valid_frame_id = 0;
        auto frame_interval_ms = std::chrono::duration<double, std::milli>(t - last_t).count();
        uint64_t frame_gap = frame_id - last_valid_frame_id;

        // 相机理论帧率 = 1000 / (实际间隔ms / 实际帧数差)
        double camera_theoretical_fps = frame_gap * 1000.0 / frame_interval_ms;

        last_t = t;
        last_valid_frame_id = frame_id;

        // 🔍 同步调试日志（每50帧打印一次）
        static int sync_log_counter = 0;
        if (++sync_log_counter % 50 == 0) {
          tools::logger()->info("[SYNC] ✅ frame_id={}, trigger_imu_count={}, 间隔={:.1f}ms, 跳过{}帧, 相机理论帧率≈{:.0f}fps",
            frame_id, trigger_imu_count, frame_interval_ms, frame_gap - 1, camera_theoretical_fps);
        }

        // ✅ 在获取到有效IMU数据后立即设置solver，确保姿态同步
        solver.set_R_gimbal2world(q);

#ifdef AMENT_CMAKE_FOUND
        // 发布动态TF: world -> gimbal（使用MCU四元数）
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          t.time_since_epoch()).count();
        rclcpp::Time ros_time(ns);
        Eigen::Vector3d zero_trans(0, 0, 0);  // world和gimbal原点重合
        visualizer->publish_dynamic_tf("world", "gimbal", q, zero_trans, ros_time);
#endif
      } else {
        // 🔍 同步调试：IMU数据无效（可能是时序问题或环形缓冲区被覆盖）
        tools::logger()->warn("[SYNC] ❌ frame_id={}, trigger_imu_count={}, IMU数据无效（可能还未到达或已被覆盖）",
          frame_id, trigger_imu_count);
      }
    mode = cboard.mode;
    frame_id_last=frame_id;
     }
    auto t1 = std::chrono::steady_clock::now();  // 🔍 性能监控
    // YOLO26 color memory is measured in frames. Pass a real increasing frame
    // index; the implicit -1 previously kept stale color memories forever.
    const bool full_frame_search_used = roi_reacquisition_enabled && full_frame_search;
    auto armors = detector.detect(img, camera_frame_count, full_frame_search_used);
    const auto detection_diagnostics = infantry_diagnostics_enabled
      ? detector.diagnostics() : auto_aim::YoloDetectionDiagnostics{};
    const std::size_t armors_before_color = armors.size();
    const std::size_t armors_enemy_color = infantry_diagnostics_enabled
      ? static_cast<std::size_t>(std::count_if(
          armors.begin(), armors.end(),
          [diagnostic_enemy_color](const auto & armor) {
            return armor.color == diagnostic_enemy_color;
          }))
      : 0;
    auto t2 = std::chrono::steady_clock::now();
    // Query the same image-relative pose after inference, when later IMU samples
    // can already be in the history. Do not wait or replace the requested time
    // with the current pose; PnP, tracker and diagnostics must share this result.
    if (xuc.enabled()) {
      if (xuc.rx_valid()) {
        frame_imu_requested_time = t + xuc_imu_alignment;
        frame_imu_query_time = std::chrono::steady_clock::now();
        xuc_imu_sync_valid = xuc.imu_at(frame_imu_requested_time, frame_imu);
        // Preserve a pose for tracker continuity if the timestamp is outside
        // the bounded history, but block new control/fire requests on that frame.
        const double imu_yaw = xuc_imu_sync_valid ? frame_imu.yaw : xuc.imu_yaw();
        const double imu_pitch = xuc_imu_sync_valid ? frame_imu.pitch : xuc.imu_pitch();
        q = Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(imu_pitch, Eigen::Vector3d::UnitY());
        q.normalize();
        solver.set_R_gimbal2world(q);
        cboard.mode = xuc_imu_sync_valid
          ? static_cast<io::Mode>(xuc.mode()) : io::Mode::idle;
        const double lower_projectile_speed_mps = xuc.bullet_speed();
        const bool lower_projectile_speed_valid =
          std::isfinite(lower_projectile_speed_mps) &&
          lower_projectile_speed_mps >= 5.0 && lower_projectile_speed_mps <= 40.0;
        cboard.bullet_speed = lower_projectile_speed_valid
          ? lower_projectile_speed_mps : xuc_projectile_speed_mps;
        if (infantry_diagnostics_enabled && shot_prediction_debug) {
          shot_prediction_debug->feedback_speed_mps = lower_projectile_speed_mps;
          shot_prediction_debug->latest_rx_ns = xuc.last_rx_ns();
          shot_prediction_debug->input_source = lower_projectile_speed_valid
            ? "feedback_field_unverified" : "xuc_projectile_speed_mps";
        }
        mode = cboard.mode;

        static int xuc_imu_log_counter = 0;
        static bool xuc_startup_sync_logged = false;
        const int xuc_log_period_frames = (xuc.mode() == 1) ? 5 : 50;
        if (!xuc_startup_sync_logged ||
            (infantry_diagnostics_enabled &&
             ++xuc_imu_log_counter % xuc_log_period_frames == 0)) {
          tools::logger()->info(
            "[SYNC][XUC] mode={} robot_id={} yaw={:.2f} deg pitch={:.2f} deg "
            "projectile_speed_feedback={:.2f} projectile_speed_used={:.2f} "
            "feedback_seq={} tx_control={} tx_shoot={} "
            "frame_imu_sync={} interpolated={} extrapolated={} alignment_ms={:+.1f} bracket_ms={:.2f} "
            "nearest_offset_ms={:+.2f}",
            static_cast<int>(xuc.mode()),
            static_cast<unsigned>(xuc.robot_id()),
            imu_yaw * 180.0 / M_PI, imu_pitch * 180.0 / M_PI,
            lower_projectile_speed_mps, cboard.bullet_speed,
            static_cast<unsigned>(xuc.bullet_count()),
            xuc.last_tx_control() ? 1 : 0,
            static_cast<unsigned>(xuc.last_tx_shoot()),
            xuc_imu_sync_valid ? 1 : 0,
            xuc_imu_sync_valid && frame_imu.interpolated ? 1 : 0,
            xuc_imu_sync_valid && frame_imu.extrapolated ? 1 : 0,
            xuc_imu_alignment_ms,
            xuc_imu_sync_valid ? frame_imu.bracket_ms : NAN,
            xuc_imu_sync_valid ? frame_imu.nearest_offset_ms : NAN);
          xuc_startup_sync_logged = true;
        }
      } else {
        cboard.mode = io::Mode::idle;
        mode = io::Mode::idle;
      }
    }
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }
    // recorder.record(img, q, t);
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto targets = tracker.track(armors, t);
    if (roi_reacquisition_enabled) {
      if (tracker.state() == "tracking") {
        const auto * observation = tracker.primary_observation();
        const cv::Rect2f tracking_roi(
          configured_detection_roi.x, configured_detection_roi.y,
          configured_detection_roi.width == -1 ? img.cols : configured_detection_roi.width,
          configured_detection_roi.height == -1 ? img.rows : configured_detection_roi.height);
        full_frame_search = !(observation != nullptr && observation->points.size() == 4 &&
          std::all_of(observation->points.begin(), observation->points.end(),
            [&](const auto & point) { return tracking_roi.contains(point); }));
      } else if (tracker.state() != "temp_lost" || tracker.temp_lost_count() >= 3) {
        full_frame_search = true;
      }
      // One/two missed associations retain the current search region. A third
      // miss enables full-frame detection on the next image, without resetting Tracker.
    }
    const std::size_t armors_after_color = armors.size();
    auto t3 = std::chrono::steady_clock::now();

    bool tracker_converged = false;
    bool yaw_motion_model_trusted = false;
    bool vehicle_model_fresh = false;
    bool gyro_model_fresh = false;
    bool gyro_phase_model_trusted = false;
    bool gyro_center_follow_active = false;
    double gyro_center_yaw = 0.0;
    double gyro_body_phase_rad = NAN;
    double gyro_omega_rad_s = 0.0;
    double gyro_radius_m = 0.0;
    if (!targets.empty()) {
      auto & tracked_target = targets.front();
      tracker_converged = tracked_target.convergened();
      yaw_motion_model_trusted = tracked_target.yaw_eskf_trusted();
      const auto tracked_x = tracked_target.ekf_x();
      if (tracked_x.size() >= 9) {
        gyro_center_yaw = std::atan2(tracked_x[2], tracked_x[0]);
        gyro_body_phase_rad = tracked_x[6];
        gyro_omega_rad_s = tracked_x[7];
        gyro_radius_m = tracked_x[8];
      }
      static int yaw_eskf_log_counter = 0;
      if (++yaw_eskf_log_counter % 50 == 0) {
        const Eigen::Vector3d armor_velocity =
          tracked_target.armor_motion_velocity(tracked_target.last_id);
        tools::logger()->info(
          "[YAWESKF] trusted={} angle={:+.3f} omega={:+.3f} alpha={:+.3f} "
          "raw_ekf_omega={:+.3f} nis={:.2f} accepted={} rejected={} "
          "whole_iekf_nis={:.2f} armor_cv_trusted={} armor_v=({:+.2f},{:+.2f},{:+.2f}) "
          "armor_cv_nis={:.2f}",
          tracked_target.yaw_eskf_trusted() ? 1 : 0,
          tracked_target.yaw_eskf_angle(), tracked_target.yaw_eskf_rate(),
          tracked_target.yaw_eskf_acceleration(),
          tracked_target.ekf().x.size() >= 8 ? tracked_target.ekf().x[7] : NAN,
          tracked_target.yaw_eskf_nis(),
          tracked_target.yaw_eskf_accepted_updates(),
          tracked_target.yaw_eskf_rejected_updates(),
          tracked_target.ekf().last_nis,
          tracked_target.armor_motion_trusted(tracked_target.last_id) ? 1 : 0,
          armor_velocity.x(), armor_velocity.y(), armor_velocity.z(),
          tracked_target.armor_motion_nis(tracked_target.last_id));
      }
    }
    const double model_now_s = std::chrono::duration<double>(
      t.time_since_epoch()).count();
    const bool prior_phase_rate_valid = gyro_phase_scheduler.period_valid();
    const double prior_phase_omega_rad_s = prior_phase_rate_valid
      ? (M_PI / 2.0) / gyro_phase_scheduler.face_period_s()
      : NAN;
    const auto rotation_model = rotation_model_manager.update(
      model_now_s, tracker_converged && yaw_motion_model_trusted,
      gyro_omega_rad_s,
      prior_phase_rate_valid, prior_phase_omega_rad_s);
    if (rotation_model.phase_reset) {
      gyro_phase_scheduler.reset();
      gyro_face_selector.reset();
      event_last_fired_cycle = -1;
      xuc_one_face_armed = true;
      tools::logger()->warn(
        "[MODEL] phase reset mode={} ekf_omega={:+.3f} pll_omega={:+.3f} "
        "rate_delta={:.3f}",
        auto_aim::RotationModelManager::name(rotation_model.mode),
        gyro_omega_rad_s, prior_phase_omega_rad_s,
        rotation_model.rate_disagreement_rad_s);
    }
    if (rotation_model.mode_changed) {
      tools::logger()->info(
        "[MODEL] switched to {} ekf_omega={:+.3f}rad_s",
        auto_aim::RotationModelManager::name(rotation_model.mode),
        gyro_omega_rad_s);
    }
    if (tracker.frame_diagnostics().initialized_this_frame) {
      xuc_gyro_center_memory_valid = false;
      direct_yaw_control_confirmed = false;
    }
    vehicle_model_fresh = xuc_gyro_center_follow_enabled &&
      tracker.primary_observation() != nullptr &&
      rotation_model.tracker_trusted &&
      rotation_model.mode != auto_aim::RotationModelManager::Mode::armor;
    gyro_model_fresh = vehicle_model_fresh &&
      rotation_model.mode ==
        auto_aim::RotationModelManager::Mode::high_speed_phase;
    gyro_phase_model_trusted = gyro_model_fresh &&
      rotation_model.phase_fire_trusted;
    if (vehicle_model_fresh) {
      // Both low/medium rotation and high-speed gyro share the same stable
      // vehicle-centre gimbal reference.  Only their firing clocks differ:
      // continuous future-face prediction below the high-speed threshold and
      // the physical phase-event scheduler above it.
      gyro_center_follow_active = true;
      xuc_gyro_center_memory_valid = true;
      xuc_gyro_center_memory_yaw = gyro_center_yaw;
      xuc_gyro_center_last_fresh = t;
    } else if (xuc_gyro_center_memory_valid) {
      const double center_age_s =
        std::chrono::duration<double>(t - xuc_gyro_center_last_fresh).count();
      if (center_age_s <= xuc_gyro_center_hold_s) {
        // Keep the last observed centre through a short association gap.
        // Prediction-only frames do not renew this memory or permit firing.
        gyro_center_follow_active = true;
        gyro_center_yaw = xuc_gyro_center_memory_yaw;
      } else {
        xuc_gyro_center_memory_valid = false;
      }
    }
    if (gyro_center_follow_active != xuc_gyro_center_follow_logged) {
      tools::logger()->info(
        "[XUC][GYRO] center_follow={} tracker={} omega={:+.3f}rad_s radius={:.3f}m",
        gyro_center_follow_active, tracker.state(), gyro_omega_rad_s, gyro_radius_m);
      xuc_gyro_center_follow_logged = gyro_center_follow_active;
    }

#ifdef AMENT_CMAKE_FOUND
    // Publish only solved association poses, including identity corrections.
    if (!tracker.last_associations().empty()) {
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        t.time_since_epoch()).count();
      rclcpp::Time ros_time(ns);

      visualization_msgs::msg::MarkerArray marker_array;
      int marker_id = 0;
      for (const auto & association : tracker.last_associations()) {
        if (!association.observation || !association.observation->xyz_in_world.allFinite()) continue;
        const auto & armor = *association.observation;
        auto marker = visualizer->create_sphere_marker(
          "world", "armors", marker_id++,
          armor.xyz_in_world.x(),
          armor.xyz_in_world.y(),
          armor.xyz_in_world.z(),
          1.0, 0.0, 0.0, 0.8,  // 红色，80%不透明
          0.1,  // 10cm直径
          ros_time
        );
        marker_array.markers.push_back(marker);
      }
      visualizer->publish_marker_array(marker_array);
    }
#endif

    // Use exactly the solved observation that updated the tracker, including
    // its corrected physical plate model. Never recover pose from raw detections.
    const auto valid_measurement_pose = [](const auto_aim::Armor & armor) {
      return armor.xyz_in_world.allFinite() && armor.xyz_in_gimbal.allFinite() &&
        armor.ypr_in_world.allFinite() && armor.ypd_in_world.allFinite() &&
        armor.ypd_in_world[2] > 0.0;
    };
    const auto_aim::Armor * direct_measurement_armor = tracker.primary_observation();
    if (direct_measurement_armor != nullptr &&
        !valid_measurement_pose(*direct_measurement_armor)) {
      direct_measurement_armor = nullptr;
    }
    const bool direct_measurement_from_tracker = direct_measurement_armor != nullptr;
    int direct_measurement_detection_index = direct_measurement_from_tracker
      ? tracker.primary_detection_index() : -1;
    std::optional<auto_aim::Armor> direct_measurement_fallback;
    if (direct_measurement_armor == nullptr) {
      // Preserve measurement-only following when association is unavailable.
      // Solve current-frame local copies before comparing their bearings; leave
      // the detector's identity untouched and never retain an old frame's pose.
      double best_yaw_distance = INFINITY;
      int detection_index = 0;
      for (const auto & armor : armors) {
        const int current_detection_index = detection_index++;
        if (armor.points.size() != 4) continue;
        auto solved_armor = armor;
        solved_armor.xyz_in_world.setConstant(NAN);
        solved_armor.xyz_in_gimbal.setConstant(NAN);
        solved_armor.ypr_in_world.setConstant(NAN);
        solved_armor.ypr_in_gimbal.setConstant(NAN);
        solved_armor.ypd_in_world.setConstant(NAN);
        try {
          solver.solve(solved_armor);
        } catch (const cv::Exception &) {
          continue;
        }
        if (!valid_measurement_pose(solved_armor)) continue;
        const double armor_yaw = std::atan2(
          solved_armor.xyz_in_world.y(), solved_armor.xyz_in_world.x());
        const double yaw_distance = direct_yaw_initialized
          ? std::abs(std::remainder(armor_yaw - direct_yaw_filtered, 2.0 * M_PI))
          : 0.0;
        if (yaw_distance < best_yaw_distance) {
          best_yaw_distance = yaw_distance;
          direct_measurement_fallback = std::move(solved_armor);
          direct_measurement_detection_index = current_detection_index;
        }
        if (!direct_yaw_initialized) break;
      }
      if (direct_measurement_fallback) {
        direct_measurement_armor = &*direct_measurement_fallback;
      }
    }

    // Use the rigid-body geometry as the range source for trajectory timing.
    // At a face-centre gyro event the near armor is approximately one radius
    // closer than the vehicle centre.  Raw oblique-PnP depth is only a small,
    // bounded correction; the latest field log ranged from 3.24 to 5.92 m for
    // a nominal 5 m target and otherwise injected more than 30 degrees of
    // time-of-flight phase error.
    double raw_pnp_range_m = NAN;
    if (direct_measurement_armor != nullptr) {
      raw_pnp_range_m = std::hypot(
        direct_measurement_armor->xyz_in_world[0],
        direct_measurement_armor->xyz_in_world[1]);
    }
    double modeled_armor_range_m = NAN;
    if (!targets.empty()) {
      const auto tracked_x = targets.front().ekf_x();
      if (gyro_model_fresh && tracked_x.size() >= 9) {
        const double center_range_m = std::hypot(tracked_x[0], tracked_x[2]);
        if (std::isfinite(center_range_m) && std::isfinite(tracked_x[8])) {
          modeled_armor_range_m = center_range_m - std::abs(tracked_x[8]);
        }
      } else {
        const int primary_face_id = tracker.primary_face_id();
        const auto modeled_faces = targets.front().armor_xyza_list();
        if (primary_face_id >= 0 &&
            primary_face_id < static_cast<int>(modeled_faces.size())) {
          modeled_armor_range_m = std::hypot(
            modeled_faces[primary_face_id][0], modeled_faces[primary_face_id][1]);
        }
      }
    }
    const auto robust_range = robust_range_filter.update(
      model_now_s, modeled_armor_range_m, raw_pnp_range_m);
    const double robust_target_range_m = robust_range.valid
      ? robust_range.range_m
      : raw_pnp_range_m;
    static int range_log_counter = 0;
    if (++range_log_counter % 50 == 0) {
      tools::logger()->info(
        "[RANGE] raw={:.3f}m model={:.3f}m filtered={:.3f}m "
        "model_used={} raw_limited={}",
        raw_pnp_range_m, modeled_armor_range_m, robust_target_range_m,
        robust_range.used_model ? 1 : 0, robust_range.raw_limited ? 1 : 0);
    }

    if (infantry_diagnostics_enabled && shot_prediction_debug) {
      shot_prediction_debug->aimer_input_mps = cboard.bullet_speed;
      shot_prediction_debug->aimer_begin_ns = duration_cast<nanoseconds>(
        steady_clock::now().time_since_epoch()).count();
    }
    // Selected-face identity is needed by the fire gate even with debug off.
    auto_aim::AimPredictionTrace native_fire_prediction;
    auto command = aimer.aim(
      targets, t, cboard.bullet_speed, true,
      &native_fire_prediction,
      vehicle_model_fresh && !gyro_model_fresh
        ? auto_aim::AimGeometry::whole_ekf : auto_aim::AimGeometry::existing);
    if (infantry_diagnostics_enabled && shot_prediction_debug) {
      shot_prediction_debug->native = native_fire_prediction;
      shot_prediction_debug->aimer_end_ns = duration_cast<nanoseconds>(
        steady_clock::now().time_since_epoch()).count();
    }
    const bool native_prediction_valid =
      command.control && !targets.empty() && std::isfinite(command.yaw);
    const bool native_prediction_trusted =
      native_prediction_valid && tracker_converged && yaw_motion_model_trusted;
    const double native_predicted_yaw = command.yaw;
    double gyro_predicted_impact_yaw_error_rad = INFINITY;
    bool gyro_predicted_impact_yaw_valid = false;
    bool gyro_event_schedule_valid = false;
    bool gyro_event_interval_safe = false;
    std::int64_t gyro_event_hit_cycle = -1;
    double gyro_event_lead_error_s = INFINITY;
    double gyro_event_face_period_s = NAN;
    double gyro_event_half_window_rad = NAN;
    double gyro_event_worst_phase_rad = NAN;
    double gyro_event_combined_phase_rad = NAN;
    double gyro_event_phase_uncertainty_rad = NAN;
    bool gyro_event_face_valid = false;
    int gyro_event_face_id = -1;
    double gyro_event_face_alignment_rad = INFINITY;
    double gyro_event_gimbal_tracking_error_rad = INFINITY;
    std::chrono::steady_clock::time_point gyro_event_command_due_time{};

    // Phase 2D.5 integration guard.  The detector pose is much more stable
    // than the current high-dynamic EKF tuning at close range.  Keep the
    // complete Tracker/Aimer prediction path, but prevent a divergent state
    // from commanding a yaw far away from the armor actually in the image.
    // If the tracker briefly rejects convergence while a valid enemy armor is
    // still detected, degrade to measurement-only yaw tracking and never fire.
    bool measurement_fallback = false;
    bool direct_yaw_measurement_fresh = false;
    double direct_yaw_measurement_rad = NAN;
    bool direct_pitch_measurement_fresh = false;
    double direct_yaw_pixel_error_rad = INFINITY;
    double direct_pitch_pixel_error_rad = INFINITY;
    double direct_pitch_raw_pixel_error_rad = INFINITY;
    double ballistic_pitch_compensation_rad = 0.0;
    double ballistic_camera_range_m = NAN;
    double ballistic_total_holdover_m = 0.0;
    // A newly constructed tracker returns provisional targets before its
    // normal convergence window.  Keep those detections visible, but never let
    // a one-frame pose regain actuator authority after a genuine track loss.
    if (targets.empty()) direct_yaw_control_confirmed = false;
    if (direct_yaw_guard_enabled) {
      bool valid_measurement = false;
      double measured_yaw = 0.0;
      if (direct_measurement_armor != nullptr) {
        const auto & measured_armor = *direct_measurement_armor;
        measured_yaw = gyro_center_follow_active
          ? gyro_center_yaw
          : std::atan2(measured_armor.xyz_in_world.y(), measured_armor.xyz_in_world.x());
        valid_measurement = std::isfinite(measured_yaw) &&
          (!gyro_center_follow_active || vehicle_model_fresh);
        if (valid_measurement) direct_yaw_measurement_rad = measured_yaw;
        if (valid_measurement && direct_yaw_focal_x_px > 0.0 && !img.empty()) {
          double center_x_px = measured_armor.center.x;
          if (!measured_armor.points.empty()) {
            center_x_px = 0.0;
            for (const auto & point : measured_armor.points) center_x_px += point.x;
            center_x_px /= static_cast<double>(measured_armor.points.size());
          }
          const double raw_yaw_pixel_error_rad = std::atan2(
            center_x_px - 0.5 * static_cast<double>(img.cols), direct_yaw_focal_x_px);
          // On the installed yaw convention, a negative impact correction moves
          // the armor to image-left by the same angle.  Fire on the residual from
          // that requested offset, not on distance from the image centre.
          direct_yaw_pixel_error_rad =
            raw_yaw_pixel_error_rad - direct_yaw_impact_correction_rad;
          direct_yaw_measurement_fresh = direct_measurement_from_tracker &&
            std::isfinite(direct_yaw_pixel_error_rad);
        }
      }

      if (valid_measurement) {
        if (!direct_yaw_initialized) {
          direct_yaw_filtered = measured_yaw;
          direct_yaw_initialized = true;
        } else {
          // Cap dt so a camera stall or a long loss cannot turn the rate limit
          // into one large step on the first recovered frame.
          const double dt = std::clamp(
            std::chrono::duration<double>(t - direct_yaw_last_update).count(),
            0.0, 0.05);
          const double yaw_error =
            std::remainder(measured_yaw - direct_yaw_filtered, 2.0 * M_PI);
          const double alpha = direct_yaw_filter_tau_s > 0.0
            ? 1.0 - std::exp(-dt / direct_yaw_filter_tau_s)
            : 1.0;
          double yaw_step = alpha * yaw_error;
          if (direct_yaw_max_rate_rad_s > 0.0) {
            const double max_step = direct_yaw_max_rate_rad_s * dt;
            yaw_step = std::clamp(yaw_step, -max_step, max_step);
          }
          direct_yaw_filtered = std::remainder(
            direct_yaw_filtered + yaw_step, 2.0 * M_PI);
        }
        direct_yaw_last_update = t;
        direct_yaw_last_measurement = t;
        if (direct_yaw_long_loss) {
          tools::logger()->info("[XUC][YAW] measurement reacquired with continuous slew");
        }
        direct_yaw_long_loss = false;

        measurement_fallback = true;
        command.shoot = false;
        if (!direct_yaw_control_confirmed && tracker_converged) {
          direct_yaw_control_confirmed = true;
          if (infantry_diagnostics_enabled) {
            tools::logger()->info(
              "[XUC][YAW] tracker convergence confirmed; actuator reacquired");
          }
        }
        command.control = direct_yaw_control_confirmed;
        if (!direct_yaw_control_confirmed) {
          command.yaw = direct_yaw_filtered;
        } else if (gyro_center_follow_active) {
          // In every rotating-target mode the visible plate bearing oscillates
          // around the vehicle. Following the EKF rotation center avoids
          // switching the gimbal at each of the four plate hand-overs.
          command.yaw = std::remainder(
            direct_yaw_filtered + direct_yaw_command_correction_rad,
            2.0 * M_PI);
        } else if (prediction_lead_limit_configured && native_prediction_trusted) {
          const double prediction_lead =
            std::remainder(native_predicted_yaw - measured_yaw, 2.0 * M_PI);
          command.yaw = std::remainder(
            direct_yaw_filtered + std::clamp(
              prediction_lead, -max_prediction_lead_rad, max_prediction_lead_rad) +
              direct_yaw_command_correction_rad,
            2.0 * M_PI);
        } else {
          command.yaw = std::remainder(
            direct_yaw_filtered + direct_yaw_command_correction_rad,
            2.0 * M_PI);
        }
      } else if (direct_yaw_initialized) {
        const double visual_gap =
          std::chrono::duration<double>(t - direct_yaw_last_measurement).count();
        if (direct_yaw_control_confirmed && visual_gap <= direct_yaw_hold_s) {
          // Hold the centre through a short association gap. Other targets
          // retain the existing bounded prediction.
          command.control = true;
          command.shoot = false;
          command.yaw = std::remainder(
            direct_yaw_filtered + direct_yaw_command_correction_rad,
            2.0 * M_PI);
          if (!gyro_center_follow_active &&
              prediction_lead_limit_configured && native_prediction_trusted) {
            const double prediction_delta =
              std::remainder(native_predicted_yaw - direct_yaw_filtered, 2.0 * M_PI);
            command.yaw = std::remainder(
              direct_yaw_filtered + std::clamp(
                prediction_delta, -max_prediction_lead_rad, max_prediction_lead_rad) +
                direct_yaw_command_correction_rad,
              2.0 * M_PI);
          }
          measurement_fallback = true;
        } else {
          // A genuine loss must stop control, but retaining the filtered yaw and
          // refreshing last_update prevents a full-angle jump on reacquisition.
          if (!direct_yaw_long_loss) {
            tools::logger()->warn("[XUC][YAW] visual loss: control stopped, yaw state retained");
          }
          direct_yaw_long_loss = true;
          direct_yaw_control_confirmed = false;
          command.control = false;
          command.shoot = false;
          measurement_fallback = true;
        }
        direct_yaw_last_update = t;
      } else {
        command.control = false;
        command.shoot = false;
        measurement_fallback = true;
      }
    }
    if (direct_pitch_guard_enabled) {
      bool valid_pitch_measurement = false;
      double measured_pitch = 0.0;
      if (direct_measurement_armor != nullptr && direct_pitch_focal_y_px > 0.0 && !img.empty()) {
        const auto & measured_armor = *direct_measurement_armor;
        double center_y_px = measured_armor.center.y;
        if (!measured_armor.points.empty()) {
          center_y_px = 0.0;
          for (const auto & point : measured_armor.points) center_y_px += point.y;
          center_y_px /= static_cast<double>(measured_armor.points.size());
        }
        const double pixel_pitch_error = std::atan2(
          center_y_px - 0.5 * static_cast<double>(img.rows),
          direct_pitch_focal_y_px);
        direct_pitch_raw_pixel_error_rad = pixel_pitch_error;

        if (xuc_ballistic_pitch_enabled) {
          const double raw_camera_range_m = std::hypot(
            measured_armor.xyz_in_gimbal.x(), measured_armor.xyz_in_gimbal.y());
          ballistic_camera_range_m = std::isfinite(robust_target_range_m)
            ? robust_target_range_m
            : raw_camera_range_m;
          if (std::isfinite(ballistic_camera_range_m) && ballistic_camera_range_m >= 0.10) {
            // Reuse the same closed-form low-arc solver that is exercised by
            // the Gestalt Tongji/SHtech Aimer.  The camera is forward of and
            // above the muzzle, while camera/gimbal Z points down.
            const double range_scale = raw_camera_range_m > 0.10
              ? ballistic_camera_range_m / raw_camera_range_m
              : 1.0;
            const double muzzle_x_m = measured_armor.xyz_in_gimbal.x() * range_scale +
              xuc_camera_forward_of_muzzle_m;
            const double muzzle_y_m = measured_armor.xyz_in_gimbal.y() * range_scale;
            const double muzzle_range_m = std::hypot(muzzle_x_m, muzzle_y_m);
            const double target_height_from_muzzle_m =
              -measured_armor.xyz_in_gimbal.z() + xuc_camera_above_muzzle_m;
            const double camera_los_up_rad = std::atan2(
              -measured_armor.xyz_in_gimbal.z(), ballistic_camera_range_m);
            const double reference_muzzle_range_m =
              xuc_ballistic_reference_camera_range_m + xuc_camera_forward_of_muzzle_m;
            const double reference_measured_compensation_rad = std::atan2(
              xuc_ballistic_reference_total_drop_m, reference_muzzle_range_m);
            const tools::Trajectory reference_trajectory(
              xuc_ballistic_reference_projectile_speed_mps,
              reference_muzzle_range_m,
              xuc_camera_above_muzzle_m);
            const tools::Trajectory current_trajectory(
              xuc_projectile_speed_mps, muzzle_range_m, target_height_from_muzzle_m);
            if (!reference_trajectory.unsolvable && !current_trajectory.unsolvable) {
              // The one real-shot datum calibrates only the fixed mechanical /
              // boresight bias.  Distance, height, gravity and flight time are
              // then solved physically instead of extrapolating a measured miss
              // with distance squared.
              const double fixed_boresight_bias_rad =
                reference_measured_compensation_rad - reference_trajectory.pitch;
              ballistic_pitch_compensation_rad = std::clamp(
                fixed_boresight_bias_rad + current_trajectory.pitch - camera_los_up_rad +
                  xuc_ballistic_pitch_trim_rad,
                0.0,
                xuc_ballistic_max_compensation_rad);
              ballistic_total_holdover_m =
                std::tan(ballistic_pitch_compensation_rad) * muzzle_range_m;
            } else {
              ballistic_pitch_compensation_rad = NAN;
              ballistic_total_holdover_m = NAN;
            }
          } else {
            ballistic_pitch_compensation_rad = NAN;
          }
        }
        const double compensated_pitch_error =
          pixel_pitch_error - ballistic_pitch_compensation_rad;
        // Physical acceptance established that a target below image centre
        // requires a larger lower-board IMU pitch target. Closing the loop in
        // image space avoids the currently inconsistent pitch hand-eye offset.
        // Positive ballistic compensation deliberately settles the armor below
        // image centre so the muzzle is elevated above the visual line of sight.
        measured_pitch = xuc.imu_pitch() + direct_pitch_error_gain * compensated_pitch_error;
        valid_pitch_measurement = std::isfinite(measured_pitch);
        direct_pitch_pixel_error_rad = compensated_pitch_error;
        direct_pitch_measurement_fresh = direct_measurement_from_tracker && valid_pitch_measurement &&
          std::isfinite(direct_pitch_pixel_error_rad);
      }

      if (valid_pitch_measurement) {
        if (!direct_pitch_initialized) {
          direct_pitch_filtered = measured_pitch;
          direct_pitch_initialized = true;
        } else {
          const double dt = std::clamp(
            std::chrono::duration<double>(t - direct_pitch_last_update).count(),
            0.0, 0.05);
          const double pitch_error = measured_pitch - direct_pitch_filtered;
          const double alpha = direct_pitch_filter_tau_s > 0.0
            ? 1.0 - std::exp(-dt / direct_pitch_filter_tau_s)
            : 1.0;
          double pitch_step = alpha * pitch_error;
          if (direct_pitch_max_rate_rad_s > 0.0) {
            const double max_step = direct_pitch_max_rate_rad_s * dt;
            pitch_step = std::clamp(pitch_step, -max_step, max_step);
          }
          direct_pitch_filtered += pitch_step;
        }
        direct_pitch_last_update = t;
        direct_pitch_last_measurement = t;
        if (direct_pitch_long_loss) {
          tools::logger()->info("[XUC][PITCH] measurement reacquired with continuous slew");
        }
        direct_pitch_long_loss = false;
        command.pitch = direct_pitch_filtered;
      } else if (direct_pitch_initialized) {
        const double visual_gap =
          std::chrono::duration<double>(t - direct_pitch_last_measurement).count();
        if (visual_gap <= direct_pitch_hold_s) {
          command.pitch = direct_pitch_filtered;
        } else {
          if (!direct_pitch_long_loss) {
            tools::logger()->warn("[XUC][PITCH] visual loss: control stopped, pitch state retained");
          }
          direct_pitch_long_loss = true;
          command.control = false;
          command.shoot = false;
        }
        direct_pitch_last_update = t;
      } else {
        command.control = false;
        command.shoot = false;
      }
    }

    // The final rate limiter includes the bounded ESKF lead and every model
    // hand-over.  Synchronize it to the actual lower-board yaw outside AUTO or
    // on a genuine visual loss, so re-entering AUTO cannot dump a stale target
    // into the inner position loop in one frame.
    // omni-code exposes AUTO readiness through mode only. robot_id is an
    // identity value (103 on this lower board), not a bit field of gate states.
    const bool lower_auto_ready_now = xuc.enabled() && xuc.rx_valid() &&
      xuc.mode() == static_cast<uint8_t>(io::Mode::auto_aim);
    const double yaw_before_final_rate_rad = command.yaw;
    if (final_yaw_command_max_rate_rad_s > 0.0 && xuc.enabled()) {
      const auto limiter_now = std::chrono::steady_clock::now();
      const bool actual_yaw_valid = xuc.rx_valid() &&
        std::isfinite(xuc.raw_imu_yaw());
      const double actual_command_yaw = actual_yaw_valid
        ? xuc.raw_yaw_correction_to_command(xuc.raw_imu_yaw())
        : 0.0;

      if (!lower_auto_ready_now || !command.control || !std::isfinite(command.yaw)) {
        if (actual_yaw_valid) {
          final_yaw_command_filtered = actual_command_yaw;
          final_yaw_command_initialized = true;
        }
        final_yaw_command_last_update = limiter_now;
        final_yaw_lower_armed = false;
      } else {
        if (!final_yaw_command_initialized) {
          final_yaw_command_filtered = actual_yaw_valid
            ? actual_command_yaw
            : command.yaw;
          final_yaw_command_initialized = true;
          final_yaw_command_last_update = limiter_now;
        }
        if (!final_yaw_lower_armed) {
          final_yaw_lower_armed = true;
          final_yaw_command_last_update = limiter_now;
        }
        const double dt = std::clamp(
          std::chrono::duration<double>(
            limiter_now - final_yaw_command_last_update).count(),
          0.0, 0.05);
        final_yaw_command_last_update = limiter_now;
        const double desired_step = std::remainder(
          command.yaw - final_yaw_command_filtered, 2.0 * M_PI);
        const double max_step = final_yaw_command_max_rate_rad_s * dt;
        final_yaw_command_filtered = std::remainder(
          final_yaw_command_filtered +
            std::clamp(desired_step, -max_step, max_step),
          2.0 * M_PI);
        command.yaw = final_yaw_command_filtered;
      }
    }
    const double yaw_after_final_rate_rad = command.yaw;
    if (xuc.enabled() && !xuc_imu_sync_valid) {
      command.control = false;
      command.shoot = false;
    }
    const bool native_shoot = shooter.shoot(command, aimer, targets, ypr);
    fire_gate_snapshot_valid = false;
    fire_first_blocker = -1;
    if (xuc_fire_guard_enabled) {
      bool face_valid = false;
      double face_error_rad = INFINITY;
      if (!targets.empty() && aimer.debug_aim_point.valid) {
        const auto tracked_x = targets.front().ekf_x();
        if (tracked_x.size() >= 3) {
          const double center_bearing = std::atan2(tracked_x[2], tracked_x[0]);
          face_error_rad = std::abs(std::remainder(
            aimer.debug_aim_point.xyza[3] - center_bearing, 2.0 * M_PI));
          face_valid = std::isfinite(face_error_rad);
        }
      }
      const bool gyro_predicted_fire_active =
        vehicle_model_fresh && gyro_model_fresh &&
        xuc_gyro_predicted_fire_enabled;
      const bool vehicle_predicted_fire_active =
        vehicle_model_fresh && !gyro_model_fresh &&
        xuc_gyro_predicted_fire_enabled;
      // One-shot-per-face is meaningful only for the high-speed phase clock.
      // At low/medium speed, a valid aim window may last much longer than one
      // face passage; let the command hold/rearm window pace repeated requests.
      const bool one_face_policy_active =
        xuc_one_face_once_fire && gyro_predicted_fire_active;
      const bool predictive_fire_active =
        gyro_predicted_fire_active || vehicle_predicted_fire_active;
      if (infantry_diagnostics_enabled && shot_prediction_debug) {
        shot_prediction_debug->gate_branch = gyro_predicted_fire_active
          ? "high_speed_prediction" : (vehicle_predicted_fire_active
              ? "vehicle_prediction" : "stationary_measurement");
      }
      const bool command_cadence_gate_open = !xuc.enabled() ||
        (!xuc_pending_shot && xuc.shoot_request_ready());
      if (xuc.enabled() && !command_cadence_gate_open) {
        if (!xuc_gyro_cadence_hold_logged) {
          tools::logger()->info(
            "[XUC][FIRE] request inhibited until command rearm window completes");
          xuc_gyro_cadence_hold_logged = true;
        }
      } else if (xuc_gyro_cadence_hold_logged) {
        tools::logger()->info(
          "[XUC][FIRE] command rearmed; evaluating current frame");
        xuc_gyro_cadence_hold_logged = false;
      }
      if (predictive_fire_active && native_prediction_valid) {
        // native_predicted_yaw is produced from the same future AimPoint used by
        // face_error_rad and already includes configured actuation delay plus the
        // iterated projectile flight time. Compare it with actual gimbal attitude,
        // not the centre-follow command, so servo lag is included in qualification.
        gyro_predicted_impact_yaw_error_rad = std::remainder(
          native_predicted_yaw - ypr[0] - xuc_gyro_predicted_yaw_trim_rad,
          2.0 * M_PI);
        gyro_predicted_impact_yaw_valid =
          std::isfinite(gyro_predicted_impact_yaw_error_rad);
      }
      const bool convergence_gate_open =
        !xuc_fire_require_tracker_converged || tracker_converged;
      const bool stationary_yaw_gate_open = direct_yaw_measurement_fresh &&
        std::abs(direct_yaw_pixel_error_rad) <= xuc_fire_yaw_tolerance_rad;
      // A held gyro centre is useful for smooth reacquisition, but it is neither
      // a fresh moving model nor a stationary-target solution. Do not fall through
      // to the stationary fire policy until centre-memory mode has ended.
      const bool motion_transition_gate_open =
        !gyro_center_follow_active || vehicle_model_fresh;
      // ---- Event-clock phase scheduler ----
      // Fold the EKF body yaw modulo one face (pi/2).  Every armor id therefore
      // shares the same physical front-center zero and a detector association
      // switch cannot move the event clock by an entire face.  Raw PnP armor
      // yaw is intentionally not used as the clock because field logs showed
      // paired long/short zero crossings and clustered hit/miss intervals.
      const bool ev_gyro_active = gyro_predicted_fire_active && gyro_model_fresh;
      if (ev_gyro_active && direct_measurement_armor != nullptr &&
          direct_yaw_measurement_fresh && std::isfinite(gyro_center_yaw) &&
          std::isfinite(gyro_body_phase_rad)) {
        const double observed_phase_error = std::remainder(
          gyro_body_phase_rad - gyro_center_yaw, M_PI / 2.0);
        const double observation_time_s = std::chrono::duration<double>(
          t.time_since_epoch()).count();
        const auto event_observation = gyro_phase_scheduler.observe(
          observation_time_s, observed_phase_error, gyro_omega_rad_s);
        if (event_observation.period_updated) {
          tools::logger()->info(
            "[GYROPLL] phase_zero cycle={} sample={:.1f} ms periods={} alias_guard={} "
            "filtered={:.1f} ms residual={:+.1f} ms correction={:+.1f} ms "
            "uncertainty={:.1f} ms "
            "locks={} locked={} reacquired={} rejects={} omega={:.3f} ekf_omega={:+.3f}",
            gyro_phase_scheduler.crossing_cycle(),
            event_observation.period_sample_s * 1000.0,
            event_observation.periods_elapsed,
            event_observation.omega_alias_guarded ? 1 : 0,
            event_observation.face_period_s * 1000.0,
            event_observation.phase_residual_s * 1000.0,
            event_observation.applied_phase_correction_s * 1000.0,
            event_observation.phase_uncertainty_s * 1000.0,
            event_observation.lock_updates,
            event_observation.locked ? 1 : 0,
            event_observation.reacquired ? 1 : 0,
            event_observation.consecutive_rejections,
            (M_PI / 2.0) / event_observation.face_period_s,
            gyro_omega_rad_s);
        } else if (event_observation.crossing && event_observation.rejected) {
          tools::logger()->warn(
            "[GYROPLL] rejected phase crossing sample={:.1f} ms periods={} alias_guard={} "
            "residual={:+.1f} ms rejects={} ekf_omega={:+.3f}",
            event_observation.period_sample_s * 1000.0,
            event_observation.periods_elapsed,
            event_observation.omega_alias_guarded ? 1 : 0,
            event_observation.phase_residual_s * 1000.0,
            event_observation.consecutive_rejections,
            gyro_omega_rad_s);
        }
      } else if (!gyro_center_follow_active) {
        gyro_phase_scheduler.reset();
        gyro_face_selector.reset();
        event_last_fired_cycle = -1;
      }
      if (!ev_gyro_active) gyro_face_selector.reset();

      // The flight-time estimate uses the current measured armor range.  The
      // interval gate below separately accounts for flight-time uncertainty.
      double event_flight_time_s = NAN;
      if (ev_gyro_active && direct_measurement_armor != nullptr) {
        const auto & measured_xyz = direct_measurement_armor->xyz_in_world;
        const double d_face_m = std::isfinite(robust_target_range_m)
          ? robust_target_range_m
          : std::hypot(measured_xyz[0], measured_xyz[1]);
        event_flight_time_s = d_face_m / xuc_projectile_speed_mps;
        const tools::Trajectory trj(
          xuc_projectile_speed_mps, d_face_m, measured_xyz[2]);
        if (!trj.unsolvable && trj.fly_time > 0.0) {
          event_flight_time_s = trj.fly_time;
        }
      }

      auto event_solution = auto_aim::GyroPhaseScheduler::Solution{};
      if (infantry_diagnostics_enabled && shot_prediction_debug) {
        shot_prediction_debug->event_flight_time_s = event_flight_time_s;
        shot_prediction_debug->event_delay_s = shot_delay_adapter.delay_s();
      }
      if (ev_gyro_active && std::isfinite(event_flight_time_s)) {
        const double decision_now_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        event_solution = gyro_phase_scheduler.solve(
          decision_now_s,
          shot_delay_adapter.delay_s(),
          shot_delay_adapter.delay_min_s(),
          shot_delay_adapter.delay_max_s(),
          event_flight_time_s,
          xuc_event_flight_time_uncertainty_s,
          gyro_radius_m,
          xuc_event_armor_width_m,
          xuc_event_armor_edge_margin_m,
          xuc_event_model_phase_uncertainty_rad);
        if (event_solution.valid) {
          gyro_event_schedule_valid = true;
          // The PLL is tied to directly observed physical face crossings.  Use
          // its temporal uncertainty as the primary event gate.  Deployments
          // that trust the EKF face phase may additionally require the harder
          // combined face-consistency check below.
          const double relaxed_half_window_rad =
            event_solution.half_hit_window_rad + xuc_event_phase_grace_rad;
          gyro_event_interval_safe =
            !xuc_event_require_face_consistency &&
            event_solution.worst_interval_phase_rad <= relaxed_half_window_rad;
          gyro_event_hit_cycle = event_solution.hit_cycle;
          gyro_event_lead_error_s = event_solution.time_until_command_s;
          gyro_event_face_period_s = event_solution.face_period_s;
          gyro_event_half_window_rad = event_solution.half_hit_window_rad;
          gyro_event_worst_phase_rad = event_solution.worst_interval_phase_rad;
          gyro_event_phase_uncertainty_rad = event_solution.phase_uncertainty_rad;
          gyro_event_command_due_time = std::chrono::steady_clock::time_point(
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(event_solution.command_due_time_s)));

          // Predict the complete rigid body to this exact impact event, then
          // commit one physical face id.  The commitment is held for repeated
          // frames of the same event cycle and hands over only at the next
          // cycle (or if the prior face becomes geometrically impossible).
          if (!targets.empty()) {
            auto future_target = targets.front();
            const double processing_age_s = std::max(
              0.0, std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t).count());
            future_target.predict(
              processing_age_s + event_solution.time_until_hit_center_s);
            if (infantry_diagnostics_enabled && shot_prediction_debug) {
              shot_prediction_debug->event_prediction_horizon_s =
                processing_age_s + event_solution.time_until_hit_center_s;
            }
            const auto future_x = future_target.ekf_x();
            if (future_x.size() >= 3) {
              const double future_center_yaw = std::atan2(future_x[2], future_x[0]);
              std::vector<auto_aim::GyroFaceSelector::Candidate> face_candidates;
              const auto future_faces = future_target.armor_xyza_list();
              face_candidates.reserve(future_faces.size());
              for (std::size_t face_id = 0; face_id < future_faces.size(); ++face_id) {
                face_candidates.push_back({
                  static_cast<int>(face_id), future_faces[face_id][3]});
              }
              const auto face_selection = gyro_face_selector.select(
                face_candidates, future_center_yaw, gyro_event_hit_cycle);
              gyro_event_face_valid = face_selection.valid;
              gyro_event_face_id = face_selection.face_id;
              gyro_event_face_alignment_rad = face_selection.alignment_rad;
              if (infantry_diagnostics_enabled && shot_prediction_debug &&
                  face_selection.valid && face_selection.face_id >= 0 &&
                  static_cast<std::size_t>(face_selection.face_id) < future_faces.size()) {
                const auto & face = future_faces[face_selection.face_id];
                std::copy_n(face.data(), 4, shot_prediction_debug->event_xyza.begin());
              }
              if (face_selection.valid) {
                // The temporal PLL interval and future-face alignment are
                // partially independent error sources.  Root-sum-square keeps
                // the face check meaningful without the excessive fire-rate
                // loss caused by linearly adding two conservative bounds.
                gyro_event_combined_phase_rad =
                  std::hypot(
                    event_solution.worst_interval_phase_rad,
                    face_selection.alignment_rad);
                if (xuc_event_require_face_consistency) {
                  gyro_event_interval_safe =
                    event_solution.worst_interval_phase_rad <= relaxed_half_window_rad &&
                    gyro_event_combined_phase_rad <= relaxed_half_window_rad;
                }
              }
              if (face_selection.valid && face_selection.changed) {
                tools::logger()->info(
                  "[FACELOCK] handoff cycle={} face={} alignment={:.2f}deg",
                  gyro_event_hit_cycle, gyro_event_face_id,
                  gyro_event_face_alignment_rad * 180.0 / M_PI);
              }
            }
          }

          gyro_event_gimbal_tracking_error_rad = std::abs(std::remainder(
            command.yaw - ypr[0], 2.0 * M_PI));

          static int event_schedule_log = 0;
          if (++event_schedule_log % 10 == 0) {
            tools::logger()->info(
              "[GYROSCHED] cycle={} due_in={:+.1f} ms hit_in={:.1f} ms "
              "period={:.1f} ms window={:.2f}deg worst={:.2f}deg "
              "combined={:.2f}deg "
              "phase_unc={:.2f}deg face={} face_align={:.2f}deg "
              "gimbal_err={:.2f}deg safe={}",
              gyro_event_hit_cycle, gyro_event_lead_error_s * 1000.0,
              event_solution.time_until_hit_center_s * 1000.0,
              gyro_event_face_period_s * 1000.0,
              gyro_event_half_window_rad * 180.0 / M_PI,
              gyro_event_worst_phase_rad * 180.0 / M_PI,
              gyro_event_combined_phase_rad * 180.0 / M_PI,
              gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
              gyro_event_face_id,
              gyro_event_face_alignment_rad * 180.0 / M_PI,
              gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI,
              gyro_event_interval_safe);
          }
        }
      }

      // A new physical face-center cycle re-arms a shot.  Outside AUTO the
      // estimator may continue learning the period, but fire state and cooldown
      // are never consumed.
      const bool auto_mode_active = mode == io::Mode::auto_aim;
      if (!auto_mode_active) {
        xuc_one_face_armed = true;
        event_last_fired_cycle = -1;
      } else if (one_face_policy_active) {
        if (gyro_event_schedule_valid) {
          // A PLL correction may move a scheduled event back to an already
          // consumed cycle.  Explicitly disarm in that case; merely rearming
          // on a different future cycle allowed duplicate requests in field
          // log cycles 41 and 146.
          xuc_one_face_armed = gyro_event_hit_cycle > event_last_fired_cycle;
        } else if (!gyro_event_schedule_valid &&
                   (!face_valid || face_error_rad >= xuc_one_face_rearm_rad)) {
          xuc_one_face_armed = true;
        }
      }
      if (!gyro_center_follow_active) xuc_one_face_armed = true;
      const bool one_face_gate_open = !one_face_policy_active ||
        (gyro_predicted_fire_active
           ? (gyro_event_schedule_valid
                ? xuc_one_face_armed
                : (!xuc_gyro_require_event_lock_for_fire && face_valid &&
                   face_error_rad <= xuc_one_face_limit_rad &&
                   xuc_one_face_armed))
           : (face_valid && face_error_rad <= xuc_one_face_limit_rad &&
              xuc_one_face_armed));

      // In event mode, issue only before the due instant and wait up to one
      // frame for a precise steady-clock send.  Never accept a late sample.
      const bool gyro_yaw_gate_native_open = gyro_predicted_impact_yaw_valid &&
        direct_yaw_measurement_fresh &&
        std::abs(gyro_predicted_impact_yaw_error_rad) <=
          xuc_gyro_predicted_yaw_tolerance_rad &&
        std::abs(direct_yaw_pixel_error_rad) <=
          xuc_gyro_visibility_yaw_tolerance_rad;
      const bool gyro_event_gate_open = gyro_event_schedule_valid &&
        gyro_phase_model_trusted && gyro_event_interval_safe &&
        (!xuc_event_require_face_consistency || gyro_event_face_valid) &&
        gyro_event_gimbal_tracking_error_rad <=
          xuc_event_gimbal_tracking_tolerance_rad &&
        gyro_event_lead_error_s >= 0.0 &&
        gyro_event_lead_error_s <= xuc_event_trigger_window_s;
      const bool yaw_gate_open = gyro_predicted_fire_active
        ? (gyro_event_schedule_valid
             ? gyro_event_gate_open
             : (!xuc_gyro_require_event_lock_for_fire &&
                gyro_yaw_gate_native_open))
        : (vehicle_predicted_fire_active
             ? gyro_yaw_gate_native_open
             : stationary_yaw_gate_open);
      const bool fire_candidate = command.control &&
        auto_mode_active &&
        convergence_gate_open && one_face_gate_open && motion_transition_gate_open &&
        command_cadence_gate_open &&
        direct_yaw_measurement_fresh && direct_pitch_measurement_fresh &&
        yaw_gate_open &&
        std::abs(direct_pitch_pixel_error_rad) <= xuc_fire_pitch_tolerance_rad;
      bool stable_window_open = false;
      // Fire-gate diagnostic for both medium-speed prediction and the high-speed
      // event clock; low-speed dead zones were previously invisible here.
      static int b2_diag_log = 0;
      if (infantry_diagnostics_enabled && predictive_fire_active &&
          ++b2_diag_log % 50 == 0) {
        tools::logger()->info(
          "[B2DIAG] auto={} control={} conv={} oneface_gate={} armed={} motion={} "
          "cadence={} dyaw_fresh={} dpitch_fresh={} yawgate={} event={} safe={} "
          "face={} face_ok={} face_align_deg={:.2f} phase_unc_deg={:.2f} "
          "gimbal_err_deg={:.2f} due_ms={:+.1f} window_deg={:.2f} "
          "worst_deg={:.2f} combined_deg={:.2f} "
          "yawerr_deg={:.2f} pitcherr_deg={:.2f}",
          auto_mode_active ? 1 : 0, command.control ? 1 : 0,
          convergence_gate_open ? 1 : 0,
          one_face_gate_open ? 1 : 0, xuc_one_face_armed ? 1 : 0,
          motion_transition_gate_open ? 1 : 0,
          command_cadence_gate_open ? 1 : 0,
          direct_yaw_measurement_fresh ? 1 : 0,
          direct_pitch_measurement_fresh ? 1 : 0,
          yaw_gate_open ? 1 : 0, gyro_event_schedule_valid ? 1 : 0,
          gyro_event_interval_safe ? 1 : 0,
          gyro_event_face_id, gyro_event_face_valid ? 1 : 0,
          gyro_event_face_alignment_rad * 180.0 / M_PI,
          gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
          gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI,
          gyro_event_lead_error_s * 1000.0,
          gyro_event_half_window_rad * 180.0 / M_PI,
          gyro_event_worst_phase_rad * 180.0 / M_PI,
          gyro_event_combined_phase_rad * 180.0 / M_PI,
          direct_yaw_pixel_error_rad * 180.0 / M_PI,
          direct_pitch_pixel_error_rad * 180.0 / M_PI);
      }
      const auto & fire_tracker_frame = tracker.frame_diagnostics();
      const bool prediction_observation_valid = predictive_fire_active &&
        !targets.empty() && tracker.state() == "tracking" &&
        tracker.primary_observation() != nullptr && convergence_gate_open &&
        direct_yaw_measurement_fresh && direct_pitch_measurement_fresh &&
        (!xuc.enabled() || xuc_imu_sync_valid) &&
        native_prediction_valid && native_fire_prediction.valid &&
        gyro_predicted_impact_yaw_valid && aimer.debug_aim_point.valid;
      rotation_fire_window.update(
        prediction_observation_valid,
        fire_tracker_frame.initialized_this_frame || fire_tracker_frame.nis_reset ||
          fire_tracker_frame.diverged,
        targets.empty() ? -1 : static_cast<int>(targets.front().name),
        targets.empty() ? -1 : static_cast<int>(targets.front().armor_type),
        targets.empty() ? 0 : targets.front().armor_num(),
        native_fire_prediction.face_id,
        gyro_event_face_valid,
        !prediction_observation_valid ? NAN : (gyro_event_face_valid
          ? static_cast<double>(gyro_event_face_id) * M_PI / 2.0
          : aimer.debug_aim_point.xyza[3]),
        gyro_predicted_impact_yaw_error_rad, t, xuc_gyro_fire_min_consecutive_frames,
        xuc_gyro_candidate_max_phase_step_rad, xuc_gyro_predicted_yaw_tolerance_rad);
      if (predictive_fire_active) {
        // The current gates and cadence apply only to this frame's request.
        // Qualification above keeps observing outside the angle window and
        // during the sender's hold/rearm interval. It never latches a request.
        stable_window_open = rotation_fire_window.two_frames_ready;
        command.shoot = stable_window_open && fire_candidate &&
          gyro_yaw_gate_native_open && rotation_fire_window.current_frame_is_better;
        // Keep the stationary branch's dwell state separate from rotation.
        xuc_fire_candidate_active = false;
        xuc_fire_ready_logged = false;
        xuc_fire_candidate_frames = 0;
        if (command.shoot) {
          tools::logger()->info(
            "[XUC][FIRE] rotation qualified frames={} face={} "
            "predicted_impact_yaw_error_deg={:.2f} next_error_deg={:.2f}",
            rotation_fire_window.frames, rotation_fire_window.face_id,
            gyro_predicted_impact_yaw_error_rad * 180.0 / M_PI,
            rotation_fire_window.next_error_rad * 180.0 / M_PI);
          if (one_face_policy_active) {
            xuc_one_face_armed = false;
            if (gyro_event_schedule_valid) event_last_fired_cycle = gyro_event_hit_cycle;
          }
        }
      } else if (fire_candidate) {
        if (!xuc_fire_candidate_active) {
          xuc_fire_candidate_active = true;
          xuc_fire_ready_logged = false;
          xuc_fire_candidate_frames = 1;
          xuc_fire_candidate_since = t;
          tools::logger()->info("[XUC][FIRE] stable-window candidate started");
        } else {
          ++xuc_fire_candidate_frames;
        }
        const double stable_time =
          std::chrono::duration<double>(t - xuc_fire_candidate_since).count();
        const double required_stable_s = one_face_policy_active
          ? xuc_gyro_fire_stable_s
          : xuc_fire_stable_s;
        stable_window_open = stable_time >= required_stable_s;
        // XUC also receives a one-frame request for stationary targets. Once
        // the command rearm window closes this shot, a still-valid gate starts
        // a new qualification without flooding the serial link with requests.
        command.shoot = stable_window_open &&
          (!xuc.enabled() || !xuc_fire_ready_logged);
        if (command.shoot && !xuc_fire_ready_logged) {
          tools::logger()->info(
            "[XUC][FIRE] qualified after {:.0f} ms/{} frames, "
            "current_pixel_error_deg=({:.2f},{:.2f}) "
            "predicted_impact_yaw_error_deg={:.2f} gyro_predict={} "
            "face_error={:.2f}deg committed_face={} face_align={:.2f}deg "
            "phase_unc={:.2f}deg gimbal_err={:.2f}deg omega={:+.3f}rad_s armed={}",
            stable_time * 1000.0,
            xuc_fire_candidate_frames,
            direct_yaw_pixel_error_rad * 180.0 / M_PI,
            direct_pitch_pixel_error_rad * 180.0 / M_PI,
            gyro_predicted_impact_yaw_error_rad * 180.0 / M_PI,
            predictive_fire_active, face_error_rad * 180.0 / M_PI,
            gyro_event_face_id, gyro_event_face_alignment_rad * 180.0 / M_PI,
            gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
            gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI,
            gyro_omega_rad_s, xuc_one_face_armed);
          xuc_fire_ready_logged = true;
          if (one_face_policy_active) {
            xuc_one_face_armed = false;
            if (gyro_event_schedule_valid) {
              event_last_fired_cycle = gyro_event_hit_cycle;
            }
          }
        }
      } else {
        if (xuc_fire_candidate_active) {
          tools::logger()->info("[XUC][FIRE] stable window reset");
        }
        xuc_fire_candidate_active = false;
        xuc_fire_ready_logged = false;
        xuc_fire_candidate_frames = 0;
        command.shoot = false;
      }
      if (infantry_diagnostics_enabled) {
        fire_gate_open = {
          armors_after_color != 0, !targets.empty(), convergence_gate_open,
          !xuc.enabled() || xuc_imu_sync_valid, auto_mode_active, command.control,
          direct_yaw_measurement_fresh, direct_pitch_measurement_fresh,
          yaw_gate_open,
          std::abs(direct_pitch_pixel_error_rad) <= xuc_fire_pitch_tolerance_rad,
          one_face_gate_open, motion_transition_gate_open,
          command_cadence_gate_open, stable_window_open};
        fire_gate_snapshot_valid = true;
      }
    } else {
      command.shoot = native_shoot;
      if (measurement_fallback) command.shoot = false;
    }
    if (infantry_diagnostics_enabled && fire_gate_snapshot_valid) {
      const auto gate_now = std::chrono::steady_clock::now();
      for (std::size_t gate = 0; gate < fire_gate_open.size(); ++gate) {
        if (!fire_gate_open[gate]) {
          fire_first_blocker = static_cast<int>(gate);
          break;
        }
      }
      const double sampled_ms = fire_last_gate_sample.time_since_epoch().count() == 0
        ? 0.0
        : std::clamp(std::chrono::duration<double, std::milli>(
            gate_now - fire_last_gate_sample).count(), 0.0, 200.0);
      fire_last_gate_sample = gate_now;
      if (fire_first_blocker != fire_previous_blocker) fire_current_episode_ms = 0.0;
      if (fire_first_blocker >= 0) {
        const auto index = static_cast<std::size_t>(fire_first_blocker);
        ++fire_blocked_frames[index];
        fire_blocked_ms[index] += sampled_ms;
        fire_current_episode_ms += sampled_ms;
        fire_longest_episode_ms[index] = std::max(
          fire_longest_episode_ms[index], fire_current_episode_ms);
      }
      fire_previous_blocker = fire_first_blocker;
      if (fire_last_gate_summary.time_since_epoch().count() == 0 ||
          gate_now - fire_last_gate_summary >= std::chrono::seconds(5)) {
        nlohmann::json summary;
        for (std::size_t gate = 0; gate < fire_gate_names.size(); ++gate) {
          summary[fire_gate_names[gate]] = {
            {"first_block_frames", fire_blocked_frames[gate]},
            {"first_block_ms", fire_blocked_ms[gate]},
            {"longest_episode_ms", fire_longest_episode_ms[gate]}};
        }
        tools::logger()->info("[FIRE_GATE_SUMMARY] {}", summary.dump());
        fire_last_gate_summary = gate_now;
      }
    }

    // Final actuator tracking layer.  Form the error before adding PI output so
    // the integrator converges to the extra command needed to make actual raw IMU
    // yaw follow the ESKF/direct-guard reference instead of chasing its own
    // boosted output.  Reset on loss/exit to prevent stale wind-up at reacquire.
    double yaw_tracking_error_raw_rad = NAN;
    double yaw_tracking_output_raw_rad = 0.0;
    if (!lower_auto_ready_now) {
      // The vision pipeline continues producing targets while the RC switch is
      // outside AUTO.  Never integrate that physically untrackable error and
      // then dump it into the gimbal on the AUTO edge.
      yaw_tracking_integral_raw_rad = 0.0;
      yaw_tracking_initialized = false;
      yaw_tracking_lower_armed = false;
    } else if (!yaw_tracking_lower_armed) {
      yaw_tracking_lower_armed = true;
      yaw_tracking_armed_since = std::chrono::steady_clock::now();
      yaw_tracking_last_update = yaw_tracking_armed_since;
      yaw_tracking_integral_raw_rad = 0.0;
      yaw_tracking_initialized = true;
    }

    if (yaw_tracking_pi_enabled && lower_auto_ready_now &&
        yaw_tracking_lower_armed && xuc.rx_valid() && command.control &&
        std::isfinite(command.yaw)) {
      const auto pi_now = std::chrono::steady_clock::now();
      const double armed_age_s = std::chrono::duration<double>(
        pi_now - yaw_tracking_armed_since).count();
      const double dt = yaw_tracking_initialized &&
          armed_age_s >= yaw_tracking_arm_dwell_s
        ? std::clamp(
            std::chrono::duration<double>(pi_now - yaw_tracking_last_update).count(),
            0.0,
            0.05)
        : 0.0;
      yaw_tracking_last_update = pi_now;
      yaw_tracking_initialized = true;

      const double reference_raw_rad = xuc.command_yaw_to_raw(command.yaw);
      const double feedback_raw_rad = xuc.raw_imu_yaw();
      yaw_tracking_error_raw_rad = std::remainder(
        reference_raw_rad - feedback_raw_rad, 2.0 * M_PI);
      const bool low_dynamic_measurement = direct_yaw_measurement_fresh &&
        tracker_converged &&
        (!gyro_model_fresh || std::abs(gyro_omega_rad_s) <
          xuc_gyro_omega_threshold_rad_s);
      if (armed_age_s >= yaw_tracking_arm_dwell_s &&
          low_dynamic_measurement &&
          std::isfinite(yaw_tracking_error_raw_rad) &&
          std::abs(yaw_tracking_error_raw_rad) <=
            yaw_tracking_integrate_error_limit_rad) {
        yaw_tracking_integral_raw_rad = std::clamp(
          yaw_tracking_integral_raw_rad +
            yaw_tracking_ki_per_s * yaw_tracking_error_raw_rad * dt,
          -yaw_tracking_integral_limit_rad,
          yaw_tracking_integral_limit_rad);
      }
      if (armed_age_s >= yaw_tracking_arm_dwell_s) {
        // Proportional action is permitted only on a fresh, low-dynamic
        // measurement.  During gyro motion retain at most the slowly learned
        // static trim; ESKF/phase prediction remains the sole dynamic lead.
        const double proportional_raw_rad = low_dynamic_measurement
          ? yaw_tracking_kp * yaw_tracking_error_raw_rad
          : 0.0;
        yaw_tracking_output_raw_rad = std::clamp(
          proportional_raw_rad + yaw_tracking_integral_raw_rad,
          -yaw_tracking_output_limit_rad,
          yaw_tracking_output_limit_rad);
      }
      command.yaw = std::remainder(
        command.yaw +
          xuc.raw_yaw_correction_to_command(yaw_tracking_output_raw_rad),
        2.0 * M_PI);
    } else {
      yaw_tracking_integral_raw_rad = 0.0;
      yaw_tracking_initialized = false;
      if (!lower_auto_ready_now) yaw_tracking_lower_armed = false;
    }

    // Dry-run diagnostics: world-coordinate stability and final command.
    static int dryrun_log_counter = 0;
    if (infantry_diagnostics_enabled && ++dryrun_log_counter % 20 == 0) {
      constexpr double RAD2DEG = 180.0 / M_PI;
      if (!targets.empty()) {
        auto tracked_xyza_list = targets.front().armor_xyza_list();
        if (!tracked_xyza_list.empty() && direct_measurement_armor != nullptr) {
          const auto & measured_gimbal = direct_measurement_armor->xyz_in_gimbal;
          const auto & measured_world = direct_measurement_armor->xyz_in_world;
          const auto & tracked = tracked_xyza_list.front();
          double armor_center_y_px = direct_measurement_armor->center.y;
          if (!direct_measurement_armor->points.empty()) {
            armor_center_y_px = 0.0;
            for (const auto & point : direct_measurement_armor->points) armor_center_y_px += point.y;
            armor_center_y_px /= static_cast<double>(direct_measurement_armor->points.size());
          }
          const double pixel_pitch_error_deg = direct_pitch_focal_y_px > 0.0
            ? std::atan2(
                armor_center_y_px - 0.5 * static_cast<double>(img.rows),
                direct_pitch_focal_y_px) * RAD2DEG
            : 0.0;

          const double target_gimbal_yaw_deg =
            std::atan2(measured_gimbal.y(), measured_gimbal.x()) * RAD2DEG;
          const double target_gimbal_pitch_deg =
            std::atan2(
              measured_gimbal.z(),
              std::hypot(measured_gimbal.x(), measured_gimbal.y())) * RAD2DEG;
          const double target_world_yaw_deg =
            std::atan2(measured_world.y(), measured_world.x()) * RAD2DEG;
          const double target_world_pitch_deg =
            std::atan2(
              measured_world.z(),
              std::hypot(measured_world.x(), measured_world.y())) * RAD2DEG;

          tools::logger()->info(
            "[DRYRUN] measured_gimbal=({:.3f},{:.3f},{:.3f}) "
            "measured_world=({:.3f},{:.3f},{:.3f}) "
            "tracked_world=({:.3f},{:.3f},{:.3f}) "
            "target_gimbal_deg=({:.2f},{:.2f}) "
            "target_world_deg=({:.2f},{:.2f}) "
            "gimbal_deg=({:.2f},{:.2f}) "
            "armor_y_px={:.1f} pixel_error_deg=({:.2f},{:.2f}) "
            "ballistic=(range={:.3f}m holdover={:.3f}m comp={:.2f}deg residual={:.2f}deg) "
            "q_wxyz=({:.6f},{:.6f},{:.6f},{:.6f}) "
            "cmd_deg=({:.2f},{:.2f}) packed_pitch_deg={:.2f} "
            "yaw_pi=(error={:.2f}deg integral={:.2f}deg output={:.2f}deg) "
            "control={} shoot={}",
            measured_gimbal.x(), measured_gimbal.y(), measured_gimbal.z(),
            measured_world.x(), measured_world.y(), measured_world.z(),
            tracked.x(), tracked.y(), tracked.z(),
            target_gimbal_yaw_deg, target_gimbal_pitch_deg,
            target_world_yaw_deg, target_world_pitch_deg,
            ypr[0] * RAD2DEG, ypr[1] * RAD2DEG,
            armor_center_y_px, direct_yaw_pixel_error_rad * RAD2DEG, pixel_pitch_error_deg,
            ballistic_camera_range_m, ballistic_total_holdover_m,
            ballistic_pitch_compensation_rad * RAD2DEG,
            direct_pitch_pixel_error_rad * RAD2DEG,
            q.w(), q.x(), q.y(), q.z(),
            command.yaw * RAD2DEG, command.pitch * RAD2DEG,
            xuc_pitch_sign_for_log * command.pitch * RAD2DEG,
            yaw_tracking_error_raw_rad * RAD2DEG,
            yaw_tracking_integral_raw_rad * RAD2DEG,
            yaw_tracking_output_raw_rad * RAD2DEG,
            command.control, command.shoot);
        }
      } else {
        tools::logger()->info(
          "[DRYRUN] target=none gimbal_deg=({:.2f},{:.2f}) control={} shoot={}",
          ypr[0] * RAD2DEG, ypr[1] * RAD2DEG,
          command.control, command.shoot);
      }
    }

    // For an event-qualified gyro shot, wait for the exact steady-clock due
    // instant instead of quantizing the request to the next camera frame.  A
    // sample that was already late before this deliberate wait is discarded;
    // the small tolerance only covers OS wake-up jitter after sleep_until().
    double gyro_event_send_lateness_ms = NAN;
    if (xuc.enabled() && command.shoot) {
      if (xuc.mode() != static_cast<uint8_t>(io::Mode::auto_aim)) {
        tools::logger()->warn(
          "[SHOT] request suppressed because lower-board mode={} is not AUTO",
          static_cast<int>(xuc.mode()));
        command.shoot = false;
      } else if (gyro_event_schedule_valid) {
        const auto wait_started = std::chrono::steady_clock::now();
        if (wait_started > gyro_event_command_due_time) {
          gyro_event_send_lateness_ms = std::chrono::duration<double, std::milli>(
            wait_started - gyro_event_command_due_time).count();
          tools::logger()->warn(
            "[GYROSCHED] cycle={} missed before send by {:.2f} ms; shot discarded",
            gyro_event_hit_cycle, gyro_event_send_lateness_ms);
          command.shoot = false;
        } else {
          std::this_thread::sleep_until(gyro_event_command_due_time);
          const auto woke_at = std::chrono::steady_clock::now();
          gyro_event_send_lateness_ms = std::chrono::duration<double, std::milli>(
            woke_at - gyro_event_command_due_time).count();
          if (gyro_event_send_lateness_ms >
              xuc_event_send_late_tolerance_s * 1000.0) {
            tools::logger()->warn(
              "[GYROSCHED] cycle={} wake late by {:.2f} ms; shot discarded",
              gyro_event_hit_cycle, gyro_event_send_lateness_ms);
            command.shoot = false;
          }
        }
      }
    }

    // Attach a software shot id to each qualified request. XucSender converts
    // this one-frame request into the lower board's high/low single-shot sequence;
    // that protocol still supplies no confirmed projectile-exit time.
    uint64_t issued_shot_id = 0;
    if (xuc.enabled() && command.shoot) {
      const auto issue_time = std::chrono::steady_clock::now();
      const uint64_t shot_id = xuc_next_shot_id++;
      issued_shot_id = shot_id;
      if (xuc_pending_shot) {
        tools::logger()->warn(
          "[SHOT] id={} superseded_before_rearm by={}",
          xuc_pending_shot_id, shot_id);
      }
      xuc_pending_shot = true;
      xuc_pending_shot_tx_ok = false;
      xuc_pending_slot_estimate_logged = false;
      xuc_pending_shot_id = shot_id;
      xuc_pending_shot_time = issue_time;
      xuc_pending_feeder_turns_at_request = xuc.feeder_motor_turns_estimate();
      const double processing_age_ms =
        std::chrono::duration<double, std::milli>(issue_time - t).count();
      tools::logger()->info(
        "[SHOT] id={} issue mode={} gyro={} event={} cycle={} "
        "processing_age_ms={:.1f} due_error_ms={:+.2f} "
        "period_ms={:.1f} window_deg={:.2f} worst_deg={:.2f} combined_deg={:.2f} "
        "face={} face_align_deg={:.2f} phase_unc_deg={:.2f} gimbal_err_deg={:.2f} "
        "imu_interp={} imu_extrap={} imu_alignment_ms={:+.1f} "
        "imu_bracket_ms={:.2f} imu_nearest_offset_ms={:+.2f}",
        shot_id, static_cast<int>(xuc.mode()), gyro_model_fresh ? 1 : 0,
        gyro_event_schedule_valid ? 1 : 0, gyro_event_hit_cycle,
        processing_age_ms, gyro_event_send_lateness_ms,
        gyro_event_face_period_s * 1000.0,
        gyro_event_half_window_rad * 180.0 / M_PI,
        gyro_event_worst_phase_rad * 180.0 / M_PI,
        gyro_event_combined_phase_rad * 180.0 / M_PI,
        gyro_event_face_id,
        gyro_event_face_alignment_rad * 180.0 / M_PI,
        gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
        gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI,
        frame_imu.interpolated ? 1 : 0,
        frame_imu.extrapolated ? 1 : 0, xuc_imu_alignment_ms,
        frame_imu.bracket_ms, frame_imu.nearest_offset_ms);
    }

    // 发送命令到 PlotJuggler（显示实际发送的值）
    nlohmann::json plot_data;
    plot_data["t"] = std::chrono::duration<double>(t - std::chrono::steady_clock::time_point()).count();
    plot_data["cmd_yaw"] = command.yaw * 180.0 / M_PI;
    plot_data["cmd_pitch"] =
      xuc_pitch_sign_for_log * command.pitch * 180.0 / M_PI;
    plot_data["yaw_tracking_error_raw_deg"] = std::isfinite(yaw_tracking_error_raw_rad)
      ? nlohmann::json(yaw_tracking_error_raw_rad * 180.0 / M_PI)
      : nlohmann::json(nullptr);
    plot_data["yaw_tracking_integral_raw_deg"] =
      yaw_tracking_integral_raw_rad * 180.0 / M_PI;
    plot_data["yaw_tracking_output_raw_deg"] =
      yaw_tracking_output_raw_rad * 180.0 / M_PI;
    plot_data["control"] = command.control;
    plot_data["shoot"] = command.shoot;
    if (gyro_predicted_impact_yaw_valid) {
      plot_data["gyro_predicted_impact_yaw_error_deg"] =
        gyro_predicted_impact_yaw_error_rad * 180.0 / M_PI;
    } else {
      plot_data["gyro_predicted_impact_yaw_error_deg"] = nullptr;
    }
    if (gyro_event_schedule_valid) {
      plot_data["gyro_event_due_ms"] = gyro_event_lead_error_s * 1000.0;
      plot_data["gyro_event_period_ms"] = gyro_event_face_period_s * 1000.0;
      plot_data["gyro_event_interval_safe"] = gyro_event_interval_safe;
      plot_data["gyro_event_half_window_deg"] =
        gyro_event_half_window_rad * 180.0 / M_PI;
      plot_data["gyro_event_worst_phase_deg"] =
        gyro_event_worst_phase_rad * 180.0 / M_PI;
      plot_data["gyro_event_combined_phase_deg"] =
        gyro_event_combined_phase_rad * 180.0 / M_PI;
      plot_data["gyro_event_phase_uncertainty_deg"] =
        gyro_event_phase_uncertainty_rad * 180.0 / M_PI;
      plot_data["gyro_event_face_id"] = gyro_event_face_id;
      plot_data["gyro_event_face_alignment_deg"] =
        gyro_event_face_alignment_rad * 180.0 / M_PI;
      plot_data["gyro_event_gimbal_tracking_error_deg"] =
        gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI;
    } else {
      plot_data["gyro_event_due_ms"] = nullptr;
      plot_data["gyro_event_period_ms"] = nullptr;
      plot_data["gyro_event_interval_safe"] = false;
      plot_data["gyro_event_half_window_deg"] = nullptr;
      plot_data["gyro_event_worst_phase_deg"] = nullptr;
      plot_data["gyro_event_combined_phase_deg"] = nullptr;
      plot_data["gyro_event_phase_uncertainty_deg"] = nullptr;
      plot_data["gyro_event_face_id"] = nullptr;
      plot_data["gyro_event_face_alignment_deg"] = nullptr;
      plot_data["gyro_event_gimbal_tracking_error_deg"] = nullptr;
    }
    // 电控的欧拉角（从MCU获取的姿态）
    plot_data["mcu_yaw"] = ypr[0] * 180.0 / M_PI;
    plot_data["mcu_pitch"] = ypr[1] * 180.0 / M_PI;
    plot_data["mcu_roll"] = ypr[2] * 180.0 / M_PI;
    // 电控的四元数（从MCU获取）
    plot_data["mcu_q_w"] = q.w();
    plot_data["mcu_q_x"] = q.x();
    plot_data["mcu_q_y"] = q.y();
    plot_data["mcu_q_z"] = q.z();
    plotter.plot(plot_data);

    // Send the command computed from this frame. The camera runs independently
    // in continuous acquisition mode; sending does not timestamp its exposure.
    const uint64_t tx_count_before = xuc.enabled()
      ? xuc.successful_tx_count() : 0;
    if (xuc.enabled()) {
      if (infantry_diagnostics_enabled && shot_prediction_debug) {
        const io::XucShotDebugContext context{
          issued_shot_id, diagnostics_run_id, static_cast<uint64_t>(camera_frame_count),
          static_cast<int64_t>(frame_id),
          duration_cast<nanoseconds>(t.time_since_epoch()).count()};
        const bool accepted = xuc.send_debug(command, &context);
        if (issued_shot_id) {
          // All values below still belong to this frame. Construct only after
          // send(), and let the bounded worker perform dump()/logging.
          try {
            const auto capture_begin = steady_clock::now();
            const auto number = [](double value) -> nlohmann::json {
              return std::isfinite(value) ? nlohmann::json(value) : nlohmann::json(nullptr);
            };
            const auto & debug = *shot_prediction_debug;
            const bool aimer_speed_fallback = !std::isfinite(debug.aimer_input_mps) ||
              debug.aimer_input_mps < 5.0 || debug.aimer_input_mps > 35.0;
            nlohmann::json snapshot = {
              {"kind", "request_snapshot"}, {"schema", 2}, {"id", issued_shot_id},
              {"run", context.run}, {"frame_seq", context.frame_seq},
              {"frame_id", context.frame_id}, {"frame_ns", context.frame_ns},
              {"accepted", accepted}, {"timestamp_basis", "host_steady_clock"},
              {"request_issue_ns", duration_cast<nanoseconds>(
                xuc_pending_shot_time.time_since_epoch()).count()},
              {"snapshot_built_after_send", true},
              {"image_timestamp_basis", "host_receive_with_configured_offset"},
              {"physical_exposure_center_ns", nullptr},
              {"motion_mode", auto_aim::RotationModelManager::name(rotation_model.mode)},
              {"fire_branch", gyro_event_schedule_valid ? "high_speed_event" : debug.gate_branch},
              {"primary_observed_face", tracker.primary_face_id()},
              {"future_native_face_id", debug.native.valid
                ? nlohmann::json(debug.native.face_id) : nlohmann::json(nullptr)},
              {"future_event_face_id", gyro_event_face_id},
              {"native_prediction_valid", native_prediction_valid},
              {"native_geometry_source", debug.native.whole_ekf_geometry
                ? "whole_ekf_rigid" : "existing"},
              {"native_prediction_trusted", native_prediction_trusted},
              {"aimer_call_begin_ns", debug.aimer_begin_ns},
              {"aimer_call_end_ns", debug.aimer_end_ns},
              {"native_prediction_time_ns", debug.native.valid
                ? nlohmann::json(debug.native.prediction_ns) : nlohmann::json(nullptr)},
              {"native_flight_time_s", number(debug.native.flight_time_s)},
              {"native_flight_time_used_s", number(debug.native.flight_time_used_s)},
              {"native_iteration_converged", debug.native.converged},
              {"event_prediction_horizon_s", number(debug.event_prediction_horizon_s)},
              {"event_flight_time_s", number(debug.event_flight_time_s)},
              {"event_delay_s", number(debug.event_delay_s)},
              {"planned_send_ns", gyro_event_schedule_valid
                ? nlohmann::json(duration_cast<nanoseconds>(
                    gyro_event_command_due_time.time_since_epoch()).count())
                : nlohmann::json(nullptr)},
              {"aimer_speed_input_mps", number(debug.aimer_input_mps)},
              {"aimer_speed_effective_mps", number(aimer_speed_fallback ? 23.0 : debug.aimer_input_mps)},
              {"aimer_speed_source", aimer_speed_fallback
                ? "aimer_internal_fallback_23_mps" : debug.input_source},
              {"feedback_speed_field_raw", number(debug.feedback_speed_mps)},
              {"feedback_speed_field_age_ms", nullptr},
              {"feedback_speed_field_units", "unverified"},
              {"latest_rx_ns_observed_at_speed_selection", debug.latest_rx_ns},
              {"latest_rx_age_ms_at_aim", debug.latest_rx_ns > 0
                ? number((debug.aimer_begin_ns - debug.latest_rx_ns) / 1.0e6)
                : nlohmann::json(nullptr)},
              {"speed_and_rx_timestamp_atomic_pair", false},
              {"direct_pitch_speed_mps", xuc_projectile_speed_mps},
              {"event_speed_mps", xuc_projectile_speed_mps},
              {"direct_pitch_and_event_speed_source", "xuc_projectile_speed_mps"},
              {"aimer_high_speed_delay_config_s", integration_yaml["high_speed_delay_time"].as<double>()},
              {"aimer_low_speed_delay_config_s", integration_yaml["low_speed_delay_time"].as<double>()},
              {"aimer_delay_adjustment_s", nullptr},
              {"range_raw_m", number(raw_pnp_range_m)},
              {"range_model_m", number(modeled_armor_range_m)},
              {"range_filtered_m", number(robust_target_range_m)},
              {"omega_rad_s", number(gyro_omega_rad_s)},
              {"gate_reference_yaw_rad", number(ypr[0])},
              {"predicted_yaw_rad", number(native_predicted_yaw)},
              {"predicted_yaw_trim_rad", xuc_gyro_predicted_yaw_trim_rad},
              {"predicted_yaw_error_rad", number(gyro_predicted_impact_yaw_error_rad)},
              {"predicted_yaw_tolerance_rad", xuc_gyro_predicted_yaw_tolerance_rad},
              {"stationary_yaw_tolerance_rad", xuc_fire_yaw_tolerance_rad},
              {"pixel_yaw_error_rad", number(direct_yaw_pixel_error_rad)},
              {"pitch_residual_rad", number(direct_pitch_pixel_error_rad)},
              {"pitch_tolerance_rad", xuc_fire_pitch_tolerance_rad},
              {"event_half_window_rad", number(gyro_event_half_window_rad)},
              {"event_worst_phase_rad", number(gyro_event_worst_phase_rad)},
              {"event_combined_phase_rad", number(gyro_event_combined_phase_rad)},
              {"event_phase_uncertainty_rad", number(gyro_event_phase_uncertainty_rad)},
              {"event_interval_safe", gyro_event_interval_safe},
              {"event_cycle", gyro_event_hit_cycle},
              {"yaw_boresight_correction_rad", direct_yaw_boresight_correction_rad},
              {"yaw_impact_correction_rad", direct_yaw_impact_correction_rad},
              {"output_yaw_rad", number(command.yaw)},
              {"output_pitch_rad", number(command.pitch)},
              {"tx_record_join_keys", {"run", "id"}},
              {"projectile_exit", "unknown"}, {"actual_projectile_count", nullptr}};
            snapshot["future_native_xyza"] = nullptr;
            if (native_prediction_valid && aimer.debug_aim_point.valid) {
              snapshot["future_native_xyza"] = nlohmann::json::array();
              for (int i = 0; i < 4; ++i) {
                snapshot["future_native_xyza"].push_back(number(aimer.debug_aim_point.xyza[i]));
              }
            }
            snapshot["future_event_xyza"] = nlohmann::json::array();
            for (double value : debug.event_xyza) {
              snapshot["future_event_xyza"].push_back(number(value));
            }
            snapshot["target"] = nullptr;
            snapshot["observation_xyz_world_m"] = nullptr;
            snapshot["observation_source"] = direct_measurement_from_tracker
              ? "tracker_accepted" : "current_frame_solved_fallback";
            if (direct_measurement_armor != nullptr) {
              snapshot["observation_xyz_world_m"] = {
                number(direct_measurement_armor->xyz_in_world[0]),
                number(direct_measurement_armor->xyz_in_world[1]),
                number(direct_measurement_armor->xyz_in_world[2])};
            }
            if (!targets.empty()) {
              const auto & target = targets.front();
              snapshot["target"] = {
                {"name", static_cast<int>(target.name)},
                {"armor_type", static_cast<int>(target.armor_type)},
                {"armor_count", target.armor_num()}, {"state_at_frame", nlohmann::json::array()},
                {"state_order", {"cx", "vx", "cy", "vy", "cz", "vz",
                  "body_yaw", "omega", "r1", "r2_delta", "height_delta"}}};
              const auto state = target.ekf_x();
              for (int i = 0; i < state.size(); ++i) {
                snapshot["target"]["state_at_frame"].push_back(number(state[i]));
              }
            }
            snapshot["fire_gates"] = nlohmann::json::object();
            if (fire_gate_snapshot_valid) {
              for (std::size_t i = 0; i < fire_gate_names.size(); ++i) {
                snapshot["fire_gates"][fire_gate_names[i]] = fire_gate_open[i];
              }
            }
            snapshot["capture_us"] = duration<double, std::micro>(
              steady_clock::now() - capture_begin).count();
            xuc.record_shot_debug(std::move(snapshot));
          } catch (...) {
            xuc.note_shot_debug_drop();
          }
        }
      } else {
        xuc.send(command);
      }
      if (command.shoot) {
        const bool shot_tx_ok = xuc.successful_tx_count() > tx_count_before &&
          xuc.last_tx_shoot() == 1;
        xuc_pending_shot_tx_ok = shot_tx_ok;
        if (infantry_diagnostics_enabled) tools::logger()->info(
          "[SHOT_TX] id={} tx_ok={} tx_attempt_ns={} tx_write_complete_ns={} "
          "tx_yaw_raw_deg={:.3f} "
          "tx_pitch_raw_deg={:.3f} feeder_feedback_raw={:.1f} "
          "feeder_motor_turns_est_at_request={:+.2f} "
          "feed_slot_observation=unknown projectile_exit=unknown",
          issued_shot_id, shot_tx_ok,
          shot_tx_ok ? xuc.last_tx_ns() : 0,
          shot_tx_ok ? xuc.last_tx_complete_ns() : 0,
          shot_tx_ok ? xuc.last_tx_yaw_raw() * 180.0 / M_PI : NAN,
          shot_tx_ok ? xuc.last_tx_pitch_raw() * 180.0 / M_PI : NAN,
          xuc.feeder_speed_rpm(), xuc_pending_feeder_turns_at_request);
        const int64_t tx_ns = xuc.last_shoot_tx_ns();
        if (shot_tx_ok && tx_ns > 0) {
          xuc_pending_shot_time = std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(tx_ns));
        }
      }
    } else {
      cboard.send(command);
    }

    // Continuous debug evidence, independent of request cadence/permissions.
    // Observe only: no extra aim(), predict(), qualification, wait or send.
    if (infantry_diagnostics_enabled && shot_prediction_debug && prediction_session_debug) {
      try {
        const auto capture_begin = steady_clock::now();
        const auto number = [](double value) -> nlohmann::json {
          return std::isfinite(value) ? nlohmann::json(value) : nlohmann::json(nullptr);
        };
        const auto & debug = *shot_prediction_debug;
        const auto & native = debug.native;
        auto & session = *prediction_session_debug;
        const auto shooter_feedback = xuc.take_shooter_feedback();
        if (!shooter_feedback.empty()) {
          nlohmann::json samples = nlohmann::json::array();
          for (const auto & sample : shooter_feedback) {
            samples.push_back({{"host_rx_ns", sample.host_rx_ns},
              {"feedback_sequence", sample.feedback_sequence},
              {"fire_flag", sample.fire_flag}, {"fire_tick_ms", sample.fire_tick_ms},
              {"friction_current_raw", sample.friction_current_raw},
              {"feeder_speed_rpm", sample.feeder_speed_rpm}});
          }
          xuc.record_shot_debug({{"kind", "shooter_feedback"}, {"run", diagnostics_run_id},
            {"frame_seq", camera_frame_count}, {"samples", std::move(samples)},
            {"samples_dropped_total", xuc.shooter_feedback_dropped()},
            {"host_timestamp_basis", "host_steady_clock_parse"},
            {"lower_timestamp_basis", "freertos_tick_1khz"},
            {"fire_flag_basis", "lower_current_threshold"},
            {"friction_current_units", "raw_can_feedback"},
            {"feeder_speed_units", "motor_rotor_rpm"}, {"projectile_exit", "unknown"}});
        }
        const auto & td = tracker.frame_diagnostics();
        const bool has_target = !targets.empty();
        const int name = has_target ? static_cast<int>(targets.front().name) : -1;
        const int type = has_target ? static_cast<int>(targets.front().armor_type) : -1;
        const int count = has_target ? targets.front().armor_num() : 0;
        const bool epoch_reset = has_target &&
          (!session.had_target || td.initialized_this_frame || td.nis_reset || td.diverged ||
           name != session.name || type != session.type || count != session.armor_count);
        if (epoch_reset) ++session.track_epoch;
        session.had_target = has_target;
        session.name = name;
        session.type = type;
        session.armor_count = count;
        const bool tx_ok = xuc.successful_tx_count() > tx_count_before;
        const auto rx_ns = xuc.last_rx_ns();
        io::XucTimedImu feedback_pose;
        const bool feedback_valid = xuc.rx_valid() && rx_ns > 0 && xuc.imu_at(
          steady_clock::time_point(nanoseconds(rx_ns)), feedback_pose);
        const auto coefficient = [&](const char * key, double fallback) {
          return integration_yaml[key] ? integration_yaml[key].as<double>() : fallback;
        };
        nlohmann::json trace = {
          {"kind", "prediction_frame"}, {"schema", 2}, {"run", diagnostics_run_id},
          {"frame_seq", camera_frame_count}, {"frame_id", frame_id},
          {"frame_ns", duration_cast<nanoseconds>(t.time_since_epoch()).count()},
          {"timestamp_basis", "host_steady_clock"},
          {"image_timestamp_basis", "host_receive_with_configured_offset"},
          {"physical_exposure_center_ns", nullptr},
          {"track_epoch", session.track_epoch}, {"track_epoch_reset", epoch_reset},
          {"target_present", has_target}, {"target_name", name}, {"target_type", type},
          {"armor_count", count}, {"tracker_converged", tracker_converged},
          {"frame_imu_valid", xuc_imu_sync_valid},
          {"frame_imu_query_ns", duration_cast<nanoseconds>(
            frame_imu_requested_time.time_since_epoch()).count()},
          {"frame_imu_pose_rad", {number(ypr[0]), number(ypr[1])}},
          {"frame_imu_quaternion_wxyz", {number(q.w()), number(q.x()), number(q.y()), number(q.z())}},
          {"image_size_px", {img.cols, img.rows}},
          {"native_prediction_valid", native_prediction_valid && native.valid},
          {"native_prediction_trusted", native_prediction_trusted && native.valid},
          {"native_geometry_source", native.whole_ekf_geometry
            ? "whole_ekf_rigid" : "existing"},
          {"native_prediction_ns", native.valid
            ? nlohmann::json(native.prediction_ns) : nlohmann::json(nullptr)},
          {"native_base_prediction_ns", native.valid
            ? nlohmann::json(native.base_prediction_ns) : nlohmann::json(nullptr)},
          {"native_face_id", native.valid ? native.face_id : -1},
          {"native_iteration_converged", native.converged},
          {"native_iterations", native.iterations},
          {"native_flight_time_used_s", number(native.flight_time_used_s)},
          {"native_flight_time_s", number(native.flight_time_s)},
          {"native_delay_s", number(native.delay_s)},
          {"speed_mps", number(native.speed_mps)}, {"speed_source", debug.input_source},
          {"aimer_call_begin_ns", debug.aimer_begin_ns}, {"aimer_call_end_ns", debug.aimer_end_ns},
          {"omega_rad_s", number(gyro_omega_rad_s)},
          {"motion_mode", auto_aim::RotationModelManager::name(rotation_model.mode)},
          {"fire_branch", gyro_event_schedule_valid ? "high_speed_event" : debug.gate_branch},
          {"request_id", issued_shot_id}, {"request_shoot", command.shoot},
          {"gate_error_rad", number(gyro_predicted_impact_yaw_error_rad)},
          {"rotation_fire_window", {
            {"frames", rotation_fire_window.frames},
            {"two_frames_ready", rotation_fire_window.two_frames_ready},
            {"current_frame_is_better", rotation_fire_window.current_frame_is_better},
            {"next_error_rad", number(rotation_fire_window.next_error_rad)}}},
          {"gate_tolerance_rad", xuc_gyro_predicted_yaw_tolerance_rad},
          {"gate_reference_yaw_rad", number(ypr[0])},
          {"native_yaw_rad", number(native_predicted_yaw)},
          {"output_yaw_rad", number(command.yaw)}, {"output_pitch_rad", number(command.pitch)},
          {"yaw_before_final_rate_rad", number(yaw_before_final_rate_rad)},
          {"yaw_after_final_rate_rad", number(yaw_after_final_rate_rad)},
          {"yaw_boresight_correction_rad", direct_yaw_boresight_correction_rad},
          {"yaw_impact_correction_rad", direct_yaw_impact_correction_rad},
          {"gyro_center_follow_active", gyro_center_follow_active},
          {"tx_yaw_scale", coefficient("xuc_yaw_sign", 1.0)},
          {"tx_pitch_scale", coefficient("xuc_pitch_sign", 1.0)},
          {"output_control", command.control}, {"tx_ok", tx_ok},
          {"tx_write_complete_ns", tx_ok
            ? nlohmann::json(xuc.last_tx_complete_ns()) : nlohmann::json(nullptr)},
          {"tx_raw_yaw_rad", tx_ok ? number(xuc.last_tx_yaw_raw()) : nlohmann::json(nullptr)},
          {"tx_raw_pitch_rad", tx_ok ? number(xuc.last_tx_pitch_raw()) : nlohmann::json(nullptr)},
          {"tx_control", tx_ok && xuc.last_tx_control()},
          {"tx_shoot", tx_ok ? nlohmann::json(xuc.last_tx_shoot()) : nlohmann::json(nullptr)},
          {"feedback_pose_ns", feedback_valid
            ? nlohmann::json(duration_cast<nanoseconds>(feedback_pose.timestamp.time_since_epoch()).count())
            : nlohmann::json(nullptr)},
          {"feedback_world_yaw_rad", feedback_valid ? number(feedback_pose.yaw) : nlohmann::json(nullptr)},
          {"feedback_world_pitch_rad", feedback_valid ? number(feedback_pose.pitch) : nlohmann::json(nullptr)},
          {"feedback_imu_yaw_scale", coefficient("xuc_imu_yaw_sign", 1.0)},
          {"feedback_imu_yaw_offset_rad", coefficient("xuc_imu_yaw_offset_rad", 0.0)},
          {"feedback_imu_pitch_scale", coefficient("xuc_imu_pitch_sign", 1.0)},
          {"feedback_imu_pitch_offset_rad", coefficient("xuc_imu_pitch_offset_rad", 0.0)},
          {"projectile_exit", "unknown"}, {"actual_projectile_count", nullptr},
          {"native_xyza", nullptr}, {"state_at_frame", nullptr},
          {"whole_ekf_state_at_frame", nullptr},
          {"state_before_update", nullptr}, {"tracker_initialized", td.initialized_this_frame},
          {"tracker_update_attempted", td.update_attempted},
          {"primary_face_id", tracker.primary_face_id()},
          {"detected_armor_count", armors.size()},
          {"association_reject_reason", td.association_reject_reason},
          {"fire_gates", nlohmann::json::object()},
          {"detections", nlohmann::json::array()},
          {"associations", nlohmann::json::array()},
          {"observations", nlohmann::json::array()}};
        // Exact transform used by this Solver frame, avoiding Euler/sign reconstruction.
        trace["solver_R_gimbal2world_rowmajor"] = nlohmann::json::array();
        const auto & solver_rotation = solver.R_gimbal2world();
        for (int row = 0; row < 3; ++row)
          for (int col = 0; col < 3; ++col)
            trace["solver_R_gimbal2world_rowmajor"].push_back(number(solver_rotation(row, col)));
        // Repeat context so a dropped first record does not invalidate an entire run.
        if (!session.calibration_recorded || camera_frame_count % 60 == 0) {
          trace["calibration"] = {{"solver_model", "ippe_fixed_pitch_yaw_grid_v1"}};
          for (const char * key : {"camera_matrix", "distort_coeffs", "R_camera2gimbal",
                                  "t_camera2gimbal", "R_gimbal2imubody"}) {
            trace["calibration"][key] = nlohmann::json::array();
            for (double value : integration_yaml[key].as<std::vector<double>>())
              trace["calibration"][key].push_back(number(value));
          }
          session.calibration_recorded = true;
        }
        if (has_target && td.update_attempted && !td.initialized_this_frame && !td.nis_reset && !td.diverged) {
          trace["state_before_update"] = nlohmann::json::array();
          for (int i = 0; i < td.state_before.size(); ++i)
            trace["state_before_update"].push_back(number(td.state_before[i]));
        }
        const auto corners = [&](const auto_aim::Armor & armor) {
          auto points = nlohmann::json::array();
          for (std::size_t i = 0; i < std::min<std::size_t>(armor.points.size(), 4); ++i)
            points.push_back({number(armor.points[i].x), number(armor.points[i].y)});
          return points;
        };
        // Bound evidence size, without truncating the detector/tracker input itself.
        constexpr std::size_t evidence_limit = 8;
        int detection_index = 0;
        for (const auto & armor : armors) {
          if (trace["detections"].size() >= evidence_limit) break;
          trace["detections"].push_back({
            {"detection_index", detection_index++}, {"name", static_cast<int>(armor.name)},
            {"type", static_cast<int>(armor.type)}, {"class_id", armor.class_id},
            {"confidence", number(armor.confidence)},
            {"keypoint_confidence", number(armor.keypoint_confidence)}, {"corners_px", corners(armor)}});
        }
        trace["detections_truncated"] = armors.size() > evidence_limit;
        trace["associations_total"] = tracker.last_associations().size();
        for (const auto & association : tracker.last_associations()) {
          if (trace["associations"].size() >= evidence_limit) break;
          trace["associations"].push_back({
            {"detection_index", association.detection_index}, {"face_id", association.face_id},
            {"cost", number(association.cost)}, {"used_for_update", association.used_for_update},
            {"identity_fallback", association.identity_fallback},
            {"observed_name", static_cast<int>(association.observed_name)},
            {"observed_type", static_cast<int>(association.observed_type)},
            {"type_corrected", association.identity_fallback_type_corrected}});
        }
        trace["associations_truncated"] = tracker.last_associations().size() > evidence_limit;
        if (fire_gate_snapshot_valid) {
          for (std::size_t i = 0; i < fire_gate_names.size(); ++i) {
            trace["fire_gates"][fire_gate_names[i]] = fire_gate_open[i];
          }
        }
        if (native_prediction_valid && native.valid && aimer.debug_aim_point.valid) {
          trace["native_xyza"] = nlohmann::json::array();
          for (int i = 0; i < 4; ++i) trace["native_xyza"].push_back(number(aimer.debug_aim_point.xyza[i]));
        }
        if (has_target) {
          trace["state_at_frame"] = nlohmann::json::array();
          const auto state = targets.front().ekf_x();
          for (int i = 0; i < state.size(); ++i) trace["state_at_frame"].push_back(number(state[i]));
          trace["whole_ekf_state_at_frame"] = nlohmann::json::array();
          const auto & whole_state = targets.front().ekf().x;
          for (int i = 0; i < whole_state.size(); ++i)
            trace["whole_ekf_state_at_frame"].push_back(number(whole_state[i]));
        }
        // Raw solved observations accepted for this frame, not future EKF states.
        for (const auto & association : tracker.last_associations()) {
          if (trace["observations"].size() >= evidence_limit) break;
          if (!association.used_for_update || !association.observation) continue;
          const auto & armor = *association.observation;
          if (!armor.xyz_in_world.allFinite()) continue;
          trace["observations"].push_back({
            {"face_id", association.face_id}, {"identity_fallback", association.identity_fallback},
            {"detection_index", association.detection_index}, {"association_cost", number(association.cost)},
            {"type_corrected", association.identity_fallback_type_corrected},
            {"name", static_cast<int>(armor.name)}, {"type", static_cast<int>(armor.type)},
            {"confidence", number(armor.confidence)},
            {"keypoint_confidence", number(armor.keypoint_confidence)}, {"corners_px", corners(armor)},
            {"xyz_gimbal_m", {number(armor.xyz_in_gimbal[0]), number(armor.xyz_in_gimbal[1]),
                               number(armor.xyz_in_gimbal[2])}},
            {"ypr_world_rad", {number(armor.ypr_in_world[0]), number(armor.ypr_in_world[1]),
                                number(armor.ypr_in_world[2])}},
            // yaw_raw is not initialized on the balance branch; never read it there.
            {"pnp_yaw_before_optimization_rad",
              armor.type == auto_aim::ArmorType::big &&
                (armor.name == auto_aim::ArmorName::three || armor.name == auto_aim::ArmorName::four ||
                 armor.name == auto_aim::ArmorName::five)
                ? nlohmann::json(nullptr) : number(armor.yaw_raw)},
            {"xyz_world_m", {number(armor.xyz_in_world[0]), number(armor.xyz_in_world[1]),
                              number(armor.xyz_in_world[2])}},
            {"body_yaw_rad", number(armor.ypr_in_world[0])}});
        }
        const auto observation_count = std::count_if(
          tracker.last_associations().begin(), tracker.last_associations().end(),
          [](const auto & association) { return association.used_for_update; });
        trace["observations_total"] = observation_count;
        trace["observations_truncated"] = observation_count > evidence_limit;
        trace["capture_us"] = duration<double, std::micro>(steady_clock::now() - capture_begin).count();
        xuc.record_shot_debug(std::move(trace));
      } catch (...) {
        xuc.note_shot_debug_drop();
      }
    }

    // Keep the compact console status aligned with the packet sent for this
    // frame.  XucSender may extend a one-frame request into its configured
    // shoot hold, so report the transmitted bit rather than the request bit.
    if (!competition_headless) {
      const bool debug_tx_ok =
        !xuc.enabled() || xuc.successful_tx_count() > tx_count_before;
      const int debug_shoot = xuc.enabled()
        ? (debug_tx_ok ? static_cast<int>(xuc.last_tx_shoot()) : 0)
        : (command.shoot ? 1 : 0);
      fmt::print(
        "[DEBUG] armors={}, targets={}, shoot={}\n",
        armors.size(), targets.size(), debug_shoot);
    }

    // This is intentionally after send() and before drawing on img: one record
    // refers to one camera frame, its detector/tracker result, the transmitted
    // yaw packet, and the freshest available lower-board IMU feedback.
    if (infantry_diagnostics_enabled) {
      const auto diagnostic_now = std::chrono::steady_clock::now();
      const auto diagnostic_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        diagnostic_now.time_since_epoch()).count();
      const auto frame_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        t.time_since_epoch()).count();
      const auto rx_ns = xuc.enabled() ? xuc.last_rx_ns() : 0;
      const bool rx_valid = xuc.enabled() && xuc.rx_valid();
      const bool tx_ok = xuc.enabled() && xuc.successful_tx_count() > tx_count_before;
      const auto tx_ns = tx_ok ? xuc.last_tx_ns() : 0;
      const auto tx_control = tx_ok && xuc.last_tx_control();
      const auto tx_yaw_raw = tx_ok ? xuc.last_tx_yaw_raw() : NAN;
      const auto tx_pitch_raw = tx_ok ? xuc.last_tx_pitch_raw() : NAN;
      const auto rx_yaw_raw = rx_valid ? xuc.raw_imu_yaw() : NAN;
      const auto rx_yaw_world = rx_valid ? xuc.imu_yaw() : NAN;
      const auto rx_pitch_world = rx_valid ? xuc.imu_pitch() : NAN;
      const bool loss_transition =
        (diagnostics_previous_armor && armors_after_color == 0) ||
        (diagnostics_previous_target && targets.empty()) ||
        (diagnostics_previous_control && !command.control);
      const bool observation_missing = armors_after_color == 0 || targets.empty() ||
        !std::isfinite(direct_yaw_measurement_rad);
      const auto & tracker_frame_diagnostics = tracker.frame_diagnostics();
      const bool tracker_diagnostic_event = tracker_frame_diagnostics.diverged ||
        tracker_frame_diagnostics.nis_reset;
      const bool periodic_record = diagnostic_now - diagnostics_last_record >=
        std::chrono::milliseconds(100);
      if (diagnostics_records < 3000 &&
          (periodic_record || loss_transition || tracker_diagnostic_event)) {
        std::string image_path;
        if (diagnostics_images_available && diagnostics_images < 20 &&
            loss_transition && observation_missing && !img.empty() &&
            diagnostic_now - diagnostics_last_image >= std::chrono::seconds(1)) {
          image_path = fmt::format(
            "{}/{}_{}_{}.jpg", diagnostics_image_dir.string(), diagnostics_run_id,
            camera_frame_count, frame_id);
          try {
            cv::Mat image_bgr;
            cv::cvtColor(img, image_bgr, cv::COLOR_RGB2BGR);
            if (cv::imwrite(image_path, image_bgr, {cv::IMWRITE_JPEG_QUALITY, 85})) {
              ++diagnostics_images;
              diagnostics_last_image = diagnostic_now;
            } else {
              tools::logger()->warn("[INFANTRY_DIAG] image write returned false: {}", image_path);
              image_path.clear();
              diagnostics_images_available = false;
            }
          } catch (const cv::Exception & e) {
            tools::logger()->warn("[INFANTRY_DIAG] image write failed: {}", e.what());
            image_path.clear();
            diagnostics_images_available = false;
          }
        }

        const auto finite_value = [](double value) -> nlohmann::json {
          return std::isfinite(value) ? nlohmann::json(value) : nlohmann::json(nullptr);
        };
        constexpr double RAD2DEG = 180.0 / M_PI;
        const auto tracker_state_json = [&](const Eigen::VectorXd & state) -> nlohmann::json {
          if (state.size() < 11) return nullptr;
          nlohmann::json value;
          value["center_x_m"] = finite_value(state[0]);
          value["center_vx_mps"] = finite_value(state[1]);
          value["center_y_m"] = finite_value(state[2]);
          value["center_vy_mps"] = finite_value(state[3]);
          value["center_z_m"] = finite_value(state[4]);
          value["center_vz_mps"] = finite_value(state[5]);
          value["body_yaw_deg"] = finite_value(state[6] * RAD2DEG);
          value["raw_omega_rad_s"] = finite_value(state[7]);
          value["r1_m"] = finite_value(state[8]);
          value["r2_delta_m"] = finite_value(state[9]);
          value["r2_m"] = finite_value(state[8] + state[9]);
          value["height_delta_m"] = finite_value(state[10]);
          return value;
        };
        const auto tracker_observation_json = [&](const auto_aim::Armor & armor) {
          nlohmann::json value;
          value["class_id"] = armor.class_id;
          value["name"] = static_cast<int>(armor.name);
          value["type"] = static_cast<int>(armor.type);
          value["color"] = static_cast<int>(armor.color);
          value["confidence"] = finite_value(armor.confidence);
          value["keypoint_confidence"] = finite_value(armor.keypoint_confidence);
          value["center_x_px"] = finite_value(armor.center.x);
          value["center_y_px"] = finite_value(armor.center.y);
          value["world_x_m"] = finite_value(armor.xyz_in_world.x());
          value["world_y_m"] = finite_value(armor.xyz_in_world.y());
          value["world_z_m"] = finite_value(armor.xyz_in_world.z());
          value["gimbal_x_m"] = finite_value(armor.xyz_in_gimbal.x());
          value["gimbal_y_m"] = finite_value(armor.xyz_in_gimbal.y());
          value["gimbal_z_m"] = finite_value(armor.xyz_in_gimbal.z());
          value["bearing_yaw_deg"] = finite_value(armor.ypd_in_world[0] * RAD2DEG);
          value["bearing_pitch_deg"] = finite_value(armor.ypd_in_world[1] * RAD2DEG);
          value["range_m"] = finite_value(armor.ypd_in_world[2]);
          value["pnp_yaw_deg"] = finite_value(armor.ypr_in_world[0] * RAD2DEG);
          value["pnp_pitch_deg"] = finite_value(armor.ypr_in_world[1] * RAD2DEG);
          value["pnp_roll_deg"] = finite_value(armor.ypr_in_world[2] * RAD2DEG);
          return value;
        };
        nlohmann::json trace;
        trace["run"] = diagnostics_run_id;
        trace["enemy_color"] = diagnostic_enemy_color_name;
        trace["frame_seq"] = camera_frame_count;
        trace["frame_id"] = frame_id;
        trace["frame_ns"] = frame_ns;
        trace["frame_imu_sync"] = xuc_imu_sync_valid;
        trace["frame_imu_query_stage"] = xuc.enabled()
          ? nlohmann::json("after_detection") : nlohmann::json(nullptr);
        trace["frame_imu_query_kind"] = !xuc.enabled() ? "disabled"
          : !xuc_imu_sync_valid ? "invalid"
          : frame_imu.interpolated ? "interpolated"
          : frame_imu.extrapolated ? "extrapolated" : "nearest";
        const bool frame_imu_query_attempted =
          frame_imu_query_time != std::chrono::steady_clock::time_point{};
        trace["frame_imu_query_ns"] = frame_imu_query_attempted
          ? nlohmann::json(std::chrono::duration_cast<std::chrono::nanoseconds>(
              frame_imu_query_time.time_since_epoch()).count()) : nlohmann::json(nullptr);
        trace["frame_imu_requested_ns"] = frame_imu_query_attempted
          ? nlohmann::json(std::chrono::duration_cast<std::chrono::nanoseconds>(
              frame_imu_requested_time.time_since_epoch()).count()) : nlohmann::json(nullptr);
        trace["frame_imu_pose_ns"] = xuc_imu_sync_valid
          ? nlohmann::json(std::chrono::duration_cast<std::chrono::nanoseconds>(
              frame_imu.timestamp.time_since_epoch()).count()) : nlohmann::json(nullptr);
        trace["frame_imu_query_after_frame_ms"] = frame_imu_query_attempted
          ? finite_value(std::chrono::duration<double, std::milli>(
              frame_imu_query_time - t).count()) : nlohmann::json(nullptr);
        trace["frame_imu_pose_offset_ms"] = xuc_imu_sync_valid
          ? finite_value(std::chrono::duration<double, std::milli>(
              frame_imu.timestamp - frame_imu_requested_time).count())
          : nlohmann::json(nullptr);
        trace["frame_imu_interpolated"] =
          xuc_imu_sync_valid && frame_imu.interpolated;
        trace["frame_imu_extrapolated"] =
          xuc_imu_sync_valid && frame_imu.extrapolated;
        trace["frame_imu_alignment_ms"] = xuc_imu_alignment_ms;
        trace["frame_imu_bracket_ms"] = xuc_imu_sync_valid
          ? finite_value(frame_imu.bracket_ms) : nlohmann::json(nullptr);
        trace["frame_imu_nearest_offset_ms"] = xuc_imu_sync_valid
          ? finite_value(frame_imu.nearest_offset_ms) : nlohmann::json(nullptr);
        trace["frame_imu_yaw_deg"] = xuc_imu_sync_valid
          ? finite_value(frame_imu.yaw * RAD2DEG) : nlohmann::json(nullptr);
        trace["frame_imu_pitch_deg"] = xuc_imu_sync_valid
          ? finite_value(frame_imu.pitch * RAD2DEG) : nlohmann::json(nullptr);
        trace["record_ns"] = diagnostic_ns;
        trace["image"] = image_path.empty() ? nlohmann::json(nullptr) : nlohmann::json(image_path);
        trace["loss_transition"] = loss_transition;
        trace["raw_rows"] = detection_diagnostics.raw_rows;
        trace["image_empty"] = img.empty();
        trace["detector_available"] = detection_diagnostics.available;
        trace["raw_max_conf"] = detection_diagnostics.max_confidence;
        trace["raw_max_class"] = detection_diagnostics.max_class;
        trace["candidates"] = detection_diagnostics.candidates;
        trace["after_nms"] = detection_diagnostics.after_nms;
        trace["accepted"] = detection_diagnostics.accepted;
        trace["reject_not_armor"] = detection_diagnostics.rejected_not_armor;
        trace["reject_confidence"] = detection_diagnostics.rejected_confidence;
        trace["reject_type"] = detection_diagnostics.rejected_type;
        trace["color_direct"] = detection_diagnostics.color_direct;
        trace["color_memory"] = detection_diagnostics.color_memory;
        trace["color_unconfirmed"] = detection_diagnostics.color_unconfirmed;
        trace["armors_before_color"] = armors_before_color;
        trace["armors_enemy_color"] = armors_enemy_color;
        trace["armors"] = armors_after_color;
        trace["targets"] = targets.size();
        trace["tracker_state"] = tracker.state();
        trace["full_frame_search"] = full_frame_search_used;
        trace["temp_lost_count"] = tracker.state() == "temp_lost" ? tracker.temp_lost_count() : 0;
        trace["associations"] = tracker.last_associations().size();
        trace["primary_detection"] = tracker.primary_detection_index();
        trace["primary_face"] = tracker.primary_face_id();
        trace["tracker_converged"] = tracker_converged;
        trace["tracker_target_state_available"] =
          tracker_frame_diagnostics.target_state_available;
        trace["tracker_initialized_this_frame"] =
          tracker_frame_diagnostics.initialized_this_frame;
        trace["tracker_update_attempted"] = tracker_frame_diagnostics.update_attempted;
        trace["tracker_update_used_batch"] = tracker_frame_diagnostics.update_used_batch;
        trace["tracker_update_observations"] = tracker_frame_diagnostics.update_observations;
        trace["tracker_identity_fallback_used"] =
          tracker_frame_diagnostics.identity_fallback_used;
        trace["tracker_identity_fallback_used_type_corrected"] =
          tracker_frame_diagnostics.identity_fallback_used_type_corrected;
        trace["tracker_identity_fallback_best_cost"] =
          finite_value(tracker_frame_diagnostics.identity_fallback_best_cost);
        trace["tracker_identity_fallback_observed_name"] =
          std::isfinite(tracker_frame_diagnostics.identity_fallback_best_cost)
          ? nlohmann::json(static_cast<int>(
              tracker_frame_diagnostics.identity_fallback_observed_name))
          : nlohmann::json(nullptr);
        trace["tracker_identity_fallback_observed_type"] =
          std::isfinite(tracker_frame_diagnostics.identity_fallback_best_cost)
          ? nlohmann::json(static_cast<int>(
              tracker_frame_diagnostics.identity_fallback_observed_type))
          : nlohmann::json(nullptr);
        trace["tracker_identity_fallback_best_type_corrected"] =
          tracker_frame_diagnostics.identity_fallback_best_type_corrected;
        trace["tracker_association_input_detections"] =
          tracker_frame_diagnostics.association_input_detections;
        trace["tracker_association_same_identity_detections"] =
          tracker_frame_diagnostics.association_same_identity_detections;
        trace["tracker_association_type_compatible_detections"] =
          tracker_frame_diagnostics.association_type_compatible_detections;
        trace["tracker_association_projected_pairs"] =
          tracker_frame_diagnostics.association_projected_pairs;
        trace["tracker_association_nonfinite_pairs"] =
          tracker_frame_diagnostics.association_nonfinite_pairs;
        trace["tracker_association_over_cost_pairs"] =
          tracker_frame_diagnostics.association_over_cost_pairs;
        trace["tracker_association_best_detection"] =
          tracker_frame_diagnostics.association_best_detection >= 0
          ? nlohmann::json(tracker_frame_diagnostics.association_best_detection)
          : nlohmann::json(nullptr);
        trace["tracker_association_best_face"] =
          tracker_frame_diagnostics.association_best_face >= 0
          ? nlohmann::json(tracker_frame_diagnostics.association_best_face)
          : nlohmann::json(nullptr);
        trace["tracker_association_best_observed_name"] =
          std::isfinite(tracker_frame_diagnostics.association_best_cost)
          ? nlohmann::json(static_cast<int>(
              tracker_frame_diagnostics.association_best_observed_name))
          : nlohmann::json(nullptr);
        trace["tracker_association_best_cost"] =
          finite_value(tracker_frame_diagnostics.association_best_cost);
        trace["tracker_association_best_center_error"] =
          finite_value(tracker_frame_diagnostics.association_best_center_error);
        trace["tracker_association_best_angle_error"] =
          finite_value(tracker_frame_diagnostics.association_best_angle_error);
        trace["tracker_association_best_range_error"] =
          finite_value(tracker_frame_diagnostics.association_best_range_error);
        trace["tracker_association_best_confidence_bonus"] =
          finite_value(tracker_frame_diagnostics.association_best_confidence_bonus);
        trace["tracker_association_cost_threshold"] =
          finite_value(tracker_frame_diagnostics.association_cost_threshold);
        trace["tracker_association_best_within_threshold"] =
          tracker_frame_diagnostics.association_best_within_threshold;
        trace["tracker_association_reject_reason"] =
          tracker_frame_diagnostics.association_reject_reason;
        trace["tracker_target_name"] = tracker_frame_diagnostics.target_state_available
          ? nlohmann::json(static_cast<int>(tracker_frame_diagnostics.target_name))
          : nlohmann::json(nullptr);
        trace["tracker_target_armor_count"] = tracker_frame_diagnostics.target_state_available
          ? nlohmann::json(tracker_frame_diagnostics.target_armor_count)
          : nlohmann::json(nullptr);
        trace["tracker_update_diverged"] = tracker_frame_diagnostics.diverged;
        trace["tracker_nis_reset"] = tracker_frame_diagnostics.nis_reset;
        trace["tracker_state_prior"] =
          tracker_state_json(tracker_frame_diagnostics.state_before);
        trace["tracker_state_post"] =
          tracker_state_json(tracker_frame_diagnostics.state_after);
        trace["tracker_whole_nis"] = finite_value(tracker_frame_diagnostics.whole_nis);
        trace["tracker_whole_nees"] = finite_value(tracker_frame_diagnostics.whole_nees);
        trace["tracker_yaw_eskf_trusted"] = tracker_frame_diagnostics.yaw_eskf_trusted;
        trace["tracker_yaw_eskf_angle_deg"] =
          finite_value(tracker_frame_diagnostics.yaw_eskf_angle * RAD2DEG);
        trace["tracker_yaw_eskf_rate_rad_s"] =
          finite_value(tracker_frame_diagnostics.yaw_eskf_rate);
        trace["tracker_yaw_eskf_acceleration_rad_s2"] =
          finite_value(tracker_frame_diagnostics.yaw_eskf_acceleration);
        trace["tracker_yaw_eskf_nis"] =
          finite_value(tracker_frame_diagnostics.yaw_eskf_nis);
        trace["tracker_yaw_eskf_accepted"] = tracker_frame_diagnostics.yaw_eskf_accepted;
        trace["tracker_yaw_eskf_rejected"] = tracker_frame_diagnostics.yaw_eskf_rejected;
        trace["tracker_center_acceleration_variance"] =
          finite_value(tracker_frame_diagnostics.center_acceleration_variance);
        nlohmann::json association_details = nlohmann::json::array();
        for (const auto & association : tracker.last_associations()) {
          nlohmann::json detail;
          detail["detection"] = association.detection_index;
          detail["face"] = association.face_id;
          detail["cost"] = finite_value(association.cost);
          detail["identity_fallback"] = association.identity_fallback;
          detail["observed_name"] = static_cast<int>(association.observed_name);
          detail["observed_type"] = static_cast<int>(association.observed_type);
          detail["identity_fallback_type_corrected"] =
            association.identity_fallback_type_corrected;
          detail["used_for_update"] = association.used_for_update;
          detail["observation"] = association.observation &&
              valid_measurement_pose(*association.observation)
            ? tracker_observation_json(*association.observation) : nlohmann::json(nullptr);
          association_details.push_back(std::move(detail));
        }
        trace["tracker_association_details"] = std::move(association_details);
        const auto * primary_observation = tracker.primary_observation();
        trace["tracker_primary_observation"] = primary_observation != nullptr &&
            valid_measurement_pose(*primary_observation)
          ? tracker_observation_json(*primary_observation) : nlohmann::json(nullptr);
        trace["direct_measurement_source"] = direct_measurement_from_tracker
          ? "tracker" : (direct_measurement_armor != nullptr ? "measurement_fallback" : "unknown");
        trace["direct_measurement_detection"] = direct_measurement_detection_index;
        trace["direct_measurement_observation"] = direct_measurement_armor != nullptr
          ? tracker_observation_json(*direct_measurement_armor) : nlohmann::json(nullptr);
        trace["range_raw_m"] = finite_value(raw_pnp_range_m);
        trace["range_model_m"] = finite_value(modeled_armor_range_m);
        trace["range_filtered_m"] = finite_value(robust_target_range_m);
        trace["measurement_fresh"] = direct_yaw_measurement_fresh;
        trace["gyro_center_follow"] = gyro_center_follow_active;
        trace["measurement_fallback"] = measurement_fallback;
        trace["yaw_measurement_deg"] = finite_value(direct_yaw_measurement_rad * RAD2DEG);
        trace["yaw_filtered_deg"] = direct_yaw_initialized
          ? finite_value(direct_yaw_filtered * RAD2DEG) : nlohmann::json(nullptr);
        trace["yaw_native_deg"] = finite_value(native_predicted_yaw * RAD2DEG);
        trace["yaw_pre_rate_deg"] = finite_value(yaw_before_final_rate_rad * RAD2DEG);
        trace["yaw_post_rate_deg"] = finite_value(yaw_after_final_rate_rad * RAD2DEG);
        trace["yaw_final_deg"] = finite_value(command.yaw * RAD2DEG);
        trace["pitch_command_deg"] = finite_value(command.pitch * RAD2DEG);
        trace["pitch_raw_pixel_error_deg"] =
          finite_value(direct_pitch_raw_pixel_error_rad * RAD2DEG);
        trace["pitch_ballistic_comp_deg"] =
          finite_value(ballistic_pitch_compensation_rad * RAD2DEG);
        trace["pitch_residual_deg"] =
          finite_value(direct_pitch_pixel_error_rad * RAD2DEG);
        trace["pitch_gate_open"] = direct_pitch_measurement_fresh &&
          std::abs(direct_pitch_pixel_error_rad) <= xuc_fire_pitch_tolerance_rad;
        trace["fire_first_blocker"] = fire_gate_snapshot_valid && fire_first_blocker >= 0
          ? nlohmann::json(fire_gate_names[static_cast<std::size_t>(fire_first_blocker)])
          : nlohmann::json(nullptr);
        if (fire_gate_snapshot_valid) {
          nlohmann::json gates;
          for (std::size_t gate = 0; gate < fire_gate_names.size(); ++gate) {
            gates[fire_gate_names[gate]] = fire_gate_open[gate];
          }
          trace["fire_gates"] = std::move(gates);
        }
        trace["control"] = command.control;
        trace["lower_auto_ready"] = lower_auto_ready_now;
        trace["rate_limiter_armed"] = final_yaw_lower_armed;
        trace["mode"] = xuc.enabled() ? static_cast<int>(xuc.mode()) : -1;
        trace["robot_id"] = xuc.enabled() ? static_cast<int>(xuc.robot_id()) : -1;
        trace["xuc_feedback_aux_raw"] = xuc.enabled()
          ? static_cast<int>(xuc.robot_id()) : -1;
        trace["xuc_feedback_count_raw"] = xuc.enabled()
          ? static_cast<int>(xuc.bullet_count()) : -1;
        trace["rx_valid"] = rx_valid;
        trace["rx_ns"] = rx_ns;
        trace["rx_age_ms"] = rx_ns > 0
          ? finite_value(static_cast<double>(diagnostic_ns - rx_ns) / 1.0e6)
          : nlohmann::json(nullptr);
        trace["rx_raw_deg"] = rx_valid
          ? finite_value(rx_yaw_raw * RAD2DEG) : nlohmann::json(nullptr);
        trace["rx_world_deg"] = rx_valid
          ? finite_value(rx_yaw_world * RAD2DEG) : nlohmann::json(nullptr);
        trace["rx_pitch_deg"] = rx_valid
          ? finite_value(rx_pitch_world * RAD2DEG) : nlohmann::json(nullptr);
        trace["tx_ok"] = tx_ok;
        trace["tx_ns"] = tx_ns;
        trace["tx_write_complete_ns"] = tx_ok
          ? xuc.last_tx_complete_ns() : 0;
        trace["tx_control"] = tx_control;
        trace["tx_shoot"] = tx_ok
          ? nlohmann::json(static_cast<int>(xuc.last_tx_shoot()))
          : nlohmann::json(nullptr);
        trace["shoot_hold_active"] = tx_ok && xuc.last_tx_shoot() == 1;
        trace["shoot_hold_continuation"] = tx_ok && xuc.last_tx_shoot() == 1 &&
          !command.shoot;
        trace["feeder_feedback_raw"] = xuc.enabled()
          ? finite_value(xuc.feeder_speed_rpm()) : nlohmann::json(nullptr);
        trace["feeder_feedback_units"] = "motor_rotor_rpm";
        trace["feeder_motor_turns_est"] = xuc.enabled()
          ? finite_value(xuc.feeder_motor_turns_estimate()) : nlohmann::json(nullptr);
        trace["pending_shot_id"] = xuc_pending_shot
          ? nlohmann::json(xuc_pending_shot_id) : nlohmann::json(nullptr);
        const double feeder_turns_since_request_est = xuc_pending_shot
          ? xuc.feeder_motor_turns_estimate() - xuc_pending_feeder_turns_at_request
          : NAN;
        trace["feeder_motor_turns_since_request_est"] =
          finite_value(feeder_turns_since_request_est);
        trace["one_slot_rotation_est"] = xuc_pending_shot &&
          feeder_turns_since_request_est >= 4.5;
        trace["feed_slot_observation"] = "unknown";
        trace["projectile_exit"] = "unknown";
        trace["tx_raw_deg"] = tx_ok
          ? finite_value(tx_yaw_raw * RAD2DEG) : nlohmann::json(nullptr);
        trace["tx_pitch_raw_deg"] = tx_ok
          ? finite_value(tx_pitch_raw * RAD2DEG) : nlohmann::json(nullptr);
        // Shadow comparisons only: keep the control pose, solver and observation unchanged.
        // This block shares the bounded INFANTRY_DIAG sampling and is never run in compact mode.
        if (tools::logger()->should_log(spdlog::level::debug)) {
          const auto sync_begin = std::chrono::steady_clock::now();
          const auto time_ns = [](std::chrono::steady_clock::time_point time) {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
              time.time_since_epoch()).count();
          };
          const bool observation_valid = direct_measurement_armor != nullptr &&
            valid_measurement_pose(*direct_measurement_armor);
          nlohmann::json sync;
          sync["run"] = diagnostics_run_id;
          sync["frame_seq"] = camera_frame_count;
          sync["frame_id"] = frame_id;
          sync["frame_ns"] = frame_ns;
          sync["timestamp_basis"] = "host_receive";
          sync["control_alignment_ms"] = xuc_imu_alignment_ms;
          sync["control_imu_valid"] = xuc_imu_sync_valid;
          sync["control_pose_ns"] = xuc_imu_sync_valid
            ? nlohmann::json(time_ns(frame_imu.timestamp)) : nlohmann::json(nullptr);
          sync["observation_valid"] = observation_valid;
          sync["observation_source"] = direct_measurement_from_tracker
            ? "tracker" : (observation_valid ? "measurement_fallback" : "unknown");
          sync["observation"] = observation_valid
            ? tracker_observation_json(*direct_measurement_armor) : nlohmann::json(nullptr);
          sync["candidates"] = nlohmann::json::array();
          const Eigen::Matrix3d gimbal_to_imubody = solver.R_gimbal2imubody();
          for (const int offset_ms : std::array<int, 5>{0, 5, 10, 15, 20}) {
            const auto requested = t + std::chrono::milliseconds(offset_ms);
            const auto query_time = std::chrono::steady_clock::now();
            io::XucTimedImu candidate_imu;
            const bool pose_valid = xuc.enabled() && xuc.imu_at(requested, candidate_imu);
            nlohmann::json candidate;
            candidate["offset_ms"] = offset_ms;
            candidate["requested_ns"] = time_ns(requested);
            candidate["query_ns"] = time_ns(query_time);
            candidate["pose_valid"] = pose_valid;
            candidate["query_kind"] = !pose_valid ? "invalid" :
              (candidate_imu.interpolated ? "interpolated" :
               (candidate_imu.extrapolated ? "extrapolated" : "nearest"));
            candidate["pose_ns"] = pose_valid
              ? nlohmann::json(time_ns(candidate_imu.timestamp)) : nlohmann::json(nullptr);
            candidate["pose_offset_ms"] = pose_valid
              ? finite_value(std::chrono::duration<double, std::milli>(
                  candidate_imu.timestamp - requested).count()) : nlohmann::json(nullptr);
            candidate["bracket_ms"] = pose_valid
              ? finite_value(candidate_imu.bracket_ms) : nlohmann::json(nullptr);
            candidate["nearest_offset_ms"] = pose_valid
              ? finite_value(candidate_imu.nearest_offset_ms) : nlohmann::json(nullptr);
            candidate["imu_yaw_deg"] = pose_valid
              ? finite_value(candidate_imu.yaw * RAD2DEG) : nlohmann::json(nullptr);
            candidate["imu_pitch_deg"] = pose_valid
              ? finite_value(candidate_imu.pitch * RAD2DEG) : nlohmann::json(nullptr);
            candidate["world_xyz_m"] = nullptr;
            candidate["bearing_yaw_deg"] = nullptr;
            candidate["bearing_pitch_deg"] = nullptr;
            candidate["range_m"] = nullptr;
            candidate["delta_control_yaw_deg"] = nullptr;
            candidate["delta_control_pitch_deg"] = nullptr;
            candidate["world_valid"] = false;
            if (pose_valid && observation_valid) {
              Eigen::Quaterniond candidate_q =
                Eigen::AngleAxisd(candidate_imu.yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(candidate_imu.pitch, Eigen::Vector3d::UnitY());
              candidate_q.normalize();
              const Eigen::Vector3d world_xyz = gimbal_to_imubody.transpose() *
                candidate_q.toRotationMatrix() * gimbal_to_imubody *
                direct_measurement_armor->xyz_in_gimbal;
              const Eigen::Vector3d world_ypd = tools::xyz2ypd(world_xyz);
              if (world_xyz.allFinite() && world_ypd.allFinite()) {
                candidate["world_valid"] = true;
                candidate["world_xyz_m"] = {world_xyz.x(), world_xyz.y(), world_xyz.z()};
                candidate["bearing_yaw_deg"] = world_ypd[0] * RAD2DEG;
                candidate["bearing_pitch_deg"] = world_ypd[1] * RAD2DEG;
                candidate["range_m"] = world_ypd[2];
                candidate["delta_control_yaw_deg"] = finite_value(std::remainder(
                  world_ypd[0] - direct_measurement_armor->ypd_in_world[0], 2.0 * M_PI) * RAD2DEG);
                candidate["delta_control_pitch_deg"] = finite_value(
                  (world_ypd[1] - direct_measurement_armor->ypd_in_world[1]) * RAD2DEG);
              }
            }
            sync["candidates"].push_back(std::move(candidate));
          }
          // Preserve repeated values and actual packet parse times; never synthesize samples.
          const auto raw_window_begin = t - std::chrono::milliseconds(30);
          const auto raw_window_end = t + std::chrono::milliseconds(30);
          const auto raw_query_time = std::chrono::steady_clock::now();
          const auto raw_window = xuc.imu_history_window(raw_window_begin, raw_window_end);
          sync["raw_imu_query_ns"] = time_ns(raw_query_time);
          sync["raw_imu_window_begin_ns"] = time_ns(raw_window_begin);
          sync["raw_imu_window_end_ns"] = time_ns(raw_window_end);
          sync["raw_imu_total_samples"] = raw_window.total_samples;
          sync["raw_imu_truncated"] = raw_window.truncated;
          sync["raw_imu_timestamp_basis"] = "host_packet_parse";
          sync["raw_imu_sensor_sample_ns"] = nullptr;  // Not supplied by the current protocol.
          sync["raw_imu_samples"] = nlohmann::json::array();
          for (const auto & sample : raw_window.samples) {
            nlohmann::json raw;
            raw["sample_ns"] = time_ns(sample.timestamp);
            raw["offset_ms"] = finite_value(std::chrono::duration<double, std::milli>(
              sample.timestamp - t).count());
            raw["imu_yaw_deg"] = finite_value(sample.yaw * RAD2DEG);
            raw["imu_pitch_deg"] = finite_value(sample.pitch * RAD2DEG);
            sync["raw_imu_samples"].push_back(std::move(raw));
          }
          sync["compute_us"] = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - sync_begin).count();
          tools::logger()->debug("[P2_SYNC] {}", sync.dump());
        }
        tools::logger()->info("[INFANTRY_DIAG] {}", trace.dump());
        diagnostics_last_record = diagnostic_now;
        ++diagnostics_records;
      }
      diagnostics_previous_armor = armors_after_color != 0;
      diagnostics_previous_target = !targets.empty();
      diagnostics_previous_control = command.control;
    }

    if (!competition_headless) {
      // 绘制识别与预测结果
      if (!targets.empty() && tracker_converged && yaw_motion_model_trusted) {
        auto target = targets.front();

        // 绘制跟踪到的所有装甲板位置（绿色）
        std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(img, image_points, {0, 255, 0});
        }

        // 绘制预测的瞄准位置（红色）
        if (aimer.debug_aim_point.valid) {
          Eigen::Vector4d aim_xyza = aimer.debug_aim_point.xyza;
          auto image_points =
            solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
          tools::draw_points(img, image_points, {0, 0, 255});
        }
      }

      cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
      // 相机输出为 RGB 格式，imshow 需要 BGR 格式
      cv::Mat img_bgr;
      cv::cvtColor(img, img_bgr, cv::COLOR_RGB2BGR);
      cv::imshow("reprojection", img_bgr);
      auto key = cv::waitKey(1);
      if (key == 'q') break;
    }

    // 🔍 性能监控：打印各环节耗时
    auto loop_end = std::chrono::steady_clock::now();
    static int perf_counter = 0;
    if (++perf_counter % 50 == 0) {
      auto detect_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
      auto track_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
      auto total_ms = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
      tools::logger()->info("[PERF] 总耗时={:.1f}ms (检测={:.1f}ms, 跟踪={:.1f}ms), FPS={:.1f}",
        total_ms, detect_ms, track_ms, 1000.0 / total_ms);
    }
  }

  // 清理 ROS2
#ifdef AMENT_CMAKE_FOUND
  visualizer.reset();
  rclcpp::shutdown();
  tools::logger()->info("[ROS2] Shutdown complete");
#endif

  return 0;
}
