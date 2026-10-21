#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <list>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "gyro_face_selector.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

enum class AimGeometry { existing, whole_ekf };
// Explicit timing in standard; automatic preserves other callers' legacy API.
enum class AimTiming { automatic, stationary, low_speed, high_speed };

// Optional observer of the existing iterations. Never used for control.
struct AimPredictionTrace
{
  bool valid = false, converged = false;
  bool whole_ekf_geometry = false;
  int face_id = -1, iterations = 0;
  int64_t base_prediction_ns = 0, prediction_ns = 0;
  double delay_s = std::numeric_limits<double>::quiet_NaN();
  double speed_mps = std::numeric_limits<double>::quiet_NaN();
  double flight_time_used_s = std::numeric_limits<double>::quiet_NaN();
  double flight_time_s = std::numeric_limits<double>::quiet_NaN();
};

// Runtime geometry for one PLL event. No Aimer delay or ballistic iteration is
// added: the scheduler has already chosen the absolute impact time.
struct EventAimPrediction
{
  bool valid = false, face_changed = false;
  int face_id = -1;
  int64_t prediction_ns = 0;
  Eigen::Vector4d xyza = Eigen::Vector4d::Constant(
    std::numeric_limits<double>::quiet_NaN());
  double yaw_rad = std::numeric_limits<double>::quiet_NaN();
  double alignment_rad = std::numeric_limits<double>::infinity();

  double yaw_in_command_frame(double command_correction_rad) const
  {
    return valid && std::isfinite(command_correction_rad)
      ? std::remainder(yaw_rad + command_correction_rad, 2.0 * std::acos(-1.0))
      : std::numeric_limits<double>::quiet_NaN();
  }
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true, AimPredictionTrace * trace = nullptr,
    AimGeometry geometry = AimGeometry::existing, AimTiming timing = AimTiming::automatic);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true, AimPredictionTrace * trace = nullptr,
    AimGeometry geometry = AimGeometry::existing, AimTiming timing = AimTiming::automatic);

  // Explicit profiles own separate bounded corrections. The default preserves
  // the legacy automatic caller's correction; it cannot leak into explicit modes.
  void set_delay_adjustment(double adjustment_s, AimTiming timing = AimTiming::automatic);

  EventAimPrediction predict_event(
    const Target & target, std::chrono::steady_clock::time_point impact_time,
    GyroFaceSelector & selector, int64_t hit_cycle) const;

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double stationary_delay_time_;
  std::array<double, 4> delay_adjustment_times_{};
  double decision_speed_;

  AimPoint choose_aim_point(
    const Target & target, bool whole_ekf_geometry, int * face_id = nullptr);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
