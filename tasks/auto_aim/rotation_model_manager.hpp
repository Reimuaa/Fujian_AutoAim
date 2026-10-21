#ifndef AUTO_AIM__ROTATION_MODEL_MANAGER_HPP
#define AUTO_AIM__ROTATION_MODEL_MANAGER_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace auto_aim
{

// Motion classification is independent of the optional phase-clock predictor.
// Disabling PLL must not collapse low/high speed into the same motion mode.
// It deliberately does not estimate pose: its job is to decide which existing
// estimator may drive the gimbal/fire policy, and when a phase clock is stale.
class RotationModelManager
{
public:
  enum class Mode { armor = 0, vehicle = 1, high_speed_phase = 2 };
  enum class MotionMode { stationary = 0, low_speed = 1, high_speed = 2 };

  struct Config
  {
    double vehicle_enter_rad_s = 0.8;
    double vehicle_exit_rad_s = 0.5;
    bool high_speed_phase_enabled = true;
    double high_speed_enter_rad_s = 3.2;
    double high_speed_exit_rad_s = 2.5;
    double mode_dwell_s = 0.15;
    double tracker_dropout_hold_s = 0.0;
    double rate_filter_tau_s = 0.0;
    double max_rate_disagreement_rad_s = 1.2;
    double speed_step_reset_rad_s = 1.2;
    int rate_agreement_updates = 2;
    int rate_disagreement_updates = 3;
  };

  struct Result
  {
    Mode mode = Mode::armor;
    MotionMode motion_mode = MotionMode::stationary;
    bool mode_changed = false;
    bool motion_mode_changed = false;
    bool phase_reset = false;
    bool tracker_trusted = false;
    bool rate_consistent = false;
    bool phase_fire_trusted = false;
    double rate_disagreement_rad_s = std::numeric_limits<double>::infinity();
    double filtered_abs_rate_rad_s = std::numeric_limits<double>::quiet_NaN();
  };

  RotationModelManager() = default;
  explicit RotationModelManager(const Config & config) : config_(config)
  {
    if (!std::isfinite(config.vehicle_enter_rad_s) || !std::isfinite(config.vehicle_exit_rad_s) ||
        !std::isfinite(config.high_speed_enter_rad_s) || !std::isfinite(config.high_speed_exit_rad_s) ||
        config.vehicle_exit_rad_s < 0.0 || config.vehicle_exit_rad_s >= config.vehicle_enter_rad_s ||
        config.vehicle_enter_rad_s >= config.high_speed_exit_rad_s ||
        config.high_speed_exit_rad_s >= config.high_speed_enter_rad_s ||
        !std::isfinite(config.mode_dwell_s) || config.mode_dwell_s < 0.0 ||
        !std::isfinite(config.rate_filter_tau_s) || config.rate_filter_tau_s < 0.0 ||
        !std::isfinite(config.tracker_dropout_hold_s) || config.tracker_dropout_hold_s < 0.0)
      throw std::invalid_argument("Invalid stationary/low/high rotation mode thresholds");
  }

  void reset()
  {
    mode_ = Mode::armor;
    motion_mode_ = MotionMode::stationary;
    candidate_mode_ = MotionMode::stationary;
    candidate_since_s_ = 0.0;
    have_candidate_ = false;
    have_moving_candidate_ = false;
    moving_candidate_since_s_ = 0.0;
    have_last_rate_ = false;
    last_abs_ekf_rate_ = 0.0;
    have_filtered_rate_ = false;
    filtered_abs_rate_ = 0.0;
    last_update_s_ = 0.0;
    agreement_updates_ = 0;
    disagreement_updates_ = 0;
    have_last_trusted_update_ = false;
    last_trusted_update_s_ = 0.0;
    have_input_time_ = false;
    last_input_s_ = 0.0;
  }

  Result update(
    double now_s, bool tracker_converged, double ekf_omega_rad_s,
    bool phase_rate_valid = false,
    double phase_omega_rad_s = std::numeric_limits<double>::quiet_NaN())
  {
    Result out;
    if (!std::isfinite(now_s) || (have_input_time_ && now_s <= last_input_s_)) {
      out.mode_changed = mode_ != Mode::armor;
      out.motion_mode_changed = motion_mode_ != MotionMode::stationary;
      out.phase_reset = mode_ == Mode::high_speed_phase;
      reset();
      return out;
    }
    last_input_s_ = now_s;
    have_input_time_ = true;
    out.tracker_trusted = tracker_converged && std::isfinite(now_s) &&
      std::isfinite(ekf_omega_rad_s);
    if (!out.tracker_trusted) {
      // Unobserved time never completes a pending classification dwell.
      have_candidate_ = false;
      have_moving_candidate_ = false;
      // A detector association hand-over can make convergence false for only a
      // few frames.  Keep the selected motion regime through that gap (fire is
      // still inhibited by tracker_trusted=false) instead of immediately
      // collapsing high-speed -> armor and rebuilding the model on every face.
      if (have_last_trusted_update_ && std::isfinite(now_s) &&
          now_s - last_trusted_update_s_ <=
            std::max(0.0, config_.tracker_dropout_hold_s)) {
        out.mode = mode_;
        out.motion_mode = motion_mode_;
        out.filtered_abs_rate_rad_s = filtered_abs_rate_;
        return out;
      }
      out.mode_changed = mode_ != Mode::armor;
      out.motion_mode_changed = motion_mode_ != MotionMode::stationary;
      out.phase_reset = mode_ == Mode::high_speed_phase;
      reset();
      out.mode = mode_;
      return out;
    }
    have_last_trusted_update_ = true;
    last_trusted_update_s_ = now_s;

    const double raw_abs_rate = std::abs(ekf_omega_rad_s);
    if (!have_filtered_rate_) {
      filtered_abs_rate_ = raw_abs_rate;
      have_filtered_rate_ = true;
    } else {
      const double dt = std::clamp(now_s - last_update_s_, 0.0, 0.25);
      const double tau = std::max(0.0, config_.rate_filter_tau_s);
      const double alpha = tau > 0.0 ? 1.0 - std::exp(-dt / tau) : 1.0;
      filtered_abs_rate_ += alpha * (raw_abs_rate - filtered_abs_rate_);
    }
    last_update_s_ = now_s;
    const double abs_rate = filtered_abs_rate_;
    out.filtered_abs_rate_rad_s = abs_rate;
    const MotionMode desired = desired_motion_mode(abs_rate);
    if (motion_mode_ == MotionMode::stationary && abs_rate >= config_.vehicle_enter_rad_s) {
      if (!have_moving_candidate_) {
        have_moving_candidate_ = true;
        moving_candidate_since_s_ = now_s;
      }
    } else {
      have_moving_candidate_ = false;
    }
    if (!have_candidate_ || desired != candidate_mode_) {
      candidate_mode_ = desired;
      candidate_since_s_ = now_s;
      have_candidate_ = true;
    }

    MotionMode selected = desired;
    bool dwell_complete = now_s - candidate_since_s_ >= config_.mode_dwell_s;
    // At startup, noise around the low/high boundary must not keep a clearly
    // rotating target in stationary forever. Enter the proven low-speed path
    // after continuous motion, while high speed still needs its own full dwell.
    if (!dwell_complete && motion_mode_ == MotionMode::stationary && have_moving_candidate_ &&
        now_s - moving_candidate_since_s_ >= config_.mode_dwell_s) {
      selected = MotionMode::low_speed;
      dwell_complete = true;
    }
    if (selected != motion_mode_ && dwell_complete) {
      const Mode old_mode = mode_;
      motion_mode_ = selected;
      out.motion_mode_changed = true;
      mode_ = motion_mode_ == MotionMode::stationary ? Mode::armor
        : (motion_mode_ == MotionMode::high_speed && config_.high_speed_phase_enabled
          ? Mode::high_speed_phase : Mode::vehicle);
      out.mode_changed = mode_ != old_mode;
      out.phase_reset = old_mode == Mode::high_speed_phase ||
        mode_ == Mode::high_speed_phase;
      agreement_updates_ = 0;
      disagreement_updates_ = 0;
    }

    if (mode_ == Mode::high_speed_phase && !phase_rate_valid && have_last_rate_ &&
        std::abs(abs_rate - last_abs_ekf_rate_) >=
          std::max(0.0, config_.speed_step_reset_rad_s)) {
      // With a mature physical crossing clock, one-frame ESKF speed steps are
      // usually armor hand-over artifacts. Preserve the clock and let its own
      // robust residual gate decide whether the motion actually changed.
      out.phase_reset = true;
      agreement_updates_ = 0;
      disagreement_updates_ = 0;
    }
    have_last_rate_ = true;
    last_abs_ekf_rate_ = abs_rate;

    if (mode_ == Mode::high_speed_phase && phase_rate_valid &&
        std::isfinite(phase_omega_rad_s)) {
      out.rate_disagreement_rad_s =
        std::abs(abs_rate - std::abs(phase_omega_rad_s));
      out.rate_consistent = out.rate_disagreement_rad_s <=
        std::max(0.0, config_.max_rate_disagreement_rad_s);
      if (out.rate_consistent) {
        ++agreement_updates_;
        disagreement_updates_ = 0;
      } else {
        agreement_updates_ = 0;
        ++disagreement_updates_;
        if (disagreement_updates_ >=
            std::max(1, config_.rate_disagreement_updates)) {
          out.phase_reset = true;
          disagreement_updates_ = 0;
        }
      }
    } else {
      agreement_updates_ = 0;
      disagreement_updates_ = 0;
    }

    out.mode = mode_;
    out.motion_mode = motion_mode_;
    out.phase_fire_trusted = mode_ == Mode::high_speed_phase &&
      phase_rate_valid && out.rate_consistent &&
      agreement_updates_ >= std::max(1, config_.rate_agreement_updates) &&
      !out.phase_reset;
    return out;
  }

  Mode mode() const { return mode_; }
  MotionMode motion_mode() const { return motion_mode_; }

  static const char * name(MotionMode mode)
  {
    switch (mode) {
      case MotionMode::stationary: return "stationary";
      case MotionMode::low_speed: return "low_speed";
      case MotionMode::high_speed: return "high_speed";
    }
    return "unknown";
  }

  static const char * name(Mode mode)
  {
    switch (mode) {
      case Mode::armor: return "armor";
      case Mode::vehicle: return "vehicle";
      case Mode::high_speed_phase: return "high_speed_phase";
    }
    return "unknown";
  }

private:
  MotionMode desired_motion_mode(double abs_rate) const
  {
    if (motion_mode_ == MotionMode::high_speed && abs_rate >= config_.high_speed_exit_rad_s)
      return MotionMode::high_speed;
    if (abs_rate >= config_.high_speed_enter_rad_s) return MotionMode::high_speed;
    if (motion_mode_ != MotionMode::stationary && abs_rate >= config_.vehicle_exit_rad_s)
      return MotionMode::low_speed;
    return abs_rate >= config_.vehicle_enter_rad_s
      ? MotionMode::low_speed : MotionMode::stationary;
  }

  Config config_{};
  Mode mode_ = Mode::armor;
  MotionMode motion_mode_ = MotionMode::stationary;
  MotionMode candidate_mode_ = MotionMode::stationary;
  double candidate_since_s_ = 0.0;
  bool have_candidate_ = false;
  bool have_moving_candidate_ = false;
  double moving_candidate_since_s_ = 0.0;
  bool have_last_rate_ = false;
  double last_abs_ekf_rate_ = 0.0;
  bool have_filtered_rate_ = false;
  double filtered_abs_rate_ = 0.0;
  double last_update_s_ = 0.0;
  int agreement_updates_ = 0;
  int disagreement_updates_ = 0;
  bool have_last_trusted_update_ = false;
  double last_trusted_update_s_ = 0.0;
  bool have_input_time_ = false;
  double last_input_s_ = 0.0;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ROTATION_MODEL_MANAGER_HPP
