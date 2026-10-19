#ifndef AUTO_AIM__WORK_ZONE_TARGET_HPP
#define AUTO_AIM__WORK_ZONE_TARGET_HPP

#include <cmath>
#include <cstdint>

namespace auto_aim
{
// MCU owns scanning. This gate only decides whether vision currently owns the
// target; no scan angles, encoder conversion or return trigger are generated.
class WorkZoneTarget
{
public:
  enum class State { inactive, scanning, confirming, tracking, lost_hold };
  struct Config
  {
    int confirm_frames = 3;
    double loss_hold_s = 0.150;
  };
  struct Input
  {
    double now_s = 0.0, frame_s = 0.0;
    bool work_zone_ready = false, feedback_valid = false, observation_valid = false;
    double yaw = 0.0, pitch = 0.0;
    uint64_t target_key = 0;
  };
  struct Output
  {
    State state = State::inactive;
    bool control = false, inhibit_fire = true, hold_angles = false;
    double yaw = 0.0, pitch = 0.0, valid_until_s = 0.0;
  };

  WorkZoneTarget() = default;
  explicit WorkZoneTarget(const Config & config) : config_(config) {}
  State state() const { return state_; }
  bool needs_full_frame() const { return state_ != State::tracking; }
  void remember_final_command(double yaw, double pitch)
  {
    if (state_ == State::tracking && std::isfinite(yaw) && std::isfinite(pitch)) {
      yaw_ = yaw;
      pitch_ = pitch;
    }
  }

  Output update(const Input & in)
  {
    if (!in.work_zone_ready || !in.feedback_valid || !std::isfinite(in.now_s) ||
        config_.confirm_frames < 2 || config_.confirm_frames > 3 ||
        !std::isfinite(config_.loss_hold_s) || config_.loss_hold_s < 0.100 ||
        config_.loss_hold_s > 0.200 || (have_time_ && in.now_s <= last_update_s_)) {
      reset();
      return output(false, true, false);
    }
    last_update_s_ = in.now_s;
    have_time_ = true;
    const bool fresh = in.observation_valid && in.target_key != 0 &&
      std::isfinite(in.yaw) && std::isfinite(in.pitch) && std::isfinite(in.frame_s) &&
      in.frame_s <= in.now_s && in.now_s - in.frame_s < config_.loss_hold_s &&
      (!have_frame_ || in.frame_s > last_frame_s_);
    const bool same_target = in.target_key == target_key_;
    // An expired lock always requires confirmation again, even if the next
    // processed image happens to contain the old target.
    if (locked_ && in.now_s >= last_seen_s_ + config_.loss_hold_s) {
      locked_ = false;
      consecutive_ = 0;
    }
    if (fresh) {
      if (!same_target) { locked_ = false; consecutive_ = 0; }
      target_key_ = in.target_key;
      if (have_frame_ && in.frame_s - last_frame_s_ >= config_.loss_hold_s)
        consecutive_ = 0;
      last_frame_s_ = in.frame_s;
      have_frame_ = true;
      last_seen_s_ = in.frame_s;
      yaw_ = in.yaw;
      pitch_ = in.pitch;
      if (++consecutive_ >= config_.confirm_frames) {
        consecutive_ = config_.confirm_frames;
        locked_ = true;
      }
      state_ = locked_ ? State::tracking : State::confirming;
      return output(locked_, !locked_, false);
    }
    consecutive_ = 0;
    if (locked_) {
      state_ = State::lost_hold;
      return output(true, true, true); // Last valid command, never new extrapolated angles/fire.
    }
    state_ = State::scanning;
    return output(false, true, false);
  }

  static const char * name(State state)
  {
    switch (state) {
      case State::scanning: return "scanning";
      case State::confirming: return "confirming";
      case State::tracking: return "tracking";
      case State::lost_hold: return "lost_hold";
      default: return "inactive";
    }
  }

private:
  Config config_;
  State state_ = State::inactive;
  bool locked_ = false, have_time_ = false, have_frame_ = false;
  int consecutive_ = 0;
  uint64_t target_key_ = 0;
  double last_update_s_ = 0.0, last_frame_s_ = 0.0, last_seen_s_ = 0.0;
  double yaw_ = 0.0, pitch_ = 0.0;
  void reset()
  {
    state_ = State::inactive;
    locked_ = have_time_ = have_frame_ = false;
    consecutive_ = 0;
    target_key_ = 0;
  }
  Output output(bool control, bool inhibit_fire, bool hold_angles) const
  {
    return {state_, control, inhibit_fire, hold_angles, yaw_, pitch_,
      control ? last_seen_s_ + config_.loss_hold_s : 0.0};
  }
};
}  // namespace auto_aim
#endif
