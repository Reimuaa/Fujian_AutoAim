#ifndef AUTO_AIM__TARGET_DEATH_RETURN_HPP
#define AUTO_AIM__TARGET_DEATH_RETURN_HPP

#include <cmath>
#include <stdexcept>

namespace auto_aim
{
// Vision infers lights-out from continuous fresh full-frame absence only after
// the MCU grants mode=1 and WorkZoneTarget confirms a real illuminated target.
class TargetDeathReturn
{
public:
  struct Config
  {
    bool enabled = false;
    double missing_s = 1.5;
  };
  struct Input
  {
    double now_s = 0.0, frame_s = 0.0;
    bool work_zone_ready = false, feedback_valid = false;
    bool frame_valid = false, full_frame = false;
    bool target_visible = false, target_confirmed = false;
  };
  struct Output
  {
    bool armed = false, request_return = false, triggered = false;
    double missing_s = 0.0;
    int missing_frames = 0;
  };

  TargetDeathReturn() = default;
  explicit TargetDeathReturn(const Config & config) : config_(config)
  {
    if (!std::isfinite(config_.missing_s) || config_.missing_s < 0.5 ||
        config_.missing_s > 10.0)
      throw std::invalid_argument("XUC target-death absence must be 500-10000ms");
  }
  bool armed() const { return armed_; }

  Output update(const Input & in)
  {
    // One return event per process, matching XucSender's existing return latch.
    // Neither mode=0 during return nor a later mode=1 can reactivate it.
    if (return_requested_) return output();
    if (!config_.enabled || !std::isfinite(in.now_s) ||
        (have_time_ && in.now_s <= last_now_s_)) {
      reset_phase();
      return output();
    }
    last_now_s_ = in.now_s;
    have_time_ = true;
    if (!in.feedback_valid) {
      // A link outage is not evidence of target death. Keep prior acquisition,
      // but require a completely new absence window after valid feedback resumes.
      clear_missing();
      have_frame_ = false;
      return output();
    }
    if (!in.work_zone_ready) {
      reset_phase();
      return output();
    }
    constexpr double max_frame_gap_s = 0.250;
    const bool fresh = in.frame_valid && std::isfinite(in.frame_s) &&
      in.frame_s <= in.now_s && in.now_s - in.frame_s <= max_frame_gap_s &&
      (!have_frame_ || in.frame_s > last_frame_s_);
    if (!fresh) {
      clear_missing();
      return output();
    }
    if (have_frame_ && in.frame_s - last_frame_s_ >= max_frame_gap_s)
      clear_missing();
    have_frame_ = true;
    last_frame_s_ = in.frame_s;
    if (in.target_visible) {
      if (in.target_confirmed) armed_ = true;
      clear_missing();
      return output();
    }
    if (!armed_ || !in.full_frame) {
      clear_missing();
      return output();
    }
    if (missing_frames_ == 0) missing_started_s_ = in.frame_s;
    ++missing_frames_;
    missing_s_ = in.frame_s - missing_started_s_;
    if (missing_frames_ >= 3 && missing_s_ + 1e-9 >= config_.missing_s) {
      return_requested_ = true;
      auto out = output();
      out.triggered = true;
      return out;
    }
    return output();
  }

private:
  Config config_;
  bool armed_ = false, return_requested_ = false, have_time_ = false, have_frame_ = false;
  double last_now_s_ = 0.0, last_frame_s_ = 0.0, missing_started_s_ = 0.0, missing_s_ = 0.0;
  int missing_frames_ = 0;
  void clear_missing() { missing_frames_ = 0; missing_started_s_ = missing_s_ = 0.0; }
  void reset_phase()
  {
    armed_ = have_time_ = have_frame_ = false;
    clear_missing();
  }
  Output output() const { return {armed_, return_requested_, false, missing_s_, missing_frames_}; }
};
}  // namespace auto_aim
#endif
