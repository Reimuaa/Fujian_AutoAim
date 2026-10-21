#include "rotation_model_manager.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  using Manager = auto_aim::RotationModelManager;
  Manager manager;

  auto result = manager.update(0.00, true, 2.0);
  assert(result.mode == Manager::Mode::armor);
  result = manager.update(0.20, true, 2.0);
  assert(result.mode == Manager::Mode::vehicle);
  assert(result.mode_changed);
  assert(!result.phase_reset);

  result = manager.update(0.30, true, 4.5);
  assert(result.mode == Manager::Mode::vehicle);
  result = manager.update(0.50, true, 4.5);
  assert(result.mode == Manager::Mode::high_speed_phase);
  assert(result.phase_reset);

  result = manager.update(0.51, true, 4.5, true, 4.4);
  assert(!result.phase_fire_trusted);
  result = manager.update(0.52, true, 4.5, true, 4.4);
  assert(result.rate_consistent);
  assert(result.phase_fire_trusted);

  result = manager.update(0.53, true, 4.5, true, 6.2);
  assert(!result.phase_fire_trusted);
  manager.update(0.54, true, 4.5, true, 6.2);
  result = manager.update(0.55, true, 4.5, true, 6.2);
  assert(result.phase_reset);

  manager.update(0.60, true, 2.4);
  result = manager.update(0.80, true, 2.4);
  assert(result.mode == Manager::Mode::vehicle);
  assert(result.phase_reset);

  result = manager.update(1.00, false, 0.0);
  assert(result.mode == Manager::Mode::armor);
  assert(!result.phase_fire_trusted);

  // Filtered rate plus separate enter/exit thresholds must prevent mode
  // chatter when a slow target hovers around the vehicle-model boundary.
  Manager::Config smooth_config;
  smooth_config.vehicle_enter_rad_s = 0.9;
  smooth_config.vehicle_exit_rad_s = 0.45;
  smooth_config.mode_dwell_s = 0.20;
  smooth_config.rate_filter_tau_s = 0.10;
  smooth_config.rate_disagreement_updates = 18;
  Manager smooth(smooth_config);
  smooth.update(0.00, true, 1.2);
  result = smooth.update(0.25, true, 1.2);
  assert(result.mode == Manager::Mode::vehicle);
  int changes = result.mode_changed ? 1 : 0;
  for (int i = 1; i <= 80; ++i) {
    const double noisy_rate = i % 2 == 0 ? 0.65 : 1.0;
    result = smooth.update(0.25 + 0.02 * i, true, noisy_rate);
    changes += result.mode_changed ? 1 : 0;
    assert(result.mode == Manager::Mode::vehicle);
  }
  assert(changes == 1);

  // Short tracker convergence dropouts must inhibit trust/fire without
  // destroying the selected motion model.  A sustained dropout still resets.
  Manager::Config dropout_config;
  dropout_config.mode_dwell_s = 0.10;
  dropout_config.tracker_dropout_hold_s = 0.25;
  Manager dropout(dropout_config);
  dropout.update(0.00, true, 4.5);
  result = dropout.update(0.11, true, 4.5);
  assert(result.mode == Manager::Mode::high_speed_phase);
  result = dropout.update(0.20, false, 0.0);
  assert(result.mode == Manager::Mode::high_speed_phase);
  assert(!result.tracker_trusted);
  assert(!result.phase_reset);
  result = dropout.update(0.40, false, 0.0);
  assert(result.mode == Manager::Mode::armor);
  assert(result.phase_reset);

  // With phase mode disabled, crossing either old high-speed boundary must
  // keep vehicle prediction, including when a phase clock is supplied.
  Manager::Config two_mode_config;
  two_mode_config.high_speed_phase_enabled = false;
  two_mode_config.vehicle_exit_rad_s = 0.45;
  two_mode_config.mode_dwell_s = 0.60;
  two_mode_config.tracker_dropout_hold_s = 0.25;
  Manager two_mode(two_mode_config);
  result = two_mode.update(0.00, true, 0.0);
  assert(result.mode == Manager::Mode::armor);
  result = two_mode.update(0.70, true, 0.7);
  assert(result.mode == Manager::Mode::armor);
  result = two_mode.update(1.00, true, 4.5);
  assert(result.mode == Manager::Mode::armor);
  result = two_mode.update(1.61, true, 4.5);
  assert(result.mode == Manager::Mode::vehicle && result.mode_changed);
  assert(!result.phase_reset && !result.phase_fire_trusted);
  double two_mode_time = 1.61;
  for (double omega : {3.1, 3.3, 8.0, 2.6, 2.4, -4.5, -8.0, 0.6}) {
    two_mode_time += 0.70;
    result = two_mode.update(two_mode_time, true, omega, true, std::abs(omega));
    assert(result.mode == Manager::Mode::vehicle && !result.mode_changed);
    assert(!result.phase_reset && !result.phase_fire_trusted);
  }
  result = two_mode.update(two_mode_time + 0.10, false, 0.0);
  assert(result.mode == Manager::Mode::vehicle && !result.tracker_trusted);
  result = two_mode.update(two_mode_time + 0.20, true, 4.5);
  assert(result.mode == Manager::Mode::vehicle && !result.mode_changed);
  two_mode_time += 0.30;
  result = two_mode.update(two_mode_time, true, 0.0);
  assert(result.mode == Manager::Mode::vehicle);
  result = two_mode.update(two_mode_time + 0.61, true, 0.0);
  assert(result.mode == Manager::Mode::armor && result.mode_changed);
  result = two_mode.update(two_mode_time + 0.70, true, -4.5);
  assert(result.mode == Manager::Mode::armor);
  result = two_mode.update(two_mode_time + 1.31, true, -4.5);
  assert(result.mode == Manager::Mode::vehicle);
  result = two_mode.update(two_mode_time + 1.60, false, 0.0);
  assert(result.mode == Manager::Mode::armor && !result.tracker_trusted);
  assert(!result.phase_reset && !result.phase_fire_trusted);

  // Motion classification remains three-state with the PLL disabled. Legacy
  // low-speed prediction stays vehicle; high speed does not enable the PLL.
  using Motion = Manager::MotionMode;
  for (double sign : {-1.0, 1.0}) {
    Manager::Config cfg;
    cfg.high_speed_phase_enabled = false;
    cfg.vehicle_exit_rad_s = 0.45;
    cfg.mode_dwell_s = 0.6;
    cfg.tracker_dropout_hold_s = 0.25;
    Manager three(cfg);
    assert(three.update(0.0, true, 0.0).motion_mode == Motion::stationary);
    assert(three.update(0.1, true, sign * 1.5).motion_mode == Motion::stationary);
    result = three.update(0.71, true, sign * 1.5);
    assert(result.motion_mode == Motion::low_speed && result.motion_mode_changed);
    assert(result.mode == Manager::Mode::vehicle);
    three.update(0.8, true, sign * 4.5);
    result = three.update(1.41, true, sign * 4.5);
    assert(result.motion_mode == Motion::high_speed && result.motion_mode_changed);
    assert(result.mode == Manager::Mode::vehicle && !result.mode_changed);
    assert(!result.phase_reset && !result.phase_fire_trusted);
    // Both hysteresis bands retain their current mode, even for long periods.
    result = three.update(2.2, true, sign * 2.8, true, 2.8);
    assert(result.motion_mode == Motion::high_speed && !result.phase_fire_trusted);
    three.update(2.3, true, sign * 2.4);
    result = three.update(2.91, true, sign * 2.4);
    assert(result.motion_mode == Motion::low_speed);
    assert(three.update(3.6, true, sign * 0.6).motion_mode == Motion::low_speed);
    three.update(3.7, true, 0.0);
    assert(three.update(4.31, true, 0.0).motion_mode == Motion::stationary);
    // A brief excursion above 3.2 cannot enter high speed.
    three.update(4.4, true, sign * 4.5);
    assert(three.update(4.6, true, sign * 1.5).motion_mode == Motion::stationary);
    assert(three.update(5.21, true, sign * 1.5).motion_mode == Motion::low_speed);
    // Lost observations cannot complete the high-speed dwell.
    three.update(5.3, true, sign * 4.5);
    result = three.update(5.4, false, 0.0);
    assert(result.motion_mode == Motion::low_speed && !result.tracker_trusted);
    assert(three.update(5.5, true, sign * 4.5).motion_mode == Motion::low_speed);
    assert(three.update(6.11, true, sign * 4.5).motion_mode == Motion::high_speed);
    result = three.update(6.12, false, 0.0);
    assert(result.motion_mode == Motion::high_speed && !result.tracker_trusted);
    result = three.update(6.5, false, 0.0);
    assert(result.motion_mode == Motion::stationary && !result.tracker_trusted);
    three.update(6.6, true, sign * 4.5);
    result = three.update(7.21, true, sign * 4.5);
    assert(result.motion_mode == Motion::high_speed);
    assert(!three.update(7.21, true, sign * 4.5).tracker_trusted); // Duplicate frame.
    assert(three.motion_mode() == Motion::stationary);
    assert(!three.update(NAN, true, sign * 4.5).tracker_trusted);
  }
  for (int bad = 0; bad < 4; ++bad) {
    Manager::Config cfg;
    if (bad == 0) cfg.vehicle_exit_rad_s = cfg.vehicle_enter_rad_s;
    if (bad == 1) cfg.high_speed_exit_rad_s = cfg.vehicle_enter_rad_s;
    if (bad == 2) cfg.high_speed_enter_rad_s = cfg.high_speed_exit_rad_s;
    if (bad == 3) cfg.rate_filter_tau_s = NAN;
    bool rejected = false;
    try { Manager invalid(cfg); } catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  {
    Manager::Config cfg;
    cfg.high_speed_phase_enabled = false;
    cfg.mode_dwell_s = 0.6;
    cfg.rate_filter_tau_s = 0.1;
    Manager boundary(cfg);
    for (int i = 0; i < 160; ++i) {
      result = boundary.update(i * 0.02, true, i % 2 ? 3.3 : 3.1);
      if (i >= 32) assert(result.motion_mode == Motion::low_speed);
      assert(!result.phase_fire_trusted);
    }
  }
  std::cout << "rotation_model_manager_test: PASS\n";
  return 0;
}
