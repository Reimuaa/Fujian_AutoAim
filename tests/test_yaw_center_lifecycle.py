#!/usr/bin/env python3
"""Exercise the actual standard.cpp centre/Yaw blocks offline; no hardware."""
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class YawCenterLifecycleTests(unittest.TestCase):
    def test_observation_loss_rebuild_and_continuous_follow(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        center_start = source.index("    vehicle_model_fresh = xuc_gyro_center_follow_enabled &&")
        reset_start = source.find("    if (tracker.frame_diagnostics().initialized_this_frame) {")
        if reset_start >= 0:
            center_start = reset_start
        center = source[center_start:
                        source.index("    if (gyro_center_follow_active != xuc_gyro_center_follow_logged)")]
        yaw = source[source.index("    if (targets.empty()) direct_yaw_control_confirmed = false;"):
                     source.index("    if (direct_pitch_guard_enabled) {", source.index("    // Phase 2D.5 integration guard."))]
        harness = r'''
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>
using Clock = std::chrono::steady_clock;
struct Point { double x, y; };
struct Vec { double a, b; double x() const { return a; } double y() const { return b; } };
namespace auto_aim {
struct Armor { Vec xyz_in_world; Point center{720,540}; std::vector<Point> points; };
struct RotationModelManager { enum class Mode { armor, vehicle, high_speed_phase }; };
}
struct Tracker {
  struct Diagnostics { bool initialized_this_frame = false; } diagnostics;
  bool observed = true;
  const void * primary_observation() const { return observed ? this : nullptr; }
  const Diagnostics & frame_diagnostics() const { return diagnostics; }
};
struct Logger {
  template<class... A> void info(A...) {}
  template<class... A> void warn(A...) {}
};
namespace tools { Logger * logger() { static Logger l; return &l; } }
struct Image { int cols=1440; bool empty() const { return false; } };
struct Command { bool control=true, shoot=true; double yaw=0, pitch=0; };
struct Controller {
  Tracker tracker;
  struct Rotation {
    bool tracker_trusted=true, phase_fire_trusted=true;
    auto_aim::RotationModelManager::Mode mode=auto_aim::RotationModelManager::Mode::vehicle;
  } rotation_model;
  bool xuc_gyro_center_follow_enabled=true, xuc_gyro_center_memory_valid=false;
  double xuc_gyro_center_memory_yaw=0, xuc_gyro_center_hold_s=3;
  Clock::time_point xuc_gyro_center_last_fresh;
  bool direct_yaw_initialized=false, direct_yaw_control_confirmed=false, direct_yaw_long_loss=false;
  double direct_yaw_filtered=0;
  Clock::time_point direct_yaw_last_update, direct_yaw_last_measurement;
  bool direct_yaw_guard_enabled=true, infantry_diagnostics_enabled=false;
  double direct_yaw_filter_tau_s=.06, direct_yaw_hold_s=.3;
  double direct_yaw_max_rate_rad_s=M_PI/2, direct_yaw_focal_x_px=2366;
  double direct_yaw_impact_correction_rad=0, direct_yaw_command_correction_rad=-1.3*M_PI/180;
  bool prediction_lead_limit_configured=true, native_prediction_trusted=true;
  double native_predicted_yaw=0, max_prediction_lead_rad=6*M_PI/180;
  bool vehicle_model_fresh=false, gyro_model_fresh=false, gyro_phase_model_trusted=false;
  bool gyro_center_follow_active=false, direct_yaw_measurement_fresh=false;
  double gyro_center_yaw=0;
  Command command;

  void step(double seconds, double center_yaw, bool associated=true, bool detected=true,
            bool rebuilt=false, bool converged=true) {
    auto t=Clock::time_point(std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1000+seconds)));
    tracker.observed=associated;
    tracker.diagnostics.initialized_this_frame=rebuilt;
    rotation_model.tracker_trusted=converged;
    gyro_center_yaw=center_yaw;
    gyro_center_follow_active=false;
    command={};
    CENTER_BLOCK
    auto_aim::Armor observation{{std::cos(center_yaw),std::sin(center_yaw)}};
    const auto * direct_measurement_armor=detected ? &observation : nullptr;
    const bool direct_measurement_from_tracker=associated && detected;
    const bool tracker_converged=converged;
    const std::vector<int> targets{1};
    const Image img;
    bool measurement_fallback=false;
    direct_yaw_measurement_fresh=false;
    double direct_yaw_measurement_rad=NAN, direct_yaw_pixel_error_rad=INFINITY;
    YAW_BLOCK
  }
};
bool near(double a,double b) { return std::abs(a-b)<1e-10; }
int main() {
  Controller gap;
  gap.step(0,1);
  assert(gap.command.control && gap.vehicle_model_fresh);
  const auto fresh_time=gap.xuc_gyro_center_last_fresh;
  const auto measurement_time=gap.direct_yaw_last_measurement;
  const double held_yaw=gap.command.yaw;
  // A drifting prediction plus an unassociated current-frame detection must
  // neither renew the centre nor add the old native lead to the held command.
  gap.native_predicted_yaw=1.5;
  gap.step(.1,1.2,false,true);
  assert(!gap.vehicle_model_fresh && gap.gyro_center_follow_active);
  assert(near(gap.xuc_gyro_center_memory_yaw,1));
  assert(gap.xuc_gyro_center_last_fresh==fresh_time);
  assert(gap.direct_yaw_last_measurement==measurement_time);
  assert(gap.command.control && !gap.command.shoot && near(gap.command.yaw,held_yaw));
  gap.step(.29,1.4,false,true);
  assert(gap.command.control && near(gap.command.yaw,held_yaw));
  gap.step(.31,1.6,false,true);
  assert(!gap.command.control && !gap.command.shoot);
  assert(gap.direct_yaw_last_measurement==measurement_time);

  Controller rebuild;
  rebuild.step(0,1);
  rebuild.step(.1,2,true,true,true,false);
  assert(!rebuild.xuc_gyro_center_memory_valid && !rebuild.gyro_center_follow_active);
  assert(!rebuild.command.control);
  // A short gap in the new, unconfirmed target must not regain authority.
  rebuild.step(.12,2,false,false,false,false);
  assert(!rebuild.command.control);
  const double before=rebuild.direct_yaw_filtered;
  rebuild.step(.14,2,true,true,false,true);
  assert(rebuild.command.control && rebuild.vehicle_model_fresh);
  assert(near(rebuild.xuc_gyro_center_memory_yaw,2));
  assert(std::abs(std::remainder(rebuild.direct_yaw_filtered-before,2*M_PI))<=M_PI/2*.02+1e-10);

  Controller moving;
  moving.native_prediction_trusted=false;
  moving.step(0,1);
  moving.step(.02,1.03);
  assert(moving.command.control && moving.vehicle_model_fresh);
  assert(near(moving.xuc_gyro_center_memory_yaw,1.03));
  assert(near(moving.direct_yaw_filtered,1+.03*(1-std::exp(-.02/.06))));
  moving.step(.04,1.06);
  assert(moving.command.control && near(moving.xuc_gyro_center_memory_yaw,1.06));
  moving.step(.1,1.08,false,true);
  moving.step(.12,1.10,true,true);
  assert(moving.command.control && near(moving.xuc_gyro_center_memory_yaw,1.10));

  Controller wrap;
  wrap.step(0,M_PI-.01);
  wrap.step(.02,-M_PI+.01);
  assert(std::abs(std::remainder(wrap.direct_yaw_filtered-(M_PI-.01),2*M_PI))<.02);

  Controller armor;
  armor.rotation_model.mode=auto_aim::RotationModelManager::Mode::armor;
  armor.native_prediction_trusted=false;
  armor.step(0,.5,false,true);
  assert(!armor.gyro_center_follow_active && armor.command.control);
  assert(near(armor.direct_yaw_filtered,.5));
  std::cout << "PASS: observed centre, bounded loss, rebuild, moving target, angle wrap, armor mode\n";
}
'''.replace("CENTER_BLOCK", center).replace("YAW_BLOCK", yaw)
        binary = ROOT / "build/yaw_center_lifecycle_test"
        binary.parent.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="yaw-center-test-") as folder:
            cpp = Path(folder) / "test.cpp"
            cpp.write_text(harness, encoding="utf-8")
            result = subprocess.run(["g++", "-std=c++17", "-O1", "-Wall", "-Wextra",
                                     str(cpp), "-o", str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
            self.assertEqual(run.returncode, 0, run.stderr)
            print(run.stdout.strip())


if __name__ == "__main__":
    unittest.main()
