"""Exercise actual PLL geometry and production qualification without device I/O."""
from pathlib import Path
import unittest

from test_lightbar_search import compile_run

ROOT = Path(__file__).resolve().parents[1]


class PllEventTimingTests(unittest.TestCase):
    def test_absolute_event_geometry_and_production_gates(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")

        def statement(start):
            begin = source.index(start)
            return source[begin:source.index(";", begin) + 1]

        start = source.index("struct RotationFireWindow")
        window = source[start:source.index("\n};", start) + 3]
        code = r'''
#include <cassert>
#include <iostream>
#include <list>
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/gyro_phase_scheduler.hpp"
using namespace std::chrono;
WINDOW
struct Gates {
  bool control=true, auto_mode=true, convergence=true, face=true, motion=true;
  bool cadence=true, yaw_fresh=true, pitch_fresh=true, imu=true, phase=true;
  bool interval=true, geometry=true, inhibit=false, rebuilt=false;
  double lead=.020, gimbal=0., error=0., pitch=0.;
};
bool qualify(auto_aim::Aimer & aimer, const auto_aim::Target & target,
  RotationFireWindow & rotation_fire_window, auto_aim::EventAimPrediction prediction,
  int64_t cycle, int native_face, int sequence, Gates gates={}) {
  const auto t=steady_clock::time_point(seconds(10)+milliseconds(sequence*20));
  const bool gyro_event_schedule_valid=true, predictive_fire_active=true;
  prediction.valid &= gates.geometry;
  const auto & gyro_event_prediction=prediction;
  const int gyro_event_face_id=prediction.face_id;
  const int64_t gyro_event_hit_cycle=cycle;
  const bool gyro_event_face_valid=prediction.valid;
  const bool gyro_phase_model_trusted=gates.phase, gyro_event_interval_safe=gates.interval;
  const bool xuc_event_require_face_consistency=false;
  const double gyro_event_gimbal_tracking_error_rad=gates.gimbal;
  const double xuc_event_gimbal_tracking_tolerance_rad=4.*M_PI/180.;
  const double gyro_event_lead_error_s=gates.lead, xuc_event_trigger_window_s=.050;
  const bool gyro_predicted_impact_yaw_valid=prediction.valid;
  const double gyro_predicted_impact_yaw_error_rad=gates.error;
  const double xuc_gyro_predicted_yaw_tolerance_rad=.3*M_PI/180.;
  const double direct_yaw_pixel_error_rad=0., xuc_gyro_visibility_yaw_tolerance_rad=8.*M_PI/180.;
  const bool direct_yaw_measurement_fresh=gates.yaw_fresh;
  const bool direct_pitch_measurement_fresh=gates.pitch_fresh;
  const bool convergence_gate_open=gates.convergence, one_face_gate_open=gates.face;
  const bool motion_transition_gate_open=gates.motion, command_cadence_gate_open=gates.cadence;
  const bool auto_mode_active=gates.auto_mode, xuc_imu_sync_valid=gates.imu;
  const double direct_pitch_pixel_error_rad=gates.pitch, xuc_fire_pitch_tolerance_rad=.02;
  struct { bool open=true; } high_speed_pitch_quality;
  const bool high_speed_prediction_quality_open=true; // PLL bypasses non-PLL history quality.
  const bool high_speed_prediction_current_valid=true; // The current-forecast veto is also non-PLL only.
  io::Command command{gates.control,false,0.,0.}; command.inhibit_shoot=gates.inhibit;
  struct { bool enabled() const { return true; } } xuc;
  struct {
    std::string state() const { return "tracking"; }
    const void * primary_observation() const { return this; }
  } tracker;
  struct { bool initialized_this_frame=false, nis_reset=false, diverged=false; } fire_tracker_frame;
  fire_tracker_frame.initialized_this_frame=gates.rebuilt;
  const bool face_correction_changed=false, native_prediction_valid=false;
  auto_aim::AimPredictionTrace native_fire_prediction;
  native_fire_prediction.face_id=native_face;
  std::list<auto_aim::Target> targets{target};
  const int xuc_gyro_fire_min_consecutive_frames=2;
  const double xuc_gyro_candidate_max_phase_step_rad=25.*M_PI/180.;
  const uint64_t camera_frame_count=sequence;
  const double selection_phase_rad=NAN, selection_omega_rad_s=NAN;
  const double selection_coming_rad=60./57.3, selection_leaving_rad=20./57.3;
YAW_GATE
EVENT_GATE
  const bool yaw_gate_open=gyro_event_gate_open;
FIRE_CANDIDATE
OBSERVATION
WINDOW_UPDATE
POINT_GATE
  const bool stable_window_open=rotation_fire_window.two_frames_ready;
REQUEST
  return command.shoot;
}
int main(int argc,char ** argv) {
  assert(argc==3);
  auto_aim::Aimer aimer(argv[1]), changed_delays(argv[2]);
  const auto stamp=steady_clock::time_point(seconds(10));
  auto_aim::Armor armor(11,.9f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
  armor.xyz_in_world={3.3,0.,.05}; armor.ypr_in_world={0.,0.,0.};
  auto_aim::EventAimPrediction prediction;
  auto_aim::Target target(armor,stamp,.3,4,Eigen::VectorXd::Ones(11));
  int checked=0;
  for(double direction : {-1.,1.}) for(int processing_ms : {5,35,75}) {
    const auto impact=stamp+milliseconds(275);
    auto & state=const_cast<tools::ExtendedKalmanFilter &>(target.ekf()).x;
    const double bearing=std::atan2(.1-.05*.275,3.6+.1*.275);
    state << 3.6,.1,.1,-.05,.05,0.,bearing-direction*4.5*.275+.025,
      direction*4.5,.28,.04,.03;
    const auto before=target.ekf_x();
    auto_aim::GyroFaceSelector selector(15.*M_PI/180.), other(15.*M_PI/180.);
    // Deliberate processing wait is represented by a later decision time;
    // neither geometry API reads the wall clock or adds that age a second time.
    const auto decision=stamp+milliseconds(processing_ms);
    assert(decision<impact);
    prediction=aimer.predict_event(target,impact,selector,12);
    const auto isolated=changed_delays.predict_event(target,impact,other,12);
    assert(prediction.valid && isolated.valid);
    assert(prediction.prediction_ns==10275000000LL);
    assert(prediction.face_id==isolated.face_id && prediction.yaw_rad==isolated.yaw_rad);
    assert((prediction.xyza-isolated.xyza).norm()==0.);
    auto expected=target; expected.predict(impact);
    const auto xyza=expected.armor_xyza_list()[prediction.face_id];
    assert((prediction.xyza-xyza).norm()<1e-12);
    assert(std::abs(prediction.yaw_rad-std::atan2(xyza[1],xyza[0])-1.25/57.3)<1e-12);
    assert((target.ekf_x()-before).norm()==0.);
    const auto repeated=aimer.predict_event(target,impact,selector,12);
    assert(repeated.valid && repeated.prediction_ns==prediction.prediction_ns &&
      repeated.face_id==prediction.face_id && repeated.yaw_rad==prediction.yaw_rad);
    ++checked;
  }
  assert(checked==6);
  aimer.debug_aim_point.valid=false; // Invalid/different native forecast cannot block PLL.
  RotationFireWindow w;
  assert(!qualify(aimer,target,w,prediction,12,0,1));
  assert(qualify(aimer,target,w,prediction,12,3,2));
  assert(w.face_id==prediction.face_id && w.hit_cycle==12);
  // Even the same physical face cannot borrow a frame from another event.
  assert(!qualify(aimer,target,w,prediction,13,2,3));
  assert(qualify(aimer,target,w,prediction,13,0,4));
  auto changed=prediction; changed.face_id=(prediction.face_id+1)%4;
  assert(!qualify(aimer,target,w,changed,13,0,5));
  Gates invalid; invalid.geometry=false;
  assert(!qualify(aimer,target,w,prediction,13,0,6,invalid));
  assert(!qualify(aimer,target,w,prediction,13,0,7));
  assert(qualify(aimer,target,w,prediction,13,0,8));
  Gates rebuilt; rebuilt.rebuilt=true;
  assert(!qualify(aimer,target,w,prediction,13,0,9,rebuilt));
  // Apply each production gate to a prequalified same-event observation.
  for(int blocked=0;blocked<19;++blocked) {
    Gates g;
    switch(blocked) {
      case 0:g.control=false;break; case 1:g.auto_mode=false;break;
      case 2:g.convergence=false;break; case 3:g.face=false;break;
      case 4:g.motion=false;break; case 5:g.cadence=false;break;
      case 6:g.yaw_fresh=false;break; case 7:g.pitch_fresh=false;break;
      case 8:g.imu=false;break; case 9:g.phase=false;break;
      case 10:g.interval=false;break; case 11:g.geometry=false;break;
      case 12:g.inhibit=true;break; case 13:g.lead=-.001;break;
      case 14:g.lead=.051;break; case 15:g.gimbal=4.01*M_PI/180.;break;
      case 16:g.error=.301*M_PI/180.;break;
      case 17:g.error=-.301*M_PI/180.;break; case 18:g.pitch=.021;break;
    }
    RotationFireWindow gate_window;
    qualify(aimer,target,gate_window,prediction,20,0,1);
    assert(!qualify(aimer,target,gate_window,prediction,20,3,2,g));
  }
  for(double error : {-.299,.299}) {
    Gates g; g.error=error*M_PI/180.; RotationFireWindow allowed;
    qualify(aimer,target,allowed,prediction,30,0,1,g);
    assert(qualify(aimer,target,allowed,prediction,30,3,2,g));
  }
  auto_aim::GyroFaceSelector selector(15.*M_PI/180.);
  assert(!aimer.predict_event(target,stamp,selector,-1).valid);
  auto three=auto_aim::Target(armor,stamp,.3,3,Eigen::VectorXd::Ones(11));
  assert(!aimer.predict_event(three,stamp+milliseconds(275),selector,1).valid);
  auto nonfinite=target;
  const_cast<tools::ExtendedKalmanFilter &>(nonfinite.ekf()).x[0]=NAN;
  assert(!aimer.predict_event(nonfinite,stamp+milliseconds(275),selector,1).valid);
  std::cout<<"PASS PLL absolute event geometry: both directions, timing isolation, "
    "native forecast independence, same-event/face qualification, 19 closed gates and +/-0.3 deg\n";
}
'''
        replacements = {
            "WINDOW": window,
            "YAW_GATE": statement("      const bool prediction_yaw_gate_open ="),
            "EVENT_GATE": statement("      const bool gyro_event_gate_open ="),
            "FIRE_CANDIDATE": statement("      const bool fire_candidate ="),
            "OBSERVATION": statement("      const bool prediction_observation_valid ="),
            "WINDOW_UPDATE": statement("      rotation_fire_window.update("),
            "POINT_GATE": statement("      const bool prediction_point_gate_open ="),
            "REQUEST": statement("        command.shoot = stable_window_open && fire_candidate &&"),
        }
        # Whole-line replacement avoids replacing WINDOW inside WINDOW_UPDATE.
        code = "\n".join(replacements.get(line, line) for line in code.splitlines())
        config = ("yaw_offset: 1.25\npitch_offset: 0\ncomming_angle: 60\nleaving_angle: 20\n"
                  "high_speed_delay_time: 0.330\nlow_speed_delay_time: 0.330\ndecision_speed: 0.8\n")
        compile_run(code, "test_pll_event_timing", sources=(
            "tasks/auto_aim/aimer.cpp", "tasks/auto_aim/target.cpp", "tasks/auto_aim/armor.cpp",
            "tools/math_tools.cpp", "tools/trajectory.cpp", "tools/extended_kalman_filter.cpp",
            "tools/logger.cpp"), libraries=("opencv_core", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"config.yaml": config, "changed.yaml": config.replace("0.330", "0.090")},
            arguments=("config.yaml", "changed.yaml"))


if __name__ == "__main__":
    unittest.main()
