"""Production tracker isolation for the restored stationary/low-speed profile; no hardware."""
from pathlib import Path
import unittest
from test_lightbar_search import compile_run

ROOT = Path(__file__).resolve().parents[1]


class StationaryLowSpeedRestoreTests(unittest.TestCase):
    def test_real_trackers_keep_high_speed_state_and_restore_legacy_prediction(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        begin = source.index("    configured_tracker.set_high_speed_batch_projection(")
        selection = source[begin:source.index("    if (roi_reacquisition_enabled)", begin)]
        code = r'''
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/rotation_model_manager.hpp"
#include "tasks/auto_aim/gyro_phase_scheduler.hpp"
#include "tools/math_tools.hpp"
using namespace std::chrono;
namespace auto_aim {
Solver::Solver(const std::string &) {}
int batch_projection_calls=0;
void Solver::solve(Armor &, bool batch) const { if(batch) ++batch_projection_calls; }
std::vector<cv::Point2f> Solver::reproject_armor(
 const Eigen::Vector3d &,double,ArmorType,ArmorName) const {
 return {{0,0},{100,0},{100,40},{0,40}};
}
}
auto_aim::Armor measurement(double phase) {
 auto_aim::Armor a(9,.95f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
 a.color=auto_aim::Color::blue;a.name=auto_aim::ArmorName::three;a.type=auto_aim::ArmorType::small;
 int face=static_cast<int>(std::round(-phase/(M_PI/2.)))%4;if(face<0)face+=4;
 const double angle=phase+face*M_PI/2.;
 a.xyz_in_world={3.6-.3*std::cos(angle),-.3*std::sin(angle),.05};
 a.xyz_in_gimbal=a.xyz_in_world;a.ypr_in_world={std::remainder(angle,2*M_PI),0.,0.};
 a.ypd_in_world=tools::xyz2ypd(a.xyz_in_world);return a;
}
void same_targets(const std::list<auto_aim::Target> & a,const std::list<auto_aim::Target> & b) {
 assert(a.size()==b.size());if(a.empty())return;
 assert((a.front().ekf().x-b.front().ekf().x).norm()<1e-12);
 assert((a.front().ekf().P-b.front().ekf().P).norm()<1e-12);
 assert((a.front().ekf_x()-b.front().ekf_x()).norm()<1e-12);
 assert(a.front().yaw_eskf_enabled()==b.front().yaw_eskf_enabled());
 assert(a.front().yaw_eskf_accepted_updates()==b.front().yaw_eskf_accepted_updates());
}
void run(const char * off,const char * on,bool restore) {
 auto_aim::Solver solver(off);
 auto_aim::Tracker configured_tracker(off,solver),high_reference(off,solver),legacy_reference(on,solver);
 legacy_reference.set_high_speed_batch_projection(true); // ESKF must reject opting in.
 const int batch_calls_before=auto_aim::batch_projection_calls;
 std::unique_ptr<auto_aim::Tracker> legacy_tracker;
 if(restore)legacy_tracker=std::make_unique<auto_aim::Tracker>(off,solver,true);
 auto_aim::RotationModelManager::Config cfg;cfg.high_speed_phase_enabled=false;
 auto_aim::RotationModelManager rotation_model_manager(cfg);
 auto_aim::GyroPhaseScheduler gyro_phase_scheduler;
 const bool configured_yaw_eskf_enabled=false;
 bool previous_legacy_tracker_active=false,high_speed_camera_mode=false;
 auto_aim::Aimer actual_aimer(off),reference_aimer(on),high_aimer(off);
 int high_checks=0,legacy_checks=0,cv_checks=0,aim_checks=0,frame=0;
 double phase=0.;
 const auto stamp=steady_clock::now();
 for(double rate : {0.,.5,1.6,-1.6,4.5,-4.5,1.6,0.}) for(int j=0;j<130;++j) {
  const auto t=stamp+milliseconds(20*frame++);phase+=rate*.02;
  std::list<auto_aim::Armor> armors{measurement(phase)};
  auto reference_armors=armors;
  const auto high_targets=high_reference.track(reference_armors,t);
  auto legacy_input=armors;
SELECTION
  // Restore output must never modify the configured chain used for high speed.
  const auto & diag=configured_tracker.frame_diagnostics();
  const auto & reference_diag=high_reference.frame_diagnostics();
  assert(diag.state_after.size()==reference_diag.state_after.size());
  if(diag.state_after.size())assert((diag.state_after-reference_diag.state_after).norm()<1e-12);
  assert(diag.yaw_eskf_accepted==0 && !diag.yaw_eskf_enabled);
  if(legacy_tracker_active) {
   assert(!tracker.frame_diagnostics().high_speed_batch_projection);
   const auto expected=legacy_reference.track(legacy_input,t);same_targets(targets,expected);
   ++legacy_checks;
   if(!targets.empty() && targets.front().armor_motion_trusted(targets.front().last_id))++cv_checks;
   const auto timing=high_speed_mode?auto_aim::AimTiming::high_speed:
    (rotation_model.motion_mode==auto_aim::RotationModelManager::MotionMode::low_speed
     ?auto_aim::AimTiming::low_speed:auto_aim::AimTiming::stationary);
   const auto geometry=std::abs(rate)>.8?auto_aim::AimGeometry::whole_ekf:auto_aim::AimGeometry::existing;
   auto_aim::AimPredictionTrace actual,baseline;
   const auto a=actual_aimer.aim(targets,t,20.,false,&actual,geometry,timing);
   // The historical automatic delay chose 330ms in both speed branches.
   const auto b=reference_aimer.aim(expected,t,20.,false,&baseline,geometry);
   assert(a.control==b.control);
   if(a.control) {
    assert(std::abs(a.yaw-b.yaw)<1e-12 && std::abs(a.pitch-b.pitch)<1e-12);
    assert(std::abs(actual.delay_s-.330)<1e-12);++aim_checks;
   }
  } else {
   same_targets(targets,high_targets);
   if(high_speed_mode && !targets.empty()) {
    auto_aim::AimPredictionTrace a,b;
    const auto x=actual_aimer.aim(targets,t,20.,false,&a,auto_aim::AimGeometry::whole_ekf,auto_aim::AimTiming::high_speed);
    const auto y=high_aimer.aim(high_targets,t,20.,false,&b,auto_aim::AimGeometry::whole_ekf,auto_aim::AimTiming::high_speed);
    assert(x.control==y.control);
    if(x.control)assert(std::abs(x.yaw-y.yaw)<1e-12 && std::abs(x.pitch-y.pitch)<1e-12);
    assert(std::abs(a.delay_s-.355)<1e-12);++high_checks;
   }
  }
 }
 assert(high_checks>50);
 if(restore) assert(auto_aim::batch_projection_calls>batch_calls_before);
 else assert(auto_aim::batch_projection_calls==batch_calls_before);
 if(restore)assert(legacy_checks>100 && cv_checks>100 && aim_checks>100);
 else assert(legacy_checks==0);
 std::cout<<"PASS restored="<<restore<<" legacy="<<legacy_checks<<" cv="<<cv_checks
  <<" high="<<high_checks<<" matched legacy prediction="<<aim_checks<<"\n";
}
int main(int argc,char ** argv) { assert(argc==3);run(argv[1],argv[2],false);run(argv[1],argv[2],true); }
'''
        code = code.replace("SELECTION", selection)
        config = (
            "yaw_eskf_enabled: false\nyaw_eskf_max_trusted_abs_rate_rad_s: 8\n"
            "enemy_color: blue\nmin_detect_count: 1\nmax_temp_lost_count: 15\n"
            "outpost_max_temp_lost_count: 75\nyaw_offset: 0\npitch_offset: 0\n"
            "comming_angle: 60\nleaving_angle: 20\nhigh_speed_delay_time: 0.355\n"
            "low_speed_delay_time: 0.330\nstationary_delay_time: 0.330\ndecision_speed: 0.8\n"
        )
        compile_run(code, "test_stationary_low_speed_restore", sources=(
            "tasks/auto_aim/aimer.cpp", "tasks/auto_aim/target.cpp", "tasks/auto_aim/tracker.cpp",
            "tasks/auto_aim/armor.cpp", "tools/math_tools.cpp", "tools/trajectory.cpp",
            "tools/extended_kalman_filter.cpp", "tools/logger.cpp"),
            libraries=("opencv_core", "opencv_imgproc", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"off.yaml": config, "on.yaml": config.replace(
                "yaw_eskf_enabled: false", "yaw_eskf_enabled: true").replace(
                "high_speed_delay_time: 0.355", "high_speed_delay_time: 0.330")},
            arguments=("off.yaml", "on.yaml"))


if __name__ == "__main__":
    unittest.main()
