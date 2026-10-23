"""Production EKF-only prediction and calibrated PLL residual; no device I/O."""
from pathlib import Path
import unittest

from test_lightbar_search import compile_run

ROOT = Path(__file__).resolve().parents[1]


class EkfOnlyPredictionTests(unittest.TestCase):
    def test_model_switch_tracker_wiring_and_pll_reference(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        begin = source.index("gyro_event_command_yaw_rad = gyro_event_prediction.yaw_in_command_frame(")
        command_yaw = source[begin:source.index(";", begin) + 1]
        begin = source.index("gyro_predicted_impact_yaw_error_rad = std::remainder(", begin)
        residual = source[begin:source.index(";", begin) + 1]
        code = r'''
#include <cassert>
#include <cmath>
#include <iostream>
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/rotation_model_manager.hpp"
#include "tools/math_tools.hpp"
using namespace std::chrono;
// The tracker receives solved synthetic poses. PnP and projection are boundary
// stubs; the actual tracker config, association, target and filter code run.
namespace auto_aim {
Solver::Solver(const std::string &) {}
void Solver::solve(Armor &, bool) const {}
std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d &,double,ArmorType,ArmorName) const {
  return {{0,0},{100,0},{100,40},{0,40}};
}
}
auto_aim::Armor measurement(double phase,int face,double t,double speed=.08) {
  auto_aim::Armor a(9,.95f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
  a.color=auto_aim::Color::blue;a.name=auto_aim::ArmorName::three;
  a.type=auto_aim::ArmorType::small;
  const double angle=phase+face*M_PI/2.;
  a.xyz_in_world={3.6+speed*t-.3*std::cos(angle),.1-.03*t-.3*std::sin(angle),.05};
  a.xyz_in_gimbal=a.xyz_in_world;a.ypr_in_world={angle,0.,0.};
  a.ypd_in_world=tools::xyz2ypd(a.xyz_in_world);
  return a;
}
auto_aim::Target train(double rate,bool enabled) {
  const auto stamp=steady_clock::time_point(seconds(10));
  auto_aim::YawESKF::Config cfg;cfg.enabled=enabled;cfg.max_trusted_abs_rate_rad_s=8.;
  auto a=measurement(0.,0,0.);
  auto_aim::Target target(a,stamp,.3,4,Eigen::VectorXd::Ones(11),cfg);
  if(!enabled) assert(!target.yaw_model_trusted());
  for(int i=1;i<=400;++i) {
    const double t=.01*i,phase=rate*t;
    int face=static_cast<int>(std::round(-phase/(M_PI/2.)))%4;if(face<0)face+=4;
    a=measurement(phase,face,t);target.predict(stamp+milliseconds(10*i));target.update(a,face);
  }
  assert(target.convergened());
  return target;
}
double production_error(const auto_aim::EventAimPrediction & gyro_event_prediction,
  double direct_yaw_command_correction_rad,double reference,double xuc_gyro_predicted_yaw_trim_rad=0.) {
  std::array<double,3> ypr{reference,0.,0.};
  double gyro_event_command_yaw_rad=NAN,gyro_predicted_impact_yaw_error_rad=NAN;
COMMAND_YAW
RESIDUAL
  return gyro_predicted_impact_yaw_error_rad;
}
int main(int argc,char **argv) {
  assert(argc==3);
  auto_aim::YawESKF::Config disabled;disabled.enabled=false;
  auto_aim::YawESKF filter(disabled);
  filter.reset(.3,4.5);filter.predict(.6);
  assert(!filter.update(.5,.01).accepted);
  assert(!filter.update_equivalent(.5,.01,M_PI/2.).accepted);
  assert(!filter.enabled() && !filter.initialized() && !filter.trusted());
  assert(filter.accepted_updates()==0 && filter.rejected_updates()==0);
  auto_aim::Aimer aimer(argv[1]);int event_checks=0,aim_checks=0;
  for(double rate : {-4.5,-1.5,0.,.6,1.5,4.5}) {
    auto target=train(rate,false);
    assert(!target.yaw_eskf_enabled() && !target.yaw_eskf_trusted());
    assert(target.yaw_eskf_accepted_updates()==0 && target.yaw_eskf_rejected_updates()==0);
    assert(target.yaw_model_trusted() && std::string(target.yaw_model_source())=="whole_ekf");
    assert((target.ekf_x()-target.ekf().x).norm()==0.);
    assert(std::abs(target.ekf().x[7]-rate)<.1);
    const auto stamp=steady_clock::time_point(seconds(14));
    for(int ms : {50,330,550,800,1000}) {
      auto future=target;future.predict(stamp+milliseconds(ms));
      const auto state=future.ekf_x();
      assert((state-future.ekf().x).norm()==0.);
      assert(std::abs(std::remainder(state[6]-target.ekf().x[6]-target.ekf().x[7]*ms/1000.,2*M_PI))<1e-10);
      const auto direct_requested=future.armor_xyza_list(true);
      const auto expected=future.whole_ekf_armor_xyza_list();
      for(int i=0;i<4;++i) assert((direct_requested[i]-expected[i]).norm()==0.);
    }
    for(auto timing : {auto_aim::AimTiming::stationary,auto_aim::AimTiming::low_speed,
                       auto_aim::AimTiming::high_speed}) {
      auto_aim::AimPredictionTrace trace;
      const auto cmd=aimer.aim({target},stamp,20.,false,&trace,auto_aim::AimGeometry::existing,timing);
      assert(trace.whole_ekf_geometry);
      if(cmd.control) {
        auto future=target;future.predict(steady_clock::time_point(nanoseconds(trace.prediction_ns)));
        assert(trace.valid);
        assert((future.whole_ekf_armor_xyza_list()[trace.face_id]-aimer.debug_aim_point.xyza).norm()<1e-10);
        ++aim_checks;
      }
    }
    if(std::abs(rate)>3.) {
      auto_aim::GyroFaceSelector selector(15.*M_PI/180.);
      for(int ms=330;ms<=950;ms+=10) {
        const auto impact=stamp+milliseconds(ms);
        const auto event=aimer.predict_event(target,impact,selector,ms/100);
        if(!event.valid)continue;
        auto future=target;future.predict(impact);
        assert((event.xyza-future.whole_ekf_armor_xyza_list()[event.face_id]).norm()<1e-10);
        ++event_checks;
      }
      auto_aim::RotationModelManager::Config cfg;cfg.high_speed_phase_enabled=true;
      auto_aim::RotationModelManager manager(cfg);auto_aim::RotationModelManager::Result out;
      for(int i=0;i<100;++i)out=manager.update(14.+i*.02,target.yaw_model_trusted(),
        target.ekf_x()[7],true,rate);
      assert(out.motion_mode==auto_aim::RotationModelManager::MotionMode::high_speed);
      assert(out.mode==auto_aim::RotationModelManager::Mode::high_speed_phase && out.phase_fire_trusted);
    }
    auto & ekf=const_cast<tools::ExtendedKalmanFilter &>(target.ekf());
    const double variance=ekf.P(7,7);ekf.P(7,7)=4.01;
    assert(!target.yaw_model_trusted());ekf.P(7,7)=variance;
    const double omega=ekf.x[7];ekf.x[7]=8.01;
    assert(!target.yaw_model_trusted());ekf.x[7]=omega;
    const double radius=ekf.x[8];ekf.x[8]=.01;
    assert(!target.yaw_model_trusted());ekf.x[8]=radius;
    assert(target.yaw_model_trusted());
  }
  assert(aim_checks>10 && event_checks>10);
  const auto restored=train(4.5,true);
  assert(restored.yaw_eskf_enabled() && restored.yaw_eskf_accepted_updates()>0);
  // Actual tracker YAML wiring: static solved poses exercise initialization,
  // association and capture without relying on ESKF trust to open the EKF path.
  for(int index : {1,2}) {
    auto_aim::Solver solver(argv[index]);auto_aim::Tracker tracker(argv[index],solver);
    const auto stamp=steady_clock::now();std::list<auto_aim::Target> targets;
    auto a=measurement(0.,0,0.,0.);
    for(int i=0;i<100;++i) {
      std::list<auto_aim::Armor> armors{a};targets=tracker.track(armors,stamp+milliseconds(i*20));
    }
    assert(!targets.empty() && tracker.state()=="tracking");
    const auto & target=targets.front();const auto & diag=tracker.frame_diagnostics();
    assert(target.yaw_eskf_enabled()==(index==2));
    assert(diag.yaw_eskf_enabled==(index==2));
    assert(target.yaw_model_trusted() && diag.yaw_model_trusted);
    if(index==1) {
      assert(target.yaw_eskf_accepted_updates()==0 && diag.yaw_eskf_accepted==0);
      assert(std::string(diag.yaw_model_source)=="whole_ekf");
    } else assert(target.yaw_eskf_accepted_updates()>0);
  }
  for(double raw : {-M_PI+.005,.4,M_PI-.005}) for(double correction : {-1.3,1.3}) {
    auto_aim::EventAimPrediction event;event.valid=true;event.yaw_rad=raw;
    const double c=correction*M_PI/180.;
    const double actual=std::remainder(raw+c,2*M_PI);
    assert(std::abs(production_error(event,c,actual))<1e-12);
    assert(std::abs(production_error(event,c,actual,.002)+.002)<1e-12);
    assert(std::abs(event.yaw_rad-raw)<1e-12);
    assert(std::abs(production_error(event,c,actual-.6*M_PI/180.)-.6*M_PI/180.)<1e-12);
  }
  std::cout<<"PASS whole-EKF trial: ESKF does not run, tracker switch/restoration, six rates, "
    "all timing modes, EKF geometry/phase through 1s, valid raw-model trust and PLL entry; "
    "calibrated PLL residual signs/wrap/trim and unchanged raw geometry\n";
}
'''
        code = "\n".join({"COMMAND_YAW": command_yaw, "RESIDUAL": residual}.get(line, line)
                         for line in code.splitlines())
        config = ("yaw_eskf_enabled: false\nyaw_eskf_max_trusted_abs_rate_rad_s: 8\n"
                  "enemy_color: blue\nmin_detect_count: 1\nmax_temp_lost_count: 15\n"
                  "outpost_max_temp_lost_count: 75\nyaw_offset: 0\npitch_offset: 0\n"
                  "comming_angle: 60\nleaving_angle: 20\nhigh_speed_delay_time: 0.330\n"
                  "low_speed_delay_time: 0.330\nstationary_delay_time: 0.330\ndecision_speed: 0.8\n")
        compile_run(code, "test_ekf_only_prediction", sources=(
            "tasks/auto_aim/aimer.cpp", "tasks/auto_aim/target.cpp", "tasks/auto_aim/tracker.cpp",
            "tasks/auto_aim/armor.cpp", "tools/math_tools.cpp", "tools/trajectory.cpp",
            "tools/extended_kalman_filter.cpp", "tools/logger.cpp"),
            libraries=("opencv_core", "opencv_imgproc", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"off.yaml": config, "on.yaml": config.replace(
                "yaw_eskf_enabled: false", "yaw_eskf_enabled: true")},
            arguments=("off.yaml", "on.yaml"))


if __name__ == "__main__":
    unittest.main()
