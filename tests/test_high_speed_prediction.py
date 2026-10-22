"""Production high-speed prediction and source ownership; no hardware access."""
import unittest

from test_lightbar_search import compile_run


class HighSpeedPredictionTests(unittest.TestCase):
    def test_final_face_selection_and_legacy_mode_isolation(self):
        compile_run(r'''
#include "tasks/auto_aim/aimer.hpp"
#include <cassert>
#include <iostream>
using namespace std::chrono;
int main(int argc,char **argv) {
  auto_aim::Armor a(9,.95f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
  a.name=auto_aim::ArmorName::three;a.type=auto_aim::ArmorType::small;
  a.priority=auto_aim::ArmorPriority::first;a.xyz_in_world={3.3,0,.05};a.ypr_in_world={0,0,0};
  Eigen::VectorXd p(11);p.setOnes();auto_aim::YawESKF::Config yaw;yaw.enabled=false;
  auto_aim::Aimer aimer(argv[1]);int rescued=0,valid=0;
  for(int sign:{-1,1}) for(int k=0;k<360;++k) {
    const auto t=steady_clock::now();auto_aim::Target base(a,t,.3,4,p,yaw);
    auto state=base.ekf().x;state[0]=3.6;state[2]=.05;state[4]=.05;
    state[7]=sign*4.5;state[8]=.30;state[9]=.02;state[10]=.002;
    const double los=std::atan2(state[2],state[0]);
    // k=0 puts ALL faces outside the original intermediate window, while
    // a final face is legal. The rest sweep both directions and wrap edges.
    state[6]=los+(sign*25.+k)*M_PI/180.-state[7]*.345;
    auto fixed=base.with_fixed_center_prediction(state);assert(fixed);
    const auto before=base.ekf().x;auto_aim::AimPredictionTrace trace;
    const auto c=aimer.aim({*fixed},t,20.,false,&trace,auto_aim::AimGeometry::whole_ekf,
      auto_aim::AimTiming::high_speed);
    assert(trace.final_point_selection);
    assert((base.ekf().x-before).norm()==0);
    if(c.control) {
      ++valid;assert(trace.valid && trace.converged && trace.face_id>=0 && trace.face_id<4);
      assert(std::abs(trace.flight_time_s-trace.flight_time_used_s)<.001);
      auto future=*fixed;future.predict(steady_clock::time_point(nanoseconds(trace.prediction_ns)));
      const auto selected=future.whole_ekf_armor_xyza_list()[trace.face_id];
      assert((selected-aimer.debug_aim_point.xyza).norm()<1e-10);
      const double phase=std::remainder(selected[3]-los,2*M_PI);
      assert(std::abs(phase)<=60./57.3);
      assert(sign>0?phase<20./57.3:phase> -20./57.3);
    } else {assert(!trace.valid && !aimer.debug_aim_point.valid);}
    if(k==0) {
      assert(c.control);++rescued;
      for(auto timing:{auto_aim::AimTiming::low_speed,auto_aim::AimTiming::stationary}) {
        auto_aim::AimPredictionTrace old;
        const auto legacy=aimer.aim({*fixed},t,20.,false,&old,
          auto_aim::AimGeometry::whole_ekf,timing);
        assert(!old.final_point_selection && !legacy.control);
      }
    }
  }
  assert(rescued==2 && valid>600);
  std::cout<<"PASS final-face identity/geometry/convergence in both directions; "
    "intermediate-gap recovery; stationary/low preserve historical selection\n";
}
''', "test_high_speed_prediction", sources=(
            "tasks/auto_aim/aimer.cpp", "tasks/auto_aim/target.cpp", "tasks/auto_aim/armor.cpp",
            "tools/math_tools.cpp", "tools/trajectory.cpp", "tools/extended_kalman_filter.cpp",
            "tools/logger.cpp"), libraries=("opencv_core", "opencv_imgproc", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"config.yaml": "yaw_offset: 0\npitch_offset: 0\ncomming_angle: 60\n"
                      "leaving_angle: 20\nhigh_speed_delay_time: 0.340\nlow_speed_delay_time: 0.340\n"
                      "stationary_delay_time: 0.340\ndecision_speed: 0.8\n"}, arguments=("config.yaml",))

    def test_model_ownership_readiness_blind_gap_and_reset(self):
        compile_run(r'''
#include "tasks/auto_aim/fixed_center_model.hpp"
#include <cassert>
#include <iostream>
using Owner=auto_aim::FixedCenterOwnership;
Owner::Input input(int ms,bool ready=true) {
  return {1000000000LL+ms*1000000LL,1,3,true,true,false,ready};
}
int main() {
  Owner owner;
  auto x=owner.update(input(0));assert(!x.fixed && !x.changed);
  x=owner.update(input(100));assert(!x.fixed);
  x=owner.update(input(200));assert(x.fixed && x.available && x.changed);
  x=owner.update(input(220));assert(x.fixed && x.available && !x.changed);
  x=owner.update(input(240,false));assert(x.fixed && !x.available && !x.changed);
  x=owner.update(input(260));assert(x.fixed && x.available && !x.changed);
  // No output for a blind frame; ownership survives one short missing frame.
  for(int ms:{280,320,360,400}) {
    x=owner.update(input(ms,false));assert(x.fixed && !x.available && !x.changed);
  }
  x=owner.update(input(420,false));assert(!x.fixed && !x.available && x.changed);
  for(int ms:{440,540,640}) x=owner.update(input(ms));
  assert(x.fixed && x.changed);
  auto in=input(660);in.generation=4;x=owner.update(in);
  assert(!x.fixed && !x.available && x.changed); // Fit history was rebuilt.
  for(int ms:{680,780,880}) {in=input(ms);in.generation=4;x=owner.update(in);}
  assert(x.fixed);
  in=input(900);in.generation=4;in.epoch=2;x=owner.update(in);assert(!x.fixed && x.changed);
  // Mode exit, backwards time and duplicate timestamps cannot inherit readiness.
  for(int ms:{920,1020,1120}) x=owner.update(input(ms));
  assert(x.fixed);
  in=input(1140);in.active=false;x=owner.update(in);assert(!x.fixed && x.changed);
  for(int ms:{1160,1260,1360}) x=owner.update(input(ms));assert(x.fixed);
  x=owner.update(input(1360));assert(!x.fixed && x.changed);
  for(int ms:{1380,1480,1580}) x=owner.update(input(ms));assert(x.fixed);
  in=input(1600);in.context_valid=false;x=owner.update(in);assert(!x.fixed && x.changed);
  // Sparse ready observations do not promote a source across an invalid interval.
  Owner sparse;
  for(int ms=0;ms<=1000;ms+=50){x=sparse.update(input(ms,ms%200==0));assert(!x.fixed);}
  std::cout<<"PASS ownership warm-up, short-gap hold without availability, sustained fallback, "
    "epoch/generation/mode/clock resets\n";
}
''', "test_fixed_center_ownership")


if __name__ == "__main__":
    unittest.main()
