"""Compare production high-speed geometry/ballistics to full Target predictions."""
import unittest

from test_lightbar_search import compile_run


class HighSpeedBallisticsTests(unittest.TestCase):
    def test_mean_query_and_tight_ballistics_match_full_filter_reference(self):
        compile_run(r'''
#include "tasks/auto_aim/aimer.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include <cassert>
#include <iostream>
using namespace std::chrono;
using namespace auto_aim;
struct Reference {
  bool valid=false;int face=-1;int64_t ns=0;
  Eigen::Vector4d point=Eigen::Vector4d::Zero();
  double yaw=0.,pitch=0.,gap=0.;
};
// Deliberately uses full Target copies/predict(), including covariance and
// auxiliary filters, rather than the new mean-only query under test.
Reference full_reference(Target target, steady_clock::time_point frame, double speed, double tolerance) {
  const auto base=frame+microseconds(375000);target.predict(base);
  const auto initial=target.whole_ekf_armor_xyza_list();
  for(int face=0;face<4;++face) {
    tools::Trajectory first(speed,initial[face].head<2>().norm(),initial[face][2]);
    if(first.unsolvable || !std::isfinite(first.fly_time) || first.fly_time<=0.)continue;
    double previous=first.fly_time;
    for(int iteration=0;iteration<10;++iteration) {
      const auto time=base+microseconds(static_cast<int>(previous*1e6));
      auto candidate=target;candidate.predict(time);
      const auto point=candidate.whole_ekf_armor_xyza_list()[face];
      tools::Trajectory trajectory(speed,point.head<2>().norm(),point[2]);
      if(trajectory.unsolvable || !std::isfinite(trajectory.fly_time) || trajectory.fly_time<=0.)break;
      if(std::abs(trajectory.fly_time-previous)<tolerance) {
        const auto & state=candidate.ekf().x;
        const double phase=tools::limit_rad(point[3]-std::atan2(state[2],state[0]));
        if(std::abs(phase)>60./57.3 ||
           (state[7]>0.?phase>=20./57.3:phase<=-20./57.3))break;
        return {true,face,duration_cast<nanoseconds>(time.time_since_epoch()).count(),
                point,std::atan2(point[1],point[0]),-trajectory.pitch,
                std::abs(trajectory.fly_time-previous)};
      }
      previous=trajectory.fly_time;
    }
  }
  return {};
}
int main(int argc,char **argv) {
  const auto frame=steady_clock::time_point(seconds(10));
  Armor a(9,.95f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
  a.name=ArmorName::three;a.type=ArmorType::small;a.priority=ArmorPriority::first;
  a.xyz_in_world={3.0,0.,.05};a.ypr_in_world={0.,0.,0.};
  YawESKF::Config yaw;yaw.enabled=false;
  Target base(a,frame,.3,4,Eigen::VectorXd::Ones(11),yaw);
  auto & state=const_cast<tools::ExtendedKalmanFilter &>(base.ekf()).x;
  state<<3.3,.2,.08,-.1,.05,.03,3.1,4.87,.30,.025,.012;
  base.jumped=true;
  int geometry=0,valid=0,invalid=0,prior_valid=0,became_valid=0,became_invalid=0;
  for(bool eskf:{false,true}) {
    yaw.enabled=eskf;
    Target sample(a,frame,.3,4,Eigen::VectorXd::Ones(11),yaw);
    const_cast<tools::ExtendedKalmanFilter &>(sample.ekf()).x=state;
    const auto before=sample.ekf().x;const auto covariance=sample.ekf().P;
    for(int sign:{-1,1}) {
      const_cast<tools::ExtendedKalmanFilter &>(sample.ekf()).x[7]=sign*4.87;
      for(int us:{0,1,20000,120000,500000,1200000})for(int face=0;face<4;++face) {
        const auto time=frame+microseconds(us);
        auto full=sample;full.predict(time);
        const auto query=sample.predict_rigid_armor(face,time);assert(query);
        assert((query->xyza-full.whole_ekf_armor_xyza_list()[face]).norm()<1e-11);
        assert(std::abs(query->center[0]-full.ekf().x[0])<1e-12);
        assert(std::abs(query->center[1]-full.ekf().x[2])<1e-12);++geometry;
      }
      sample.predict(frame+microseconds(50000));
      auto full=sample;full.predict(frame+microseconds(75000));
      assert((sample.predict_rigid_armor(3,frame+microseconds(75000))->xyza-
              full.whole_ekf_armor_xyza_list()[3]).norm()<1e-11);
      sample=Target(a,frame,.3,4,Eigen::VectorXd::Ones(11),yaw);
      const_cast<tools::ExtendedKalmanFilter &>(sample.ekf()).x=state;
    }
    assert((sample.ekf().x-before).norm()==0. && (sample.ekf().P-covariance).norm()==0.);
  }
  assert(!base.predict_rigid_armor(-1,frame) && !base.predict_rigid_armor(4,frame));
  assert(!base.predict_rigid_armor(0,frame-microseconds(1)));
  auto malformed=base;
  const_cast<tools::ExtendedKalmanFilter &>(malformed.ekf()).x[0]=NAN;
  assert(!malformed.predict_rigid_armor(0,frame));
  malformed=base;malformed.name=ArmorName::outpost;assert(!malformed.predict_rigid_armor(0,frame));
  Target three(a,frame,.3,3,Eigen::VectorXd::Ones(11),yaw);
  assert(!three.predict_rigid_armor(0,frame));
  auto fixed=base.with_fixed_center_prediction(state);assert(fixed);
  auto full=*fixed;full.predict(frame+milliseconds(500));
  assert((fixed->predict_rigid_armor(1,frame+milliseconds(500))->xyza-
          full.whole_ekf_armor_xyza_list()[1]).norm()<1e-11);
  const auto original_state=base.ekf().x;const auto original_covariance=base.ekf().P;
  Aimer aimer(argv[1]);double old_gap=0.,new_gap=0.;
  for(bool fixed_center:{false,true})for(int sign:{-1,1})
  for(double speed:{5.,18.,24.5,35.})for(int phase=0;phase<180;++phase) {
    auto sample=base;auto & x=const_cast<tools::ExtendedKalmanFilter &>(sample.ekf()).x;
    x[6]=-M_PI+phase*2*M_PI/180.+.00037;x[7]=sign*4.87;
    if(fixed_center) {const auto fixed_target=sample.with_fixed_center_prediction(x);
      assert(fixed_target);sample=*fixed_target;}
    const auto reference=full_reference(sample,frame,speed,high_speed_flight_tolerance_s);
    const auto prior=full_reference(sample,frame,speed,.001);
    AimPredictionTrace trace;
    const auto command=aimer.aim({sample},frame,speed,false,&trace,
                                AimGeometry::whole_ekf,AimTiming::high_speed);
    assert(command.control==reference.valid);
    if(prior.valid)++prior_valid;
    if(command.control && !prior.valid)++became_valid;
    if(!command.control && prior.valid)++became_invalid;
    if(command.control) {
      assert(trace.valid && trace.converged && aimer.debug_aim_point.valid &&
             std::abs(trace.flight_time_s-trace.flight_time_used_s)<high_speed_flight_tolerance_s);
      assert(trace.face_id==reference.face && trace.prediction_ns==reference.ns);
      assert((aimer.debug_aim_point.xyza-reference.point).norm()<1e-10);
      assert(std::abs(command.yaw-reference.yaw)<1e-11);
      assert(std::abs(command.pitch-reference.pitch)<1e-11);
      new_gap=std::max(new_gap,std::abs(trace.flight_time_s-trace.flight_time_used_s));++valid;
    } else {assert(!trace.valid && !aimer.debug_aim_point.valid);++invalid;}
    if(prior.valid)old_gap=std::max(old_gap,prior.gap);
  }
  assert(valid>1400 && invalid>600 && old_gap>0.00005);
  assert(new_gap<high_speed_flight_tolerance_s && new_gap<old_gap*.1);
  assert((base.ekf().x-original_state).norm()==0. &&
         (base.ekf().P-original_covariance).norm()==0.);
  volatile double checksum=0.;std::array<double,2> elapsed{};
  for(int mode=0;mode<2;++mode) {
    const auto begin=steady_clock::now();
    for(int k=0;k<1200;++k) {
      auto sample=*fixed;const_cast<tools::ExtendedKalmanFilter &>(sample.ekf()).x[6]=k*.037;
      if(mode==0) {const auto result=full_reference(sample,frame,24.5,.001);checksum+=result.yaw;}
      else {AimPredictionTrace trace;const auto c=aimer.aim({sample},frame,24.5,false,&trace,
            AimGeometry::whole_ekf,AimTiming::high_speed);checksum+=c.yaw;}
    }
    elapsed[mode]=duration<double,std::micro>(steady_clock::now()-begin).count()/1200.;
  }
  std::cout<<"PASS geometry="<<geometry<<" ballistic_sweeps="<<valid+invalid
    <<" valid="<<valid<<" invalid="<<invalid<<" old_max_gap_us="<<old_gap*1e6
    <<" prior_valid="<<prior_valid<<" gained="<<became_valid<<" lost="<<became_invalid
    <<" new_max_gap_us="<<new_gap*1e6<<" full_reference_us="<<elapsed[0]
    <<" production_us="<<elapsed[1]<<"; numerical checks, not hit rate\n";
}
''', "test_high_speed_ballistics", sources=(
            "tasks/auto_aim/aimer.cpp", "tasks/auto_aim/target.cpp", "tasks/auto_aim/armor.cpp",
            "tools/math_tools.cpp", "tools/trajectory.cpp", "tools/extended_kalman_filter.cpp",
            "tools/logger.cpp"), libraries=("opencv_core", "opencv_imgproc", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"config.yaml": "yaw_offset: 0\npitch_offset: 0\ncomming_angle: 60\n"
                "leaving_angle: 20\nhigh_speed_delay_time: 0.370\nlow_speed_delay_time: 0.330\n"
                "stationary_delay_time: 0.330\ndecision_speed: 0.8\n"}, arguments=("config.yaml",))


    def test_ten_us_boundary_and_exhausted_iterations_keep_legacy_modes(self):
        compile_run(r'''
#include "tasks/auto_aim/aimer.hpp"
#include "tools/trajectory.hpp"
#include <cassert>
#include <iostream>
using namespace auto_aim;
int calls=0;double variation=.00005;
namespace tools {
Trajectory::Trajectory(double,double,double) {
  unsolvable=false;pitch=.1;fly_time=.15+(calls++%2 ? variation : 0.);
}
}
int main(int argc,char **argv) {
  assert(argc==2);const auto frame=std::chrono::steady_clock::time_point(std::chrono::seconds(10));
  Armor armor(9,.95f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
  armor.name=ArmorName::three;armor.type=ArmorType::small;
  armor.xyz_in_world={3.,0.,.05};armor.ypr_in_world={0.,0.,0.};
  YawESKF::Config yaw;yaw.enabled=false;
  Target target(armor,frame,.3,4,Eigen::VectorXd::Ones(11),yaw);
  const_cast<tools::ExtendedKalmanFilter &>(target.ekf()).x[7]=4.87;
  target.jumped=false;
  Aimer aimer(argv[1]);AimPredictionTrace trace;
  calls=0;auto command=aimer.aim({target},frame,23.,false,&trace,
    AimGeometry::whole_ekf,AimTiming::high_speed);
  assert(!command.control && !trace.valid && !trace.converged);
  assert(trace.iterations==10 && !aimer.debug_aim_point.valid);
  for(auto mode:{AimTiming::stationary,AimTiming::low_speed,AimTiming::automatic}) {
    calls=0;command=aimer.aim({target},frame,23.,false,&trace,AimGeometry::whole_ekf,mode);
    assert(command.control && trace.valid && trace.converged && trace.iterations==1);
    assert(std::abs(trace.flight_time_s-trace.flight_time_used_s)>.00001);
    assert(std::abs(trace.flight_time_s-trace.flight_time_used_s)<.001);
  }
  variation=.000009;calls=0;
  command=aimer.aim({target},frame,23.,false,&trace,AimGeometry::whole_ekf,AimTiming::high_speed);
  assert(command.control && trace.converged && trace.iterations==1);
  assert(std::abs(trace.flight_time_s-trace.flight_time_used_s)<high_speed_flight_tolerance_s);
  variation=.00005;target.jumped=true;calls=0;
  command=aimer.aim({target},frame,23.,false,&trace,AimGeometry::whole_ekf,AimTiming::high_speed);
  assert(!command.control && !trace.valid && !aimer.debug_aim_point.valid && calls>=40);
  std::cout<<"PASS 10us: 9us accepted, 50us exhausted and rejected in both high-speed paths; "
    "stationary/low/automatic retain 1ms termination\n";
}
''', "test_high_speed_ballistic_tolerance", sources=(
            "tasks/auto_aim/aimer.cpp", "tasks/auto_aim/target.cpp", "tasks/auto_aim/armor.cpp",
            "tools/math_tools.cpp", "tools/extended_kalman_filter.cpp", "tools/logger.cpp"),
            libraries=("opencv_core", "opencv_imgproc", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"config.yaml": "yaw_offset: 0\npitch_offset: 0\ncomming_angle: 60\n"
                "leaving_angle: 20\nhigh_speed_delay_time: 0.335\nlow_speed_delay_time: 0.320\n"
                "stationary_delay_time: 0.330\ndecision_speed: 0.8\n"}, arguments=("config.yaml",))


if __name__ == "__main__":
    unittest.main()
