"""Production high-speed qualification and independent fixed-centre tests; no hardware."""
from pathlib import Path
import unittest

from test_lightbar_search import compile_run
from test_face_prediction_residual import DRIVER

ROOT = Path(__file__).resolve().parents[1]


class HighSpeedFireQualityTests(unittest.TestCase):
    def test_high_speed_pitch_trim_is_additive_and_keeps_non_high_unchanged(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")

        def statement(start):
            begin = source.index(start)
            return source[begin:source.index(";", begin) + 1]

        code = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <yaml-cpp/yaml.h>
double compensate(bool high_speed_mode,const char * yaml,double model_deg,double common_deg=0.) {
  const auto integration_yaml=YAML::Load(yaml);
  const double rad=M_PI/180.;
  const double xuc_ballistic_pitch_trim_rad=common_deg*rad;
CONFIG
SELECTION
  const double fixed_boresight_bias_rad=0.,camera_los_up_rad=0.;
  const double xuc_ballistic_max_compensation_rad=20.*rad;
  struct { double pitch; } current_trajectory{model_deg*rad};
  double ballistic_pitch_compensation_rad=0.;
COMPENSATION
  return ballistic_pitch_compensation_rad/rad;
}
int main() {
  const char * trim="xuc_high_speed_ballistic_pitch_trim_deg: -0.5";
  for(double model:{1.,3.,8.})for(double common:{-.2,0.,.3}) {
    const auto baseline=compensate(false,"{}",model,common);
    assert(compensate(false,trim,model,common)==baseline);
    assert(compensate(true,"{}",model,common)==baseline);
    assert(std::abs(compensate(true,trim,model,common)-(baseline-.5))<1e-12);
  }
  assert(compensate(true,trim,.2)==0.); // Existing total-compensation bounds remain.
  assert(std::abs(compensate(true,trim,25.)-20.)<1e-12);
  assert(std::abs(compensate(true,"xuc_high_speed_ballistic_pitch_trim_deg: 999",2.)-7.)<1e-12);
  std::cout<<"PASS production high-speed -0.5 degree pitch trim: additive, missing-key "
    "compatibility, exact non-high isolation and original compensation limits\n";
}
'''
        replacements = {
            "CONFIG": statement("  const double xuc_high_speed_ballistic_pitch_trim_rad ="),
            "SELECTION": statement("    const double effective_ballistic_pitch_trim_rad ="),
            "COMPENSATION": statement("              ballistic_pitch_compensation_rad = std::clamp("),
        }
        code = "\n".join(replacements.get(line, line) for line in code.splitlines())
        compile_run(code, "test_high_speed_pitch_trim", libraries=("yaml-cpp",))

    def test_rotation_window_requires_history_and_preserves_other_gates(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")

        def statement(start):
            begin = source.index(start)
            return source[begin:source.index(";", begin) + 1]

        begin = source.index("struct RotationFireWindow")
        window = source[begin:source.index("\n};", begin) + 3]
        code = r'''
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
WINDOW
struct Result { bool shoot, stable, point; };
Result qualify(RotationFireWindow & rotation_fire_window, int seq,
  unsigned blocked=0, bool valid=true, int face=0, double phase=0., double error=0.) {
  const auto t=std::chrono::steady_clock::time_point(std::chrono::milliseconds(20*seq));
  rotation_fire_window.update(valid,false,3,1,4,face,false,phase,error,t,2,
    20.*M_PI/180.,.25*M_PI/180.,seq,0.,4.5,60./57.3,20./57.3);
  auto passes=[&](int bit) { return !(blocked & (1u<<bit)); };
  struct { bool inhibit_shoot,control,shoot=false; } command{!passes(0),passes(1)};
  const bool auto_mode_active=passes(2),convergence_gate_open=passes(3);
  const bool one_face_gate_open=passes(4),motion_transition_gate_open=passes(5);
  const bool command_cadence_gate_open=passes(6),direct_yaw_measurement_fresh=passes(7);
  const bool direct_pitch_measurement_fresh=passes(8),yaw_gate_open=passes(9);
  const double direct_pitch_pixel_error_rad=passes(10)?0.:1.,xuc_fire_pitch_tolerance_rad=.02;
  struct { bool open; } high_speed_pitch_quality{passes(11)};
  const bool high_speed_prediction_quality_open=passes(12),prediction_yaw_gate_open=passes(13);
  const bool high_speed_prediction_current_valid=passes(14);
  const bool gyro_event_schedule_valid=false;
CANDIDATE
POINT
  bool stable_window_open=false;
STABLE
REQUEST
  return {command.shoot,stable_window_open,prediction_point_gate_open};
}
int main() {
  RotationFireWindow initial;
  assert(!qualify(initial,1).shoot);
  assert(qualify(initial,2).shoot);
  for(int gate=0;gate<15;++gate) {
    RotationFireWindow w;
    assert(!qualify(w,1).shoot);
    assert(!qualify(w,2,1u<<gate).shoot);
  }
  RotationFireWindow w;
  assert(!qualify(w,1).shoot);
  assert(qualify(w,2).shoot);
  assert(!qualify(w,3,0,false).shoot); // Invalid observations clear history.
  assert(!qualify(w,4,0,true,-1).shoot);
  assert(!qualify(w,5,0,true,0,NAN).shoot);
  assert(!qualify(w,6,0,true,0,0.,NAN).shoot);
  assert(!qualify(w,7,0,true,1).shoot); // A new face must rebuild two-frame history.
  assert(qualify(w,8,0,true,1).shoot);
  RotationFireWindow point_window;
  const double rad=M_PI/180.;
  assert(!qualify(point_window,1,0,true,0,0.,.2*rad).shoot);
  const auto waiting=qualify(point_window,2,0,true,0,0.,.1*rad);
  assert(waiting.stable && !waiting.point && !waiting.shoot);
  assert(qualify(point_window,3,0,true,0,0.,0.).shoot);
  std::cout<<"PASS production fixed rotation window: two-frame and best-frame waits, "
    "invalid observations/face reset and 15 independent fire vetoes\n";
}
'''
        replacements = {
            "WINDOW": window,
            "CANDIDATE": statement("      const bool fire_candidate ="),
            "POINT": statement("      const bool prediction_point_gate_open ="),
            "STABLE": statement("        stable_window_open = rotation_fire_window.two_frames_ready;"),
            "REQUEST": statement("        command.shoot = stable_window_open && fire_candidate &&"),
        }
        code = "\n".join(replacements.get(line, line) for line in code.splitlines())
        compile_run(code, "test_high_speed_rotation_window")

    def test_pitch_dwell_boundaries_and_past_only_forecast_quality(self):
        prefix = DRIVER[:DRIVER.index("void tests()")]
        compile_run(prefix + r'''
#include "tasks/auto_aim/high_speed_fire_quality.hpp"
int main() {
  using Gate=auto_aim::HighSpeedPitchGate;
  Gate::Config cfg;cfg.enabled=true;
  Gate gate(cfg);
  Gate::Input in{1000000000,1,true,true,.4*rad,-.55*rad};
  assert(!gate.update(in).open);
  in.frame_ns+=17000000;assert(!gate.update(in).open);
  in.frame_ns+=17000000;assert(gate.update(in).open); // 34ms, not an exact 30ms period.
  for(int i=0;i<10;++i){in.frame_ns+=17000000;assert(gate.update(in).open);}
  in.frame_ns+=17000000;in.residual_rad=.450001*rad;assert(!gate.update(in).open);
  in.residual_rad=.45*rad;in.tracking_error_rad=.8*rad;
  in.frame_ns+=17000000;assert(!gate.update(in).open);
  in.frame_ns+=17000000;assert(!gate.update(in).open);
  in.frame_ns+=17000000;assert(gate.update(in).open); // Inclusive limits.
  in.frame_ns+=17000000;in.tracking_error_rad=.80001*rad;assert(!gate.update(in).open);
  in.frame_ns+=17000000;in.tracking_error_rad=.55*rad;assert(!gate.update(in).open);
  in.frame_ns+=17000000;in.residual_rad=.1*rad;assert(!gate.update(in).open); // Unsettled.
  in.frame_ns+=17000000;in.residual_rad=.4*rad;assert(!gate.update(in).open);
  in.frame_ns+=17000000;assert(!gate.update(in).open);
  in.frame_ns+=17000000;assert(gate.update(in).open);
  in.epoch=2;in.frame_ns+=17000000;assert(!gate.update(in).open);
  assert(!gate.update(in).open); // Duplicate frame cannot complete dwell.
  in.frame_ns+=101000000;assert(!gate.update(in).open);
  in.high_speed=false;in.residual_rad=NAN;in.context_valid=false;
  auto out=gate.update(in);assert(out.open && !out.active); // Exact non-high bypass.
  in.high_speed=true;in.context_valid=true;in.residual_rad=.3*rad;
  in.frame_ns+=17000000;assert(!gate.update(in).open);
  in.frame_ns+=17000000;in.context_valid=false;assert(!gate.update(in).open);
  Gate disabled;assert(disabled.update(in).open);
  assert(auto_aim::past_forecast_quality_allows(false,true,false,NAN,rad));
  assert(!auto_aim::past_forecast_quality_allows(true,false,false,NAN,rad));
  assert(!auto_aim::past_forecast_quality_allows(true,false,true,0.,rad)); // Finite is not mature.
  assert(!auto_aim::past_forecast_quality_allows(true,true,true,NAN,rad));
  assert(!auto_aim::past_forecast_quality_allows(true,true,true,1.01*rad,rad));
  assert(auto_aim::past_forecast_quality_allows(true,true,true,rad,rad));
  assert(!auto_aim::past_forecast_quality_allows(true,true,false,0.,rad));
  assert(!auto_aim::past_forecast_quality_allows(true,true,true,-rad,rad));
  assert(!auto_aim::past_forecast_quality_allows(true,true,true,0.,NAN));
  assert(!auto_aim::past_forecast_quality_allows(true,true,true,0.,0.));
  for(bool ready:{false,true})for(bool eligible:{false,true})
    assert(auto_aim::past_forecast_quality_allows(false,ready,eligible,NAN,NAN));

  auto permits=[](const Monitor::Output & out) {
    return auto_aim::past_forecast_quality_allows(
      true,out.quality_ready,out.quality_eligible,out.past_error_bound_rad,.45*rad);
  };

  auto run=[&](double raw) {
    Monitor m({true});Monitor::Output out;
    for(int i=0;i<=400;++i){auto f=input(i);f.forecast_alignment_rad=0.;
      f.raw_yaw_rad=raw*rad;out=m.update(f);
      if(i<356){assert(!out.quality_ready);assert(!permits(out));}
    }
    assert(out.quality_ready && out.quality_cycles==3);
    return std::make_pair(m,out);
  };
  auto good=run(.1);assert(good.second.past_error_bound_rad<.11*rad && permits(good.second));
  auto bad=run(3.5); // Large residuals excluded from bias training still count against quality.
  assert(bad.second.past_error_bound_rad>3.4*rad && !bad.second.ready && !permits(bad.second));
  auto f=input(401);f.forecast_alignment_rad=0.;f.epoch=2;
  auto reset=good.first.update(f);assert(!reset.quality_ready && !permits(reset));
  f=input(402);f.forecast_alignment_rad=0.;f.epoch=2;
  assert(!permits(good.first.update(f))); // A valid new frame cannot reuse the old epoch.
  Monitor a({true}),b({true});
  for(int i=0;i<=650;++i){auto x=input(i),y=x;x.forecast_alignment_rad=y.forecast_alignment_rad=0.;
    if(i>=469 && y.observations[0])y.observations[0]=std::array<double,2>{5.,.5};
    auto aa=a.update(x),bb=b.update(y);
    if(i<469){assert(aa.quality_cycles==bb.quality_cycles);
      if(aa.quality_ready)assert(aa.past_error_bound_rad==bb.past_error_bound_rad);}
  }
  std::cout<<"PASS high-speed pitch boundaries, non-high bypass, causal independent-pass quality\n";
}
''', "test_high_speed_fire_quality", libraries=("yaml-cpp",))

    def test_standard_quality_wiring_keeps_non_high_pll_and_disabled_bypass(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        begin = source.index("      high_speed_prediction_quality_active = high_speed_mode &&")
        wiring = source[begin:source.index("      const bool fire_candidate", begin)]
        compile_run(r'''
#include "tasks/auto_aim/face_prediction_residual.hpp"
#include "tasks/auto_aim/high_speed_fire_quality.hpp"
#include <cassert>
#include <iostream>
int main() {
  for(bool high_speed_mode:{false,true})
    for(bool high_speed_prediction_quality_enabled:{false,true})
      for(bool gyro_model_fresh:{false,true})
        for(int history=0;history<6;++history) {
          auto_aim::FacePredictionResidual::Output face_residual;
          face_residual.quality_ready=history>0;
          face_residual.eligible=history!=2 && history!=4 && history!=5;
          face_residual.quality_eligible=history!=2 && history!=4;
          face_residual.quality_retained=history==4;
          face_residual.past_error_bound_rad=history==3?.46*M_PI/180.:.1*M_PI/180.;
          double high_speed_prediction_quality_limit_rad=.45*M_PI/180.;
          bool high_speed_prediction_quality_active=false;
          bool high_speed_prediction_quality_open=false;
''' + wiring + r'''
          if(!high_speed_mode || !high_speed_prediction_quality_enabled || gyro_model_fresh)
            assert(high_speed_prediction_quality_open);
          else
            assert(high_speed_prediction_quality_open==(history==1 || history==4 || history==5));
          const bool active=high_speed_mode && high_speed_prediction_quality_enabled && !gyro_model_fresh;
          assert(high_speed_prediction_current_valid==(!active || (history!=2 && history!=4 && history!=5)));
          if(active && (history==4 || history==5))
            assert(!high_speed_prediction_current_valid); // Held or no current accepted observation.
        }
  std::cout<<"PASS production standard wiring: high-speed maturity; non-high/PLL/disabled bypass\n";
}
''', "test_high_speed_quality_wiring")

    def test_joint_fixed_center_and_prediction_value_isolation(self):
        compile_run(r'''
#include "tasks/auto_aim/fixed_center_model.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include <cassert>
#include <iostream>
using namespace std::chrono;
auto_aim::FixedCenterModel::Input frame(int i,int sign) {
  auto_aim::FixedCenterModel::Input in;
  in.active=in.context_valid=true;in.epoch=1;in.frame_ns=1000000000LL+i*20000000LL;
  const double t=i*.02,phase=sign*4.5*t;
  in.seed.resize(11);in.seed<<3.67,.1,.10,-.1,.05,.02,std::remainder(phase+.08,2*M_PI),
    sign*4.6,.29,.015,0.;
  int face=static_cast<int>(std::round(-phase/(M_PI/2)))%4;if(face<0)face+=4;
  const double a=phase+face*M_PI/2,r=face%2?.31:.30;
  Eigen::Vector3d xyz{3.6-r*std::cos(a),.05-r*std::sin(a),face%2?.052:.05};
  const Eigen::Vector2d u=xyz.head<2>().normalized();
  xyz.head<2>()+=.02*std::sin(i*31.)*u+.001*std::sin(i*7.)*Eigen::Vector2d(-u.y(),u.x());
  in.observations.push_back({face,xyz});return in;
}
int main(int argc,char **argv) {
  int ready=0;
  for(int sign:{-1,1}) {
    auto_aim::FixedCenterModel model;
    for(int i=0;i<420;++i){auto in=frame(i,sign);auto out=model.update(in);
      if(i<135)assert(!out.ready);
      if(out.ready){++ready;assert(out.state[1]==0 && out.state[3]==0 && out.state[5]==0);
        assert(std::hypot(out.state[0]-3.6,out.state[2]-.05)<.02);
        assert(std::abs(out.state[7]-sign*4.5)<.03);
        assert(out.heldout_yaw_error_rad<.1*M_PI/180.);}
    }
    auto in=frame(420,sign);in.epoch=2;assert(!model.update(in).ready);
    for(int i=421;i<430;++i){in=frame(i,sign);in.active=false;assert(!model.update(in).ready);}
    in=frame(430,sign);assert(!model.update(in).ready);
  }
  assert(ready>300);
  auto_aim::FixedCenterModel m;
  for(int i=0;i<200;++i)m.update(frame(i,1));
  auto blind=frame(200,1);blind.observations.clear();assert(!m.update(blind).ready);
  blind=frame(211,1);assert(!m.update(blind).ready); // >200ms observation loss.
  auto_aim::Armor a(9,.95f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
  a.name=auto_aim::ArmorName::three;a.type=auto_aim::ArmorType::small;
  a.priority=auto_aim::ArmorPriority::first;a.xyz_in_world={3.3,0,.05};a.ypr_in_world={0,0,0};
  Eigen::VectorXd p(11);p.setOnes();auto_aim::YawESKF::Config yaw;yaw.enabled=false;
  const auto t=steady_clock::now();auto_aim::Target base(a,t,.3,4,p,yaw);
  const auto before=base.ekf().x;
  const auto covariance=base.ekf().P;
  auto state=before;state[1]=.2;state[3]=-.1;state[7]=4.5;
  auto fixed=base.with_fixed_center_prediction(state);assert(fixed);
  fixed->predict(t+milliseconds(600));assert(fixed->ekf().x[0]==state[0]);
  assert(fixed->ekf().x[2]==state[2] && fixed->ekf().x[1]==0);
  assert(std::abs(std::remainder(fixed->ekf().x[6]-state[6]-2.7,2*M_PI))<1e-12);
  assert((base.ekf().x-before).norm()==0 && (base.ekf().P-covariance).norm()==0);
  yaw.enabled=true;auto_aim::Target legacy(a,t,.3,4,p,yaw);
  assert(!legacy.with_fixed_center_prediction(state));
  auto_aim::Aimer native(argv[1]),shadow(argv[1]);auto_aim::AimPredictionTrace trace;
  const auto predicted=base.with_fixed_center_prediction(state);
  const auto c=shadow.aim({*predicted},t,20.,false,&trace,auto_aim::AimGeometry::whole_ekf,
    auto_aim::AimTiming::high_speed);
  assert(c.control && trace.valid && trace.whole_ekf_geometry && trace.flight_time_s>0);
  assert((base.ekf().x-before).norm()==0);
  std::cout<<"PASS joint centre/radii/phase in both directions; shadow Target/Aimer isolation\n";
}
''', "test_fixed_center_model", sources=(
            "tasks/auto_aim/aimer.cpp", "tasks/auto_aim/target.cpp", "tasks/auto_aim/armor.cpp",
            "tools/math_tools.cpp", "tools/trajectory.cpp", "tools/extended_kalman_filter.cpp",
            "tools/logger.cpp"), libraries=("opencv_core", "opencv_imgproc", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"config.yaml": "yaw_offset: 0\npitch_offset: 0\ncomming_angle: 60\n"
                      "leaving_angle: 20\nhigh_speed_delay_time: 0.340\nlow_speed_delay_time: 0.330\n"
                      "stationary_delay_time: 0.330\ndecision_speed: 0.8\n"}, arguments=("config.yaml",))


if __name__ == "__main__":
    unittest.main()
