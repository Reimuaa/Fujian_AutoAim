"""Exercise real Aimer direction profiles and production qualification resets offline."""
from pathlib import Path
import unittest

from test_lightbar_search import compile_run

ROOT = Path(__file__).resolve().parents[1]


class HighSpeedDirectionalDelayTests(unittest.TestCase):
    def test_profiles_predictions_fallback_and_mode_isolation(self):
        standard = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        window = standard[standard.index("struct RotationFireWindow"):
                          standard.index("// A coordinate hypothesis")]
        reset = standard[standard.index("    // A changed high-speed direction"):
                         standard.index("    const Eigen::VectorXd fire_prediction_state =")]
        metadata = "\n".join(
            standard[standard.index(statement):standard.index(";", standard.index(statement)) + 1]
            for statement in ("fixed_center_residual_input.delay_s =",
                              "fixed_center_residual_input.speed_mps ="))
        code = r'''
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/face_prediction_residual.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <stdexcept>
using namespace auto_aim;
using namespace std::chrono;
WINDOW
void check_qualification_reset() {
  RotationFireWindow rotation_fire_window;
  bool high_speed_mode=true,gyro_model_fresh=false;
  bool xuc_fire_candidate_active=true,xuc_fire_ready_logged=true;
  int xuc_fire_candidate_frames=4,high_speed_delay_rotation_sign=0;
  AimPredictionTrace native_fire_prediction;
  const auto seed=[&] {
    rotation_fire_window.frames=2;xuc_fire_candidate_active=true;
    xuc_fire_ready_logged=true;xuc_fire_candidate_frames=4;
  };
  const auto update=[&] { RESET };
  const auto cleared=[&] {
    return rotation_fire_window.frames==0 && !xuc_fire_candidate_active &&
      !xuc_fire_ready_logged && xuc_fire_candidate_frames==0;
  };
  native_fire_prediction.high_speed_rotation_sign=-1;seed();update();assert(cleared());
  seed();update();assert(rotation_fire_window.frames==2 && xuc_fire_candidate_frames==4);
  native_fire_prediction.high_speed_rotation_sign=1;update();assert(cleared());
  native_fire_prediction.high_speed_rotation_sign=0;seed();update();assert(cleared());
  high_speed_mode=false;seed();update();
  assert(rotation_fire_window.frames==2 && xuc_fire_candidate_frames==4);
  high_speed_mode=true;native_fire_prediction.high_speed_rotation_sign=-1;update();assert(cleared());
  gyro_model_fresh=true;seed();native_fire_prediction.high_speed_rotation_sign=1;update();
  assert(rotation_fire_window.frames==2 && xuc_fire_candidate_frames==4 &&
         high_speed_delay_rotation_sign==0);
  FacePredictionResidual::Input fixed_center_residual_input;
  fixed_center_residual_input.delay_s=.370;fixed_center_residual_input.speed_mps=20.;
  AimPredictionTrace fixed_center_prediction;
  fixed_center_prediction.delay_s=.355;fixed_center_prediction.speed_mps=23.;
  METADATA
  assert(fixed_center_residual_input.delay_s==.355 &&
         fixed_center_residual_input.speed_mps==23.);
}
bool same(const io::Command & a,const io::Command & b,
          const AimPredictionTrace & x,const AimPredictionTrace & y) {
  return a.control==b.control && a.shoot==b.shoot && a.yaw==b.yaw && a.pitch==b.pitch &&
    x.valid==y.valid && x.converged==y.converged && x.face_id==y.face_id &&
    x.base_prediction_ns==y.base_prediction_ns && x.prediction_ns==y.prediction_ns &&
    x.delay_s==y.delay_s && x.speed_mps==y.speed_mps;
}
int main(int argc,char ** argv) {
  assert(argc==11);
  Aimer legacy(argv[1]),split(argv[2]),cw_reference(argv[3]),ccw_reference(argv[4]);
  Aimer cw_changed(argv[5]),ccw_changed(argv[6]),cw_only(argv[7]),formal(argv[10]);
  // Field tuning is independent of the controlled CW370/CCW355 fixtures below.
  const auto formal_config=YAML::LoadFile(argv[10]);
  const double formal_fallback=formal_config["high_speed_delay_time"].as<double>();
  const double formal_cw=formal_config["high_speed_delay_time_cw"]
    ? formal_config["high_speed_delay_time_cw"].as<double>() : formal_fallback;
  const double formal_ccw=formal_config["high_speed_delay_time_ccw"]
    ? formal_config["high_speed_delay_time_ccw"].as<double>() : formal_fallback;
  const auto frame=steady_clock::time_point(seconds(10));
  Armor armor(9,.95f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
  armor.name=ArmorName::three;armor.type=ArmorType::small;
  armor.xyz_in_world={3.,0.,.05};armor.ypr_in_world={0.,0.,0.};
  YawESKF::Config yaw;yaw.enabled=false;
  Target initial(armor,frame,.3,4,Eigen::VectorXd::Ones(11),yaw);
  int cases=0,valid_cw=0,valid_ccw=0,isolated=0;
  for(bool fixed:{false,true})for(bool jumped:{false,true})for(int sign:{-1,1})
  for(int phase=0;phase<36;++phase) {
    auto target=initial;
    auto & state=const_cast<tools::ExtendedKalmanFilter &>(target.ekf()).x;
    state[6]=-M_PI+phase*2*M_PI/36.;state[7]=sign*4.6;
    if(fixed) {const auto candidate=target.with_fixed_center_prediction(state);
      assert(candidate);target=*candidate;}
    target.jumped=jumped;
    const auto saved=target.ekf().x;
    const auto covariance=target.ekf().P;
    AimPredictionTrace a,b,c,d;
    auto & reference=sign<0 ? cw_reference : ccw_reference;
    const auto actual=split.aim({target},frame,23.,false,&a,
      AimGeometry::whole_ekf,AimTiming::high_speed);
    const auto expected=reference.aim({target},frame,23.,false,&b,
      AimGeometry::whole_ekf,AimTiming::high_speed);
    assert(same(actual,expected,a,b));
    assert(a.delay_s==(sign<0 ? .370 : .355) && a.high_speed_rotation_sign==sign);
    if(actual.control) {if(sign<0)++valid_cw;else ++valid_ccw;}
    const auto only_cw=cw_changed.aim({target},frame,23.,false,&c,
      AimGeometry::whole_ekf,AimTiming::high_speed);
    const auto only_ccw=ccw_changed.aim({target},frame,23.,false,&d,
      AimGeometry::whole_ekf,AimTiming::high_speed);
    assert(c.delay_s==(sign<0 ? .390 : .355));
    assert(d.delay_s==(sign<0 ? .370 : .375));
    if(sign>0)assert(same(actual,only_cw,a,c));
    if(sign<0)assert(same(actual,only_ccw,a,d));
    for(auto mode:{AimTiming::stationary,AimTiming::low_speed,AimTiming::automatic}) {
      const auto original=legacy.aim({target},frame,23.,false,&a,AimGeometry::whole_ekf,mode);
      const auto unchanged=cw_changed.aim({target},frame,23.,false,&b,AimGeometry::whole_ekf,mode);
      assert(same(original,unchanged,a,b) && b.high_speed_rotation_sign==0);++isolated;
    }
    assert((target.ekf().x-saved).norm()==0. && (target.ekf().P-covariance).norm()==0.);
    ++cases;
  }
  assert(cases==288 && valid_cw>20 && valid_ccw>20 && isolated==864);
  auto target=initial;
  auto & state=const_cast<tools::ExtendedKalmanFilter &>(target.ekf()).x;
  for(double omega:{-4.6,4.6,0.,-4.6,4.6}) {
    state[7]=omega;AimPredictionTrace a,b;
    const int sign=omega>0 ? 1 : (omega<0 ? -1 : 0);
    cw_only.aim({target},frame,23.,false,&a,AimGeometry::existing,AimTiming::high_speed);
    assert(a.delay_s==(sign<0 ? .370 : .365) && a.high_speed_rotation_sign==sign);
    formal.aim({target},frame,23.,false,&a,AimGeometry::whole_ekf,AimTiming::high_speed);
    assert(a.delay_s==(sign<0 ? formal_cw : (sign>0 ? formal_ccw : formal_fallback)));
    split.set_delay_adjustment(.010,AimTiming::high_speed);
    split.aim({target},frame,23.,false,&a,AimGeometry::whole_ekf,AimTiming::high_speed);
    assert(std::abs(a.delay_s-((sign<0 ? .370 : (sign>0 ? .355 : .365))+.010))<1e-12);
    split.aim({target},frame,23.,false,&a,AimGeometry::whole_ekf,AimTiming::low_speed);
    assert(a.delay_s==.320 && a.high_speed_rotation_sign==0);
    split.set_delay_adjustment(0.,AimTiming::high_speed);
    const auto old=legacy.aim({target},frame,23.,false,&a,AimGeometry::whole_ekf,AimTiming::high_speed);
    auto & reference=omega>0 ? ccw_reference : cw_reference;
    if(omega==0.) {
      const auto fallback=split.aim({target},frame,23.,false,&b,AimGeometry::whole_ekf,AimTiming::high_speed);
      assert(same(old,fallback,a,b) && b.high_speed_rotation_sign==0);
    }
    (void)reference;
  }
  for(int bad:{8,9}) {
    bool rejected=false;try {Aimer invalid(argv[bad]);}
    catch(const std::invalid_argument &) {rejected=true;}
    assert(rejected);
  }
  const auto now=steady_clock::now();
  Target live(armor,now,.3,4,Eigen::VectorXd::Ones(11),yaw);
  const_cast<tools::ExtendedKalmanFilter &>(live.ekf()).x[7]=-4.6;
  AimPredictionTrace trace;const auto begin=steady_clock::now();
  split.aim({live},now,23.,true,&trace,AimGeometry::whole_ekf,AimTiming::high_speed);
  const auto end=steady_clock::now();
  assert(trace.delay_s==.370 && trace.high_speed_rotation_sign==-1);
  const auto base=steady_clock::time_point(nanoseconds(trace.base_prediction_ns))-milliseconds(370);
  assert(base>=begin-microseconds(1) && base<=end);
  check_qualification_reset();
  std::cout<<"PASS configured production profile, fixtures CW370/CCW355: "<<cases<<" tracker/fixed-centre comparisons, "<<isolated
    <<" exact stationary/low/automatic checks, one-direction isolation, optional-key fallback, "
      "mode corrections, live-clock base, invalid values and production reversal/PLL isolation\n";
}
'''
        code = code.replace("WINDOW", window).replace("RESET", reset).replace("METADATA", metadata)
        common = ("yaw_offset: 0\npitch_offset: 0\ncomming_angle: 60\nleaving_angle: 20\n"
                  "high_speed_delay_time: 0.365\nlow_speed_delay_time: 0.320\n"
                  "stationary_delay_time: 0.330\ndecision_speed: 0.8\n")
        split = common + "high_speed_delay_time_cw: 0.370\nhigh_speed_delay_time_ccw: 0.355\n"
        compile_run(code, "test_high_speed_directional_delay", sources=(
            "tasks/auto_aim/aimer.cpp", "tasks/auto_aim/target.cpp", "tasks/auto_aim/armor.cpp",
            "tools/math_tools.cpp", "tools/trajectory.cpp", "tools/extended_kalman_filter.cpp",
            "tools/logger.cpp"), libraries=("opencv_core", "opencv_imgproc", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"legacy.yaml": common, "split.yaml": split,
                      "cw_reference.yaml": common.replace("0.365", "0.370"),
                      "ccw_reference.yaml": common.replace("0.365", "0.355"),
                      "cw_changed.yaml": split.replace("time_cw: 0.370", "time_cw: 0.390"),
                      "ccw_changed.yaml": split.replace("time_ccw: 0.355", "time_ccw: 0.375"),
                      "cw_only.yaml": common + "high_speed_delay_time_cw: 0.370\n",
                      "bad_cw.yaml": split.replace("time_cw: 0.370", "time_cw: -0.001"),
                      "bad_ccw.yaml": split.replace("time_ccw: 0.355", "time_ccw: .nan")},
            arguments=("legacy.yaml", "split.yaml", "cw_reference.yaml", "ccw_reference.yaml",
                       "cw_changed.yaml", "ccw_changed.yaml", "cw_only.yaml", "bad_cw.yaml",
                       "bad_ccw.yaml", str(ROOT / "configs/standard.yaml")))


if __name__ == "__main__":
    unittest.main()
