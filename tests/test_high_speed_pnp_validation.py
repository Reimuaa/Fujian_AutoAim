"""High-speed result/exception checks and rejection propagation; no devices or PLL."""
import unittest

from test_lightbar_search import compile_run


CALIBRATION = (
    "R_gimbal2imubody: [1,0,0,0,1,0,0,0,1]\n"
    "R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0]\n"
    "t_camera2gimbal: [0.02,-0.01,0.005]\n"
    "camera_matrix: [1500,0,720,0,1490,540,0,0,1]\n"
    "distort_coeffs: [0.03,-0.01,0.0002,-0.0001,0.001]\n"
    "enemy_color: blue\nmin_detect_count: 1\nmax_temp_lost_count: 15\n"
    "outpost_max_temp_lost_count: 75\nyaw_eskf_enabled: false\n"
    "tracker_identity_fallback_enabled: true\n"
)


class HighSpeedPnpValidationTests(unittest.TestCase):
    def test_solver_failure_results_depth_and_exception_boundary(self):
        compile_run(r'''
#include "tasks/auto_aim/solver.hpp"
#include <cassert>
#include <iostream>
#include <limits>
int scenario=0;
namespace cv {
bool reviewSolvePnP(InputArray,InputArray,InputArray,InputArray,
                   OutputArray rvec,OutputArray tvec,bool,int) {
  if(scenario==1) return false;
  if(scenario==6) CV_Error(Error::StsError,"injected PnP exception");
  Mat r=Mat::zeros(3,1,CV_64F),t=Mat::zeros(3,1,CV_64F);
  t.at<double>(2)=3.;
  if(scenario==2) r.at<double>(1)=std::numeric_limits<double>::quiet_NaN();
  if(scenario==3) t.at<double>(0)=std::numeric_limits<double>::infinity();
  if(scenario==4) t.at<double>(2)=-3.;
  // Positive centre depth is insufficient: two corners are behind the camera.
  if(scenario==5) {r.at<double>(0)=CV_PI/2.;t.at<double>(2)=.01;}
  r.copyTo(rvec);t.copyTo(tvec);return true;
}
}
#define solvePnP reviewSolvePnP
#include "tasks/auto_aim/solver.cpp"
#undef solvePnP
int main(int argc,char **argv) {
  assert(argc==2);auto_aim::Solver solver(argv[1]);
  auto_aim::Armor a(9,.95f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
  a.type=auto_aim::ArmorType::small;a.name=auto_aim::ArmorName::three;
  assert(solver.solve(a,true));
  for(scenario=1;scenario<=6;++scenario) assert(!solver.solve(a,true));
  scenario=6;bool propagated=false;
  try {solver.solve(a);} catch(const cv::Exception &) {propagated=true;}
  assert(propagated); // The non-high exception contract is unchanged.
  scenario=0;assert(solver.solve(a,true));assert(solver.solve(a));
  std::cout<<"PASS production PnP: failed return, NaN/Inf, negative/all-corner depth, "
    "OpenCV exception, legacy propagation and recovery\n";
}
''', "test_high_speed_pnp_failure_paths", sources=("tasks/auto_aim/armor.cpp",
            "tools/math_tools.cpp", "tools/logger.cpp"),
            libraries=("opencv_core", "opencv_imgproc", "opencv_calib3d", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"pose.yaml": CALIBRATION}, arguments=("pose.yaml",))

    def test_real_tracker_discards_failed_initialization_and_association(self):
        compile_run(r'''
#include "tasks/auto_aim/tracker.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace auto_aim;
int main(int argc,char **argv) {
  assert(argc==2);Solver solver(argv[1]);Tracker tracker(argv[1],solver,false);
  tracker.set_high_speed_batch_projection(true);
  const auto points=solver.reproject_armor({3.,0.,.08},.2,ArmorType::small,ArmorName::three);
  auto stamp=std::chrono::steady_clock::now();
  const auto track=[&](Armor a) {
    a.color=Color::blue;a.type=ArmorType::small;
    if(a.name!=ArmorName::four) a.name=ArmorName::three;
    std::list<Armor> inputs{a};stamp+=std::chrono::milliseconds(10);
    return tracker.track(inputs,stamp);
  };
  for(int kind=0;kind<5;++kind) {
    Armor bad(9,.95f,cv::Rect(0,0,100,40),points);
    if(kind==0) bad.points.clear();
    if(kind==1) bad.points.pop_back();
    if(kind==2) bad.points.push_back(bad.points.front());
    if(kind==3) bad.points[0].x=std::numeric_limits<float>::quiet_NaN();
    if(kind==4) bad.points[1].y=std::numeric_limits<float>::infinity();
    assert(!solver.solve(bad,true));assert(track(bad).empty());
    assert(tracker.state()=="lost" && !tracker.frame_diagnostics().initialized_this_frame);
    assert(tracker.frame_diagnostics().high_speed_pose_rejections==1);
  }
  Armor valid(9,.95f,cv::Rect(0,0,100,40),points);valid.name=ArmorName::three;
  solver.set_R_gimbal2world(Eigen::Quaterniond(std::numeric_limits<double>::quiet_NaN(),0,0,0));
  assert(!solver.solve(valid,true));
  solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());assert(solver.solve(valid,true));
  for(int k=0;k<7;++k) assert(!track(valid).empty());
  auto mature=track(valid);assert(!mature.empty() && mature.front().convergened());
  for(bool fallback:{false,true}) {
    auto bad=valid;bad.points[0].x=std::numeric_limits<float>::quiet_NaN();
    if(fallback) bad.name=ArmorName::four;
    track(bad);
    assert(tracker.frame_diagnostics().high_speed_pose_rejections==1);
    assert(tracker.frame_diagnostics().update_observations==0 && tracker.last_associations().empty());
    assert(!track(valid).empty());
    assert(tracker.frame_diagnostics().high_speed_pose_rejections==0);
    assert(tracker.frame_diagnostics().update_observations==1);
  }
  std::cout<<"PASS real high-speed Solver/Tracker: invalid input/transform, rejected "
    "initialization, mature normal/fallback association and recovery\n";
}
''', "test_high_speed_pnp_tracker", sources=("tasks/auto_aim/solver.cpp",
            "tasks/auto_aim/tracker.cpp", "tasks/auto_aim/target.cpp",
            "tools/extended_kalman_filter.cpp", "tasks/auto_aim/armor.cpp",
            "tools/math_tools.cpp", "tools/logger.cpp"),
            libraries=("opencv_core", "opencv_imgproc", "opencv_calib3d", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"pose.yaml": CALIBRATION}, arguments=("pose.yaml",))


if __name__ == "__main__":
    unittest.main()
