"""Compare production batch projection with the unchanged one-degree solver; no hardware."""
import unittest
from pathlib import Path

from test_lightbar_search import compile_run


class BatchedYawProjectionTests(unittest.TestCase):
    def test_original_grid_pose_and_excluded_balance_are_preserved(self):
        calibration = (
            "R_gimbal2imubody: [1,0,0,0,1,0,0,0,1]\n"
            "R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0]\n"
            "t_camera2gimbal: [0.02,-0.01,0.005]\n"
            "camera_matrix: [1500,0,720,0,1490,540,0,0,1]\n"
            "distort_coeffs: [0.03,-0.01,0.0002,-0.0001,0.001]\n"
        )
        keys = {"R_gimbal2imubody", "R_camera2gimbal", "t_camera2gimbal",
                "camera_matrix", "distort_coeffs"}
        current_calibration = "\n".join(
            line for line in (Path(__file__).resolve().parents[1] / "configs/standard.yaml")
            .read_text(encoding="utf-8").splitlines() if line.split(":", 1)[0] in keys) + "\n"
        compile_run(r'''
#include "tasks/auto_aim/solver.hpp"
#include "tools/math_tools.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <iostream>
#include <optional>
using namespace auto_aim;
constexpr double degree=M_PI/180.;
int main(int argc,char **argv) {
  for(int fixture=1;fixture<argc;++fixture) {
    Solver solver(argv[fixture]);int cases=0,excluded=0;
    std::optional<Armor> timed;
    for(double heading:{-179.*degree,-90.*degree,0.,90.*degree,179.*degree})
    for(double distance:{1.,3.,6.})
    for(double phase:{-69.,-65.37,-30.49,-.51,0.,.49,30.37,65.37,69.})
    for(double noise:{0.,.12,.7}) {
      solver.set_R_gimbal2world(Eigen::Quaterniond(
        Eigen::AngleAxisd(heading,Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(.025,Eigen::Vector3d::UnitY())));
      const double bearing=tools::eulers(solver.R_gimbal2world(),2,1,0)[0];
      const Eigen::Vector3d xyz(distance*std::cos(bearing),distance*std::sin(bearing),.08);
      const double truth=tools::limit_rad(bearing+phase*degree);
      auto points=solver.reproject_armor(xyz,truth,ArmorType::small,ArmorName::three);
      for(int j=0;j<4;++j) {
        points[j].x+=static_cast<float>(noise*(j%2?1:-1));
        points[j].y+=static_cast<float>(noise*(j<2?1:-1));
      }
      Armor input(9,.95f,cv::Rect(0,0,100,40),points);
      input.type=ArmorType::small;input.name=ArmorName::three;
      auto legacy=input,batched=input;
      solver.solve(legacy);solver.solve(batched,true);
      assert((legacy.xyz_in_world-batched.xyz_in_world).norm()==0.);
      assert((legacy.xyz_in_gimbal-batched.xyz_in_gimbal).norm()==0.);
      assert((legacy.ypr_in_gimbal-batched.ypr_in_gimbal).norm()==0.);
      assert((legacy.ypd_in_world-batched.ypd_in_world).norm()==0.);
      assert(legacy.yaw_raw==batched.yaw_raw);
      const double difference=std::abs(tools::limit_rad(
        legacy.ypr_in_world[0]-batched.ypr_in_world[0]));
      if(difference>1e-12) std::cerr<<"grid mismatch fixture="<<fixture
        <<" heading="<<heading/degree<<" range="<<distance<<" phase="<<phase
        <<" noise="<<noise<<" old="<<legacy.ypr_in_world[0]/degree
        <<" batch="<<batched.ypr_in_world[0]/degree<<std::endl;
      assert(difference<1e-12);
      const double start=tools::limit_rad(
        tools::eulers(solver.R_gimbal2world(),2,1,0)[0]-70.*degree);
      const double index=std::remainder(batched.ypr_in_world[0]-start,2.*M_PI)/degree;
      assert(std::abs(index-std::round(index))<1e-10);
      ++cases;timed=input;
    }
    // Preserve the balance bypass and the original outpost/base geometry.
    solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
    for(auto kind:{std::make_pair(ArmorType::big,ArmorName::three),
                   std::make_pair(ArmorType::big,ArmorName::four),
                   std::make_pair(ArmorType::big,ArmorName::five),
                   std::make_pair(ArmorType::big,ArmorName::base),
                   std::make_pair(ArmorType::small,ArmorName::outpost)}) {
      const auto points=solver.reproject_armor({3.,0.,.08},.2,kind.first,kind.second);
      Armor legacy(9,.95f,cv::Rect(0,0,100,40),points);
      legacy.type=kind.first;legacy.name=kind.second;
      auto batched=legacy;solver.solve(legacy);solver.solve(batched,true);
      assert((legacy.xyz_in_world-batched.xyz_in_world).norm()==0.);
      assert((legacy.ypr_in_world-batched.ypr_in_world).norm()==0.);++excluded;
    }
    std::array<double,2> elapsed{};
    for(int mode=0;mode<2;++mode) {
      const auto begin=std::chrono::steady_clock::now();
      for(int i=0;i<300;++i) {auto a=*timed;solver.solve(a,mode!=0);}
      elapsed[mode]=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-begin).count()/300.;
    }
    std::cout<<"PASS batch fixture="<<fixture<<" exact_grid_cases="<<cases
      <<" excluded_or_other_geometry="<<excluded<<" legacy_ms="<<elapsed[0]
      <<" batch_ms="<<elapsed[1]<<"; microbenchmark, not frame throughput\n";
  }
}
''', "test_batched_yaw_projection", sources=("tasks/auto_aim/solver.cpp",
            "tasks/auto_aim/armor.cpp", "tools/math_tools.cpp", "tools/logger.cpp"),
            libraries=("opencv_core", "opencv_imgproc", "opencv_calib3d", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"pose.yaml": calibration, "rounded.yaml": calibration.replace(
                "[0,0,1,-1,0,0,0,-1,0]",
                "[0.0002,0.0001,1.0001,-1,0.00015,0.00005,0.0001,-1.0001,0.0001]"),
                      "current.yaml": current_calibration},
            arguments=("pose.yaml", "rounded.yaml", "current.yaml"))


if __name__ == "__main__":
    unittest.main()
