"""Local-only C++ image/search/integration tests; never opens a camera or serial port."""
import importlib.util
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def compile_run(code, name, *, sources=(), libraries=(), fixtures=None, arguments=()):
    """Use the installed local compiler. Temporary sources are removed; binaries use build/."""
    env = os.environ.copy()
    binary = ROOT / "build" / (name + (".exe" if sys.platform == "win32" else ""))
    binary.parent.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="lightbar-search-") as directory:
        temporary = Path(directory)
        source = temporary / "test.cpp"
        source.write_text(code, encoding="utf-8")
        for path, content in (fixtures or {}).items():
            p = temporary / path
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(content, encoding="utf-8")
        if sys.platform == "win32":
            vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / \
                "Microsoft Visual Studio/Installer/vswhere.exe"
            installation = subprocess.check_output(
                [str(vswhere), "-latest", "-products", "*", "-property", "installationPath"],
                text=True).strip()
            setup = Path(installation) / "VC/Auxiliary/Build/vcvars64.bat"
            environment_script = temporary / "compiler_env.cmd"
            environment_script.write_text(
                f'@echo off\ncall "{setup}" >nul\nif errorlevel 1 exit /b %errorlevel%\nset\n',
                encoding="utf-8")
            setup_result = subprocess.run(["cmd.exe", "/d", "/c", str(environment_script)],
                                          capture_output=True, text=True, check=True)
            for line in setup_result.stdout.splitlines():
                if "=" in line and not line.startswith("="):
                    key, value = line.split("=", 1)
                    env[key.upper()] = value
            cache = (ROOT / "build/CMakeCache.txt").read_text(encoding="utf-8")
            installed = re.search(r"^VCPKG_INSTALLED_DIR:PATH=(.+)$", cache, re.MULTILINE)
            vcpkg = Path(installed.group(1).strip() if installed else "C:/vcpkg/installed") / "x64-windows"
            env["PATH"] = str(vcpkg / "bin") + os.pathsep + env.get("PATH", "")
            compiler = shutil.which("cl.exe", path=env.get("PATH"))
            if not compiler:
                raise RuntimeError("MSVC compiler unavailable")
            lib_names = {"opencv_core": "opencv_core4", "opencv_imgproc": "opencv_imgproc4",
                         "opencv_calib3d": "opencv_calib3d4"}
            command = [compiler, "/nologo", "/std:c++17", "/EHsc", "/utf-8", "/O2", "/MD",
                       "/D_USE_MATH_DEFINES", "/DNOMINMAX", "/DFMT_SHARED", "/DSPDLOG_SHARED_LIB",
                       "/DSPDLOG_COMPILED_LIB", "/DSPDLOG_FMT_EXTERNAL", "/DWIN32_LEAN_AND_MEAN",
                       "/I" + str(temporary), "/I" + str(ROOT),
                       "/I" + str(ROOT / "io/serial/include"), "/I" + str(vcpkg / "include"),
                       "/I" + str(vcpkg / "include/opencv4"),
                       "/I" + str(vcpkg / "include/eigen3"), str(source),
                       *[str(ROOT / p) for p in sources], "/Fo" + str(ROOT / "build") + "/",
                       "/Fe" + str(binary), "/link", "/LIBPATH:" + str(vcpkg / "lib"),
                       *[lib_names.get(lib, lib) + ".lib" for lib in libraries]]
        else:
            command = ["g++", "-std=c++17", "-O1", "-pthread", "-I" + str(temporary),
                       "-I" + str(ROOT), "-I" + str(ROOT / "io/serial/include"),
                       "-I/usr/include/opencv4", "-I/usr/include/eigen3",
                       str(source), *[str(ROOT / p) for p in sources],
                       *["-l" + lib for lib in libraries], "-o", str(binary)]
        compiled = subprocess.run(command, cwd=ROOT, env=env, text=True,
                                  capture_output=True, errors="replace", timeout=180)
        if compiled.returncode:
            raise AssertionError(compiled.stdout + compiled.stderr)
        result = subprocess.run([str(binary), *[str(temporary / a) for a in arguments]],
                                cwd=ROOT, env=env, capture_output=True, text=True,
                                errors="replace", timeout=40)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr + f" exit={result.returncode}")
        print(result.stdout.strip())


class LightbarSearchTests(unittest.TestCase):
    def test_camera_first_pending_latest_switch_and_pairing(self):
        source = (ROOT / "io/hikrobot/hikrobot.cpp").read_text(encoding="utf-8")
        begin = source.index("void HikRobot::read(")
        reads = source[begin:source.index("void HikRobot::capture_start()", begin)]
        standard = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        begin = standard.index("    const bool camera_latest_requested =")
        selection = standard[begin:standard.index("    const auto camera_read_at", begin)]
        self.assertIn("high_speed_camera_mode = high_speed_mode;", standard)
        self.assertIn("++capture_sequence_, raw.stFrameInfo.nFrameNum", source)
        self.assertIn("timestamp = data.timestamp;", reads)
        compile_run(r'''
#include <cassert>
#include <cmath>
#include <future>
#include <iostream>
#include <memory>
#include <thread>
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"
using namespace std::chrono;
using namespace std::chrono_literals;
struct Frame { int seq; int64_t ns; double yaw; std::shared_ptr<int> payload; };
Frame frame(int seq) { return {seq, seq*6666667LL, seq*.001, std::make_shared<int>(seq)}; }
void paired(const Frame & f) {
  assert(f.ns==f.seq*6666667LL && f.yaw==f.seq*.001 && *f.payload==f.seq);
}
namespace io {
// Only the production read methods are used. No constructor, SDK or hardware.
class HikRobot : public CameraBase {
public:
  struct CameraData { cv::Mat img; steady_clock::time_point timestamp;
    uint64_t capture_sequence; uint64_t sdk_frame_id; };
  tools::CameraFrameQueue<CameraData> queue_;
  CameraFrameInfo last_frame_info_;
  void read(cv::Mat &,steady_clock::time_point &) override;
  void read_latest(cv::Mat &,steady_clock::time_point &) override;
  void read_frame(cv::Mat &,steady_clock::time_point &,bool);
  CameraFrameInfo frame_info() const override { return last_frame_info_; }
};
READS
struct Fallback : CameraBase {
  int reads=0;
  void read(cv::Mat &,steady_clock::time_point & t) override {
    ++reads; t=steady_clock::time_point(99ms);
  }
};
}
int main() {
  tools::CameraFrameQueue<Frame> q;
  q.push(frame(1)); q.push(frame(2)); q.push(frame(3));
  assert(q.pop(false).seq==1); // Exact old single-slot semantics at low speed.
  q.push(frame(4)); q.push(frame(5)); q.push(frame(6));
  auto f=q.pop(true); paired(f); assert(f.seq==6); // Enter high: producer kept it.
  q.push(frame(7)); q.push(frame(8));
  f=q.pop(false); paired(f); assert(f.seq==7); // Exit high: never replay 4 or 5.
  q.push(frame(9)); q.push(frame(10));
  assert(q.pop(true).seq==10); // Re-enter high with pending backlog.
  std::weak_ptr<int> first, obsolete;
  { auto a=frame(11); first=a.payload; q.push(a);
    auto b=frame(12); obsolete=b.payload; q.push(b); }
  q.push(frame(13)); assert(!first.expired() && obsolete.expired());
  { auto last=q.pop(true); assert(last.seq==13); }
  assert(first.expired()); // Only the first and newest frame can be retained.
  auto waiting=std::async(std::launch::async,[&] { return q.pop(true); });
  assert(waiting.wait_for(10ms)==std::future_status::timeout); // Camera pause, no stale replay.
  q.push(frame(14)); f=waiting.get(); paired(f); assert(f.seq==14);
  // Deterministic 150Hz production / ~58Hz consumption. Compare low policy
  // frame-for-frame with the actual old queue, not with a second algorithm.
  tools::CameraFrameQueue<Frame> mixed;
  tools::ThreadSafeQueue<Frame> legacy(1);
  int next=1, previous=0; double first_age=0,latest_age=0; int samples=0;
  tools::CameraFrameQueue<Frame> high;
  for(int step=1;step<=580;++step) {
    const int64_t read_ns=step*17241379LL;
    while(next*6666667LL<=read_ns) {
      auto value=frame(next++); mixed.push(value); legacy.push(value); high.push(value);
    }
    auto low=mixed.pop(false); auto old=legacy.pop(); auto fresh=high.pop(true);
    paired(low); paired(fresh); assert(low.seq==old.seq && low.seq>previous);
    assert(fresh.seq>=low.seq && read_ns>=fresh.ns);
    previous=low.seq; first_age+=(read_ns-low.ns)/1e6; latest_age+=(read_ns-fresh.ns)/1e6; ++samples;
  }
  assert(latest_age<first_age);
  std::thread producer([&] { for(int i=15;i<=30000;++i) q.push(frame(i)); });
  int seq=14;
  while(seq<30000) { f=q.pop(true); paired(f); assert(f.seq>seq); seq=f.seq; }
  producer.join();
  io::HikRobot camera; cv::Mat img; steady_clock::time_point t;
  struct { bool enabled() const { return true; } } xuc;
  bool xuc_high_speed_latest_frame_enabled=true,high_speed_camera_mode=false;
  for(int mode=0;mode<5;++mode) {
    high_speed_camera_mode=(mode==1 || mode==3 || mode==4);
    xuc_high_speed_latest_frame_enabled=mode!=4;
    for(int i=0;i<3;++i) {
      const int seq=mode*3+i+1;
      cv::Mat image(1,1,CV_32SC1); image.at<int>(0,0)=seq;
      camera.queue_.push({image,steady_clock::time_point(milliseconds(seq)),
        uint64_t(seq),uint64_t(100+seq)});
    }
SELECTION
    const int expected=mode*3+(mode==1 || mode==3 ? 3 : 1);
    assert(img.at<int>(0,0)==expected && t==steady_clock::time_point(milliseconds(expected)));
    const auto info=camera.frame_info();
    assert(info.capture_sequence==uint64_t(expected) && info.sdk_frame_id==uint64_t(100+expected));
    assert(info.latest_supported && info.latest_selected==camera_latest_requested);
  }
  io::Fallback fallback; io::CameraBase & base=fallback;
  base.read_latest(img,t); assert(fallback.reads==1 && t==steady_clock::time_point(99ms));
  assert(!base.frame_info().latest_supported); // Other cameras retain their original read.
  std::cout << "PASS camera: legacy equality, low/high/low/re-entry/off, bounded retention, "
    "stall/wake, concurrent producer, image/time/SDK-ID pairing; synthetic mean age "
    << first_age/samples << " -> " << latest_age/samples << " ms (not field data)\n";
}
'''.replace("READS", reads).replace("SELECTION", selection),
            "camera_frame_policy_test", libraries=("opencv_core",))

    def test_latest_frame_imu_tail_limits(self):
        source = (ROOT / "io/xuc.cpp").read_text(encoding="utf-8")
        begin = source.index("bool XucSender::imu_at(")
        method = source[begin:source.index("bool XucSender::shoot_request_ready_at", begin)]
        standard = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        self.assertLess(standard.index("detector.detect(img"), standard.index("xuc.imu_at(frame_imu_requested_time"))
        self.assertIn("frame_imu_requested_time = t + xuc_imu_alignment;", standard)
        self.assertIn("? static_cast<io::Mode>(xuc.mode()) : io::Mode::idle", standard)
        compile_run(r'''
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <mutex>
#include "io/xuc.hpp"
// Compile the real imu_at body with an in-memory history; no serial sender.
using namespace std::chrono;
using namespace std::chrono_literals;
namespace offline {
using io::XucTimedImu;
class XucSender {
public:
  bool valid=true;
  mutable std::mutex imu_history_mutex_;
  std::deque<XucTimedImu> imu_history_;
  bool rx_valid() const { return valid; }
  bool imu_at(steady_clock::time_point,XucTimedImu &) const;
};
METHOD
}
int main() {
  offline::XucSender x;
  const auto t=steady_clock::time_point(10s);
  for(int i=0;i<=2;++i) {
    io::XucTimedImu sample; sample.timestamp=t+milliseconds(10*i);
    sample.yaw=.01*i; sample.pitch=.005*i; x.imu_history_.push_back(sample);
  }
  io::XucTimedImu pose;
  assert(x.imu_at(t+15ms,pose) && pose.interpolated && !pose.extrapolated);
  assert(std::abs(pose.yaw-.015)<1e-12 && pose.timestamp==t+15ms);
  // Latest frame t+20ms asks for t+35ms, nearer the history tail.
  assert(x.imu_at(t+35ms,pose) && pose.extrapolated && !pose.interpolated);
  assert(std::abs(pose.yaw-.035)<1e-12 && pose.timestamp==t+35ms);
  assert(x.imu_at(t+45ms,pose) && pose.extrapolated); // Existing 25ms maximum.
  assert(!x.imu_at(t+46ms,pose)); // No relaxed bound to accommodate fresh frames.
  assert(!x.imu_at(t-21ms,pose));
  for(int i=3;i<=4;++i) {
    io::XucTimedImu sample; sample.timestamp=t+milliseconds(10*i);
    sample.yaw=.01*i; sample.pitch=.005*i; x.imu_history_.push_back(sample);
  }
  assert(x.imu_at(t+35ms,pose) && pose.interpolated && !pose.extrapolated);
  assert(std::abs(pose.yaw-.035)<1e-12); // Same requested image pose after inference.
  x.imu_history_[3].yaw=3.; x.imu_history_[4].yaw=-3.;
  assert(!x.imu_at(t+65ms,pose)); // Rate too high; nearest also beyond 20ms.
  assert(x.imu_at(t+60ms,pose) && !pose.extrapolated && pose.nearest_offset_ms==-20.);
  x.valid=false; assert(!x.imu_at(t+35ms,pose)); // Stale feedback still blocks sync.
  std::cout << "PASS actual IMU: t+15ms, interpolation/extrapolation/nearest, "
    "25ms/20ms bounds, rate limit, invalid feedback; no waiting or hardware\n";
}
'''.replace("METHOD", method), "camera_imu_tail_test")

    def test_yaw_zero_shadow_is_read_only_and_uses_same_point(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        begin = source.index("struct YawZeroReferenceShadow")
        shadow = source[begin:source.index("\n};", begin) + 3]
        call = source.index("const auto shadow = YawZeroReferenceShadow::compare(")
        self.assertGreater(call, source.index("// Continuous debug evidence"))
        self.assertGreater(call, source.index("xuc.send(command"))
        self.assertIn("if (high_speed_mode && !gyro_model_fresh && native_prediction_valid", source[:call])
        begin = source.index("gyro_predicted_impact_yaw_error_rad = std::remainder(")
        legacy = source[begin:source.index(";", begin)+1]
        compile_run(r'''
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
SHADOW
double actual(double qualified_predicted_yaw,double reference,double xuc_gyro_predicted_yaw_trim_rad) {
  const std::array<double,3> ypr{reference,0.,0.};
  double gyro_predicted_impact_yaw_error_rad=NAN;
LEGACY
  return gyro_predicted_impact_yaw_error_rad;
}
int main() {
  const double r=M_PI/180.; int cases=0;
  for(double reference : {-179.9*r,0.,179.9*r})
  for(double error : {-.20001*r,-.2*r,0.,.2*r,.20001*r,1.3*r})
  for(double trim : {0.,.1*r})
  for(double correction : {0.,-1.3*r,1.3*r}) {
    const double predicted=reference+trim+error;
    auto s=YawZeroReferenceShadow::compare(predicted,reference,trim,correction,.2*r);
    assert(s.valid && s.legacy_error_rad==actual(predicted,reference,trim));
    const double expected=std::remainder(error+correction,2*M_PI);
    assert(std::abs(std::remainder(s.command_frame_error_rad-expected,2*M_PI))<1e-14);
    if(correction==0.) assert(s.legacy_pass==s.command_frame_pass);
    ++cases;
  }
  auto a=YawZeroReferenceShadow::compare(0.,0.,0.,-1.3*r,.2*r);
  assert(a.legacy_pass && !a.command_frame_pass);
  auto b=YawZeroReferenceShadow::compare(1.3*r,0.,0.,-1.3*r,.2*r);
  assert(!b.legacy_pass && b.command_frame_pass);
  assert(!YawZeroReferenceShadow::compare(NAN,0.,0.,0.,.2*r).valid);
  assert(!YawZeroReferenceShadow::compare(0.,0.,0.,0.,-.1).valid);
  std::cout << "PASS " << cases << " same-point zero shadow cases: current gate equality, "
    "wrap, trim, offset hypotheses, invalid input; diagnostic runs after send\n";
}
'''.replace("SHADOW", shadow).replace("LEGACY", legacy), "yaw_zero_shadow_test")

    def test_calibrated_solver_and_wire_yaw_chain(self):
        source = (ROOT / "tasks/auto_aim/solver.cpp").read_text(encoding="utf-8")
        begin = source.index("void Solver::set_R_gimbal2world(")
        rotation = source[begin:source.index("//solvePnP", begin)]
        compile_run(r'''
#include <cassert>
#include <cmath>
#include <iostream>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include "tools/math_tools.hpp"
namespace auto_aim {
class Solver {
public:
  Eigen::Matrix3d R_gimbal2imubody_, R_gimbal2world_;
  void set_R_gimbal2world(const Eigen::Quaterniond &);
};
ROTATION
}
int main(int argc,char ** argv) {
  assert(argc==2);
  const auto yaml=YAML::LoadFile(argv[1]);
  const auto d=yaml["R_gimbal2imubody"].as<std::vector<double>>();
  const double tx=yaml["xuc_yaw_sign"].as<double>();
  const double rx=yaml["xuc_imu_yaw_sign"].as<double>();
  const double offset=yaml["xuc_imu_yaw_offset_rad"].as<double>();
  auto_aim::Solver solver;
  solver.R_gimbal2imubody_=Eigen::Matrix<double,3,3,Eigen::RowMajor>(d.data());
  for(double raw : {-.5,-.2,0.,.2,.5}) for(double pitch : {-.15,0.,.15}) {
    Eigen::Quaterniond q=Eigen::AngleAxisd(rx*raw+offset,Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
    solver.set_R_gimbal2world(q);
    const double world_yaw=tools::eulers(solver.R_gimbal2world_,2,1,0)[0];
    assert(std::abs(world_yaw+rx*raw+offset)<1e-12);
    assert(std::abs(tx*world_yaw-raw)<1e-8);
  }
  std::cout << "PASS configured raw IMU -> Solver conjugation -> command wire yaw: "
    "compatible sign/scale in both directions; physical bore zero remains unverified\n";
}
'''.replace("ROTATION", rotation), "yaw_frame_chain_test",
            sources=("tools/math_tools.cpp",), libraries=("opencv_core", "yaml-cpp"),
            arguments=(str(ROOT / "configs/standard.yaml"),))

    def test_actual_rgb_gate_and_yolo_acceptance(self):
        source = (ROOT / "tasks/auto_aim/yolos/yolo26.cpp").read_text(encoding="utf-8")
        check = source[source.index("bool YOLO26::check_name("):
                       source.index("bool YOLO26::has_plausible_geometry(")]
        refine = source[source.index("bool YOLO26::refine_color("):
                        source.index("std::list<Armor> YOLO26::postprocess(")]
        code = r'''
#include <cassert>
#include <iostream>
#include <limits>
#include "tasks/auto_aim/lightbar_color.hpp"
namespace auto_aim {
enum class Color { red, blue, extinguish };
enum class ArmorName { three, not_armor };
struct Armor {
  std::vector<cv::Point2f> points{{30,20},{90,20},{90,70},{30,70}};
  cv::Point2f center{60,45};
  Color color=Color::blue;
  ArmorName name=ArmorName::three;
  double confidence=.99, keypoint_confidence=1, color_score=0;
  int color_source=0;
};
class YOLO26 {
public:
  bool require_lit_bars_=true, refine_color_=true, save_samples_=false;
  double min_confidence_=.1, color_confirmed_min_confidence_=.04;
  double memory_confirmed_min_confidence_=.06, color_confirmed_min_keypoint_confidence_=.75;
  int color_hold_frames_=20;
  struct ColorMemory { cv::Point2f center; double armor_width; Color color; double score; int frame_count; };
  std::vector<ColorMemory> color_memories_;
  LightbarColorGate lightbar_color_gate_;
  bool refine_color(Armor &,const cv::Mat &,int);
  bool check_name(const Armor &) const;
  bool has_plausible_geometry(const Armor &) const { return true; }
  void save(const Armor &) const {}
};
CHECK
REFINE
}
using namespace auto_aim;
cv::Mat bars(cv::Scalar left, cv::Scalar right) {
  cv::Mat image(100,120,CV_8UC3,cv::Scalar(0));
  cv::line(image,{30,20},{30,70},left,5);
  cv::line(image,{90,20},{90,70},right,5);
  return image;
}
int main() {
  const cv::Scalar red(220,20,10), blue(10,20,220), dark(0,0,0);
  YOLO26 detector; Armor armor;
  assert(detector.refine_color(armor,bars(red,red),1));
  assert(armor.color==Color::red && detector.check_name(armor)); // Correct wrong network color.
  assert(detector.refine_color(armor,bars(blue,blue),2));
  assert(armor.color==Color::blue && detector.check_name(armor));
  for (auto image : {bars(dark,dark), bars(red,dark), bars(red,blue),
      bars({55,5,5},{55,5,5}), bars({200,200,200},{200,200,200}),
      bars({80,200,20},{80,200,20}), bars({200,195,20},{200,195,20})}) {
    assert(!detector.refine_color(armor,image,3));
    assert(armor.color==Color::extinguish && !detector.check_name(armor));
  }
  armor.color_source=2; armor.color=Color::red;
  assert(!detector.check_name(armor)); // High network confidence/history cannot bypass current light.
  assert(detector.refine_color(armor,bars(red,red),4));
  assert(!detector.refine_color(armor,cv::Mat(),5) && !detector.check_name(armor));
  auto mixed=bars(red,red);
  cv::line(mixed,{30,20},{30,45},blue,5);
  assert(!detector.refine_color(armor,mixed,6));
  auto reflection=bars(dark,dark);
  cv::line(reflection,{30,40},{30,42},red,1);
  cv::line(reflection,{90,40},{90,42},red,1);
  assert(!detector.refine_color(armor,reflection,7));
  auto saturated=bars(red,red);
  cv::line(saturated,{30,20},{30,70},{255,255,255},1);
  cv::line(saturated,{90,20},{90,70},{255,255,255},1);
  assert(detector.refine_color(armor,saturated,8));
  armor.points[0].x=std::numeric_limits<float>::quiet_NaN();
  assert(!detector.refine_color(armor,bars(red,red),9) && !detector.check_name(armor));
  std::cout << "PASS RGB: red/blue, dark/high-confidence, single/conflicting/mixed bars, "
    "green/yellow/white, reflections, saturated core, stale memory and invalid input\n";
}
'''.replace("CHECK", check).replace("REFINE", refine)
        compile_run(code, "lightbar_color_test", libraries=("opencv_core", "opencv_imgproc"))

    def test_work_zone_target_lifecycle(self):
        compile_run(r'''
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "tasks/auto_aim/work_zone_target.hpp"
using Gate=auto_aim::WorkZoneTarget;
using State=Gate::State;
bool near(double a,double b) { return std::abs(a-b)<1e-9; }
int main() {
  Gate gate; Gate::Input in{1,1,false,true,true,.3,.12,1};
  auto out=gate.update(in);assert(out.state==State::inactive && !out.control && out.inhibit_fire);
  in.work_zone_ready=true;in.now_s=in.frame_s=1.02;
  out=gate.update(in);assert(out.state==State::confirming && !out.control);
  in.now_s=in.frame_s=1.04;out=gate.update(in);assert(!out.control);
  in.now_s=in.frame_s=1.06;out=gate.update(in);assert(out.control && !out.inhibit_fire);
  assert(!gate.needs_full_frame());gate.remember_final_command(.42,.16);
  in.observation_valid=false;in.yaw=9;in.pitch=9;in.now_s=1.12;
  out=gate.update(in);assert(out.control && out.inhibit_fire && out.hold_angles);
  assert(near(out.yaw,.42) && near(out.pitch,.16) && near(out.valid_until_s,1.21));
  assert(gate.needs_full_frame());
  in.now_s=1.205;out=gate.update(in);assert(out.control && near(out.valid_until_s,1.21));
  in.now_s=1.211;out=gate.update(in);assert(!out.control && out.state==State::scanning);
  in.observation_valid=true;in.now_s=in.frame_s=1.23;out=gate.update(in);assert(!out.control);
  in.now_s=in.frame_s=1.25;out=gate.update(in);assert(!out.control);
  in.now_s=in.frame_s=1.27;out=gate.update(in);assert(out.control);
  // A stale observation/duplicate image is not fresh detection and cannot fire.
  in.now_s=1.29;out=gate.update(in);assert(out.control && out.hold_angles && out.inhibit_fire);
  in.now_s=1.45;out=gate.update(in);assert(!out.control);
  in.now_s=in.frame_s=1.47;in.target_key=2;out=gate.update(in);assert(!out.control);
  in.now_s=in.frame_s=1.49;in.target_key=3;out=gate.update(in);assert(!out.control);
  in.now_s=in.frame_s=1.51;in.target_key=2;out=gate.update(in);assert(!out.control);
  in.observation_valid=false;in.now_s=in.frame_s=1.53;gate.update(in);
  in.observation_valid=true;in.now_s=in.frame_s=1.55;assert(!gate.update(in).control);
  in.now_s=in.frame_s=1.57;assert(!gate.update(in).control);
  in.now_s=in.frame_s=1.59;assert(gate.update(in).control);
  in.feedback_valid=false;in.now_s=in.frame_s=1.61;out=gate.update(in);assert(!out.control && out.inhibit_fire);
  in.feedback_valid=true;in.now_s=in.frame_s=1.63;assert(!gate.update(in).control);
  in.work_zone_ready=false;in.now_s=in.frame_s=1.65;assert(!gate.update(in).control);
  in.work_zone_ready=true;in.now_s=in.frame_s=1.67;assert(!gate.update(in).control);
  in.yaw=std::numeric_limits<double>::quiet_NaN();in.now_s=in.frame_s=1.69;
  out=gate.update(in);assert(!out.control && out.inhibit_fire);
  Gate::Config two;two.confirm_frames=2;gate=Gate(two);
  in.yaw=.3;in.now_s=in.frame_s=2;assert(!gate.update(in).control);
  in.now_s=in.frame_s=2.02;assert(gate.update(in).control);
  in.now_s=2.5;in.frame_s=2.03;assert(!gate.update(in).control); // Camera backlog.
  in.now_s=2.6;in.frame_s=2.7;assert(!gate.update(in).control); // Future timestamp.
  two.loss_hold_s=.5;gate=Gate(two);in.now_s=in.frame_s=3;assert(!gate.update(in).control);
  std::cout << "PASS work-zone ownership: 3/2 fresh frames, association key, lost hold/expiry, cached final angles, mode/link reset, camera backlog, NaN and no host scan\n";
}
''', "work_zone_target_test")

    def test_actual_standard_target_integration(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        observe = source[source.index("    // Qualify current observations before fire/event accounting."):
                         source.index("    const bool native_shoot =")]
        block = observe + source[source.index("    // Apply target ownership after PI."):
                       source.index("    // Dry-run diagnostics: world-coordinate stability")]
        full_frame = source[source.index("    const bool full_frame_search_used ="):
                            source.index("    auto armors = detector.detect(")]
        self.assertLess(source.index("    // Final actuator tracking layer."),
                        source.index("    // Apply target ownership after PI."))
        self.assertLess(source.index("    // Qualify current observations before fire/event accounting."),
                        source.index("      const bool fire_candidate ="))
        self.assertIn("const bool fire_candidate = !command.inhibit_shoot &&", source)
        self.assertNotIn("YawSearch", source)
        compile_run(r'''
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "tasks/auto_aim/work_zone_target.hpp"
#include "io/command.hpp"
#include "tasks/auto_aim/target_death_return.hpp"
struct Logger { template<class... T> void info(T...) {} };
namespace tools { Logger * logger() {static Logger value;return &value;} }
struct Xuc {
  bool phase=false;
  bool work_zone_ready() const {return phase;}
  bool control_ready() const {return phase;} bool enabled() const {return true;}
  bool rx_valid() const {return true;}
  double raw_imu_yaw() const {return .22;}
  double raw_yaw_correction_to_command(double v) const {return v;}
} xuc;
struct Tracker {
  struct Diagnostics {bool initialized_this_frame=false;} diagnostics;
  std::string phase="detecting";
  std::string state() const {return phase;}
  const Diagnostics & frame_diagnostics() const {return diagnostics;}
} tracker;
struct Target {int name=1;};
int main() {
  auto_aim::WorkZoneTarget work_zone_target;
  auto_aim::TargetDeathReturn target_death_return; // Missing key preserves old behaviour.
  const bool target_death_frame_valid=true,target_death_full_frame=true,target_death_visible=true;
  uint64_t work_zone_track_epoch=1;
  bool xuc_imu_sync_valid=true,direct_measurement_from_tracker=true;
  bool direct_yaw_measurement_fresh=true,direct_pitch_measurement_fresh=true;
  bool tracker_profile_changed=false;
  std::vector<Target> targets(1);
  io::Command command{true,true,.3,.12};
  double yaw_tracking_integral_raw_rad=1;
  double final_yaw_command_filtered=0;
  bool final_yaw_command_initialized=false,final_yaw_lower_armed=true;
  std::chrono::steady_clock::time_point final_yaw_command_last_update;
  bool yaw_tracking_initialized=true,yaw_tracking_lower_armed=true;
  auto update = [&]() {
    auto t=std::chrono::steady_clock::now();
BLOCK
  };
  update();assert(!command.control && command.inhibit_shoot);
  xuc.phase=true;
  for(int i=0;i<3;++i) {
    tracker.phase=i<2 ? "detecting" : "tracking";
    std::this_thread::sleep_for(std::chrono::milliseconds(1));command={true,true,.3,.12};update();
    assert(command.control==(i==2));
  }
  assert(command.shoot && command.target_valid_until_ns>0);
  tracker.diagnostics.initialized_this_frame=true;
  command={true,true,.3,.12};std::this_thread::sleep_for(std::chrono::milliseconds(1));update();
  assert(!command.control && !command.shoot); // Same-class Tracker reset starts a new confirmation.
  tracker.diagnostics.initialized_this_frame=false;
  for(int i=0;i<2;++i) {
    command={true,true,.3,.12};std::this_thread::sleep_for(std::chrono::milliseconds(1));update();
    assert(command.control==(i==1));
  }
  for(int transition=0;transition<2;++transition) { // Legacy -> high -> legacy.
    const auto previous_epoch=work_zone_track_epoch;
    tracker_profile_changed=true;command={true,true,.3,.12};
    std::this_thread::sleep_for(std::chrono::milliseconds(1));update();
    assert(work_zone_track_epoch==previous_epoch+1 && !command.control && !command.shoot);
    tracker_profile_changed=false;
    for(int i=0;i<2;++i) {
      command={true,true,.3,.12};std::this_thread::sleep_for(std::chrono::milliseconds(1));update();
      assert(command.control==(i==1));
    }
  }
  command={true,true,9,9};direct_measurement_from_tracker=false;
  std::this_thread::sleep_for(std::chrono::milliseconds(1));update();
  assert(command.control && !command.shoot && command.inhibit_shoot);
  assert(std::abs(command.yaw-.3)<1e-9 && std::abs(command.pitch-.12)<1e-9);
  assert(yaw_tracking_integral_raw_rad==0 && !yaw_tracking_initialized && !yaw_tracking_lower_armed);
  assert(final_yaw_command_initialized && !final_yaw_lower_armed && std::abs(final_yaw_command_filtered-.3)<1e-9);
  bool roi_reacquisition_enabled=true,full_frame_search=false;
FULL_FRAME
  assert(full_frame_search_used);
  std::this_thread::sleep_for(std::chrono::milliseconds(160));update();
  assert(!command.control && !command.shoot);
  xuc.phase=false;update();assert(!command.control);
  std::cout << "PASS production integration: mode_TJ=1 phase, current associated observation, 3-frame confirmation, tracker-profile requalification, 150ms loss, no stale angles/fire, full-frame search\n";
}
'''.replace("BLOCK", block).replace("FULL_FRAME", full_frame), "work_zone_integration_test")

    @unittest.skipIf(sys.platform == "win32", "Linux fake-serial timing fixture; run under local WSL")
    def test_actual_xuc_single_packet_protocol(self):
        spec = importlib.util.spec_from_file_location("xuc_test", ROOT / "tests/test_xuc_shot_diagnostics.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        harness = module.HARNESS
        config = """xuc_enable: true
xuc_allow_control: true
xuc_allow_shoot: true
xuc_require_feedback_for_control: false
xuc_require_auto_mode_for_control: false
xuc_yaw_sign: 1
xuc_pitch_sign: 1
xuc_heartbeat_hz: 0
xuc_shoot_low_ms: 50
xuc_shot_rearm_ms: 0
"""
        compile_run(harness, "xuc_work_zone_protocol_test", sources=("io/xuc.cpp", "tools/crc.cpp"),
                    libraries=("yaml-cpp", "spdlog", "fmt"),
                    fixtures={"serial/serial.h": module.SERIAL_STUB,
                              "compact.yaml": config + "infantry_diagnostics_enabled: false\n",
                              "debug.yaml": config + "infantry_diagnostics_enabled: true\n"},
                    arguments=("compact.yaml", "debug.yaml"))

    @unittest.skipIf(sys.platform == "win32", "Linux fake-serial timing fixture; run under local WSL")
    def test_actual_xuc_high_speed_release_isolation(self):
        spec = importlib.util.spec_from_file_location("xuc_mode_test", ROOT / "tests/test_xuc_shot_diagnostics.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        begin = source.index("const bool high_speed_mode = rotation_model.motion_mode ==")
        classify = source[begin:source.index(";", begin) + 1]
        begin = source.index("command.high_speed_shoot = high_speed_mode;")
        tag = source[begin:source.index(";", begin) + 1]
        harness = module.HARNESS.split("int main(int argc, char ** argv)")[0] + r'''
#include "tasks/auto_aim/rotation_model_manager.hpp"
io::Command command_for(auto_aim::RotationModelManager::MotionMode mode,bool shoot=true) {
  auto_aim::RotationModelManager::Result rotation_model;rotation_model.motion_mode=mode;
  CLASSIFY
  io::Command command{true,shoot,.3,.12};
  TAG
  return command;
}
auto low_begin(io::XucSender & x) {
  return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(x.last_tx_complete_ns()));
}
std::vector<std::vector<uint8_t>> run(const char * path,bool debug) {
  serial::packets.clear();sink->lines.clear();
  using Mode=auto_aim::RotationModelManager::MotionMode;
  const auto high=command_for(Mode::high_speed),low=command_for(Mode::low_speed);
  const auto stationary=command_for(Mode::stationary);
  assert(high.high_speed_shoot && !low.high_speed_shoot && !stationary.high_speed_shoot);
  const auto high_follow=command_for(Mode::high_speed,false);
  const auto low_follow=command_for(Mode::low_speed,false);
  {
    io::XucSender x(path);assert(x.shot_debug_enabled()==debug);
    assert(x.shoot_low_ms()==50. && x.shoot_low_ms(true)==500.);
    const auto request=[&](const io::Command & cmd,uint64_t id) {
      auto c=context(id,id);assert(x.send_debug(cmd,&c)==debug);
    };
    request(high,1);assert(x.last_tx_shoot()==1);
    x.send(low_follow);assert(x.last_tx_shoot()==0);auto began=low_begin(x);
    std::this_thread::sleep_for(80ms);
    x.send(low);assert(x.last_tx_shoot()==0 && !x.shoot_request_ready());
    // A low-speed frame after a high-speed request cannot shorten its 500ms.
    std::this_thread::sleep_until(began+520ms);assert(x.shoot_request_ready());
    request(low,2);assert(x.last_tx_shoot()==1);
    x.send(high_follow);began=low_begin(x);
    // A high-speed frame after a low-speed request cannot extend its 50ms.
    std::this_thread::sleep_until(began+70ms);assert(x.shoot_request_ready());
    request(stationary,3);assert(x.last_tx_shoot()==1);
    x.send(low_follow);began=low_begin(x);
    std::this_thread::sleep_until(began+70ms);assert(x.shoot_request_ready());
    // Failed high/short low preserve the original profile until confirmed low.
    serial::fail_writes=1;request(high,4);assert(!x.shoot_request_ready());
    serial::short_writes=1;x.send(low_follow);
    std::this_thread::sleep_for(70ms);assert(!x.shoot_request_ready());
    x.send(low_follow);began=low_begin(x);
    std::this_thread::sleep_until(began+80ms);assert(!x.shoot_request_ready());
    std::this_thread::sleep_until(began+520ms);assert(x.shoot_request_ready());
  }
  size_t highs=0;
  for(const auto & p:serial::packets) {
    const auto crc=tools::get_crc16(p.data(),12);
    assert(p.size()==14 && p[12]==(crc&255) && p[13]==(crc>>8));
    highs+=p[3]==1;
  }
  assert(highs==3);
  if(debug) {
    size_t lows=0;
    for(const auto & e:events()) if(e["kind"]=="first_low") {
      const int64_t expected=e["id"]==1 || e["id"]==4 ? 500000000 : 50000000;
      assert(e["low_interval_until_ns"].get<int64_t>()-
        e["low_interval_begin_ns"].get<int64_t>()==expected);
      assert(e["low_duration_ms"].get<double>()==expected/1e6);++lows;
    }
    assert(lows==4);
  } else assert(events().empty());
  return serial::packets;
}
int main(int argc,char ** argv) {
  assert(argc==4);
  const auto compact=run(argv[1],false),debug=run(argv[2],true);
  assert(compact==debug);
  {
    io::XucSender legacy(argv[3]);
    assert(legacy.shoot_low_ms()==50. && legacy.shoot_low_ms(true)==50.);
    auto high=command_for(auto_aim::RotationModelManager::MotionMode::high_speed);
    legacy.send(high);assert(legacy.last_tx_shoot()==1);
    high.shoot=false;legacy.send(high);auto began=low_begin(legacy);
    std::this_thread::sleep_until(began+70ms);assert(legacy.shoot_request_ready());
  }
  std::cout<<"PASS actual XUC: stationary/low=50ms high=500ms, request-time profile "
    "survives both mode transitions and failed writes, single high, CRC, compact/debug "
    "wire equivalence and legacy default\n";
}
'''
        harness = harness.replace("CLASSIFY", classify).replace("TAG", tag)
        config = ("xuc_enable: true\nxuc_allow_control: true\nxuc_allow_shoot: true\n"
                  "xuc_require_feedback_for_control: false\nxuc_require_auto_mode_for_control: false\n"
                  "xuc_yaw_sign: 1\nxuc_pitch_sign: 1\nxuc_heartbeat_hz: 0\n"
                  "xuc_shoot_low_ms: 50\nxuc_shot_rearm_ms: 0\n")
        compile_run(harness, "xuc_high_speed_release_test", sources=("io/xuc.cpp", "tools/crc.cpp"),
                    libraries=("yaml-cpp", "spdlog", "fmt"),
                    fixtures={"serial/serial.h": module.SERIAL_STUB,
                              "compact.yaml": config + "xuc_high_speed_shoot_low_ms: 500\ninfantry_diagnostics_enabled: false\n",
                              "debug.yaml": config + "xuc_high_speed_shoot_low_ms: 500\ninfantry_diagnostics_enabled: true\n",
                              "legacy.yaml": config + "infantry_diagnostics_enabled: false\n"},
                    arguments=("compact.yaml", "debug.yaml", "legacy.yaml"))

    @unittest.skipIf(sys.platform == "win32", "Linux fake-serial timing fixture; run under local WSL")
    def test_actual_xuc_high_speed_minimum_request_interval(self):
        spec = importlib.util.spec_from_file_location("xuc_interval_test", ROOT / "tests/test_xuc_shot_diagnostics.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        begin = source.index("        if (xuc_pending_shot &&")
        pending = source[begin:source.index(" {", begin)]
        begin = source.index("      const bool command_cadence_gate_open =")
        cadence = source[begin:source.index(";", begin) + 1]
        harness = module.HARNESS.split("int main(int argc, char ** argv)")[0] + r'''
#include <cstring>
bool gate(io::XucSender & xuc,bool high_speed_mode,bool & xuc_pending_shot,
          std::chrono::steady_clock::time_point xuc_pending_shot_time) {
  const double xuc_shot_rearm_s=xuc.shoot_min_interval_ms()/1000.;
  const auto feedback_now=std::chrono::steady_clock::now();
PENDING { xuc_pending_shot=false; }
CADENCE
  return command_cadence_gate_open;
}
void run(const char * path,bool debug) {
  serial::packets.clear();serial::packet_times_ns.clear();sink->lines.clear();
  {
    io::XucSender x(path);assert(x.shot_debug_enabled()==debug);
    assert(x.shoot_min_interval_ms()==0. && x.shoot_min_interval_ms(true)==1000.);
    assert(x.shoot_low_ms()==50. && x.shoot_low_ms(true)==100.);
    const auto stamp=[](const io::XucShotTransmission & s) {
      return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(s.complete_ns));
    };
    bool pending=true;
    io::Command high{true,true,.3,.12};high.high_speed_shoot=true;
    auto follow=high;follow.shoot=false;
    x.send(high);const auto first=x.last_shot_transmission();x.send(follow);
    assert(first.complete_ns>0 && !x.shoot_request_ready(true));
    // High -> low: after the existing 100ms release, not the high-only 1s timer.
    std::this_thread::sleep_until(stamp(first)+150ms);
    assert(gate(x,false,pending,stamp(first)) && !pending);
    assert(!gate(x,true,pending,stamp(first)));
    io::Command low{true,true,.4,.12};x.send(low);
    const auto second=x.last_shot_transmission();assert(second.begin_ns>first.begin_ns);
    follow=low;follow.shoot=false;x.send(follow);pending=true;
    // Low -> stationary: retain the original 50ms release.
    std::this_thread::sleep_until(stamp(second)+70ms);
    assert(gate(x,false,pending,stamp(second)) && !pending);
    io::Command stationary{true,true,.5,.12};x.send(stationary);
    const auto accepted=x.last_shot_transmission();assert(accepted.begin_ns>second.begin_ns);
    follow=stationary;follow.shoot=false;x.send(follow);pending=true;
    const auto origin=stamp(accepted);
    // Re-entering high speed measures from the latest successful shot.
    while(std::chrono::steady_clock::now()<origin+800ms) {
      auto attempt=high;attempt.yaw=.65;x.send(attempt);
      assert(x.last_shot_transmission().begin_ns==accepted.begin_ns);
      std::this_thread::sleep_for(20ms);
    }
    const auto before=x.successful_tx_count();
    std::this_thread::sleep_for(80ms);
    assert(x.successful_tx_count()>before); // Heartbeat continues during cooldown.
    follow.yaw=.7;
    while(std::chrono::steady_clock::now()<origin+1100ms) {
      x.send(follow);std::this_thread::sleep_for(20ms);
    }
    assert(gate(x,true,pending,origin));
    assert(x.last_shot_transmission().begin_ns==accepted.begin_ns); // No deferred request.
    auto invalid=high;invalid.control=false;x.send(invalid);
    assert(x.last_shot_transmission().begin_ns==accepted.begin_ns);
    x.send(high);assert(x.last_shot_transmission().begin_ns>accepted.begin_ns);
  }
  std::vector<int64_t> highs;bool following_changed=false;
  for(size_t i=0;i<serial::packets.size();++i) {
    const auto & p=serial::packets[i];const auto crc=tools::get_crc16(p.data(),12);
    assert(p.size()==14 && p[12]==(crc&255) && p[13]==(crc>>8));
    if(p[3]==1) highs.push_back(serial::packet_times_ns[i]);
    float yaw;std::memcpy(&yaw,p.data()+4,sizeof(yaw));
    following_changed|=p[2]==1 && p[3]==0 && std::abs(yaw-.7)<1e-6;
  }
  assert(highs.size()==4 && following_changed);
  assert(highs[1]-highs[0]<1000000000LL && highs[2]-highs[1]<1000000000LL);
  assert(highs[3]-highs[2]>=1000000000LL && highs[3]-highs[0]>=1000000000LL);
}
int main(int argc,char ** argv) {
  assert(argc==4);run(argv[1],false);run(argv[2],true);
  { // Missing new key preserves the previous shared-interval API/configuration.
    io::XucSender legacy(argv[3]);
    assert(legacy.shoot_min_interval_ms()==1000. && legacy.shoot_min_interval_ms(true)==1000.);
    io::Command low{true,true,.3,.12};legacy.send(low);const auto first=legacy.last_shot_transmission();
    low.shoot=false;legacy.send(low);std::this_thread::sleep_for(80ms);
    assert(!legacy.shoot_request_ready() && !legacy.shoot_request_ready(true));
    low.shoot=true;legacy.send(low);assert(legacy.last_shot_transmission().begin_ns==first.begin_ns);
  }
  std::cout<<"PASS actual XUC/main gates: high-only >=1000ms, high->low and low->stationary "
    "retain 100/50ms wire release, low->high cannot bypass 1s, live following/heartbeat, "
    "no deferred request, control/CRC, compact/debug and legacy default\n";
}
'''
        harness = harness.replace("PENDING", pending).replace("CADENCE", cadence)
        config = ("xuc_enable: true\nxuc_allow_control: true\nxuc_allow_shoot: true\n"
                  "xuc_require_feedback_for_control: false\nxuc_require_auto_mode_for_control: false\n"
                  "xuc_yaw_sign: 1\nxuc_pitch_sign: 1\nxuc_heartbeat_hz: 40\n"
                  "xuc_shoot_low_ms: 50\nxuc_high_speed_shoot_low_ms: 100\n"
                  "xuc_shot_rearm_ms: 0\nxuc_high_speed_shot_rearm_ms: 1000\n")
        compile_run(harness, "xuc_minimum_request_interval_test",
                    sources=("io/xuc.cpp", "tools/crc.cpp"), libraries=("yaml-cpp", "spdlog", "fmt"),
                    fixtures={"serial/serial.h": module.SERIAL_STUB,
                              "compact.yaml": config + "infantry_diagnostics_enabled: false\n",
                              "debug.yaml": config + "infantry_diagnostics_enabled: true\n",
                              "legacy.yaml": config.replace("xuc_shot_rearm_ms: 0\nxuc_high_speed_shot_rearm_ms: 1000",
                                                            "xuc_shot_rearm_ms: 1000")},
                    arguments=("compact.yaml", "debug.yaml", "legacy.yaml"))

    @unittest.skipIf(sys.platform == "win32", "Linux fake-serial timing fixture; run under local WSL")
    def test_heartbeat_during_camera_stall(self):
        spec = importlib.util.spec_from_file_location("xuc_heartbeat_test", ROOT / "tests/test_xuc_shot_diagnostics.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        harness = module.HARNESS.split("int main(int argc, char ** argv)")[0] + r'''
void feedback(uint8_t mode) {
  io::XucTxPacket p{};p.head[0]='S';p.head[1]='P';p.mode=mode;p.robot_id=103;
  auto data=reinterpret_cast<uint8_t *>(&p);p.crc16=tools::get_crc16(data,28);
  serial::inject(std::vector<uint8_t>(data,data+30));
}
int main(int argc,char ** argv) {
 assert(argc==2);
 {
  io::XucSender x(argv[1]);
  std::this_thread::sleep_for(80ms);
  assert(x.successful_tx_count()>=2 && x.last_tx_control()==0);
  io::Command target{true,true,.3,.12};
  x.send(target);assert(x.last_tx_control()==0 && x.last_tx_shoot()==0);
  feedback(1);std::this_thread::sleep_for(20ms);assert(x.work_zone_ready());
  auto context_value=context(100,1);assert(x.send_debug(target,&context_value));
  assert(x.last_shoot_tx_ns()>0);const auto high=x.last_shot_transmission();
  assert(high.begin_ns>0 && std::abs(high.yaw_raw-.3)<1e-6);
  // No camera/main-loop send calls for 230ms: heartbeat must release high and
  // expire ownership, while its packets must never renew the target lease.
  std::this_thread::sleep_for(70ms);
  assert(x.last_tx_control()==1 && x.last_tx_shoot()==0);
  assert(x.last_shot_transmission().begin_ns==high.begin_ns);
  std::this_thread::sleep_for(160ms);assert(x.last_tx_control()==0 && x.last_tx_shoot()==0);
  target.shoot=false;x.send(target);assert(x.last_tx_control()==1);
  feedback(0);std::this_thread::sleep_for(45ms);assert(x.last_tx_control()==0);
  feedback(1);std::this_thread::sleep_for(45ms);assert(x.last_tx_control()==0); // No stale relock.
  io::Command returning{false,true,0,0};returning.request_return=true;
  x.send(returning);assert(x.last_tx_control()==2 && x.last_tx_shoot()==0);
  x.send(target);std::this_thread::sleep_for(40ms);assert(x.last_tx_control()==2 && !x.shoot_request_ready());
 }
 size_t highs=0;int64_t max_gap=0;
 for(size_t i=0;i<serial::packets.size();++i) {
  const auto & p=serial::packets[i];const auto crc=tools::get_crc16(p.data(),12);
  assert(p.size()==14 && p[12]==(crc&255) && p[13]==(crc>>8));
  highs+=p[3]==1;assert(p[2]==1 || p[3]==0);
  if(i) max_gap=std::max(max_gap,serial::packet_times_ns[i]-serial::packet_times_ns[i-1]);
 }
 assert(highs==1 && serial::packets.size()>=12 && max_gap<50000000);
 size_t high_logs=0,low_logs=0;
 for(const auto & e:events()) {high_logs+=e["kind"]=="first_high";low_logs+=e["kind"]=="first_low";}
 assert(high_logs==1 && low_logs==1);
 std::cout << "PASS camera-stall heartbeat: CRC packets=" << serial::packets.size()
  << " max_gap_ms=" << max_gap/1e6 << "; one high, ownership expires, phase exit/re-entry, latched return, debug joins\n";
}
'''
        config = """xuc_enable: true
xuc_allow_control: true
xuc_allow_shoot: true
xuc_require_feedback_for_control: true
xuc_require_auto_mode_for_control: true
xuc_yaw_sign: 1
xuc_pitch_sign: 1
xuc_heartbeat_hz: 40
xuc_target_loss_ms: 150
xuc_shoot_low_ms: 50
xuc_shot_rearm_ms: 0
infantry_diagnostics_enabled: true
"""
        compile_run(harness, "xuc_heartbeat_test", sources=("io/xuc.cpp", "tools/crc.cpp"),
                    libraries=("yaml-cpp", "spdlog", "fmt"),
                    fixtures={"serial/serial.h": module.SERIAL_STUB, "heartbeat.yaml": config},
                    arguments=("heartbeat.yaml",))


if __name__ == "__main__":
    unittest.main()
