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
            lib_names = {"opencv_core": "opencv_core4", "opencv_imgproc": "opencv_imgproc4"}
            command = [compiler, "/nologo", "/std:c++17", "/EHsc", "/utf-8", "/O2", "/MD",
                       "/D_USE_MATH_DEFINES", "/DNOMINMAX", "/DFMT_SHARED", "/DSPDLOG_SHARED_LIB",
                       "/DSPDLOG_COMPILED_LIB", "/DSPDLOG_FMT_EXTERNAL", "/DWIN32_LEAN_AND_MEAN",
                       "/I" + str(temporary), "/I" + str(ROOT), "/I" + str(vcpkg / "include"),
                       "/I" + str(vcpkg / "include/opencv4"),
                       "/I" + str(vcpkg / "include/eigen3"), str(source),
                       *[str(ROOT / p) for p in sources], "/Fo" + str(ROOT / "build") + "/",
                       "/Fe" + str(binary), "/link", "/LIBPATH:" + str(vcpkg / "lib"),
                       *[lib_names.get(lib, lib) + ".lib" for lib in libraries]]
        else:
            command = ["g++", "-std=c++17", "-O1", "-pthread", "-I" + str(temporary),
                       "-I" + str(ROOT), "-I/usr/include/opencv4", "-I/usr/include/eigen3",
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
  uint64_t work_zone_track_epoch=1;
  bool xuc_imu_sync_valid=true,direct_measurement_from_tracker=true;
  bool direct_yaw_measurement_fresh=true,direct_pitch_measurement_fresh=true;
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
  std::cout << "PASS production integration: mode_TJ=1 phase, current associated observation, 3-frame confirmation, 150ms loss, no stale angles/fire, full-frame search\n";
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
