#!/usr/bin/env python3
"""Linux offline checks of the actual ROI policy, Tracker states and YOLO26 images."""
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(sys.platform == "linux" and shutil.which("g++"), "Linux C++ compiler required")
class RoiReacquisitionTests(unittest.TestCase):
    def test_three_misses_recovery_and_existing_tracker_reset(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        start = source.index("    if (roi_reacquisition_enabled) {")
        policy = source[start:source.index("    const std::size_t armors_after_color", start)]
        tracker = (ROOT / "tasks/auto_aim/tracker.cpp").read_text(encoding="utf-8")
        start = tracker.index("void Tracker::state_machine(bool found)")
        state_machine = tracker[start:tracker.index("bool Tracker::set_target", start)]
        harness = r'''
#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/types.hpp>
enum class ArmorName { three, outpost };
struct Armor { std::vector<cv::Point2f> points; };
struct Tracker {
  std::string state_="lost";
  int detect_count_=0, temp_lost_count_=0, min_detect_count_=5;
  int max_temp_lost_count_=15, normal_temp_lost_count_=15, outpost_max_temp_lost_count_=75;
  struct Target { ArmorName name=ArmorName::three; } target_;
  Armor armor;
  bool associated=false;
  std::string state() const { return state_; }
  int temp_lost_count() const { return temp_lost_count_; }
  const Armor * primary_observation() const { return associated ? &armor : nullptr; }
  void state_machine(bool found);
};
STATE_MACHINE
const std::vector<cv::Point2f> inside{{650,500},{750,500},{750,550},{650,550}};
const std::vector<cv::Point2f> outside{{350,500},{450,500},{450,550},{350,550}};
struct Controller {
  bool roi_reacquisition_enabled=true, full_frame_search=true;
  cv::Rect configured_detection_roi{420,300,600,600};
  struct Image { int cols=1440, rows=1080; } img;
  Tracker tracker;
  bool step(bool found, const std::vector<cv::Point2f> & corners=inside) {
    const bool used=roi_reacquisition_enabled && full_frame_search;
    tracker.associated=found;
    tracker.armor.points=corners;
    tracker.state_machine(found);
    POLICY
    return used;
  }
  void confirm() {
    for(int i=0;i<5;++i) assert(step(true));
    assert(tracker.state()=="tracking" && !full_frame_search);
  }
};
int main() {
  for(int gap : {1,2}) {
    Controller c; c.confirm();
    for(int i=0;i<gap;++i) {
      assert(!c.step(false));
      assert(!c.full_frame_search && c.tracker.state()=="temp_lost");
    }
    assert(!c.step(true) && !c.full_frame_search);
    assert(c.tracker.state()=="tracking");
    assert(!c.step(false) && c.tracker.temp_lost_count()==1);
    assert(!c.full_frame_search);  // Previous gap must not accumulate.
  }
  Controller c; c.confirm();
  assert(!c.step(false) && !c.full_frame_search);
  assert(!c.step(false) && !c.full_frame_search);
  assert(!c.step(false) && c.full_frame_search);
  assert(c.tracker.state()=="temp_lost" && c.tracker.temp_lost_count()==3);
  assert(c.step(true,outside) && c.full_frame_search);
  assert(c.tracker.state()=="tracking");
  assert(c.step(false) && c.full_frame_search);
  assert(c.step(false) && c.full_frame_search);
  assert(c.step(true,outside) && c.full_frame_search);
  assert(c.step(true,inside) && !c.full_frame_search);
  assert(!c.step(true));

  // Centre inside ROI is insufficient if a lightbar crosses its boundary.
  auto corners=inside;
  corners[0].x=419.9f;
  assert(!c.step(true,corners) && c.full_frame_search);
  corners[0].x=420.f;
  assert(c.step(true,corners) && !c.full_frame_search);
  corners=inside; corners[1].x=1020.f;
  c.step(true,corners); assert(c.full_frame_search);
  c.step(true,inside); assert(!c.full_frame_search);
  corners=inside; corners[2].y=900.f;
  c.step(true,corners); assert(c.full_frame_search);
  c.step(true,inside); assert(!c.full_frame_search);

  Controller loss; loss.confirm();
  for(int i=1;i<=15;++i) {
    loss.step(false);
    assert(loss.tracker.state()=="temp_lost");
    assert(loss.tracker.temp_lost_count()==i);
  }
  assert(loss.step(false) && loss.tracker.state()=="lost");
  assert(loss.step(true) && loss.tracker.state()=="detecting");
  assert(loss.full_frame_search);

  Controller reset; reset.confirm();
  reset.tracker.state_="lost";  // Existing divergence/NIS reset bypasses the gap counter.
  assert(!reset.step(false) && reset.full_frame_search);
  assert(reset.step(true) && reset.full_frame_search);

  Controller disabled; disabled.roi_reacquisition_enabled=false;
  assert(!disabled.step(false) && !disabled.step(true));
  Controller uncropped;
  uncropped.configured_detection_roi={0,0,-1,-1};
  uncropped.confirm();
  assert(!uncropped.full_frame_search);
  std::cout << "PASS: 1/2-frame gaps, third miss, ROI-edge recovery, full-search gaps, unchanged 16-frame reset\n";
}
'''.replace("STATE_MACHINE", state_machine).replace("POLICY", policy)
        binary = ROOT / "build/roi_reacquisition_test"
        binary.parent.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="roi-policy-test-") as folder:
            cpp = Path(folder) / "test.cpp"
            cpp.write_text(harness, encoding="utf-8")
            subprocess.run(["g++", "-std=c++17", "-O1", "-Wall", "-Wextra",
                            "-I/usr/include/opencv4", str(cpp), "-lopencv_core", "-o", str(binary)],
                           check=True, timeout=60)
            subprocess.run([str(binary)], check=True, timeout=10)

    @unittest.skipUnless((ROOT / "assets/yolo26.xml").is_file() and
                         (ROOT / "build/compile_commands.json").is_file(),
                         "Field model and Linux build metadata required")
    def test_actual_yolo26_saved_images_and_roi_round_trip(self):
        import yaml

        harness = r'''
#include <algorithm>
#include <cassert>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "tasks/auto_aim/yolo.hpp"
int main(int argc, char ** argv) {
  assert(argc==3);
  auto_aim::YOLO detector(argv[1],false);
  int frame=0;
  for(const char * name : {"1791797623358_1506_0.jpg","1791797623358_1587_0.jpg"}) {
    const auto bgr=cv::imread(std::string(argv[2])+"/data/infantry_diagnostics/"+name);
    assert(!bgr.empty());
    cv::Mat rgb; cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB);
    const auto roi=detector.detect(rgb,frame++,false);
    const auto full=detector.detect(rgb,frame++,true);
    const auto roi_again=detector.detect(rgb,frame++,false);
    nlohmann::json result={{"image",name},{"roi_count",roi.size()},
                           {"full_count",full.size()},{"roi_again_count",roi_again.size()}};
    result["roi_centres"]=nlohmann::json::array();
    for(const auto & a : roi)
      result["roi_centres"].push_back({a.class_id,a.center.x,a.center.y});
    result["full_centres"]=nlohmann::json::array();
    for(const auto & a : full) {
      result["full_centres"].push_back({a.class_id,a.center.x,a.center.y});
      for(const auto & p : a.points)
        assert(p.x>=0 && p.x<rgb.cols && p.y>=0 && p.y<rgb.rows);
    }
    std::cout << result.dump() << std::endl;
    assert(!full.empty());
    // Returning to ROI must restore its offset rather than retain full-frame coordinates.
    for(const auto & a : roi) {
      assert(std::any_of(roi_again.begin(),roi_again.end(),[&](const auto & b) {
        if(a.class_id!=b.class_id || a.points.size()!=b.points.size()) return false;
        for(std::size_t i=0;i<a.points.size();++i)
          if(cv::norm(a.points[i]-b.points[i])>.01) return false;
        return true;
      }));
    }
    if(std::string(name).find("1587")!=std::string::npos) {
      // This saved target is around x=400, beyond the configured ROI's left edge.
      // A stale +420 ROI offset would make this assertion fail.
      assert(std::any_of(full.begin(),full.end(),[](const auto & a) {
        return a.center.x>300 && a.center.x<450 && a.center.y>400 && a.center.y<850;
      }));
    }
  }
  std::cout << "PASS: actual YOLO26 full-frame edge detection and ROI coordinate round trip\n";
}
'''
        binary = ROOT / "build/roi_reacquisition_image_test"
        obj = binary.with_suffix(".o")
        entries = json.loads((ROOT / "build/compile_commands.json").read_text())
        entry = next(e for e in entries if Path(e["file"]) == ROOT / "src/standard.cpp")
        with tempfile.TemporaryDirectory(prefix="roi-image-test-") as folder:
            temp = Path(folder)
            cpp = temp / "test.cpp"
            cpp.write_text(harness, encoding="utf-8")
            config = yaml.safe_load((ROOT / "configs/standard.yaml").read_text())
            for key in ["yolo26_model_path", "classify_model"]:
                config[key] = str((ROOT / config[key]).resolve())
            config["detector_debug"] = False
            config["yolo26_save_samples"] = False
            fixture = temp / "config.yaml"
            fixture.write_text(yaml.safe_dump(config), encoding="utf-8")
            (temp / "logs").mkdir()
            command = shlex.split(entry["command"])
            command[command.index("-o") + 1] = str(obj)
            command[command.index("-c") + 1] = str(cpp)
            command.append("-UNDEBUG")
            subprocess.run(command, cwd=entry["directory"], check=True, timeout=90)
            link = shlex.split((ROOT / "build/CMakeFiles/standard.dir/link.txt").read_text())
            main_obj = next(i for i, value in enumerate(link) if value.endswith("/src/standard.cpp.o"))
            link[main_obj] = str(obj)
            link[link.index("-o") + 1] = str(binary)
            subprocess.run(link, cwd=ROOT / "build", check=True, timeout=90)
            library_dirs = sorted({str(Path(arg).parent) for arg in link
                                   if Path(arg).is_absolute() and ".so" in Path(arg).name})
            env = os.environ.copy()
            env["LD_LIBRARY_PATH"] = ":".join(library_dirs + [env.get("LD_LIBRARY_PATH", "")])
            subprocess.run([str(binary), str(fixture), str(ROOT)], cwd=temp,
                           env=env, check=True, timeout=90)


if __name__ == "__main__":
    unittest.main()
