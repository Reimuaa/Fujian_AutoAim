"""Target-lights-out return and real XUC packets with an in-memory serial port."""
from pathlib import Path
import unittest

from test_lightbar_search import compile_run
from test_xuc_shot_diagnostics import SERIAL_STUB

ROOT = Path(__file__).resolve().parents[1]


class TargetDeathReturnTests(unittest.TestCase):
    def test_arrival_confirmation_absence_and_one_way_latch(self):
        compile_run(r'''
#include "tasks/auto_aim/target_death_return.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using Gate=auto_aim::TargetDeathReturn;
struct Driver {
  Gate gate{Gate::Config{true,1.5}};
  Gate::Input in;
  Gate::Output out;
  int ms=1000;
  Driver() {in.feedback_valid=in.frame_valid=in.full_frame=true;}
  Gate::Output step(int delta=20) {
    ms+=delta;in.now_s=in.frame_s=ms/1000.;out=gate.update(in);return out;
  }
  void arm() {
    in.work_zone_ready=in.target_visible=in.target_confirmed=true;
    assert(step().armed);
    in.target_visible=in.target_confirmed=false;
  }
};
int main() {
  {Driver d;
   for(int i=0;i<1000;++i)assert(!d.step().request_return && !d.out.armed);
   d.in.target_visible=d.in.target_confirmed=true;
   assert(!d.step().armed); // Seeing a target during travel is also insufficient.
   d.in.work_zone_ready=true;d.in.target_visible=d.in.target_confirmed=false;
   for(int i=0;i<500;++i)assert(!d.step().request_return && !d.out.armed);
   d.in.target_visible=true;
   for(int i=0;i<20;++i)assert(!d.step().armed); // Detection without confirmation.
  }
  {Driver d;d.arm();
   for(int i=0;i<70;++i)assert(!d.step().request_return);
   d.in.target_visible=true;assert(d.step().missing_frames==0); // Pose loss is not death.
   d.in.target_visible=false;d.step();const int first=d.ms;
   for(int i=0;i<74;++i)assert(!d.step().request_return);
   assert(d.ms-first==1480);assert(!d.step(19).request_return);
   const auto trigger=d.step(1);
   assert(trigger.triggered && trigger.request_return && trigger.missing_frames>=3);
   assert(std::abs(trigger.missing_s-1.5)<1e-9);
   for(int i=0;i<200;++i) {
     d.in.work_zone_ready=i%2;d.in.feedback_valid=i%3;
     d.in.target_visible=d.in.target_confirmed=true;
     const auto out=d.step();assert(out.request_return && !out.triggered);
   }
  }
  {Driver d;d.arm();d.in.full_frame=false;
   for(int i=0;i<200;++i)assert(!d.step().request_return && d.out.missing_frames==0);
   d.in.full_frame=true;for(int i=0;i<50;++i)assert(!d.step().request_return);
   d.in.full_frame=false;assert(d.step().missing_frames==0);
   d.in.full_frame=true;assert(!d.step().request_return && d.out.missing_s==0.);
  }
  {Driver d;d.arm();for(int i=0;i<50;++i)d.step();
   assert(!d.step(5000).request_return && d.out.missing_s==0.); // Capture/loop stopped.
   d.in.now_s+=.02;assert(d.gate.update(d.in).missing_frames==0); // Duplicate frame.
   d.step();d.in.now_s=d.in.frame_s+.3;
   assert(d.gate.update(d.in).missing_frames==0); // Old image, no absence evidence.
   d.ms+=500;assert(!d.step().request_return && d.out.missing_s==0.);
   d.in.frame_valid=false;assert(d.step().missing_frames==0);
   d.in.frame_valid=true;assert(!d.step().request_return);
  }
  {Driver d;d.arm();for(int i=0;i<70;++i)d.step();
   d.in.feedback_valid=false;assert(!d.step().request_return && d.out.missing_frames==0);
   d.in.feedback_valid=true;assert(!d.step().request_return && d.out.missing_s==0.);
   d.in.work_zone_ready=false;assert(!d.step().armed);
   d.in.work_zone_ready=true;
   for(int i=0;i<200;++i)assert(!d.step().request_return && !d.out.armed);
   d.arm();d.in.now_s-=1.;assert(!d.gate.update(d.in).armed); // Time rollback.
  }
  {Gate disabled;Gate::Input in;
   in.work_zone_ready=in.feedback_valid=in.frame_valid=in.full_frame=true;
   in.target_visible=in.target_confirmed=true;
   for(int i=0;i<300;++i){in.now_s=in.frame_s=1.+i*.02;assert(!disabled.update(in).armed);}
  }
  for(double seconds:{.2,0.,11.,std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected=false;try {Gate invalid(Gate::Config{true,seconds});}
    catch(const std::invalid_argument &) {rejected=true;}
    assert(rejected);
  }
  std::cout<<"PASS travel/never-seen blocked; confirmation required; exact 1500ms; "
    "visible-pose-loss/ROI/gap/duplicates/stale-image/link-loss rejected; one event through return\n";
}
''', "test_target_death_return")

    def test_production_wiring_and_real_control_two_packets(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        begin = source.index("    // Qualify current observations before fire/event accounting.")
        observe = source[begin:source.index("    const bool native_shoot =", begin)]
        # Only replace the observation clock, preserving the production decisions.
        self.assertEqual(observe.count("std::chrono::steady_clock::now()"), 1)
        observe = observe.replace("std::chrono::steady_clock::now()", "test_now")
        begin = source.index("    // Apply target ownership after PI.")
        apply_begin = source.index("    if (xuc.enabled()) {", begin)
        apply = source[apply_begin:source.index("    if (target_death_output.triggered)", apply_begin)]
        self.assertLess(source.index("    const bool target_death_visible ="),
                        source.index("    auto targets = configured_tracker.track("))
        code = r'''
#include "io/xuc.hpp"
#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tasks/auto_aim/work_zone_target.hpp"
#include "tasks/auto_aim/target_death_return.hpp"
#include <cassert>
#include <cstring>
#include <iostream>
#include <string>
using namespace std::chrono;
using namespace std::chrono_literals;
struct Tracker {
  struct Diagnostics {bool initialized_this_frame=false;} diagnostics;
  bool visible=false;
  const Diagnostics & frame_diagnostics() const {return diagnostics;}
  std::string state() const {return visible ? "tracking" : "temp_lost";}
};
int main(int argc,char **argv) {
  assert(argc==3);
  std::size_t first_return=0;int events=0;
  {
    io::XucSender xuc(argv[1]);
    const auto feedback=[&](int mode) {
      io::XucTxPacket p{};p.head[0]='S';p.head[1]='P';p.mode=mode;p.robot_id=103;
      p.bullet_speed=23.;p.imu_yaw=.2;p.imu_pitch=.1;
      p.crc16=tools::get_crc16(reinterpret_cast<uint8_t *>(&p),28);
      const auto count=xuc.valid_packets();
      const auto * begin=reinterpret_cast<const uint8_t *>(&p);
      serial::inject(std::vector<uint8_t>(begin,begin+sizeof(p)));
      const auto end=steady_clock::now()+1s;
      while(xuc.valid_packets()==count && steady_clock::now()<end)std::this_thread::sleep_for(1ms);
      assert(xuc.valid_packets()>count && xuc.mode()==mode);
    };
    auto_aim::WorkZoneTarget work_zone_target;
    auto_aim::TargetDeathReturn target_death_return({true,1.5});
    Tracker tracker;std::vector<int> targets(1);uint64_t work_zone_track_epoch=1;
    bool tracker_profile_changed=false,xuc_imu_sync_valid=true;
    bool direct_measurement_from_tracker=false,direct_yaw_measurement_fresh=false;
    bool direct_pitch_measurement_fresh=false;
    bool target_death_frame_valid=true,target_death_full_frame=true,target_death_visible=false;
    io::Command command{false,false,0,0};
    const auto origin=steady_clock::now()+30s;
    int ms=0;
    const auto step=[&](bool visible,bool full_frame=true) {
      ms+=20;const auto test_now=origin+milliseconds(ms);auto t=test_now;
      tracker.visible=target_death_visible=visible;target_death_full_frame=full_frame;
      direct_measurement_from_tracker=direct_yaw_measurement_fresh=direct_pitch_measurement_fresh=visible;
      command={true,true,.3,.1};
OBSERVE
APPLY
      events+=command.request_return;
      xuc.send(command);
      return target_death_output;
    };
    feedback(0);
    for(int i=0;i<150;++i){assert(!step(false).armed);assert(xuc.last_tx_control()==0);}
    feedback(1);
    for(int i=0;i<100;++i)assert(!step(false).armed); // Arrival without seeing a target.
    assert(!step(true).armed);assert(!step(true).armed);assert(step(true).armed);
    assert(xuc.last_tx_control()==1);
    for(int i=0;i<30;++i)assert(!step(false).request_return);
    feedback(0);assert(!step(false).armed && events==0); // MCU's 50-shot return/cancel.
    feedback(1);for(int i=0;i<100;++i)assert(!step(false).armed);
    step(true);step(true);assert(step(true).armed);
    for(int i=0;i<100;++i)assert(!step(false,false).request_return); // ROI-only absence.
    for(int i=0;i<75;++i)assert(!step(false).request_return);
    assert(step(false).triggered && events==1);
    assert(xuc.last_tx_control()==2 && xuc.last_tx_shoot()==0 && !command.control && command.inhibit_shoot);
    {
      std::lock_guard<std::mutex> lock(serial::tx_mutex);
      for(first_return=0;first_return<serial::packets.size();++first_return)
        if(serial::packets[first_return][2]==2)break;
      assert(first_return<serial::packets.size());
    }
    feedback(0);
    for(int i=0;i<20;++i)assert(!step(false).triggered && events==1);
    feedback(1);assert(!step(true).triggered && events==1);
    std::this_thread::sleep_for(70ms); // The existing heartbeat must retain control=2.
    assert(xuc.last_tx_control()==2 && xuc.last_tx_shoot()==0);
    io::Command stale{true,true,.5,.2};xuc.send(stale);
    assert(xuc.last_tx_control()==2 && xuc.last_tx_shoot()==0);
  }
  for(std::size_t i=0;i<serial::packets.size();++i) {
    const auto & p=serial::packets[i];assert(p.size()==14);
    const auto crc=tools::get_crc16(p.data(),12);
    assert(p[12]==(crc&255) && p[13]==(crc>>8));
    if(i>=first_return)assert(p[2]==2 && p[3]==0);
    assert(p[2]==1 || p[3]==0);
  }
  {
    io::XucSender xuc(argv[2]);
    io::Command returning{true,true,.3,.1};returning.request_return=true;
    serial::fail_writes=1;xuc.send(returning); // A failed return write must not lose the latch.
    io::Command normal{true,true,.3,.1};xuc.send(normal);
    assert(xuc.last_tx_control()==2 && xuc.last_tx_shoot()==0 && !xuc.shoot_request_ready());
  }
  std::cout<<"PASS production mode1/3-frame acquisition/lights-out wiring: exactly one event, "
    "50-shot departure cancels detection, return mode0 does not rearm, actual 14-byte "
    "control2/shoot0 CRC packets, heartbeat and failed-write latch\n";
}
'''.replace("OBSERVE", observe).replace("APPLY", apply)
        config = ("xuc_enable: true\nxuc_allow_control: true\nxuc_allow_shoot: true\n"
                  "xuc_require_feedback_for_control: true\nxuc_require_auto_mode_for_control: true\n"
                  "xuc_yaw_sign: 1\nxuc_pitch_sign: 1\nxuc_heartbeat_hz: 40\n"
                  "xuc_target_loss_ms: 150\nxuc_shoot_low_ms: 50\n"
                  "xuc_high_speed_shoot_low_ms: 1000\nxuc_shot_rearm_ms: 0\n"
                  "infantry_diagnostics_enabled: false\n")
        compile_run(code, "test_target_death_return_wire", sources=(
            "io/xuc.cpp", "tools/crc.cpp", "tools/logger.cpp"),
            libraries=("yaml-cpp", "spdlog", "fmt"),
            fixtures={"serial/serial.h": SERIAL_STUB, "config.yaml": config,
                      "no_feedback.yaml": config.replace("xuc_heartbeat_hz: 40", "xuc_heartbeat_hz: 0")
                      .replace("xuc_require_feedback_for_control: true", "xuc_require_feedback_for_control: false")
                      .replace("xuc_require_auto_mode_for_control: true", "xuc_require_auto_mode_for_control: false")},
            arguments=("config.yaml", "no_feedback.yaml"))


if __name__ == "__main__":
    unittest.main()
