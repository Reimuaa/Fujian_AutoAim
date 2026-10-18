#!/usr/bin/env python3
"""Offline diagnostic tests: compile XUC against a fake serial port, never hardware.

Requires a Linux C++17 compiler, yaml-cpp, spdlog and fmt. All fixtures live in
the system temporary directory; the test executable lives in build/.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile


SERIAL_STUB = r'''
#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <algorithm>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
namespace serial {
inline int fail_writes = 0, short_writes = 0;
inline std::vector<std::vector<uint8_t>> packets;
inline std::mutex rx_mutex;
inline std::deque<uint8_t> input;
inline void inject(const std::vector<uint8_t> & bytes) {
  std::lock_guard<std::mutex> lock(rx_mutex);
  input.insert(input.end(),bytes.begin(),bytes.end());
}
enum flowcontrol_t { flowcontrol_none };
enum parity_t { parity_none };
enum stopbits_t { stopbits_one };
enum bytesize_t { eightbits };
struct Timeout { static Timeout simpleTimeout(unsigned) { return {}; } };
class IOException : public std::runtime_error { using std::runtime_error::runtime_error; };
class Serial {
 bool open_ = false;
public:
 void setPort(const std::string &) {}
 void setBaudrate(unsigned) {}
 void setFlowcontrol(flowcontrol_t) {}
 void setParity(parity_t) {}
 void setStopbits(stopbits_t) {}
 void setBytesize(bytesize_t) {}
 void setTimeout(Timeout) {}
 void open() { open_ = true; }
 void close() { open_ = false; }
 bool isOpen() const { return open_; }
 size_t read(uint8_t * out, size_t size) {
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
   std::lock_guard<std::mutex> lock(rx_mutex);
   const auto n=std::min({size,input.size(),size_t(7)}); // Fragment wire frames.
   for(size_t i=0;i<n;++i) { out[i]=input.front();input.pop_front(); }
   return n;
 }
 size_t write(const uint8_t * p, size_t n) {
   if (fail_writes > 0) { --fail_writes; throw IOException("injected failure"); }
   if (short_writes > 0) { --short_writes; return n - 1; }
   packets.emplace_back(p, p + n); return n;
 }
};
}
'''

HARNESS = r'''
#include <atomic>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <nlohmann/json.hpp>
#include <spdlog/sinks/base_sink.h>
#include "io/xuc.hpp"
#include "tools/logger.hpp"
#include "tools/crc.hpp"
using namespace std::chrono_literals;
struct Capture : spdlog::sinks::base_sink<std::mutex> {
 std::vector<std::string> lines;
 std::atomic<bool> block{false}, entered{false};
 std::atomic<int> fail{0};
 void sink_it_(const spdlog::details::log_msg & msg) override {
   std::string s(msg.payload.data(), msg.payload.size());
   if (s.rfind("[SHOT_DEBUG]", 0) == 0) {
     entered = true;
     while (block) std::this_thread::sleep_for(1ms);
     if (fail > 0) { --fail; throw std::runtime_error("injected sink failure"); }
   }
   lines.push_back(std::move(s));
 }
 void flush_() override {}
};
auto sink = std::make_shared<Capture>();
namespace tools {
std::shared_ptr<spdlog::logger> logger() {
 static auto value = std::make_shared<spdlog::logger>("offline", sink);
 return value;
}
}
size_t thread_count() {
 size_t count = 0;
 for (const auto & f : std::filesystem::directory_iterator("/proc/self/task")) ++count;
 return count;
}
std::vector<nlohmann::json> events() {
 std::vector<nlohmann::json> out;
 for (const auto & s : sink->lines)
   if (s.rfind("[SHOT_DEBUG] ", 0) == 0) out.push_back(nlohmann::json::parse(s.substr(13)));
 return out;
}
io::XucShotDebugContext context(uint64_t id, uint64_t frame) {
 return {id, 123456, frame, static_cast<int64_t>(frame), static_cast<int64_t>(frame * 1000000)};
}
void snapshot(io::XucSender & x, uint64_t id) {
 x.record_shot_debug(nlohmann::json{{"kind","request_snapshot"},{"run",123456},{"id",id}});
}
int main(int argc, char ** argv) {
 assert(argc == 3);
 const auto threads = thread_count();
 {
   io::XucSender x(argv[2]);
   // Real captured 30-byte idle packet. CRC and layout are independent of
   // sizeof(XucTxPacket); an old 20-byte parser cannot pass this fixture.
   std::vector<uint8_t> rx={0x53,0x50,0x00,0x67,0,0,0,0,0x16,0x7a,
     0x33,0x0e,0x87,0x3e,0x0e,0xc0,0xc2,0x3e,0,0,0xa4,0x2b,0x14,0,
     0xc0,0xff,0,0,0x3a,0xfd};
   auto bad=rx;bad[29]^=1;serial::inject({0x12,0x53,0x01});serial::inject(bad);
   std::this_thread::sleep_for(30ms);assert(!x.rx_valid());
   serial::inject(rx);std::this_thread::sleep_for(30ms);
   assert(x.rx_valid());assert(x.mode()==0);assert(x.robot_id()==103);
   assert(x.bullet_count()==31254);
   assert(std::abs(x.imu_pitch()-0.2637802064418793)<1e-7);
   assert(std::abs(x.imu_yaw()-0.3803715109825134)<1e-7);
   auto samples=x.take_shooter_feedback();assert(samples.size()==1);
   assert(samples[0].fire_flag==0);assert(samples[0].fire_tick_ms==0x00142ba4);
   assert(samples[0].friction_current_raw==-64);assert(samples[0].feeder_speed_rpm==0);
   assert(samples[0].host_rx_ns==x.last_rx_ns());assert(x.take_shooter_feedback().empty());
   // Explicit signed RPM must drive integration, independently of projectile speed=0.
   rx[18]=1;rx[20]=0xf0;rx[21]=0xff;rx[22]=0xff;rx[23]=0xff;
   rx[24]=0x2e;rx[25]=0xfb; // -1234 raw current
   rx[26]=0xb0;rx[27]=0x04; // +1200 rotor RPM
   auto crc=tools::get_crc16(rx.data(),28);rx[28]=crc&255;rx[29]=crc>>8;
   serial::inject(rx);std::this_thread::sleep_for(20ms);
   serial::inject(rx);std::this_thread::sleep_for(20ms);
   assert(x.feeder_speed_rpm()==1200);assert(x.bullet_speed()==0);
   assert(x.feeder_motor_turns_estimate()>0.1);
   samples=x.take_shooter_feedback();assert(samples.size()==2);
   assert(samples[0].fire_tick_ms==0xfffffff0u);assert(samples[0].friction_current_raw==-1234);
   assert(samples[1].host_rx_ns>samples[0].host_rx_ns);
   rx[26]=0x50;rx[27]=0xfb; // -1200 rotor RPM
   crc=tools::get_crc16(rx.data(),28);rx[28]=crc&255;rx[29]=crc>>8;
   serial::inject(rx);std::this_thread::sleep_for(20ms);assert(x.feeder_speed_rpm()==-1200);
   assert(x.take_shooter_feedback()[0].feeder_speed_rpm==-1200);
   // Bound pending waveform data when the sending thread is stalled.
   for(int i=0;i<270;++i) serial::inject(rx);
   const auto deadline=std::chrono::steady_clock::now()+3s;
   while(x.valid_packets()<274 && std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(5ms);
   assert(x.valid_packets()==274);assert(x.shooter_feedback_dropped()==14);
   assert(x.take_shooter_feedback().size()==256);
   std::this_thread::sleep_for(520ms);assert(!x.rx_valid());
   serial::inject(rx);std::this_thread::sleep_for(20ms);assert(x.rx_valid());
   std::cout << "PASS 30-byte captured wire: idle heartbeat, fragmented frames, CRC, signed telemetry, bounded queue, timeout/recovery\n";
 }
 {
   io::XucSender x(argv[1]);
   std::vector<uint8_t> rx={0x53,0x50,0x00,0x67,0,0,0,0,0x16,0x7a,
     0x33,0x0e,0x87,0x3e,0x0e,0xc0,0xc2,0x3e,0,0,0xa4,0x2b,0x14,0,
     0xc0,0xff,0,0,0x3a,0xfd};
   serial::inject(rx);std::this_thread::sleep_for(20ms);assert(x.rx_valid());
   assert(x.take_shooter_feedback().empty());assert(x.shooter_feedback_dropped()==0);
   assert(x.feeder_motor_turns_estimate()==0);
 }
 assert(events().empty());sink->lines.clear();serial::packets.clear();
 const io::Command start{true,true,0.3,-0.15}, follow{true,false,0.7,0.2};
 std::vector<std::vector<uint8_t>> compact;
 {
   io::XucSender x(argv[1]);
   assert(!x.shot_debug_enabled());
   assert(thread_count() == threads + 1); // Existing receiver only.
   x.send(start); x.send(follow); std::this_thread::sleep_for(70ms); x.send(follow);
   x.record_shot_debug(nlohmann::json{{"kind","must_not_log"}});
   compact = serial::packets;
 }
 assert(events().empty());
 serial::packets.clear(); sink->lines.clear();
 {
   io::XucSender x(argv[2]);
   assert(x.shot_debug_enabled()); assert(thread_count() == threads + 2);
   auto c = context(1,10); assert(x.send_debug(start,&c)); snapshot(x,1);
   c = context(0,11); x.send_debug(follow,&c);
   std::this_thread::sleep_for(70ms); c = context(0,12); x.send_debug(follow,&c);
 }
 assert(serial::packets == compact); // Byte-for-byte command equivalence.
 auto e = events(); size_t high=0,low=0,snap=0;
 for (const auto & r:e) {
   assert(r["id"] == 1); assert(r["run"] == 123456);
   high += r["kind"] == "first_high"; low += r["kind"] == "first_low";
   snap += r["kind"] == "request_snapshot";
   if (r["kind"] == "first_low") {
     assert(r["frame_seq"] == 10); assert(r["tx_frame_seq"] == 12);
     assert(r["low_interval_until_ns"].get<int64_t>() - r["low_interval_begin_ns"].get<int64_t>() == 50000000);
   }
 }
 assert(high==1 && low==1 && snap==1);
 std::cout << "PASS compact: no debug worker/logs; wire bytes unchanged\n";
 sink->lines.clear(); serial::packets.clear();
 {
   io::XucSender x(argv[2]); auto c=context(2,20);
   serial::fail_writes=1; assert(x.send_debug(start,&c)); snapshot(x,2);
   c=context(0,21); x.send_debug(follow,&c); x.send_debug(follow,&c);
   std::this_thread::sleep_for(70ms);
   serial::short_writes=1; x.send_debug(follow,&c); assert(!x.shoot_request_ready());
   std::this_thread::sleep_for(70ms); assert(!x.shoot_request_ready());
   c=context(0,22); x.send_debug(follow,&c); assert(!x.shoot_request_ready());
   std::this_thread::sleep_for(70ms); assert(x.shoot_request_ready());
 }
 high=low=0; size_t failed=0;
 for (const auto & r:events()) {
   assert(r["id"] == 2);
   high += r["kind"] == "first_high"; low += r["kind"] == "first_low";
   failed += r["kind"] == "write_not_confirmed";
   if (r["kind"] == "first_high") { assert(r["frame_seq"]==20); assert(r["tx_frame_seq"]==21); }
   if (r["kind"] == "write_not_confirmed") assert(r["tx_write_complete_ns"].is_null());
 }
 assert(high==1 && low==1 && failed==2);
 std::cout << "PASS failure/short write/retry: original id; low interval starts on success\n";
 sink->lines.clear();
 {
   io::XucSender x(argv[2]); auto c=context(3,30); serial::fail_writes=1;
   assert(x.send_debug(start,&c)); snapshot(x,3);
   std::this_thread::sleep_for(70ms); c=context(0,31); x.send_debug(follow,&c);
 }
 high=low=0;
 for (const auto & r:events()) {
   high += r["kind"] == "first_high"; low += r["kind"] == "first_low";
   if (r["kind"]=="first_low") assert(r["first_high_ns"].is_null());
 }
 assert(high==0 && low==1);
 std::cout << "PASS no successful high: unknown remains unknown\n";
 sink->lines.clear(); sink->entered=false; sink->block=true;
 {
   io::XucSender x(argv[2]); snapshot(x,4);
   const auto deadline=std::chrono::steady_clock::now()+2s;
   while(!sink->entered && std::chrono::steady_clock::now()<deadline) std::this_thread::sleep_for(1ms);
   assert(sink->entered);
   const auto begin=std::chrono::steady_clock::now();
   for(int i=0;i<1000;++i) snapshot(x,4);
   const auto elapsed=std::chrono::steady_clock::now()-begin;
   sink->block=false;
   std::cout << "Queue producer for 1000 records (blocked sink): "
     << std::chrono::duration<double,std::milli>(elapsed).count() << " ms\n";
 }
 bool summary=false; bool dropped=false;
 for(const auto & r:events()) dropped |= r["queue_dropped_total"].get<uint64_t>()>0;
 for(const auto & s:sink->lines) summary |= s.rfind("[SHOT_DEBUG_SUMMARY]",0)==0;
 assert(dropped && summary && events().size()<=65);
 std::cout << "PASS bounded queue: producer continues; drops counted; shutdown drains\n";
 sink->lines.clear(); sink->fail=1;
 {
   io::XucSender x(argv[2]); snapshot(x,5); snapshot(x,5);
 }
 bool errors=false;
 for(const auto & r:events()) errors |= r["sink_errors_total"].get<uint64_t>()>0;
 assert(errors);
 std::cout << "PASS sink exception: counted without affecting sender\n";
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--source-overlay', type=Path)
    args = parser.parse_args()
    root = args.project_root.resolve()
    overlay = (args.source_overlay or root).resolve()
    binary = root / 'build' / 'xuc_shot_diagnostics_test'
    binary.parent.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='xuc-diagnostic-test-') as directory:
        temp = Path(directory)
        (temp / 'serial').mkdir()
        (temp / 'serial/serial.h').write_text(SERIAL_STUB, encoding='utf-8')
        (temp / 'test.cpp').write_text(HARNESS, encoding='utf-8')
        config = '''xuc_enable: true
xuc_allow_control: true
xuc_allow_shoot: true
xuc_require_feedback_for_control: false
xuc_require_auto_mode_for_control: false
xuc_yaw_sign: 1
xuc_pitch_sign: 1
xuc_shoot_hold_ms: 50
xuc_shoot_low_ms: 50
xuc_shot_rearm_ms: 0
'''
        for name, value in [('compact', 'false'), ('debug', 'true')]:
            (temp / (name + '.yaml')).write_text(
                config + 'infantry_diagnostics_enabled: ' + value + '\n', encoding='utf-8')
        subprocess.run([
            'g++', '-std=c++17', '-O1', '-pthread', '-I' + str(temp),
            '-I' + str(overlay), '-I' + str(root), str(temp / 'test.cpp'),
            str(overlay / 'io/xuc.cpp'), str(root / 'tools/crc.cpp'),
            '-lyaml-cpp', '-lspdlog', '-lfmt', '-o', str(binary),
        ], check=True, timeout=120)
        subprocess.run([str(binary), str(temp / 'compact.yaml'), str(temp / 'debug.yaml')],
                       check=True, timeout=30, cwd=root)


if __name__ == '__main__':
    main()
