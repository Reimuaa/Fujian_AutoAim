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
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
namespace serial {
inline int fail_writes = 0, short_writes = 0;
inline std::vector<std::vector<uint8_t>> packets;
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
 size_t read(uint8_t *, size_t) {
   std::this_thread::sleep_for(std::chrono::milliseconds(1)); return 0;
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
