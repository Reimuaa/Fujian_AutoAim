#!/usr/bin/env python3
"""Exercise production XUC and the real serial library over Linux PTYs only.

No camera, CAN, physical serial port, motor or shooter is opened. Generated
fixtures live in the system temporary directory; the executable lives in build/.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <pty.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <nlohmann/json.hpp>
#include <spdlog/sinks/null_sink.h>
#include "io/xuc.hpp"
#include "tools/crc.hpp"
#include "tools/logger.hpp"
using namespace std::chrono;
using namespace std::chrono_literals;
namespace tools {
std::shared_ptr<spdlog::logger> logger() {
  static auto log = std::make_shared<spdlog::logger>(
    "offline_receive", std::make_shared<spdlog::sinks::null_sink_mt>());
  return log;
}
}
int64_t ns() { return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count(); }
struct Link {
  int master = -1;
  std::unique_ptr<io::XucSender> x;
  explicit Link(const char * config) {
    int slave; char name[128]{};
    assert(openpty(&master, &slave, name, nullptr, nullptr) == 0);
    {
      std::ofstream out(config);
      out << "xuc_enable: true\nxuc_serial_port: " << name
          << "\nxuc_heartbeat_hz: 0\nxuc_allow_control: false\nxuc_allow_shoot: false"
          << "\ninfantry_diagnostics_enabled: true\n";
    }
    x = std::make_unique<io::XucSender>(config);
    close(slave);
    assert(x->enabled());
  }
  ~Link() { x.reset(); close(master); }
  void bytes(const uint8_t * data, size_t size) {
    while (size) { const auto n = write(master, data, size); assert(n > 0); data += n; size -= n; }
  }
  std::vector<uint8_t> packet(uint16_t seq) {
    io::XucTxPacket p{};
    p.head[0] = 'S'; p.head[1] = 'P'; p.mode = 1; p.robot_id = 103;
    p.bullet_count = seq; p.imu_yaw = seq * .001f; p.imu_pitch = -.125f;
    p.feeder_speed_rpm = 0;
    p.crc16 = tools::get_crc16(reinterpret_cast<uint8_t *>(&p), 28);
    std::vector<uint8_t> b(sizeof(p)); std::memcpy(b.data(), &p, sizeof(p)); return b;
  }
  void send(uint16_t seq) { const auto p = packet(seq); bytes(p.data(), p.size()); }
  void await(uint64_t count) {
    const auto end = steady_clock::now() + 300ms;
    while (x->valid_packets() < count && steady_clock::now() < end) std::this_thread::sleep_for(200us);
    assert(x->valid_packets() == count);
  }
};
struct ProfileProbe {
  bool link_enabled = false, requested = false;
  bool enabled() const { return link_enabled; }
  void set_high_speed_receive(bool value) { requested = value; }
};
void select_profile(ProfileProbe & xuc, bool xuc_high_speed_packet_receive_enabled,
                    bool high_speed_camera_mode) {
  PRODUCTION_SELECTION
}
void check_profile() {
  for (bool link : {false, true}) for (bool option : {false, true})
  for (bool high : {false, true}) {
    ProfileProbe xuc{link, true}; select_profile(xuc, option, high);
    assert(xuc.requested == (link && option && high));
    select_profile(xuc, option, false); assert(!xuc.requested);
  }
}
double median(std::vector<double> v) {
  std::sort(v.begin(), v.end()); return v[v.size() / 2];
}
struct Cadence { int coalesced = 0; double median_lag_ms = 0.; };
Cadence cadence(const char * config, bool high) {
  Link link(config); link.x->set_high_speed_receive(high);
  std::this_thread::sleep_for(30ms); // Let a previously pending read finish.
  std::vector<int64_t> sent;
  const auto start = steady_clock::now();
  constexpr int count = 80;
  for (int i = 0; i < count; ++i) {
    std::this_thread::sleep_until(start + 5ms * i);
    sent.push_back(ns()); link.send(i + 1);
  }
  link.await(count);
  const auto samples = link.x->take_shooter_feedback(); assert(samples.size() == count);
  Cadence result; std::vector<double> lag;
  for (int i = 0; i < count; ++i) {
    assert(samples[i].feedback_sequence == i + 1);
    lag.push_back((samples[i].host_rx_ns - sent[i]) / 1e6);
    if (i && samples[i].host_rx_ns - samples[i - 1].host_rx_ns < 200000) ++result.coalesced;
  }
  result.median_lag_ms = median(lag);
  std::cout << "cadence high=" << high << " count=" << count
            << " near_simultaneous_pairs=" << result.coalesced
            << " median_host_lag_ms=" << result.median_lag_ms << '\n';
  return result;
}
void protocol_and_transitions(const char * config) {
  Link link(config); link.x->set_high_speed_receive(true);
  std::this_thread::sleep_for(30ms);
  auto bad = link.packet(99); bad.back() ^= 1;
  const uint8_t noise[] = {0x12, 0x53, 0x01}; link.bytes(noise, sizeof(noise));
  link.bytes(bad.data(), bad.size());
  auto good = link.packet(1);
  link.bytes(good.data(), 7); std::this_thread::sleep_for(2ms);
  assert(!link.x->rx_valid()); // Neither garbage nor an incomplete frame is accepted.
  link.bytes(good.data() + 7, 9); std::this_thread::sleep_for(2ms);
  assert(!link.x->rx_valid());
  link.bytes(good.data() + 16, good.size() - 16); link.await(1);
  assert(link.x->crc_errors() > 0 && link.x->mode() == 1 && link.x->robot_id() == 103);
  assert(std::abs(link.x->imu_pitch() + .125) < 1e-7);
  assert(std::abs(link.x->imu_yaw() - .001) < 1e-7);
  // A mode change must preserve an in-flight partial frame in either direction.
  good = link.packet(2); link.bytes(good.data(), 11);
  std::this_thread::sleep_for(12ms); link.x->set_high_speed_receive(false);
  link.bytes(good.data() + 11, good.size() - 11); link.await(2);
  good = link.packet(3); link.bytes(good.data(), 13);
  std::this_thread::sleep_for(12ms); link.x->set_high_speed_receive(true);
  link.bytes(good.data() + 13, good.size() - 13); link.await(3);
  link.send(4); link.send(5); link.await(5); // Already-buffered packets remain valid.
  auto samples = link.x->take_shooter_feedback(); assert(samples.size() == 5);
  for (int i = 0; i < 5; ++i) assert(samples[i].feedback_sequence == i + 1);
  std::this_thread::sleep_for(520ms); assert(!link.x->rx_valid());
  assert(link.x->successful_tx_count() == 0); // This fixture never sends a control/fire command.
  std::cout << "PASS CRC/noise/fragmentation/profile transitions/stale feedback/no TX\n";
}
int main(int argc, char ** argv) {
  assert(argc == 2); check_profile();
  const auto legacy = cadence(argv[1], false);
  assert(legacy.coalesced >= 12); // Reproduce the original batching with the real library.
  for (int restart = 0; restart < 3; ++restart) {
    const auto high = cadence(argv[1], true);
    assert(high.coalesced < legacy.coalesced / 3);
    assert(high.median_lag_ms + 1.0 < legacy.median_lag_ms);
  }
  const auto restored = cadence(argv[1], false);
  assert(restored.coalesced >= 12); // Opting out retains the original non-high-speed policy.
  protocol_and_transitions(argv[1]);
  std::cout << "PASS packet-boundary receive across 3 fresh receiver instances; legacy isolation\n";
}
'''


@unittest.skipUnless(sys.platform.startswith('linux'), 'Run in local Linux/WSL; PTYs only')
class XucReceiveTimingTests(unittest.TestCase):
    def test_real_serial_cadence_and_protocol(self):
        source = (ROOT / 'src/standard.cpp').read_text(encoding='utf-8')
        begin = source.index('    const bool xuc_packet_boundary_receive_requested =')
        end = source.index('    // Classification uses this frame', begin)
        selection = source[begin:end]
        binary = ROOT / 'build/xuc_receive_timing_test'
        binary.parent.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='xuc-pty-test-') as directory:
            temp = Path(directory)
            fixture = temp / 'test.cpp'
            fixture.write_text(HARNESS.replace('PRODUCTION_SELECTION', selection), encoding='utf-8')
            subprocess.run([
                'g++', '-std=c++17', '-O1', '-pthread', '-I' + str(ROOT),
                '-I' + str(ROOT / 'io/serial/include'), str(fixture),
                str(ROOT / 'io/xuc.cpp'), str(ROOT / 'tools/crc.cpp'),
                str(ROOT / 'io/serial/src/serial.cc'),
                str(ROOT / 'io/serial/src/impl/unix.cc'),
                '-lyaml-cpp', '-lspdlog', '-lfmt', '-lutil', '-lrt', '-o', str(binary),
            ], check=True, timeout=120)
            subprocess.run([str(binary), str(temp / 'pty.yaml')], check=True,
                           timeout=20, cwd=ROOT)


if __name__ == '__main__':
    unittest.main()
