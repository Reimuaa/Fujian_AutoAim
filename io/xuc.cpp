#include "io/xuc.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <vector>
#include <nlohmann/json.hpp>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
// One producer (the sending thread), one consumer. Only the debug constructor
// allocates the queue/worker/logger. Formatting and I/O stay on the consumer.
struct XucSender::ShotDebug
{
  static constexpr std::size_t capacity = 64;
  std::array<nlohmann::json, capacity + 1> records;
  std::atomic<std::size_t> read_index{0}, write_index{0};
  std::atomic<uint64_t> dropped{0}, sink_errors{0};
  std::atomic<bool> stopping{false};
  std::mutex wait_mutex;
  std::condition_variable wake;
  std::shared_ptr<spdlog::logger> output;
  std::thread worker;
  XucShotDebugContext cycle{};
  bool cycle_active = false;
  int64_t first_high_ns = 0, first_high_complete_ns = 0;
  uint64_t enqueued = 0, written = 0;

  ShotDebug()
  {
    output = tools::logger()->clone("shot_debug");
    output->set_error_handler([this](const std::string &) {
      sink_errors.fetch_add(1, std::memory_order_relaxed);
    });
    worker = std::thread([this] { consume(); });
  }

  ~ShotDebug()
  {
    stopping.store(true, std::memory_order_release);
    wake.notify_one();
    if (worker.joinable()) worker.join();
  }

  void push(nlohmann::json && record) noexcept
  {
    const auto index = write_index.load(std::memory_order_relaxed);
    const auto next = (index + 1) % records.size();
    if (next == read_index.load(std::memory_order_acquire)) {
      dropped.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    records[index] = std::move(record);
    ++enqueued;
    write_index.store(next, std::memory_order_release);
    wake.notify_one();
  }

  void consume() noexcept
  {
    for (;;) {
      const auto index = read_index.load(std::memory_order_relaxed);
      if (index == write_index.load(std::memory_order_acquire)) {
        if (stopping.load(std::memory_order_acquire)) break;
        std::unique_lock<std::mutex> lock(wait_mutex);
        // Timed wait also bounds a notification arriving before wait().
        wake.wait_for(lock, std::chrono::milliseconds(100), [this] {
          return stopping.load(std::memory_order_acquire) ||
            read_index.load(std::memory_order_relaxed) !=
              write_index.load(std::memory_order_acquire);
        });
        continue;
      }
      auto record = std::move(records[index]);
      read_index.store((index + 1) % records.size(), std::memory_order_release);
      try {
        record["queue_dropped_total"] = dropped.load(std::memory_order_relaxed);
        record["sink_errors_total"] = sink_errors.load(std::memory_order_relaxed);
        const auto errors_before = sink_errors.load(std::memory_order_relaxed);
        output->info("[SHOT_DEBUG] {}", record.dump());
        if (sink_errors.load(std::memory_order_relaxed) == errors_before) ++written;
      } catch (...) {
        dropped.fetch_add(1, std::memory_order_relaxed);
      }
    }
    try {
      output->info(
        "[SHOT_DEBUG_SUMMARY] run={} enqueued={} written={} dropped={} sink_errors={} "
        "unfinished_id={} unfinished_first_high_ns={} unfinished_first_low_ns=unknown projectile_exit=unknown",
        cycle.run, enqueued, written, dropped.load(), sink_errors.load(),
        cycle_active ? cycle.id : 0, cycle_active ? first_high_ns : 0);
    } catch (...) {}
  }
};

XucSender::XucSender(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  if (yaml["xuc_enable"]) enabled_ = yaml["xuc_enable"].as<bool>();
  if (!enabled_) {
    tools::logger()->info("[XUC] disabled (xuc_enable=false)");
    return;
  }

  std::string port = "/dev/ttyUSB0";
  uint32_t baud = 460800;
  if (yaml["xuc_serial_port"]) port = yaml["xuc_serial_port"].as<std::string>();
  if (yaml["xuc_serial_baud"]) baud = yaml["xuc_serial_baud"].as<uint32_t>();
  if (yaml["xuc_yaw_sign"]) yaw_sign_ = yaml["xuc_yaw_sign"].as<double>();
  if (yaml["xuc_pitch_sign"]) pitch_sign_ = yaml["xuc_pitch_sign"].as<double>();
  if (yaml["xuc_imu_yaw_sign"]) imu_yaw_sign_ = yaml["xuc_imu_yaw_sign"].as<double>();
  if (yaml["xuc_imu_pitch_sign"]) imu_pitch_sign_ = yaml["xuc_imu_pitch_sign"].as<double>();
  if (yaml["xuc_imu_yaw_offset_rad"]) {
    imu_yaw_offset_rad_ = yaml["xuc_imu_yaw_offset_rad"].as<double>();
  }
  if (yaml["xuc_imu_pitch_offset_rad"]) {
    imu_pitch_offset_rad_ = yaml["xuc_imu_pitch_offset_rad"].as<double>();
  }
  if (yaml["xuc_allow_control"]) allow_control_ = yaml["xuc_allow_control"].as<bool>();
  if (yaml["xuc_allow_shoot"]) allow_shoot_ = yaml["xuc_allow_shoot"].as<bool>();
  if (yaml["infantry_diagnostics_enabled"]) {
    feeder_diagnostics_enabled_ = yaml["infantry_diagnostics_enabled"].as<bool>();
  }
  if (yaml["xuc_require_feedback_for_control"]) {
    require_feedback_for_control_ = yaml["xuc_require_feedback_for_control"].as<bool>();
  }
  if (yaml["xuc_require_auto_mode_for_control"]) {
    require_auto_mode_for_control_ = yaml["xuc_require_auto_mode_for_control"].as<bool>();
  }
  if (yaml["xuc_lock_pitch_for_control"]) {
    lock_pitch_for_control_ = yaml["xuc_lock_pitch_for_control"].as<bool>();
  }
  if (yaml["xuc_max_yaw_excursion_rad"]) {
    max_yaw_excursion_rad_ =
      std::max(0.0, yaml["xuc_max_yaw_excursion_rad"].as<double>());
  }
  if (yaml["xuc_shoot_hold_ms"]) {
    const double hold_ms = std::clamp(
      yaml["xuc_shoot_hold_ms"].as<double>(), 50.0, 1000.0);
    shoot_hold_duration_ns_ = static_cast<int64_t>(hold_ms * 1.0e6);
  }
  if (yaml["xuc_shoot_low_ms"]) {
    const double low_ms = std::clamp(
      yaml["xuc_shoot_low_ms"].as<double>(), 50.0, 2000.0);
    shoot_low_duration_ns_ = static_cast<int64_t>(low_ms * 1.0e6);
  }
  if (yaml["xuc_shot_rearm_ms"]) {
    // Zero disables the extra minimum interval; high/low release still applies.
    const double interval_ms = std::clamp(
      yaml["xuc_shot_rearm_ms"].as<double>(), 0.0, 2000.0);
    shoot_min_interval_ns_ = static_cast<int64_t>(interval_ms * 1.0e6);
  }

  try {
    serial_.setPort(port);
    serial_.setBaudrate(baud);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
    serial_.setTimeout(timeout);
    serial_.open();
    if (feeder_diagnostics_enabled_) {
      try {
        shot_debug_ = std::make_unique<ShotDebug>();
      } catch (const std::exception & e) {
        tools::logger()->warn("[SHOT_DEBUG] unavailable: {}", e.what());
      }
    }
    // 启动接收线程：解析新版 TxPacket_TJ
    recv_thread_ = std::thread([this] { recv_loop(); });
    tools::logger()->info(
      "[XUC] serial opened {} @ {} baud, angles=rad, yaw_sign={}, pitch_sign={}, "
      "imu_yaw_sign={}, imu_pitch_sign={}, allow_control={}, allow_shoot={}, "
      "require_feedback={}, require_auto_mode={}, max_yaw_excursion_deg={:.2f}, "
      "lock_pitch={}, shoot_hold_ms={:.0f}, shoot_low_ms={:.0f}, min_request_interval_ms={:.0f}",
      port, baud, yaw_sign_, pitch_sign_, imu_yaw_sign_, imu_pitch_sign_, allow_control_,
      allow_shoot_, require_feedback_for_control_, require_auto_mode_for_control_,
      max_yaw_excursion_rad_ * 180.0 / M_PI, lock_pitch_for_control_,
      static_cast<double>(shoot_hold_duration_ns_) / 1.0e6,
      static_cast<double>(shoot_low_duration_ns_) / 1.0e6,
      static_cast<double>(shoot_min_interval_ns_) / 1.0e6);
    if (!allow_control_) {
      tools::logger()->warn("[XUC][SAFE] control output is forced to zero");
    }
    if (!allow_shoot_) {
      tools::logger()->warn("[XUC][SAFE] shoot output is forced to zero");
    }
  } catch (const serial::IOException & e) {
    enabled_ = false;
    tools::logger()->warn("[XUC] failed to open serial port {}: {}", port, e.what());
  }
}

XucSender::~XucSender()
{
  recv_quit_.store(true, std::memory_order_relaxed);
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

bool XucSender::rx_valid() const
{
  if (!rx_valid_.load(std::memory_order_relaxed)) return false;

  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto last_ns = last_rx_ns_.load(std::memory_order_relaxed);

  return last_ns > 0 && now_ns - last_ns <= RX_TIMEOUT_NS;
}

XucImuHistoryWindow XucSender::imu_history_window(
  std::chrono::steady_clock::time_point begin,
  std::chrono::steady_clock::time_point end) const
{
  XucImuHistoryWindow window;
  if (!enabled_ || begin > end) return window;
  std::lock_guard<std::mutex> lock(imu_history_mutex_);
  const auto first = std::lower_bound(
    imu_history_.begin(), imu_history_.end(), begin,
    [](const XucTimedImu & sample, const auto & time) { return sample.timestamp < time; });
  const auto last = std::upper_bound(
    first, imu_history_.end(), end,
    [](const auto & time, const XucTimedImu & sample) { return time < sample.timestamp; });
  window.total_samples = static_cast<std::size_t>(std::distance(first, last));
  constexpr std::size_t max_samples = 12;
  window.truncated = window.total_samples > max_samples;
  const auto copy_begin = window.truncated ? std::prev(last, max_samples) : first;
  window.samples.assign(copy_begin, last);
  return window;
}

bool XucSender::imu_at(
  std::chrono::steady_clock::time_point timestamp, XucTimedImu & imu) const
{
  if (!rx_valid()) return false;
  std::lock_guard<std::mutex> lock(imu_history_mutex_);
  if (imu_history_.empty()) return false;

  constexpr double max_nearest_ms = 20.0;
  constexpr double max_bracket_ms = 40.0;
  const auto nearest = [&](const XucTimedImu & sample) {
    const double offset_ms =
      std::chrono::duration<double, std::milli>(sample.timestamp - timestamp).count();
    if (std::abs(offset_ms) > max_nearest_ms) return false;
    imu = sample;
    imu.nearest_offset_ms = offset_ms;
    imu.bracket_ms = 0.0;
    imu.interpolated = false;
    imu.extrapolated = false;
    return true;
  };

  const auto upper = std::lower_bound(
    imu_history_.begin(), imu_history_.end(), timestamp,
    [](const XucTimedImu & sample, const auto & time) {
      return sample.timestamp < time;
    });
  if (upper == imu_history_.begin()) return nearest(*upper);
  if (upper == imu_history_.end()) {
    const auto & last = imu_history_.back();
    const double ahead_ms =
      std::chrono::duration<double, std::milli>(timestamp - last.timestamp).count();
    if (ahead_ms > 0.0 && ahead_ms <= 25.0 && imu_history_.size() >= 2) {
      const auto & previous = imu_history_[imu_history_.size() - 2];
      const double sample_ms =
        std::chrono::duration<double, std::milli>(last.timestamp - previous.timestamp).count();
      if (sample_ms >= 5.0 && sample_ms <= 20.0) {
        const double yaw_rate =
          std::remainder(last.yaw - previous.yaw, 2.0 * M_PI) / (sample_ms / 1000.0);
        const double pitch_rate = (last.pitch - previous.pitch) / (sample_ms / 1000.0);
        if (std::abs(yaw_rate) <= M_PI && std::abs(pitch_rate) <= M_PI) {
          imu = last;
          imu.timestamp = timestamp;
          imu.yaw += yaw_rate * (ahead_ms / 1000.0);
          imu.pitch += pitch_rate * (ahead_ms / 1000.0);
          imu.nearest_offset_ms = -ahead_ms;
          imu.bracket_ms = sample_ms;
          imu.interpolated = false;
          imu.extrapolated = true;
          return true;
        }
      }
    }
    return nearest(last);
  }

  const auto & before = *(upper - 1);
  const auto & after = *upper;
  const double bracket_ms =
    std::chrono::duration<double, std::milli>(after.timestamp - before.timestamp).count();
  if (bracket_ms <= 0.0 || bracket_ms > max_bracket_ms) {
    const double before_gap_ms =
      std::chrono::duration<double, std::milli>(timestamp - before.timestamp).count();
    const double after_gap_ms =
      std::chrono::duration<double, std::milli>(after.timestamp - timestamp).count();
    return nearest(before_gap_ms <= after_gap_ms ? before : after);
  }

  const double fraction =
    std::chrono::duration<double>(timestamp - before.timestamp).count() /
    std::chrono::duration<double>(after.timestamp - before.timestamp).count();
  imu.timestamp = timestamp;
  imu.yaw = before.yaw + fraction * std::remainder(after.yaw - before.yaw, 2.0 * M_PI);
  imu.pitch = before.pitch + fraction * (after.pitch - before.pitch);
  const double before_offset_ms =
    std::chrono::duration<double, std::milli>(before.timestamp - timestamp).count();
  const double after_offset_ms =
    std::chrono::duration<double, std::milli>(after.timestamp - timestamp).count();
  imu.nearest_offset_ms =
    std::abs(before_offset_ms) <= std::abs(after_offset_ms)
      ? before_offset_ms : after_offset_ms;
  imu.bracket_ms = bracket_ms;
  imu.interpolated = true;
  imu.extrapolated = false;
  return true;
}

bool XucSender::shoot_request_ready_at(int64_t now_ns) const
{
  return enabled_ && serial_.isOpen() && allow_shoot_ && control_ready() &&
    shoot_hold_until_ns_ == 0 && !shoot_low_pending_ &&
    (shoot_low_until_ns_ == 0 || now_ns >= shoot_low_until_ns_) &&
    now_ns >= shoot_next_request_ns_;
}

bool XucSender::shoot_request_ready() const
{
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  return shoot_request_ready_at(now_ns);
}

void XucSender::note_shot_debug_drop() noexcept
{
  if (shot_debug_) shot_debug_->dropped.fetch_add(1, std::memory_order_relaxed);
}

void XucSender::record_shot_debug(nlohmann::json && snapshot) noexcept
{
  if (shot_debug_) shot_debug_->push(std::move(snapshot));
}

bool XucSender::send_debug(const Command & cmd, const XucShotDebugContext * request)
{
  if (!shot_debug_) {
    send(cmd);
    return false;
  }
  const auto ns_now = [] {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  };
  const auto dispatch_begin_ns = ns_now();
  const auto count_before = successful_tx_count();
  const auto high_before = last_shoot_tx_ns();
  const auto hold_before = shoot_hold_until_ns_;
  const auto low_before = shoot_low_until_ns_;
  send(cmd);  // Identical control, packet construction and serial-write path.
  const auto dispatch_end_ns = ns_now();
  const bool tx_ok = successful_tx_count() != count_before;
  const bool accepted = request && request->id != 0 && cmd.control && cmd.shoot &&
    hold_before == 0 && shoot_hold_until_ns_ > 0;
  auto & debug = *shot_debug_;
  if (accepted) {
    debug.cycle = *request;
    debug.cycle_active = true;
    debug.first_high_ns = debug.first_high_complete_ns = 0;
  }
  const bool first_high = tx_ok && last_tx_shoot() == 1 &&
    last_shoot_tx_ns() != high_before;
  const bool first_low = tx_ok && last_tx_shoot() == 0 &&
    shoot_low_until_ns_ > 0 && shoot_low_until_ns_ != low_before;
  if (first_high && debug.cycle_active) {
    debug.first_high_ns = last_tx_ns();
    debug.first_high_complete_ns = last_tx_complete_ns();
  }
  if (debug.cycle_active && (first_high || first_low || !tx_ok)) {
    try {
      nlohmann::json record = {
        {"kind", first_high ? "first_high" : (first_low ? "first_low" : "write_not_confirmed")},
        {"id", debug.cycle.id}, {"run", debug.cycle.run},
        {"frame_seq", debug.cycle.frame_seq}, {"frame_id", debug.cycle.frame_id},
        {"frame_ns", debug.cycle.frame_ns}, {"frame_basis", "request_frame"},
        {"tx_frame_seq", request ? nlohmann::json(request->frame_seq) : nlohmann::json(nullptr)},
        {"tx_frame_ns", request ? nlohmann::json(request->frame_ns) : nlohmann::json(nullptr)},
        {"tx_ok", tx_ok},
        {"dispatch_begin_ns", dispatch_begin_ns}, {"dispatch_end_ns", dispatch_end_ns},
        {"packet_prepare_ns", tx_ok ? nlohmann::json(last_tx_ns()) : nlohmann::json(nullptr)},
        {"serial_write_begin_ns", nullptr},
        {"tx_write_complete_ns", tx_ok ? nlohmann::json(last_tx_complete_ns()) : nlohmann::json(nullptr)},
        {"tx_shoot", tx_ok ? nlohmann::json(last_tx_shoot()) : nlohmann::json(nullptr)},
        {"tx_control", tx_ok ? nlohmann::json(last_tx_control()) : nlohmann::json(nullptr)},
        {"tx_yaw_raw_rad", tx_ok ? nlohmann::json(last_tx_yaw_raw()) : nlohmann::json(nullptr)},
        {"tx_pitch_raw_rad", tx_ok ? nlohmann::json(last_tx_pitch_raw()) : nlohmann::json(nullptr)},
        {"first_high_ns", debug.first_high_ns > 0 ? nlohmann::json(debug.first_high_ns) : nlohmann::json(nullptr)},
        {"first_high_ns_basis", "packet_prepare"},
        {"first_high_write_complete_ns", debug.first_high_complete_ns > 0 ? nlohmann::json(debug.first_high_complete_ns) : nlohmann::json(nullptr)},
        {"low_interval_begin_ns", first_low ? nlohmann::json(last_tx_complete_ns()) : nlohmann::json(nullptr)},
        {"low_interval_until_ns", first_low ? nlohmann::json(shoot_low_until_ns_) : nlohmann::json(nullptr)},
        {"projectile_exit", "unknown"}, {"timestamp_basis", "host_steady_clock"}};
      debug.push(std::move(record));
    } catch (...) {
      note_shot_debug_drop();
    }
  }
  if (first_low) debug.cycle_active = false;
  return accepted;
}

void XucSender::send(const Command & cmd)
{
  if (!enabled_ || !serial_.isOpen()) return;

  XucRxPacket pkt{};
  pkt.head[0] = 'S';
  pkt.head[1] = 'P';
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const bool link_ready = control_ready();
  if (!link_ready && shoot_hold_until_ns_ > 0) {
    // Cancel high output on loss/exit, but require a successfully sent zero
    // followed by the complete low interval. Neither deadline is reset by
    // repeated idle packets, feedback loss, or a brief AUTO switch transition.
    shoot_hold_until_ns_ = 0;
    shoot_low_pending_ = true;
  }
  if (link_ready && shoot_hold_until_ns_ > 0 && now_ns >= shoot_hold_until_ns_) {
    shoot_hold_until_ns_ = 0;
    shoot_low_pending_ = true;
  }
  if (!shoot_low_pending_ && shoot_low_until_ns_ > 0 && now_ns >= shoot_low_until_ns_) {
    shoot_low_until_ns_ = 0;
  }
  const bool new_shoot_request =
    cmd.control && cmd.shoot && shoot_request_ready_at(now_ns);
  if (new_shoot_request) {
    shoot_hold_until_ns_ = now_ns + shoot_hold_duration_ns_;
    shoot_cycle_tx_started_ = false;
    shoot_hold_yaw_command_ = cmd.yaw;
    shoot_hold_pitch_command_ = cmd.pitch;
  }
  const bool shoot_hold_active =
    link_ready && allow_shoot_ && now_ns < shoot_hold_until_ns_;
  const bool control_requested = link_ready && (cmd.control || shoot_hold_active);
  pkt.control = control_requested ? 1 : 0;
  // The lower board treats a bounded shoot=1 level as one single-shot request.
  // A successfully transmitted shoot=0 packet starts the mandatory low interval,
  // so slow frames or failed writes cannot shorten the configured release time.
  pkt.shoot = shoot_hold_active ? 1 : 0;

  const double selected_yaw_command =
    shoot_hold_active ? shoot_hold_yaw_command_ : cmd.yaw;
  const double selected_pitch_command =
    shoot_hold_active ? shoot_hold_pitch_command_ : cmd.pitch;
  double target_yaw_raw = yaw_sign_ * selected_yaw_command;
  double target_pitch_raw = pitch_sign_ * selected_pitch_command;
  const bool bounded_session =
    max_yaw_excursion_rad_ > 0.0 || lock_pitch_for_control_;

  // This diagnostic profile intentionally permits only one yaw origin per
  // process.  Detector dropouts, link recovery, or a chattering feedback mode
  // must never move that origin.  A new physical test requires stopping and
  // restarting the process, which is also the safest explicit re-arm boundary.
  if (control_requested && bounded_session) {
    if (!control_session_active_) {
      control_yaw_anchor_raw_ =
        static_cast<double>(imu_yaw_.load(std::memory_order_relaxed));
      control_session_active_ = true;
      tools::logger()->warn(
        "[XUC][BOUNDED] control session anchored at raw yaw {:.2f} deg",
        control_yaw_anchor_raw_ * 180.0 / M_PI);
    }

    if (max_yaw_excursion_rad_ > 0.0) {
      double delta = target_yaw_raw - control_yaw_anchor_raw_;
      delta = std::remainder(delta, 2.0 * M_PI);
      delta = std::clamp(delta, -max_yaw_excursion_rad_, max_yaw_excursion_rad_);
      target_yaw_raw = control_yaw_anchor_raw_ + delta;
    }
    if (lock_pitch_for_control_) {
      target_pitch_raw =
        static_cast<double>(imu_pitch_.load(std::memory_order_relaxed));
    }
  } else if (!bounded_session) {
    control_session_active_ = false;
  }

  pkt.yaw = static_cast<float>(target_yaw_raw);
  pkt.pitch = static_cast<float>(target_pitch_raw);

  // CRC16 对前 12 字节计算，低字节在前
  uint16_t crc = tools::get_crc16(reinterpret_cast<const uint8_t *>(&pkt),
                                  offsetof(XucRxPacket, crc16));
  uint8_t buf[sizeof(XucRxPacket)]{};
  std::memcpy(buf, &pkt, offsetof(XucRxPacket, crc16));
  buf[offsetof(XucRxPacket, crc16)] = static_cast<uint8_t>(crc & 0xff);
  buf[offsetof(XucRxPacket, crc16) + 1] = static_cast<uint8_t>((crc >> 8) & 0xff);

  try {
    const auto written = serial_.write(buf, sizeof(buf));
    if (written == sizeof(buf)) {
      const auto write_complete_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      if (pkt.shoot == 1 && !shoot_cycle_tx_started_) {
        // Also catches the first successful retry after an initial failed write.
        // Start the complete hold and the hard cadence from that success only;
        // continuing high packets must not extend either deadline.
        shoot_cycle_tx_started_ = true;
        shoot_hold_until_ns_ = write_complete_ns + shoot_hold_duration_ns_;
        shoot_next_request_ns_ = write_complete_ns + shoot_min_interval_ns_;
        last_shoot_tx_ns_.store(now_ns, std::memory_order_release);
        tools::logger()->info(
          "[XUC][TX] shoot=1 hold_ms={:.0f} control=1",
          static_cast<double>(shoot_hold_duration_ns_) / 1.0e6);
      }
      if (shoot_low_pending_ && pkt.shoot == 0) {
        shoot_low_pending_ = false;
        shoot_low_until_ns_ = write_complete_ns + shoot_low_duration_ns_;
        tools::logger()->info(
          "[XUC][TX] shoot=0 low_ms={:.0f} control={}",
          static_cast<double>(shoot_low_duration_ns_) / 1.0e6,
          static_cast<int>(pkt.control));
      }
      last_tx_yaw_raw_.store(pkt.yaw, std::memory_order_relaxed);
      last_tx_pitch_raw_.store(pkt.pitch, std::memory_order_relaxed);
      last_tx_control_.store(pkt.control, std::memory_order_relaxed);
      last_tx_shoot_.store(pkt.shoot, std::memory_order_relaxed);
      last_tx_ns_.store(now_ns, std::memory_order_relaxed);
      last_tx_complete_ns_.store(write_complete_ns, std::memory_order_relaxed);
      successful_tx_count_.fetch_add(1, std::memory_order_relaxed);
    } else if (written != sizeof(buf)) {
      tools::logger()->warn("[XUC] short write: {}/{} bytes", written, sizeof(buf));
    }
  } catch (const std::exception & e) {
    tools::logger()->warn("[XUC] write failed: {}", e.what());
  }
}

// 接收线程：读串口字节 → 扫描 'S''P' + CRC16 校验 → 解析 TxPacket_TJ（20 字节）
void XucSender::recv_loop()
{
  uint8_t scratch[64];
  std::vector<uint8_t> buf;
  buf.reserve(sizeof(XucTxPacket) * 2);
  int64_t previous_feedback_ns = 0;
  double previous_feeder_rpm = 0.0;

  while (!recv_quit_.load(std::memory_order_relaxed)) {
    size_t n = 0;
    try {
      n = serial_.read(scratch, sizeof(scratch));
    } catch (const std::exception &) {
      break;
    }
    if (n == 0) continue;  // 超时无数据
    rx_bytes_.fetch_add(static_cast<uint64_t>(n), std::memory_order_relaxed);
    buf.insert(buf.end(), scratch, scratch + n);

    size_t i = 0;
    while (i + sizeof(XucTxPacket) <= buf.size()) {
      if (buf[i] == 'S' && buf[i + 1] == 'P') {
        header_candidates_.fetch_add(1, std::memory_order_relaxed);
        uint16_t crc = tools::get_crc16(&buf[i], offsetof(XucTxPacket, crc16));
        if (static_cast<uint8_t>(crc & 0xff) == buf[i + offsetof(XucTxPacket, crc16)] &&
            static_cast<uint8_t>((crc >> 8) & 0xff) == buf[i + offsetof(XucTxPacket, crc16) + 1]) {
          XucTxPacket pkt{};
          std::memcpy(&pkt, &buf[i], sizeof(XucTxPacket));

          const bool fields_valid =
            pkt.mode <= 3 &&
            std::isfinite(pkt.bullet_speed) &&
            std::isfinite(pkt.imu_pitch) &&
            std::isfinite(pkt.imu_yaw);

          if (fields_valid) {
            const auto received_at = std::chrono::steady_clock::now();
            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
              received_at.time_since_epoch()).count();
            // This field is M2006 rotor RPM in the repository's infantry
            // firmware. A referee projectile-speed field would be below this
            // RPM-like range and must never be counted as feeder motion.
            const double feeder_rpm = static_cast<double>(pkt.bullet_speed);
            if (feeder_diagnostics_enabled_ && previous_feedback_ns > 0 &&
                std::abs(previous_feeder_rpm) <= 3000.0 &&
                std::abs(feeder_rpm) <= 3000.0 &&
                (std::abs(previous_feeder_rpm) >= 60.0 ||
                 std::abs(feeder_rpm) >= 60.0)) {
              const double interval_s =
                static_cast<double>(now_ns - previous_feedback_ns) / 1.0e9;
              if (interval_s > 0.0 && interval_s <= 0.05) {
                const double delta_turns =
                  0.5 * (previous_feeder_rpm + feeder_rpm) * interval_s / 60.0;
                feeder_motor_turns_estimate_.store(
                  feeder_motor_turns_estimate_.load(std::memory_order_relaxed) +
                    delta_turns,
                  std::memory_order_relaxed);
              }
            }
            previous_feedback_ns = now_ns;
            previous_feeder_rpm = feeder_rpm;
            {
              std::lock_guard<std::mutex> lock(imu_history_mutex_);
              if (imu_history_.empty() || received_at > imu_history_.back().timestamp) {
                XucTimedImu sample;
                sample.timestamp = received_at;
                sample.yaw = imu_yaw_sign_ * static_cast<double>(pkt.imu_yaw) +
                  imu_yaw_offset_rad_;
                sample.pitch = imu_pitch_sign_ * static_cast<double>(pkt.imu_pitch) +
                  imu_pitch_offset_rad_;
                imu_history_.push_back(sample);
                while (imu_history_.size() > 256 ||
                       received_at - imu_history_.front().timestamp >
                         std::chrono::seconds(2)) {
                  imu_history_.pop_front();
                }
              }
            }
            mode_.store(pkt.mode, std::memory_order_relaxed);
            robot_id_.store(pkt.robot_id, std::memory_order_relaxed);
            bullet_speed_.store(pkt.bullet_speed, std::memory_order_relaxed);
            bullet_count_.store(pkt.bullet_count, std::memory_order_relaxed);
            imu_pitch_.store(pkt.imu_pitch, std::memory_order_relaxed);
            imu_yaw_.store(pkt.imu_yaw, std::memory_order_relaxed);

            last_rx_ns_.store(now_ns, std::memory_order_relaxed);
            valid_packets_.fetch_add(1, std::memory_order_relaxed);
            rx_valid_.store(true, std::memory_order_relaxed);
          } else {
            field_errors_.fetch_add(1, std::memory_order_relaxed);
          }

          i += sizeof(XucTxPacket);
          continue;
        }
        crc_errors_.fetch_add(1, std::memory_order_relaxed);
      }
      ++i;
    }
    buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(i));
    // 防止残留缓冲无限增长（最多保留一个半包，供后续拼包）
    if (buf.size() > sizeof(XucTxPacket) + 2) {
      buf.erase(buf.begin(),
                buf.begin() + static_cast<std::ptrdiff_t>(buf.size() - sizeof(XucTxPacket) - 2));
    }
  }

  rx_valid_.store(false, std::memory_order_relaxed);
}

}  // namespace io
