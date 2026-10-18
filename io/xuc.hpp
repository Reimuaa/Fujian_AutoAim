#ifndef IO__XUC_HPP
#define IO__XUC_HPP

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <serial/serial.h>
#include <nlohmann/json_fwd.hpp>

#include "io/command.hpp"

namespace io
{
struct XucTimedImu
{
  std::chrono::steady_clock::time_point timestamp{};
  double yaw = 0.0;
  double pitch = 0.0;
  double nearest_offset_ms = 0.0;
  double bracket_ms = 0.0;
  bool interpolated = false;
  bool extrapolated = false;
};

struct XucImuHistoryWindow
{
  std::vector<XucTimedImu> samples;
  std::size_t total_samples = 0;
  bool truncated = false;
};

// Host-side diagnostics only. This is never part of either wire packet.
struct XucShotDebugContext
{
  uint64_t id = 0;
  int64_t run = 0;
  uint64_t frame_seq = 0;
  int64_t frame_id = 0;
  int64_t frame_ns = 0;
};

struct XucShooterFeedback
{
  int64_t host_rx_ns;
  uint16_t feedback_sequence;
  uint16_t fire_flag;
  uint32_t fire_tick_ms;
  int16_t friction_current_raw;
  int16_t feeder_speed_rpm;
};

// XUC 上位机→下位机：与新版下位机 RxPacket_TJ 完全一致（14 字节）
#pragma pack(push, 1)
struct XucRxPacket
{
  uint8_t head[2];   // 'S' 'P'
  uint8_t control;   // 是否控制云台
  uint8_t shoot;     // 是否开火
  float yaw;         // 目标 yaw 角
  float pitch;       // 目标 pitch 角
  uint16_t crc16;    // CRC16（低字节在前）
};
#pragma pack(pop)
static_assert(sizeof(XucRxPacket) == 14, "XucRxPacket must be 14 bytes");
static_assert(offsetof(XucRxPacket, crc16) == 12, "XucRxPacket CRC offset must be 12");

// XUC 下位机→上位机：与当前 omni-code TxPacket_TJ 一致（30 字节）。
#pragma pack(push, 1)
struct XucTxPacket
{
  uint8_t head[2];        // 'S' 'P'
  uint8_t mode;           // 0 空闲, 1 自瞄, 2 小符, 3 大符
  uint8_t robot_id;       // 3 红方, 103 蓝方
  float bullet_speed;     // 弹速 (m/s)
  uint16_t bullet_count;  // omni-code当前实现为回传包递增序号
  float imu_pitch;        // IMU 俯仰角 (rad)
  float imu_yaw;          // IMU 航向角 (rad)
  uint16_t fire_flag;     // 下位机电流阈值检测状态，不是已验证的出膛事件
  uint32_t fire_tick_ms;  // 下位机最近检测事件的 FreeRTOS tick（1 kHz）
  int16_t friction_current_raw;  // can2_motor[0] 的原始电流反馈，未换算安培
  int16_t feeder_speed_rpm;      // can2_motor[2] 电机转子 RPM
  uint16_t crc16;         // CRC16（低字节在前）
};
#pragma pack(pop)
static_assert(sizeof(XucTxPacket) == 30, "XucTxPacket must be 30 bytes");
static_assert(offsetof(XucTxPacket, fire_flag) == 18, "XucTxPacket fire flag offset must be 18");
static_assert(offsetof(XucTxPacket, fire_tick_ms) == 20, "XucTxPacket fire tick offset must be 20");
static_assert(offsetof(XucTxPacket, friction_current_raw) == 24, "XucTxPacket current offset must be 24");
static_assert(offsetof(XucTxPacket, feeder_speed_rpm) == 26, "XucTxPacket feeder RPM offset must be 26");
static_assert(offsetof(XucTxPacket, crc16) == 28, "XucTxPacket CRC offset must be 28");

// 向下位机发送目标角；接收 mode/robot_id/弹速/回传序号/IMU。
class XucSender
{
public:
  explicit XucSender(const std::string & config_path);
  ~XucSender();

  bool enabled() const { return enabled_; }
  void send(const Command & cmd);
  // Debug caller only: send() itself remains the compact-mode path unchanged.
  // Returns whether this request started a new software high/low cycle.
  bool send_debug(const Command & cmd, const XucShotDebugContext * request = nullptr);
  bool shot_debug_enabled() const { return shot_debug_ != nullptr; }
  void record_shot_debug(nlohmann::json && snapshot) noexcept;
  void note_shot_debug_drop() noexcept;
  // Called on the sending thread. Qualification waits for both the wire-level
  // release interval and the deadline retained across feedback/mode dropouts.
  bool shoot_request_ready() const;
  bool control_ready() const
  {
    return allow_control_ && (!require_feedback_for_control_ || rx_valid()) &&
           (!require_auto_mode_for_control_ || mode() == 1);
  }

  // 下位机回传数据（接收线程更新；rx_valid() 为 false 时这些值不可信）
  bool rx_valid() const;
  int64_t last_rx_ns() const
  {
    return last_rx_ns_.load(std::memory_order_relaxed);
  }
  uint64_t successful_tx_count() const
  {
    return successful_tx_count_.load(std::memory_order_relaxed);
  }
  int64_t last_tx_ns() const
  {
    return last_tx_ns_.load(std::memory_order_relaxed);
  }
  int64_t last_tx_complete_ns() const
  {
    return last_tx_complete_ns_.load(std::memory_order_relaxed);
  }
  double last_tx_yaw_raw() const
  {
    return static_cast<double>(last_tx_yaw_raw_.load(std::memory_order_relaxed));
  }
  double last_tx_pitch_raw() const
  {
    return static_cast<double>(last_tx_pitch_raw_.load(std::memory_order_relaxed));
  }
  bool last_tx_control() const
  {
    return last_tx_control_.load(std::memory_order_relaxed) != 0;
  }
  uint8_t last_tx_shoot() const
  {
    return last_tx_shoot_.load(std::memory_order_relaxed);
  }
  double shoot_hold_ms() const
  {
    return static_cast<double>(shoot_hold_duration_ns_) / 1.0e6;
  }
  double shoot_low_ms() const
  {
    return static_cast<double>(shoot_low_duration_ns_) / 1.0e6;
  }
  double shoot_min_interval_ms() const
  {
    return static_cast<double>(shoot_min_interval_ns_) / 1.0e6;
  }
  uint8_t mode() const { return mode_.load(std::memory_order_relaxed); }
  uint8_t robot_id() const { return robot_id_.load(std::memory_order_relaxed); }
  double bullet_speed() const
  {
    return static_cast<double>(bullet_speed_.load(std::memory_order_relaxed));
  }
  double feeder_speed_rpm() const
  {
    return static_cast<double>(feeder_speed_rpm_.load(std::memory_order_relaxed));
  }
  // Diagnostic only: integrate the explicit feeder rotor RPM field.
  double feeder_motor_turns_estimate() const
  {
    return feeder_motor_turns_estimate_.load(std::memory_order_relaxed);
  }
  uint16_t bullet_count() const { return bullet_count_.load(std::memory_order_relaxed); }
  // Drain on the sending thread so ShotDebug retains its single producer.
  // Capture is disabled in compact mode; at most 256 pending samples are kept.
  std::vector<XucShooterFeedback> take_shooter_feedback();
  uint64_t shooter_feedback_dropped() const
  {
    return shooter_feedback_dropped_.load(std::memory_order_relaxed);
  }
  int64_t last_shoot_tx_ns() const
  {
    return last_shoot_tx_ns_.load(std::memory_order_acquire);
  }
  double imu_pitch() const
  {
    return imu_pitch_sign_ * static_cast<double>(imu_pitch_.load(std::memory_order_relaxed)) +
           imu_pitch_offset_rad_;
  }
  double imu_yaw() const
  {
    return imu_yaw_sign_ * static_cast<double>(imu_yaw_.load(std::memory_order_relaxed)) +
           imu_yaw_offset_rad_;
  }
  // Interpolate host-receive-timestamped IMU samples at the camera frame time.
  // Returns false if the frame is outside the bounded history or feedback is stale.
  bool imu_at(std::chrono::steady_clock::time_point timestamp, XucTimedImu & imu) const;

  // Diagnostic snapshot of un-interpolated feedback, with host parse timestamps.
  // Keep at most the newest 12 samples in [begin, end], ordered oldest first.
  XucImuHistoryWindow imu_history_window(
    std::chrono::steady_clock::time_point begin,
    std::chrono::steady_clock::time_point end) const;
  // Raw lower-board yaw is the only coordinate shared by the command and IMU
  // fields.  Expose it for the upper yaw tracking loop so that actuator error is
  // never formed by subtracting two differently signed display/world angles.
  double raw_imu_yaw() const
  {
    return static_cast<double>(imu_yaw_.load(std::memory_order_relaxed));
  }
  double command_yaw_to_raw(double command_yaw_rad) const
  {
    return yaw_sign_ * command_yaw_rad;
  }
  double raw_yaw_correction_to_command(double raw_correction_rad) const
  {
    return std::abs(yaw_sign_) > 1e-9 ? raw_correction_rad / yaw_sign_ : 0.0;
  }
  uint64_t rx_bytes() const { return rx_bytes_.load(std::memory_order_relaxed); }
  uint64_t header_candidates() const
  {
    return header_candidates_.load(std::memory_order_relaxed);
  }
  uint64_t crc_errors() const { return crc_errors_.load(std::memory_order_relaxed); }
  uint64_t field_errors() const { return field_errors_.load(std::memory_order_relaxed); }
  uint64_t valid_packets() const
  {
    return valid_packets_.load(std::memory_order_relaxed);
  }

private:
  struct ShotDebug;
  std::unique_ptr<ShotDebug> shot_debug_;
  void recv_loop();
  bool shoot_request_ready_at(int64_t now_ns) const;

  serial::Serial serial_;
  bool enabled_ = false;
  double yaw_sign_ = 1.0;     // 目标 yaw 符号
  double pitch_sign_ = -1.0;  // 目标 pitch 符号（与现有 CAN 发送 -command.pitch 一致）
  double imu_yaw_sign_ = 1.0;
  double imu_pitch_sign_ = 1.0;
  double imu_yaw_offset_rad_ = 0.0;
  double imu_pitch_offset_rad_ = 0.0;
  bool allow_control_ = false;
  bool allow_shoot_ = false;
  bool feeder_diagnostics_enabled_ = false;
  bool require_feedback_for_control_ = true;
  bool require_auto_mode_for_control_ = false;
  bool lock_pitch_for_control_ = false;
  double max_yaw_excursion_rad_ = 0.0;
  bool control_session_active_ = false;
  double control_yaw_anchor_raw_ = 0.0;
  int64_t shoot_hold_duration_ns_ = 450000000;
  int64_t shoot_low_duration_ns_ = 200000000;
  int64_t shoot_min_interval_ns_ = 700000000;
  int64_t shoot_hold_until_ns_ = 0;
  int64_t shoot_low_until_ns_ = 0;
  int64_t shoot_next_request_ns_ = 0;
  bool shoot_low_pending_ = false;
  bool shoot_cycle_tx_started_ = false;
  double shoot_hold_yaw_command_ = 0.0;
  double shoot_hold_pitch_command_ = 0.0;

  std::thread recv_thread_;
  std::atomic<bool> recv_quit_{false};
  std::atomic<bool> rx_valid_{false};
  std::atomic<int64_t> last_rx_ns_{0};
  std::atomic<uint64_t> successful_tx_count_{0};
  std::atomic<int64_t> last_tx_ns_{0};
  std::atomic<int64_t> last_tx_complete_ns_{0};
  std::atomic<float> last_tx_yaw_raw_{0.0f};
  std::atomic<float> last_tx_pitch_raw_{0.0f};
  std::atomic<uint8_t> last_tx_control_{0};
  std::atomic<uint8_t> last_tx_shoot_{0};
  std::atomic<double> feeder_motor_turns_estimate_{0.0};
  static constexpr int64_t RX_TIMEOUT_NS = 500000000;
  std::atomic<uint8_t> mode_{0};
  std::atomic<uint8_t> robot_id_{0};
  std::atomic<float> bullet_speed_{0.0f};
  std::atomic<int16_t> feeder_speed_rpm_{0};
  std::atomic<uint16_t> bullet_count_{0};
  std::atomic<float> imu_pitch_{0.0f};
  std::atomic<float> imu_yaw_{0.0f};
  mutable std::mutex imu_history_mutex_;
  std::deque<XucTimedImu> imu_history_;
  std::vector<XucShooterFeedback> shooter_feedback_;
  std::atomic<uint64_t> shooter_feedback_dropped_{0};
  std::atomic<int64_t> last_shoot_tx_ns_{0};
  std::atomic<uint64_t> rx_bytes_{0};
  std::atomic<uint64_t> header_candidates_{0};
  std::atomic<uint64_t> crc_errors_{0};
  std::atomic<uint64_t> field_errors_{0};
  std::atomic<uint64_t> valid_packets_{0};
};

}  // namespace io

#endif  // IO__XUC_HPP
