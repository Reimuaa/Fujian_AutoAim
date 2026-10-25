#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  const auto directional_delay = [&](const char * key) {
    if (!yaml[key]) return high_speed_delay_time_;
    const auto value = yaml[key].as<double>();
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument(std::string(key) + " must be finite and non-negative");
    return value;
  };
  high_speed_delay_time_cw_ = directional_delay("high_speed_delay_time_cw");
  high_speed_delay_time_ccw_ = directional_delay("high_speed_delay_time_ccw");
  tools::logger()->info(
    "[Aimer] high-speed delay cw={:.1f}ms ccw={:.1f}ms fallback={:.1f}ms (viewed from above)",
    high_speed_delay_time_cw_ * 1000.0, high_speed_delay_time_ccw_ * 1000.0,
    high_speed_delay_time_ * 1000.0);
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  stationary_delay_time_ = yaml["stationary_delay_time"]
    ? yaml["stationary_delay_time"].as<double>() : low_speed_delay_time_;
  decision_speed_ = yaml["decision_speed"].as<double>();
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

void Aimer::set_delay_adjustment(double adjustment_s, AimTiming timing)
{
  const auto index = static_cast<std::size_t>(timing);
  if (index >= delay_adjustment_times_.size()) return;
  delay_adjustment_times_[index] = std::isfinite(adjustment_s)
    ? std::clamp(adjustment_s, -0.050, 0.050) : 0.0;
}

EventAimPrediction Aimer::predict_event(
  const Target & target, std::chrono::steady_clock::time_point impact_time,
  GyroFaceSelector & selector, int64_t hit_cycle) const
{
  EventAimPrediction out;
  if (target.armor_num() != 4 || hit_cycle < 0 ||
      impact_time.time_since_epoch().count() <= 0) return out;
  auto future = target;
  future.predict(impact_time);
  const auto state = future.ekf_x();
  if (state.size() != 11 || !state.allFinite()) return out;
  const auto faces = future.armor_xyza_list();
  std::vector<GyroFaceSelector::Candidate> candidates;
  candidates.reserve(faces.size());
  for (std::size_t face = 0; face < faces.size(); ++face) {
    if (faces[face].allFinite())
      candidates.push_back({static_cast<int>(face), faces[face][3]});
  }
  const auto selected = selector.select(
    candidates, std::atan2(state[2], state[0]), hit_cycle);
  if (!selected.valid || selected.face_id < 0 ||
      static_cast<std::size_t>(selected.face_id) >= faces.size()) return out;
  out.xyza = faces[selected.face_id];
  if (std::hypot(out.xyza[0], out.xyza[1]) <= 0.0) return out;
  out.yaw_rad = std::atan2(out.xyza[1], out.xyza[0]) + yaw_offset_;
  out.valid = std::isfinite(out.yaw_rad);
  out.face_changed = selected.changed;
  out.face_id = selected.face_id;
  out.alignment_rad = selected.alignment_rad;
  out.prediction_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    impact_time.time_since_epoch()).count();
  return out;
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now, AimPredictionTrace * trace, AimGeometry geometry, AimTiming timing)
{
  if (trace) *trace = AimPredictionTrace{};
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();

  const auto & raw_state = target.ekf().x;
  // Decide once at the frame boundary, so all ballistic iterations use the
  // same model. Stationary/direct-plate and outpost paths retain their model.
  const bool whole_ekf_geometry = geometry == AimGeometry::whole_ekf &&
    target.armor_num() == 4 && target.name != ArmorName::outpost &&
    std::abs(target.ekf_x()[7]) > decision_speed_ &&
    raw_state.size() == 11 && raw_state.allFinite() &&
    raw_state[8] > 0.05 && raw_state[8] < 0.5 &&
    raw_state[8] + raw_state[9] > 0.05 && raw_state[8] + raw_state[9] < 0.5;
  if (trace) trace->whole_ekf_geometry = whole_ekf_geometry || !target.yaw_eskf_enabled();
  double delay_time;
  switch (timing) {
    case AimTiming::stationary: delay_time = stationary_delay_time_; break;
    case AimTiming::low_speed: delay_time = low_speed_delay_time_; break;
    case AimTiming::high_speed: {
      // Match the angular-rate source used by this prediction's geometry.
      // Positive world yaw is CCW when viewed from above (+Z toward the origin).
      const double omega = whole_ekf_geometry ? raw_state[7] : target.ekf_x()[7];
      const int sign = std::isfinite(omega) ? (omega > 0.0 ? 1 : (omega < 0.0 ? -1 : 0)) : 0;
      delay_time = sign > 0 ? high_speed_delay_time_ccw_
        : (sign < 0 ? high_speed_delay_time_cw_ : high_speed_delay_time_);
      if (trace) trace->high_speed_rotation_sign = sign;
      break;
    }
    default:
      delay_time = std::abs(target.ekf_x()[7]) > decision_speed_
        ? high_speed_delay_time_ : low_speed_delay_time_;
  }
  const auto timing_index = static_cast<std::size_t>(timing);
  delay_time += delay_adjustment_times_[timing_index < delay_adjustment_times_.size()
    ? timing_index : 0];

  // Low-speed friction-wheel benches can be well below the competition-speed
  // default.  Preserve the fallback only for missing/invalid telemetry so the
  // trajectory and flight-time predictor can use an explicitly calibrated value.
  if (!std::isfinite(bullet_speed) || bullet_speed < 5 || bullet_speed > 35) {
    bullet_speed = 23;
  }
  if (trace) {
    trace->delay_s = delay_time;
    trace->speed_mps = bullet_speed;
  }

  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;
  if (to_now) {
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  else {
    auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
    // tools::logger()->info("dt is {:.4f} second", dt);
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  if (trace) trace->base_prediction_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    future.time_since_epoch()).count();
  if (timing == AimTiming::high_speed && whole_ekf_geometry && target.jumped) {
    // A face need not be selectable at the intermediate base time. Solve each
    // physical face consistently, then apply the existing window at its final
    // point. Explicit high-speed timing keeps the legacy paths unchanged.
    debug_aim_point = {false, Eigen::Vector4d::Zero()};
    if (trace) trace->final_point_selection = true;
    const auto base_faces = target.whole_ekf_armor_xyza_list();
    for (std::size_t face = 0; face < base_faces.size(); ++face) {
      const auto & initial = base_faces[face];
      tools::Trajectory initial_traj(bullet_speed, initial.head<2>().norm(), initial[2]);
      if (!initial.allFinite() || initial_traj.unsolvable ||
          !std::isfinite(initial_traj.fly_time) || initial_traj.fly_time <= 0.0) continue;
      double previous_flight = initial_traj.fly_time;
      for (int iteration = 0; iteration < 10; ++iteration) {
        const auto prediction_time = future + std::chrono::microseconds(
          static_cast<int>(previous_flight * 1e6));
        const auto candidate = target.predict_rigid_armor(static_cast<int>(face), prediction_time);
        if (!candidate) break;
        const auto & point = candidate->xyza;
        tools::Trajectory trajectory(bullet_speed, point.head<2>().norm(), point[2]);
        if (trajectory.unsolvable || !std::isfinite(trajectory.fly_time) ||
            trajectory.fly_time <= 0.0) break;
        if (std::abs(trajectory.fly_time - previous_flight) < high_speed_flight_tolerance_s) {
          const double phase = tools::limit_rad(
            point[3] - std::atan2(candidate->center[1], candidate->center[0]));
          const bool selectable = std::abs(phase) <= comming_angle_ &&
            (target.ekf().x[7] > 0.0 ? phase < leaving_angle_ : phase > -leaving_angle_);
          const double yaw = std::atan2(point[1], point[0]) + yaw_offset_;
          const double pitch = -(trajectory.pitch + pitch_offset_);
          if (!selectable || !std::isfinite(yaw) || !std::isfinite(pitch)) break;
          debug_aim_point = {true, point};
          if (trace) {
            trace->valid = trace->converged = true;
            trace->face_id = static_cast<int>(face);
            trace->prediction_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
              prediction_time.time_since_epoch()).count();
            trace->flight_time_used_s = previous_flight;
            trace->flight_time_s = trajectory.fly_time;
            trace->iterations = iteration + 1;
          }
          return {true, false, yaw, pitch};
        }
        previous_flight = trajectory.fly_time;
      }
    }
    return {false, false, 0, 0};
  }
  auto aim_point0 = choose_aim_point(target, whole_ekf_geometry);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // Explicit high-speed rigid-body aiming uses 10us; legacy modes keep 1ms.
  bool converged = false;
  const bool strict_high_speed = timing == AimTiming::high_speed && whole_ekf_geometry;
  const double flight_tolerance_s = strict_high_speed ? high_speed_flight_tolerance_s : 0.001;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  std::vector<Target> iteration_target(10, target);  // 创建10个目标副本用于迭代预测

  for (int iter = 0; iter < 10; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].predict(predict_time);

    // 计算瞄准点
    auto aim_point = choose_aim_point(
      iteration_target[iter], whole_ekf_geometry, trace ? &trace->face_id : nullptr);
    if (trace) {
      trace->prediction_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        predict_time.time_since_epoch()).count();
      trace->flight_time_used_s = prev_fly_time;
      trace->iterations = iter + 1;
    }
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 检查收敛条件
    if (std::abs(current_traj.fly_time - prev_fly_time) < flight_tolerance_s) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // No high-speed command may use a solution that exhausted its iteration budget.
  // Stationary/low-speed/automatic callers retain their historical behaviour.
  if (strict_high_speed && !converged) {
    debug_aim_point.valid = false;
    if (trace) trace->flight_time_s = current_traj.fly_time;
    return {false, false, 0, 0};
  }

  // 计算最终角度
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负
  if (trace) {
    trace->valid = std::isfinite(yaw) && std::isfinite(pitch);
    trace->converged = converged;
    trace->flight_time_s = current_traj.fly_time;
  }
  return {true, false, yaw, pitch};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now, AimPredictionTrace * trace, AimGeometry geometry,
  AimTiming timing)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now, trace, geometry, timing);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target, bool whole_ekf_geometry, int * face_id)
{
  Eigen::VectorXd ekf_x = whole_ekf_geometry ? target.ekf().x : target.ekf_x();
  const bool prefer_direct_motion =
    !whole_ekf_geometry && std::abs(ekf_x[7]) <= decision_speed_ &&
    target.name != ArmorName::outpost;
  std::vector<Eigen::Vector4d> armor_xyza_list =
    whole_ekf_geometry ? target.whole_ekf_armor_xyza_list()
                       : target.armor_xyza_list(prefer_direct_motion);
  auto armor_num = armor_xyza_list.size();
  const auto select = [&](bool valid, int id) -> AimPoint {
    if (face_id) *face_id = valid ? id : -1;
    return {valid, armor_xyza_list[id]};
  };
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return select(true, 0);

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  if (prefer_direct_motion) {
    // At low rotation rate the currently observed physical plate is the most
    // reliable target. Prefer its fresh CV track instead of re-selecting among
    // several model faces at every ballistic iteration; the latter produced
    // red-box jumps and corresponding gimbal reversals near a face hand-over.
    if (target.yaw_eskf_enabled() && target.last_id >= 0 &&
        target.last_id < static_cast<int>(armor_num) &&
        target.armor_motion_trusted(target.last_id) &&
        std::abs(delta_angle_list[target.last_id]) <= 75.0 / 57.3) {
      lock_id_ = target.last_id;
      return select(true, target.last_id);
    }

    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return select(false, 0);
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return select(true, static_cast<int>(lock_id_));
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return select(true, id_list[0]);
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return select(true, i);
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return select(true, i);
  }

  return select(false, 0);
}

}  // namespace auto_aim
