#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"

// ROS2 前向声明（避免强制依赖）
#ifdef AMENT_CMAKE_FOUND
namespace rclcpp {
class Node;
class Time;
}
#endif

namespace auto_aim
{
class Tracker
{
public:
  struct ArmorAssociation
  {
    int detection_index = -1;
    int face_id = -1;
    double cost = 0.0;
    bool identity_fallback = false;
    ArmorName observed_name = ArmorName::not_armor;
    ArmorType observed_type = ArmorType::small;
    bool identity_fallback_type_corrected = false;
    // Owned, solved observation; fallback copies must outlive update_target().
    std::optional<Armor> observation;
    bool used_for_update = false;
  };

  struct FrameDiagnostics
  {
    bool target_state_available = false;
    bool initialized_this_frame = false;
    bool update_attempted = false;
    bool update_used_batch = false;
    bool identity_fallback_used = false;
    bool identity_fallback_used_type_corrected = false;
    double identity_fallback_best_cost = std::numeric_limits<double>::quiet_NaN();
    ArmorName identity_fallback_observed_name = ArmorName::not_armor;
    ArmorType identity_fallback_observed_type = ArmorType::small;
    bool identity_fallback_best_type_corrected = false;
    int association_input_detections = 0;
    int association_same_identity_detections = 0;
    int association_type_compatible_detections = 0;
    int association_projected_pairs = 0;
    int association_nonfinite_pairs = 0;
    int association_over_cost_pairs = 0;
    int association_best_detection = -1;
    int association_best_face = -1;
    ArmorName association_best_observed_name = ArmorName::not_armor;
    double association_best_cost = std::numeric_limits<double>::quiet_NaN();
    double association_best_center_error = std::numeric_limits<double>::quiet_NaN();
    double association_best_angle_error = std::numeric_limits<double>::quiet_NaN();
    double association_best_range_error = std::numeric_limits<double>::quiet_NaN();
    double association_best_confidence_bonus = std::numeric_limits<double>::quiet_NaN();
    double association_cost_threshold = std::numeric_limits<double>::quiet_NaN();
    bool association_best_within_threshold = false;
    std::string association_reject_reason = "not_evaluated";
    bool diverged = false;
    bool nis_reset = false;
    int update_observations = 0;
    ArmorName target_name = ArmorName::not_armor;
    int target_armor_count = 0;
    Eigen::VectorXd state_before;
    Eigen::VectorXd state_after;
    double whole_nis = std::numeric_limits<double>::quiet_NaN();
    double whole_nees = std::numeric_limits<double>::quiet_NaN();
    bool yaw_eskf_enabled = true;
    bool yaw_model_trusted = false;
    const char * yaw_model_source = "none";
    bool yaw_eskf_trusted = false;
    double yaw_eskf_angle = std::numeric_limits<double>::quiet_NaN();
    double yaw_eskf_rate = std::numeric_limits<double>::quiet_NaN();
    double yaw_eskf_acceleration = std::numeric_limits<double>::quiet_NaN();
    double yaw_eskf_nis = std::numeric_limits<double>::quiet_NaN();
    int yaw_eskf_accepted = 0;
    int yaw_eskf_rejected = 0;
    double center_acceleration_variance = std::numeric_limits<double>::quiet_NaN();
  };

  Tracker(
    const std::string & config_path, Solver & solver,
    std::optional<bool> yaw_eskf_override = std::nullopt);

  std::string state() const;
  int temp_lost_count() const { return temp_lost_count_; }
  const std::vector<ArmorAssociation> & last_associations() const;
  int primary_detection_index() const;
  int primary_face_id() const;
  const Armor * primary_observation() const;
  const FrameDiagnostics & frame_diagnostics() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

#ifdef AMENT_CMAKE_FOUND
  // 设置 ROS2 节点用于发布 marker（可选）
  void set_ros2_node(std::shared_ptr<rclcpp::Node> node);

  // 发布当前跟踪目标的 marker 到 rviz2
  // timestamp: 可选的时间戳，如果不提供则使用ros_node_->now()
  void publish_markers(
    const std::list<Target> & targets,
    const rclcpp::Time & timestamp,
    const Eigen::Vector4d & aim_xyza = Eigen::Vector4d::Zero(),
    bool has_aim_point = false);
#endif

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  double association_max_cost_;
  bool identity_fallback_enabled_ = false;
  double identity_fallback_max_cost_ = 0.75;
  int batch_max_observations_;
  double batch_min_omega_rad_s_;
  int target_switch_min_frames_ = 3;
  ArmorName switch_candidate_name_ = ArmorName::not_armor;
  int switch_candidate_frames_ = 0;
  YawESKF::Config yaw_eskf_config_;
  Target::IteratedUpdateConfig iterated_update_config_;
  Target::CenterProcessConfig center_process_config_;
  std::vector<ArmorAssociation> last_associations_;
  int primary_detection_index_;
  int primary_face_id_;
  FrameDiagnostics frame_diagnostics_;

#ifdef AMENT_CMAKE_FOUND
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::shared_ptr<void> marker_pub_;  // 使用 void* 类型擦除，避免暴露 ROS2 具体类型
#endif

  void state_machine(bool found);
  void capture_frame_diagnostics();

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
  Armor * select_target(std::list<Armor> & armors);
  bool should_switch_target(const std::list<Armor> & armors);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
