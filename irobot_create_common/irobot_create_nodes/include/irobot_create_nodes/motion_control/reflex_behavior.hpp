// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Justin Kearns (jkearns@irobot.com)

#ifndef IROBOT_CREATE_NODES__MOTION_CONTROL__REFLEX_BEHAVIOR_HPP_
#define IROBOT_CREATE_NODES__MOTION_CONTROL__REFLEX_BEHAVIOR_HPP_

#include <atomic>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_nodes/motion_control/behaviors_scheduler.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace irobot_create_nodes
{

/**
 * @brief This class allows to create and manage Docking and Undocking action
 * servers.
 */
class ReflexBehavior
{
public:
  ReflexBehavior(
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<BehaviorsScheduler> behavior_scheduler);
  ~ReflexBehavior() = default;

  void update_hazards(const RobotState & current_state);

private:
  /// \brief Name of parameter for enabling/disabling all reflexes
  const std::string reflex_enabled_param_name_{"reflexes_enabled"};
  /// \brief Map of reflex names to hazard type (max for no support right now)
  const std::map<std::string, uint8_t> reflex_names_to_hazard_ {
    {"REFLEX_BUMP", irobot_create_msgs::msg::HazardDetection::BUMP},
    {"REFLEX_CLIFF", irobot_create_msgs::msg::HazardDetection::CLIFF},
    {"REFLEX_DOCK_AVOID", std::numeric_limits<uint8_t>::max()},
    {"REFLEX_GYRO_CAL", std::numeric_limits<uint8_t>::max()},
    {"REFLEX_PANIC", std::numeric_limits<uint8_t>::max()},
    {"REFLEX_PROXIMITY_SLOWDOWN", std::numeric_limits<uint8_t>::max()},
    {"REFLEX_STUCK", irobot_create_msgs::msg::HazardDetection::STALL},
    {"REFLEX_VIRTUAL_WALL", std::numeric_limits<uint8_t>::max()},
    {"REFLEX_WHEEL_DROP", irobot_create_msgs::msg::HazardDetection::WHEEL_DROP}};
  std::mutex hazard_reflex_mutex_;
  /// \brief Map of hazard driven reflexes with whether they are enabled
  std::map<uint8_t, bool> hazard_reflex_enabled_ {
    {irobot_create_msgs::msg::HazardDetection::BUMP, true},
    {irobot_create_msgs::msg::HazardDetection::CLIFF, true},
    {irobot_create_msgs::msg::HazardDetection::STALL, true},
    {irobot_create_msgs::msg::HazardDetection::WHEEL_DROP, true}};
  /// \brief Whether to use reflexes
  std::atomic<bool> reflexes_enabled_{true};
  /// \brief Helper function to declare ROS 2 reflex parameters
  void declare_parameters(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface);

  /// \brief Helper function to validate changes to parameters
  rcl_interfaces::msg::SetParametersResult set_parameters_callback(
    std::vector<rclcpp::Parameter> parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  /// \brief Specify whether reflex is enabled or not
  void enable_reflex(uint8_t reflex, bool enable);

  bool reflex_behavior_is_done();

  boost::optional<tf2::Vector3> get_pose_relative_to_odom(
    const irobot_create_msgs::msg::HazardDetection & hazard);

  BehaviorsScheduler::optional_output_t execute_reflex(const RobotState & current_state);

  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Logger logger_;
  std::shared_ptr<BehaviorsScheduler> behavior_scheduler_;
  std::atomic<bool> running_reflex_ {false};
  std::mutex robot_pose_mutex_;
  tf2::Transform last_robot_pose_;
  tf2::Transform reflex_start_pose_;
  tf2::Transform continuous_reflex_start_pose_;
  std::mutex hazard_mutex_;
  std::vector<irobot_create_msgs::msg::HazardDetection> last_hazards_;
  std::vector<irobot_create_msgs::msg::HazardDetection> last_trigger_hazards_;
  rclcpp::Time reflex_start_time_;
  rclcpp::Time continuous_reflex_start_time_;
  const rclcpp::Duration max_reflex_runtime_;
  const rclcpp::Duration max_continuous_reflex_runtime_;
  const double MAX_REFLEX_DISTANCE {0.15};
  const double MAX_CONTINUOUS_REFLEX_DISTANCE {0.45};
  const double MIN_REFLEX_DISTANCE {0.05};
  const double BACKUP_X_VELOCITY {-0.14};
  const double ARC_X_VELOCITY {-0.1};
  const double ARC_ANGULAR_VELOCITY {M_PI / 16.0};
  std::atomic<bool> moving_{false};
  std::atomic<bool> driving_backwards_ {false};
  const std::string odom_frame_ {"odom"};
  const std::string base_frame_ {"base_link"};
  BehaviorsScheduler::optional_output_t last_reflex_cmd_;
  rclcpp::Time last_moving_check_time_;
  const double NOT_MOVING_DISTANCE {0.001};
  const double NOT_MOVING_ANGLE {0.001};
  const rclcpp::Duration moving_check_duration_;
};

}  // namespace irobot_create_nodes
#endif  // IROBOT_CREATE_NODES__MOTION_CONTROL__REFLEX_BEHAVIOR_HPP_
