// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Alberto Soragna (asoragna@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__MOTION_CONTROL_NODE_HPP_
#define IROBOT_CREATE_TOOLBOX__MOTION_CONTROL_NODE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <irobot_create_msgs/msg/kidnap_status.hpp>
#include <irobot_create_toolbox/parameter_helper.hpp>
#include <irobot_create_toolbox/motion_control/docking_behavior.hpp>
#include <irobot_create_toolbox/motion_control/reflex_behavior.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace irobot_create_toolbox
{
class MotionControlNode : public rclcpp::Node
{
public:
  /// \brief Constructor
  MotionControlNode();

private:
  /// \brief Function to centralize velocity command for system
  void control_robot();
  /// \brief Clear any cached teleop command
  void reset_last_teleop_cmd();

  /// \brief Helper function to declare ROS 2 safety parameters
  void declare_safety_parameters();

  /// \brief Helper function to validate changes to parameters
  rcl_interfaces::msg::SetParametersResult set_parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  enum class SafetyOverrideMode
  {
    NONE,
    BACKUP_ONLY,
    FULL
  };
  const std::map<std::string, SafetyOverrideMode> safety_to_str_ {
    {"none", SafetyOverrideMode::NONE},
    {"backup_only", SafetyOverrideMode::BACKUP_ONLY},
    {"full", SafetyOverrideMode::FULL}
  };
  /// \brief Name of parameter for setting system safety mode,
  //     which dictates cliff safety and max speed
  const std::string safety_override_param_name_{"safety_override"};
  /// \brief Name of parameter for reporting system's max speed,
  //     this will be changed by system based on safety_override setting
  const std::string max_speed_param_name_{"max_speed"};

  bool set_safety_mode(const std::string & safety_mode);
  /// \brief Storage for custom parameter validation callbacks
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_{
    nullptr};

  /// \brief Callback for new velocity commands
  void commanded_velocity_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg);

  /// \brief Callback for robot odometry
  void robot_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /// \brief Callback for kidnap status
  void kidnap_callback(irobot_create_msgs::msg::KidnapStatus::ConstSharedPtr msg);

  /// \brief Given command, bound by max speed, checking each wheel
  void bound_command_by_limits(geometry_msgs::msg::Twist & cmd);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_pose_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::KidnapStatus>::SharedPtr kidnap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr backup_buffer_low_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<BehaviorsScheduler> scheduler_;
  std::shared_ptr<DockingBehavior> docking_behavior_;
  std::shared_ptr<ReflexBehavior> reflex_behavior_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist last_teleop_cmd_;
  rclcpp::Time last_teleop_ts_;
  rclcpp::Duration wheels_stop_threshold_;
  std::atomic<bool> allow_speed_param_change_ {false};
  std::atomic<SafetyOverrideMode> safety_override_mode_ {SafetyOverrideMode::NONE};
  const double SAFETY_ON_MAX_SPEED {0.306};
  const double SAFETY_OFF_MAX_SPEED {0.46};
  double max_speed_ {SAFETY_ON_MAX_SPEED};
  const double wheel_base_ {0.233};
  double backup_buffer_ {0.0};
  std::mutex robot_pose_mutex_;
  tf2::Transform last_robot_pose_;
  tf2::Transform last_backup_pose_;
  std::atomic<bool> last_kidnap_ {false};
  rclcpp::Time backup_print_ts_;
  rclcpp::Time auto_override_print_ts_;
  const rclcpp::Duration repeat_print_;
};

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__MOTION_CONTROL_NODE_HPP_
