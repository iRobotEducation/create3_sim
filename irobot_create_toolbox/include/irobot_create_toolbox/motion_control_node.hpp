// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Alberto Soragna (asoragna@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__MOTION_CONTROL_NODE_HPP_
#define IROBOT_CREATE_TOOLBOX__MOTION_CONTROL_NODE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <irobot_create_toolbox/parameter_helper.hpp>
#include <irobot_create_toolbox/motion_control/docking_behavior.hpp>
#include <rclcpp/rclcpp.hpp>
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

  /// \brief Helper function to declare ROS 2 reflex parameters
  void declare_reflex_parameters();

  /// \brief Helper function to declare ROS 2 safety parameters
  void declare_safety_parameters();

  /// \brief Helper function to validate changes to parameters
  rcl_interfaces::msg::SetParametersResult set_parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  /// \brief Name of parameter for enabling/disabling all reflexes
  const std::string reflex_enabled_param_name_{"reflexes_enabled"};
  /// \brief Vector of reflex names (these corresponds to exposed parameters)
  const std::vector<std::string> reflex_names_{
    "REFLEX_BUMP", "REFLEX_CLIFF", "REFLEX_DOCK_AVOID",
    "REFLEX_GYRO_CAL", "REFLEX_PANIC", "REFLEX_PROXIMITY_SLOWDOWN",
    "REFLEX_STUCK", "REFLEX_VIRTUAL_WALL", "REFLEX_WHEEL_DROP"};
  /// \brief Name of parameter for setting system safety mode,
  //     which dictates cliff safety and max speed
  const std::string safety_override_param_name_{"safety_override"};
  /// \brief Name of parameter for reporting system's max speed,
  //     this will be changed by system based on safety_override setting
  const std::string max_speed_param_name_{"max_speed"};
  /// \brief Storage for custom parameter validation callbacks
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_{
    nullptr};

  /// \brief Callback for new velocity commands
  void commanded_velocity_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<BehaviorsScheduler> scheduler_;
  std::shared_ptr<DockingBehavior> docking_behavior_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist last_teleop_cmd_;
  rclcpp::Time last_teleop_ts_;
  rclcpp::Duration wheels_stop_threshold_;
};

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__MOTION_CONTROL_NODE_HPP_
