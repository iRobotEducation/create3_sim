// Copyright 2021 iRobot, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// @author Alberto Soragna (asoragna@irobot.com)

#include "irobot_create_toolbox/motion_control_node.hpp"
#include <irobot_create_toolbox/behaviors_scheduler.hpp>
#include <irobot_create_toolbox/docking_behavior.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace irobot_create_toolbox
{
using namespace std::placeholders;
using namespace std::chrono_literals;

MotionControlNode::MotionControlNode()
: rclcpp::Node("motion_control"),
  m_wheels_stop_threshold(1s)
{
  // Declare ROS 2 parameters for controlling robot reflexes.
  this->declare_reflex_parameters();
  // Declare ROS 2 parameters for robot safety.
  this->declare_safety_parameters();

  // Create behaviors scheduler
  m_scheduler = std::make_shared<BehaviorsScheduler>();
  // Create Docking Behavior manager
  m_docking_behavior = std::make_shared<DockingBehavior>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_topics_interface(),
    this->get_node_waitables_interface(),
    m_scheduler);
  // Create subscription to let other applications drive the robot
  m_teleop_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS(),
    std::bind(&MotionControlNode::commanded_velocity_callback, this, _1));

  m_cmd_vel_out_pub = this->create_publisher<geometry_msgs::msg::Twist>(
    "diffdrive_controller/cmd_vel_unstamped", rclcpp::SensorDataQoS());

  // Register a callback to handle parameter changes
  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MotionControlNode::set_parameters_callback, this, _1));

  m_last_teleop_ts = this->now();
  // Create timer to periodically execute behaviors and control the robot
  constexpr auto period = 25ms;
  m_timer = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration(period),
    std::bind(&MotionControlNode::control_robot, this));
}

void MotionControlNode::declare_reflex_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = false;
  rclcpp::ParameterValue ret;

  // Declare individual reflexes parameters.
  // Eventually reflexes will be enabled, but now this is just a stub implementation
  // so we set them to false and we enforce that users do not change them.
  for (const std::string & reflex_name : reflex_names_) {
    const std::string param_name = std::string("reflexes.") + reflex_name;
    ret = this->declare_parameter(param_name, rclcpp::ParameterValue(false), descriptor);

    // Make sure user is not trying to enable reflexes at startup: this is not supported.
    if (ret.get<bool>()) {
      RCLCPP_ERROR(
        this->get_logger(), "Trying to enable reflex: '%s'. This is not supported yet.",
        param_name.c_str());
      throw std::runtime_error("User tried to enable reflexes. This are not supported yet.");
    }
  }

  // Declare parameter to control all reflexes.
  ret =
    this->declare_parameter(reflex_enabled_param_name_, rclcpp::ParameterValue(false), descriptor);
  // Make sure user is not trying to enable reflexes at startup: this is not supported.
  // See https://github.com/iRobotEducation/create3_sim/issues/65
  if (ret.get<bool>()) {
    RCLCPP_ERROR(
      this->get_logger(), "Trying to enable: '%s'. This is not supported yet.",
      reflex_enabled_param_name_.c_str());
    throw std::runtime_error("User tried to enable reflexes. This are not supported yet.");
  }
}

void MotionControlNode::declare_safety_parameters()
{
  std::stringstream long_description_string;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = false;
  long_description_string <<
    "Mode to override safety options {\"none\"(default), " <<
    "\"backup_only\"(disable backup limits, no cliff safety driving backwards), " <<
    "\"full\"(disables cliffs completely and allows for higher max drive speed " <<
    "(0.46m/s vs 0.306m/s in other modes))}";
  descriptor.description = long_description_string.str();
  auto val = this->declare_parameter<std::string>(
    safety_override_param_name_, "backup_only",
    descriptor);
  if (val != "backup_only") {
    RCLCPP_ERROR(
      this->get_logger(), "Trying to set %s. This is not supported yet on sim.",
      safety_override_param_name_);
    throw std::runtime_error(
            "User tried to set " + safety_override_param_name_ + ". This are not supported yet.");
  }

  descriptor.read_only = false;
  long_description_string.str("");
  long_description_string.clear();
  long_description_string <<
    "Maximum speed of the system in m/s, updated by robot based on safety_override mode. " <<
    "Cannot be updated externally.";
  descriptor.description = long_description_string.str();
  double default_speed = 0.306;
  auto speed = this->declare_parameter<double>(max_speed_param_name_, default_speed, descriptor);
  if (speed != default_speed) {
    RCLCPP_WARN(
      this->get_logger(), "%s %s \'%s\' parameter",
      "Ignoring user set max speed as parameter is for reporting purposes only.",
      "Max speed is only changed by updating the",
      safety_override_param_name_);
  }
}

rcl_interfaces::msg::SetParametersResult MotionControlNode::set_parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // This is just a stub implementation. Reflexes are permanently disabled
  // and user can't enable them. We reject all changes.
  // See https://github.com/iRobotEducation/create3_sim/issues/65
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = false;

  for (const rclcpp::Parameter & parameter : parameters) {
    if (parameter.get_name() == safety_override_param_name_) {
      RCLCPP_WARN(
        this->get_logger(), "Can't modify %s.  Not implemented in sim yet",
        parameter.get_name().c_str());
      result.reason = "Can't modify parameter \'" + parameter.get_name() +
        "\' not implemented in sim yet.";
    } else if (parameter.get_name() == max_speed_param_name_) {
      result.reason = "parameter \'" + parameter.get_name() +
        "\' cannot be set externally. Only updated from change in \'" +
        safety_override_param_name_ + "\' parameter";
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Can't modify reflex parameter %s.",
        parameter.get_name().c_str());
      result.reason = "reflexes can't be enabled yet.";
    }
  }

  return result;
}

void MotionControlNode::control_robot()
{
  // Handle behaviors
  BehaviorsScheduler::optional_output_t command;
  if (m_scheduler->has_behavior()) {
    command = m_scheduler->run_behavior();
    // Reset last teleoperation command if we are executing a behavior
    this->reset_last_teleop_cmd();
  } else {
    // If we don't have any behavior, let's drive the robot using teleoperation command
    // After m_wheels_stop_threshold has passed without receiving new velocity commands
    // we stop the wheels.
    rclcpp::Duration time_diff(rclcpp::Duration::max());
    {
      const std::lock_guard<std::mutex> lock(m_mutex);
      time_diff = (this->now() - m_last_teleop_ts);
    }
    if (time_diff > m_wheels_stop_threshold) {
      this->reset_last_teleop_cmd();
    } else {
      const std::lock_guard<std::mutex> lock(m_mutex);
      command = m_last_teleop_cmd;
    }
  }

  // Add reflex manager here

  // Create a null command if we don't have anything.
  // We also disable reflexes because the robot is in an idle state.
  if (!command) {
    command = geometry_msgs::msg::Twist();
  }
  auto cmd_out_msg = std::make_unique<geometry_msgs::msg::Twist>();
  *cmd_out_msg = *command;
  m_cmd_vel_out_pub->publish(std::move(cmd_out_msg));
}

void MotionControlNode::commanded_velocity_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  if (m_scheduler->has_behavior()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Ignoring velocities commanded while an autonomous behavior is running!");
    return;
  }

  const std::lock_guard<std::mutex> lock(m_mutex);

  m_last_teleop_cmd = *msg;
}

void MotionControlNode::reset_last_teleop_cmd()
{
  const std::lock_guard<std::mutex> lock(m_mutex);

  m_last_teleop_cmd.linear.x = 0;
  m_last_teleop_cmd.linear.y = 0;
  m_last_teleop_cmd.linear.z = 0;
  m_last_teleop_cmd.angular.x = 0;
  m_last_teleop_cmd.angular.y = 0;
  m_last_teleop_cmd.angular.z = 0;
  m_last_teleop_ts = this->now();
}

}  // namespace irobot_create_toolbox
