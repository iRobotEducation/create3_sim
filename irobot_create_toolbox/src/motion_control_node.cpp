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

#include <memory>
#include <string>
#include <vector>

#include "irobot_create_toolbox/motion_control_node.hpp"

namespace irobot_create_toolbox
{
using namespace std::placeholders;

MotionControlNode::MotionControlNode()
: rclcpp::Node("motion_control")
{
  // Declare ROS 2 parameters for controlling robot reflexes.
  this->declare_reflex_parameters();

  // Register a callback to handle parameter changes
  m_params_callback_handle = this->add_on_set_parameters_callback(
    std::bind(&MotionControlNode::set_parameters_callback, this, _1));
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

rcl_interfaces::msg::SetParametersResult MotionControlNode::set_parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // This is just a stub implementation. Reflexes are permanently disabled
  // and user can't enable them. We reject all changes.
  // See https://github.com/iRobotEducation/create3_sim/issues/65
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = false;
  result.reason = "reflexes can't be enabled yet.";

  for (const rclcpp::Parameter & parameter : parameters) {
    RCLCPP_WARN(
      this->get_logger(), "Can't modify reflex parameter %s.", parameter.get_name().c_str());
  }

  return result;
}

}  // namespace irobot_create_toolbox
