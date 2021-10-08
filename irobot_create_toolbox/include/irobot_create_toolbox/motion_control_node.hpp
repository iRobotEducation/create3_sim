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

#ifndef IROBOT_CREATE_TOOLBOX__MOTION_CONTROL_NODE_HPP_

#include <irobot_create_toolbox/parameter_helper.hpp>
#include <rclcpp/rclcpp.hpp>
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
  /// \brief Helper function to declare ROS 2 reflex parameters
  void declare_reflex_parameters();

  /// \brief Helper function to validate changes to parameters
  rcl_interfaces::msg::SetParametersResult set_parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  /// \brief Name of parameter for enabling/disabling all reflexes
  std::string m_reflex_enabled_param_name;
  /// \brief Vector of reflex names (these corresponds to exposed parameters)
  std::vector<std::string> m_reflex_names;
  /// \brief Storage for custom parameter validation callbacks
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_params_callback_handle;
};

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__MOTION_CONTROL_NODE_HPP_
