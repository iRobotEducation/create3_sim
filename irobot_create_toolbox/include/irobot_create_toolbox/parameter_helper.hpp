// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__PARAMETER_HELPER_HPP_
#define IROBOT_CREATE_TOOLBOX__PARAMETER_HELPER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace irobot_create_toolbox
{
template<typename T>
const T declare_and_get_parameter(std::string param_name, rclcpp::Node * node)
{
  return node->declare_parameter(param_name).get<T>();
}

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__PARAMETER_HELPER_HPP_
