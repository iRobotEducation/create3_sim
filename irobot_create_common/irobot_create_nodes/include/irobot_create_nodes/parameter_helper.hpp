// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef IROBOT_CREATE_NODES__PARAMETER_HELPER_HPP_
#define IROBOT_CREATE_NODES__PARAMETER_HELPER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace irobot_create_nodes
{
template<typename T>
const T declare_and_get_parameter(std::string param_name, rclcpp::Node * node)
{
  return node->declare_parameter<T>(param_name);
}

}  // namespace irobot_create_nodes

#endif  // IROBOT_CREATE_NODES__PARAMETER_HELPER_HPP_
