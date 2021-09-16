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
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace irobot_create_toolbox
{
template <typename T>
const T declare_and_get_parameter(std::string param_name, rclcpp::Node * node)
{
  return node->declare_parameter<T>(param_name);
}

}  // namespace irobot_create_toolbox
