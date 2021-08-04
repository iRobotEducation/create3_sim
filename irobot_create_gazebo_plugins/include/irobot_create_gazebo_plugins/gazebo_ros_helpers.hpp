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
// @author Emiliano Javier Borghi Orue (creativa_eborghi@irobot.com)

#pragma once

#include <gazebo_ros/node.hpp>
#include <sdf/sdf.hh>

namespace irobot_create_gazebo_plugins
{
namespace utils
{
// fine tuned for our specific scenario, where we check if
// t(n) - t(n-1) >= 1/(update_rate)
// with 5ms step size (assumption), t(a) - t(b) will always be a
// factor of 0.005, so we only need enough accuracy at 10^3 level on the LHS
// For the RHS, we need enough accuracy such that 1/n - 1/(n+1) > epsilon,
// because otherwise the epsilon will effectively affect the update rate
// unfortunately, 1/n - 1/(n+1) = 1/(n^2 + n) will approach 0, but we can
// decide on a cut off line for n, which is 1/step_size (maximum
// possible update rate), or 200, in which case we get
// 1/40200 ~= 0.00002488 > epsilon
// so let epsilon be 0.00001 (10^-5)
constexpr double UPDATE_RATE_EPSILON = 0.00001;

class UpdateRateEnforcer
{
public:
  UpdateRateEnforcer() {}
  void load(const double & update_freq);
  bool shouldUpdate(const double & time_elapsed);

private:
  double ideal_update_period_;
  double next_update_period_;
};

template <typename T, typename V>
inline bool initialize(T & var, sdf::ElementPtr sdf, const char * str, V default_value)
{
  if (sdf->HasElement(str)) {
    var = sdf->Get<T>(str);
    return true;
  } else {
    var = static_cast<T>(default_value);
    const auto ros_node = gazebo_ros::Node::Get(sdf);
    RCLCPP_INFO_STREAM(
      ros_node->get_logger(), "Loaded default values for " << str << ": " << default_value);
    return false;
  }
}
}  // namespace utils
}  // namespace irobot_create_gazebo_plugins
