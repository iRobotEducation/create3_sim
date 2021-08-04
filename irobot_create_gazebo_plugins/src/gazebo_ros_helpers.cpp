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

#include <irobot_create_gazebo_plugins/gazebo_ros_helpers.hpp>

namespace irobot_create_gazebo_plugins
{
namespace utils
{
void UpdateRateEnforcer::load(const double & update_freq)
{
  ideal_update_period_ = 1.0 / update_freq;
  next_update_period_ = ideal_update_period_;
}

bool UpdateRateEnforcer::shouldUpdate(const double & time_elapsed)
{
  if (time_elapsed >= next_update_period_ - UPDATE_RATE_EPSILON) {
    if (time_elapsed <= next_update_period_ + UPDATE_RATE_EPSILON) {
      // correct accumulating small errors
      next_update_period_ = ideal_update_period_;
    } else {
      next_update_period_ = ideal_update_period_ - (time_elapsed - next_update_period_);
    }
    return true;
  } else {
    return false;
  }
}
}  // namespace utils
}  // namespace irobot_create_gazebo_plugins
