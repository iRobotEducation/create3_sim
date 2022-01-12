// Copyright 2021 iRobot Corporation. All Rights Reserved.
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
