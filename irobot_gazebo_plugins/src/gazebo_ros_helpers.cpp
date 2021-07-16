#include <irobot_gazebo_plugins/gazebo_ros_helpers.h>

UpdateRateEnforcer::UpdateRateEnforcer() {
}

void UpdateRateEnforcer::load(double update_freq) {
  ideal_update_period = 1.0 / update_freq;
  next_update_period = ideal_update_period;
}

bool UpdateRateEnforcer::shouldUpdate(double time_elapsed) {
  if(time_elapsed >= next_update_period - UPDATE_RATE_EPSILON) {
    if(time_elapsed <= next_update_period + UPDATE_RATE_EPSILON) {
      // correct accumulating small errors
      next_update_period = ideal_update_period;
    }
    else {
      next_update_period = ideal_update_period - (time_elapsed - next_update_period);
    }
    return true;
  }
  else {
    return false;
  }
}