#pragma once

#include <math.h>
#include <stdint.h>
#include <string>
#include <sys/stat.h>

#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
// #include <gazebo_plugins/gazebo_ros_utils.h>
#include <geometry_msgs/msg/pose.hpp>

#include <ignition/math/Vector2.hh>

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

class UpdateRateEnforcer {
public:
  UpdateRateEnforcer();
  void load(double update_freq);
  bool shouldUpdate(double time_elapsed);

private:
  double ideal_update_period;
  double next_update_period;
};

namespace irobot_gazebo_plugins {
  namespace utils {

    template <typename T, typename V>
    inline int initialize(T &var, sdf::ElementPtr _sdf, const char *str, V default_value) {
      if(_sdf->HasElement(str)) {
        var = _sdf->Get<T>(str);
        return 1;
      }
      else {
        var = static_cast<T>(default_value);
        auto rosNode = gazebo_ros::Node::Get(_sdf);
        RCLCPP_INFO_STREAM(rosNode->get_logger(), "Loaded default values for " << str << ": " << default_value);
        return 0;
      }
    }

  } // namespace utils
} // namespace irobot_gazebo_plugins
