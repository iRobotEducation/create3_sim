// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Emiliano Javier Borghi Orue (creativa_eborghi@irobot.com)

#ifndef IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_HELPERS_HPP_
#define IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_HELPERS_HPP_

#include <algorithm>
#include <limits>
#include <vector>

#include "gazebo_ros/node.hpp"
#include "sdf/sdf.hh"

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

template<typename T, typename V>
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

/// \brief Convert radians to degrees
inline int Rad2Deg(double radians) {return radians / M_PI * 180;}

/// \brief Wrap angle between (-pi, pi]
inline double WrapAngle(double angle) {return atan2(sin(angle), cos(angle));}

/// \brief Calculate if a provided angle in radians is within the arc formed by two other angles.
/// \param target angle to test
/// \param angle1 first leg of the arc
/// \param target second leg of the arc
bool IsAngleBetween(double target, double angle1, double angle2)
{
  double t = WrapAngle(target);
  double a1 = WrapAngle(angle1);
  double a2 = WrapAngle(angle2);
  // check if it passes through zero
  if (a1 <= a2) {
    return (t >= a1) && (t <= a2);
  } else {
    return (t >= a1) || (t <= a2);
  }
  return false;
}

// Find the minimum range given a vector of 'ranges'.
// If none was found, returns a maximum double.
double FindMinimumRange(std::vector<double> & ranges)
{
  std::vector<double>::iterator detection_ptr =
    std::min_element(std::begin(ranges), std::end(ranges));
  // If a minimum range was found, return it
  if (detection_ptr != std::end(ranges)) {
    return *detection_ptr;
  }
  // Otherwise, return a maximum range
  return std::numeric_limits<double>::max();
}

// Very useful Polar Coordinates tools when working with Fields of views and distances.
struct PolarCoordinate
{
  double radius;
  double azimuth;
};

PolarCoordinate toPolar(const ignition::math::Vector2d & cartesian)
{
  return PolarCoordinate{cartesian.Length(), atan2(cartesian.Y(), cartesian.X())};
}

ignition::math::Vector2d fromPolar(const PolarCoordinate & polar)
{
  ignition::math::Vector2d cartesian{
    polar.radius * cos(polar.azimuth), polar.radius * sin(polar.azimuth)};
  return cartesian;
}

}  // namespace utils
}  // namespace irobot_create_gazebo_plugins

#endif  // IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_HELPERS_HPP_
