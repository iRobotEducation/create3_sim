// Copyright 2022 iRobot Corporation. All Rights Reserved.
// @author Alberto Soragna (asoragna@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__MATH_HPP_
#define IROBOT_CREATE_TOOLBOX__MATH_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace irobot_create_toolbox
{

/// \brief Convert radians to degrees
template<typename T>
inline int Rad2Deg(T radians) {return radians / M_PI * 180;}

/// \brief Wrap angle between (-pi, pi]
template<typename T>
inline T WrapAngle(T angle) {return atan2(sin(angle), cos(angle));}

/// \brief Calculate if a provided angle in radians is within the arc formed by two other angles.
/// \param target angle to test
/// \param angle1 first leg of the arc
/// \param target second leg of the arc
template<typename T>
bool IsAngleBetween(T target, T angle1, T angle2)
{
  T t = WrapAngle(target);
  T a1 = WrapAngle(angle1);
  T a2 = WrapAngle(angle2);
  // check if it passes through zero
  if (a1 <= a2) {
    return (t >= a1) && (t <= a2);
  } else {
    return (t >= a1) || (t <= a2);
  }
  return false;
}

// Find the minimum range given a vector of 'ranges'.
// If none was found, returns a maximum T.
template<typename T>
T FindMinimumRange(const std::vector<T> & ranges)
{
  auto detection_ptr =
    std::min_element(std::begin(ranges), std::end(ranges));
  // If a minimum range was found, return it
  if (detection_ptr != std::end(ranges)) {
    return *detection_ptr;
  }
  // Otherwise, return a maximum range
  return std::numeric_limits<T>::max();
}

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__MATH_HPP_
