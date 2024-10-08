// Copyright 2022 iRobot Corporation. All Rights Reserved.
// @author Alberto Soragna (asoragna@irobot.com)

#include <cmath>

#include "gz/math/Vector2.hh"
#include "irobot_create_toolbox/polar_coordinates.hpp"

namespace irobot_create_toolbox
{

PolarCoordinate toPolar(const gz::math::Vector2d & cartesian)
{
  return PolarCoordinate{cartesian.Length(), atan2(cartesian.Y(), cartesian.X())};
}

gz::math::Vector2d fromPolar(const PolarCoordinate & polar)
{
  gz::math::Vector2d cartesian{
    polar.radius * cos(polar.azimuth), polar.radius * sin(polar.azimuth)};
  return cartesian;
}

}  // namespace irobot_create_toolbox
