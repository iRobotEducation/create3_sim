// Copyright 2022 iRobot Corporation. All Rights Reserved.
// @author Alberto Soragna (asoragna@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__POLAR_COORDINATES_HPP_
#define IROBOT_CREATE_TOOLBOX__POLAR_COORDINATES_HPP_

#include "ignition/math/Vector2.hh"

namespace irobot_create_toolbox
{

// Very useful Polar Coordinates tools when working with Fields of views and distances.
struct PolarCoordinate
{
  double radius;
  double azimuth;
};

PolarCoordinate toPolar(const ignition::math::Vector2d & cartesian);

ignition::math::Vector2d fromPolar(const PolarCoordinate & polar);

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__POLAR_COORDINATES_HPP_
