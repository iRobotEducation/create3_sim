// Copyright 2022 iRobot Corporation. All Rights Reserved.
// @author Alberto Soragna (asoragna@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__SENSORS__BUMPERS_HPP_
#define IROBOT_CREATE_TOOLBOX__SENSORS__BUMPERS_HPP_

#include <cmath>
#include <map>
#include <string>

namespace irobot_create_toolbox
{
namespace sensors
{

// Bumper zones types
enum class BumperZoneType
{
  RIGHT,
  CENTER_RIGHT,
  CENTER,
  CENTER_LEFT,
  LEFT
};

// Auxiliary data structure to hold bumper zone details
struct BumperZone
{
  double left_limit;
  double right_limit;
  std::string name;
};

// Data structure to hold the definitions related to bumper zones
const std::map<BumperZoneType, BumperZone> BUMPER_ZONES_MAP = {
  {BumperZoneType::RIGHT, {-M_PI / 2, -3 * M_PI / 10, "bump_right"}},
  {BumperZoneType::CENTER_RIGHT, {-3 * M_PI / 10, -M_PI / 10, "bump_front_right"}},
  {BumperZoneType::CENTER, {-M_PI / 10, M_PI / 10, "bump_front_center"}},
  {BumperZoneType::CENTER_LEFT, {M_PI / 10., 3 * M_PI / 10, "bump_front_left"}},
  {BumperZoneType::LEFT, {3 * M_PI / 10, M_PI / 2, "bump_left"}}};

}  // namespace sensors
}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__SENSORS__BUMPERS_HPP_
