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
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef IROBOT_CREATE_GAZEBO_PLUGINS__DOCKING_MANAGER_HPP_
#define IROBOT_CREATE_GAZEBO_PLUGINS__DOCKING_MANAGER_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <cmath>
#include <memory>
#include <string>

namespace irobot_create_gazebo_plugins
{
struct PolarCoordinate
{
  double radius;
  double azimuth;
};

class DockingManager
{
private:
  // Link pointer to robot receiver
  gazebo::physics::LinkPtr receiver_link_{nullptr};
  // Link pointer to dock emitter
  gazebo::physics::LinkPtr emitter_link_{nullptr};
  // World pointer
  gazebo::physics::WorldPtr world_{nullptr};
  // Model names
  std::string robot_model_name_;
  std::string dock_model_name_;

  // Convert from one system to the other
  PolarCoordinate toPolar(const ignition::math::Vector2d & cartesian);
  ignition::math::Vector2d fromPolar(const PolarCoordinate & polar);

  // Once models are ready, initialize their link pointers
  void initLinks(
    const gazebo::physics::ModelPtr & dock_model, const gazebo::physics::ModelPtr & robot_model);

public:
  /// Constructor
  DockingManager(
    const gazebo::physics::WorldPtr & world, const std::string & robot_name,
    const std::string & dock_name);

  /// Use this method to check that the models are ready in gazebo before dereferencing their
  /// pointers
  bool checkIfModelsReady();

  /// Change reference frame of a cartesian point WRT emitter and return in the new frame as
  /// polar point
  PolarCoordinate emitterWRTReceiverPolarPoint(const ignition::math::Vector2d & emitter_point);

  /// Change reference frame of a cartesian point WRT receiver and return in the new frame as
  /// polar point
  PolarCoordinate receiverWRTEmitterPolarPoint(const ignition::math::Vector2d & receiver_point);
};
}  // namespace irobot_create_gazebo_plugins

#endif  // IROBOT_CREATE_GAZEBO_PLUGINS__DOCKING_MANAGER_HPP_
