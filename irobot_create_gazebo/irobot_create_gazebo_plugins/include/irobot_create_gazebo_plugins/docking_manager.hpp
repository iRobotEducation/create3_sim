// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef IROBOT_CREATE_GAZEBO_PLUGINS__DOCKING_MANAGER_HPP_
#define IROBOT_CREATE_GAZEBO_PLUGINS__DOCKING_MANAGER_HPP_

#include <cmath>
#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include "irobot_create_gazebo_plugins/gazebo_ros_helpers.hpp"
#include "irobot_create_toolbox/polar_coordinates.hpp"

namespace irobot_create_gazebo_plugins
{
class DockingManager
{
private:
  // Link pointer to robot receiver
  gazebo::physics::LinkPtr receiver_link_{nullptr};
  // Link pointer to dock emitter
  gazebo::physics::LinkPtr emitter_link_{nullptr};
  // World pointer
  gazebo::physics::WorldPtr world_{nullptr};
  // Model and their respective link names
  std::string robot_model_name_;
  std::string robot_receiver_link_name_;
  std::string dock_model_name_;
  std::string dock_emitter_link_name_;

  // Once models are ready, initialize their link pointers
  void initLinks(
    const gazebo::physics::ModelPtr & dock_model, const gazebo::physics::ModelPtr & robot_model);

public:
  /// Constructor
  DockingManager(
    const gazebo::physics::WorldPtr & world, const std::string & robot_name,
    const std::string & receiver_link_name,
    const std::string & dock_name, const std::string & emitter_link_name);

  /// Use this method to check that the models are ready in gazebo before dereferencing their
  /// pointers
  bool AreModelsReady();

  /// Change reference frame of a cartesian point WRT emitter and return in the new frame as
  /// polar point
  irobot_create_toolbox::PolarCoordinate EmitterCartesianPointToReceiverPolarPoint(
    const ignition::math::Vector2d & emitter_point);

  /// Change reference frame of a cartesian point WRT receiver and return in the new frame as
  /// polar point
  irobot_create_toolbox::PolarCoordinate ReceiverCartesianPointToEmitterPolarPoint(
    const ignition::math::Vector2d & receiver_point);
};
}  // namespace irobot_create_gazebo_plugins

#endif  // IROBOT_CREATE_GAZEBO_PLUGINS__DOCKING_MANAGER_HPP_
