// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include <irobot_create_gazebo_plugins/docking_manager.hpp>
#include <string>

namespace irobot_create_gazebo_plugins
{
DockingManager::DockingManager(
  const gazebo::physics::WorldPtr & world, const std::string & robot_name,
  const std::string & receiver_link_name,
  const std::string & dock_name, const std::string & emitter_link_name)
: world_{world}, robot_model_name_{robot_name}, robot_receiver_link_name_{receiver_link_name},
  dock_model_name_{dock_name}, dock_emitter_link_name_{emitter_link_name}
{
}

void DockingManager::initLinks(
  const gazebo::physics::ModelPtr & dock_model, const gazebo::physics::ModelPtr & robot_model)
{
  // Retrieve receiver link and assert it.
  receiver_link_ = robot_model->GetLink(robot_receiver_link_name_);
  GZ_ASSERT(receiver_link_, "Receiver link pointer is invalid!");

  // Retrieve emitter link and assert it.
  emitter_link_ = dock_model->GetLink(dock_emitter_link_name_);
  GZ_ASSERT(emitter_link_, "Emitter link pointer is invalid!");
}

bool DockingManager::AreModelsReady()
{
  const gazebo::physics::ModelPtr dock_model = world_->ModelByName(dock_model_name_);
  const gazebo::physics::ModelPtr robot_model = world_->ModelByName(robot_model_name_);
  const bool models_ready = dock_model != nullptr && robot_model != nullptr;
  if (models_ready && (receiver_link_ == nullptr || emitter_link_ == nullptr)) {
    initLinks(dock_model, robot_model);
  }
  return models_ready;
}

irobot_create_toolbox::PolarCoordinate
DockingManager::EmitterCartesianPointToReceiverPolarPoint(
  const ignition::math::Vector2d & emitter_point)
{
  const ignition::math::Vector3d emitter_point_3d =
    ignition::math::Vector3d{emitter_point.X(), emitter_point.Y(), 0.0};
  const ignition::math::Pose3d emitter_pose = emitter_link_->WorldPose();
  const ignition::math::Pose3d receiver_pose = receiver_link_->WorldPose();
  // Pose of emitter relative to the receiver
  const ignition::math::Pose3d emitter_wrt_receiver_pose = emitter_pose - receiver_pose;
  // Convert emitter point to a receiver point
  const ignition::math::Vector3d emitter_wrt_receiver_point =
    emitter_wrt_receiver_pose.CoordPositionAdd(emitter_point_3d);
  return irobot_create_toolbox::toPolar(
    ignition::math::Vector2d{emitter_wrt_receiver_point.X(), emitter_wrt_receiver_point.Y()});
}

irobot_create_toolbox::PolarCoordinate
DockingManager::ReceiverCartesianPointToEmitterPolarPoint(
  const ignition::math::Vector2d & receiver_point)
{
  const ignition::math::Vector3d receiver_point_3d =
    ignition::math::Vector3d{receiver_point.X(), receiver_point.Y(), 0.0};
  const ignition::math::Pose3d emitter_pose = emitter_link_->WorldPose();
  const ignition::math::Pose3d receiver_pose = receiver_link_->WorldPose();
  // Pose of receiver relative to the emitter
  const ignition::math::Pose3d receiver_wrt_emitter_pose = receiver_pose - emitter_pose;
  // Convert receiver point to an emitter point
  const ignition::math::Vector3d receiver_wrt_emitter_point =
    receiver_wrt_emitter_pose.CoordPositionAdd(receiver_point_3d);
  return irobot_create_toolbox::toPolar(
    ignition::math::Vector2d{receiver_wrt_emitter_point.X(), receiver_wrt_emitter_point.Y()});
}
}  // namespace irobot_create_gazebo_plugins
