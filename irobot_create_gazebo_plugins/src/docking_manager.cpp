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

#include <irobot_create_gazebo_plugins/docking_manager.hpp>
#include <string>

namespace irobot_create_gazebo_plugins
{
DockingManager::DockingManager(
  const gazebo::physics::WorldPtr & world, const std::string & robot_name,
  const std::string & dock_name)
: world_{world}, robot_model_name_{robot_name}, dock_model_name_{dock_name}
{
}

void DockingManager::initLinks(
  const gazebo::physics::ModelPtr & dock_model, const gazebo::physics::ModelPtr & robot_model)
{
  // Retrieve receiver link and assert it.
  receiver_link_ = robot_model->GetLink("ir_omni");
  GZ_ASSERT(receiver_link_, "Receiver link pointer is invalid!");

  // Retrieve emitter link and assert it.
  emitter_link_ = dock_model->GetLink("halo_link");
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

utils::PolarCoordinate DockingManager::emitterWRTReceiverPolarPoint(
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
  return utils::toPolar(
    ignition::math::Vector2d{emitter_wrt_receiver_point.X(), emitter_wrt_receiver_point.Y()});
}

utils::PolarCoordinate DockingManager::receiverWRTEmitterPolarPoint(
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
  return utils::toPolar(
    ignition::math::Vector2d{receiver_wrt_emitter_point.X(), receiver_wrt_emitter_point.Y()});
}
}  // namespace irobot_create_gazebo_plugins
