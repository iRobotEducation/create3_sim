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

#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <irobot_create_gazebo_plugins/gazebo_ros_helpers.hpp>
#include <irobot_create_msgs/msg/mouse.hpp>
#include <memory>

namespace irobot_create_gazebo_plugins
{
/*! \brief Plugin to control odometry based on the optical mouse sensor.
  *
  *  ROS Publishers
  *  - /mouse: Odometry data.
  *
  */
class GazeboRosOpticalMouse : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosOpticalMouse();

  /// Destructor
  virtual ~GazeboRosOpticalMouse();

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  /// Function is called when the world is reset
  void Reset() override;

protected:
  /// Optional callback to be called at every simulation iteration.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

private:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_{nullptr};

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Link pointer to sensor
  gazebo::physics::LinkPtr link_{nullptr};

  /// World pointer
  gazebo::physics::WorldPtr world_{nullptr};

  /// ROS publisher
  rclcpp::Publisher<irobot_create_msgs::msg::Mouse>::SharedPtr pub_{nullptr};

  /// Last time the sensor was updated
  gazebo::common::Time last_time_;

  /// \brief Pose of the sensor from previous iteration.
  ignition::math::Pose3d last_pose_;

  /// Integrated position of the sensor
  ignition::math::Vector2d integrated_position_;

  /// Helper class to enforce a specific update rate
  utils::UpdateRateEnforcer update_rate_enforcer_;
};
}  // namespace irobot_create_gazebo_plugins
