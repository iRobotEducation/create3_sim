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

#include <gazebo/plugins/ImuSensorPlugin.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/utils.hpp>
#include <irobot_create_gazebo_plugins/gazebo_ros_helpers.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace irobot_create_gazebo_plugins
{
/// Plugin to attach to a gazebo IMU sensor and publish ROS message of output
class GazeboRosImu : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  GazeboRosImu();
  /// Destructor.
  virtual ~GazeboRosImu();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publish for imu message
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  /// IMU message modified each update
  sensor_msgs::msg::Imu::SharedPtr msg_;
  /// IMU sensor this plugin is attached to
  gazebo::sensors::ImuSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;

  /// Publish latest imu data to ROS
  void OnUpdate();
};

}  // namespace irobot_create_gazebo_plugins
