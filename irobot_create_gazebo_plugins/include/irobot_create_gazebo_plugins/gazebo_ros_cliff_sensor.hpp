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
// @author Luis Enrique Chico Capistrano (lchico@irobot.com)

#pragma once

#include <gazebo/common/Assert.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <irobot_create_gazebo_plugins/gazebo_ros_helpers.hpp>
#include <irobot_create_msgs/msg/hazard_detection.hpp>
#include <limits>
#include <rclcpp/node.hpp>

namespace irobot_create_gazebo_plugins
{
class GazeboRosCliffSensor : public gazebo::SensorPlugin
{
public:
  /// \brief Constructor
  GazeboRosCliffSensor() = default;
  /// \brief Destructor
  ~GazeboRosCliffSensor() = default;
  /// \brief Called when plugin is loaded
  /// \param parent Pointer to the parent sensor to which the plugin is attached
  /// \param sdf Take in SDF root element
  void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf);
  /// \brief Reset variables on Reset event
  void Reset() override;

protected:
  /// \brief Update the controller
  virtual void OnNewLaserScans();

private:
  // Pointer to left cliff sensor
  gazebo::sensors::RaySensorPtr cliff_sensor_{nullptr};
  // Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};
  // Measured distance in meter for detecting a cliff
  double detection_threshold_;
  // Cliff message modified each update
  irobot_create_msgs::msg::HazardDetection msg_;
  // Set bound for cliff  detection
  double max_range_{std::numeric_limits<double>::max()};
  // World pointer
  gazebo::physics::WorldPtr world_{nullptr};
  // Publish for cliff message
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr pub_{nullptr};
  // The connection tied to update the laser scans OnNewLaserScans()
  gazebo::event::ConnectionPtr new_laser_scans_connection_{nullptr};
};

}  // namespace irobot_create_gazebo_plugins
