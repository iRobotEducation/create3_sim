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
// @author Emiliano Javier Borghi Orue (creativa_eborghi@irobot.com)

#ifndef IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_IR_INTENSITY_SENSOR_HPP_
#define IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_IR_INTENSITY_SENSOR_HPP_

#include <limits>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/MultiRayShape.hh"
#include "gazebo/plugins/RayPlugin.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/node.hpp"
#include "gazebo_ros/utils.hpp"
#include "irobot_create_gazebo_plugins/gazebo_ros_helpers.hpp"
#include "irobot_create_msgs/msg/ir_intensity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace irobot_create_gazebo_plugins
{
class GazeboRosIrIntensitySensor : public gazebo::SensorPlugin
{
public:
  /// \brief Constructor
  GazeboRosIrIntensitySensor();

  /// \brief Destructor
  ~GazeboRosIrIntensitySensor();

  /// \brief Load the plugin
  /// \param parent Pointer to the parent sensor to which the plugin is attached
  /// \param sdf Take in SDF root element
  void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf) override;

  /// \brief Update callback
  virtual void OnNewLaserScans();

private:
  // pointer to ros node
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  // The connection tied to RayPlugin::OnNewLaserScans()
  gazebo::event::ConnectionPtr new_laser_scans_connection_{nullptr};

  // The parent sensor
  gazebo::sensors::RaySensorPtr parent_sensor_{nullptr};

  // ROS receiver messages publisher
  rclcpp::Publisher<irobot_create_msgs::msg::IrIntensity>::SharedPtr pub_{nullptr};

  // Sensor Emitter description message
  irobot_create_msgs::msg::IrIntensity msg_;

  // Maximum range detected by the sensor
  double max_range_{std::numeric_limits<double>::max()};
};
}  // namespace irobot_create_gazebo_plugins

#endif  // IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_IR_INTENSITY_SENSOR_HPP_
