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
// @author Alexis Pojomovsky (apojomovsky@irobot.com)

#ifndef IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_BUMPER_HPP_
#define IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_BUMPER_HPP_

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/node.hpp"
#include "irobot_create_gazebo_plugins/gazebo_ros_helpers.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"

namespace irobot_create_gazebo_plugins
{
/// \brief Bumper controller
class GazeboRosBumper : public gazebo::SensorPlugin
{
public:
  /// \brief Constructor
  GazeboRosBumper() = default;

  /// \brief Destructor
  ~GazeboRosBumper() = default;

  /// \brief Load the plugin
  /// \param sensor pointer to the sensor to which the plugin is attached to
  /// \param sdf pointer to the sdf tree
  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf);

protected:
  /// \brief Update the controller
  void OnUpdate();

private:
  // Bumper zones
  enum class ZoneType { RIGHT, CENTER_RIGHT, CENTER, CENTER_LEFT, LEFT };

  // Auxiliary data structure to hold bumper zone details
  struct Zone
  {
    double left_limit;
    double right_limit;
    std::string name;
  };

  // Data structure to hold the definitions related to bumper zones
  const std::map<ZoneType, Zone> angles_map_ = {
    {ZoneType::RIGHT, {-M_PI / 2, -3 * M_PI / 10, "right"}},
    {ZoneType::CENTER_RIGHT, {-3 * M_PI / 10, -M_PI / 10, "center_right"}},
    {ZoneType::CENTER, {-M_PI / 10, M_PI / 10, "center"}},
    {ZoneType::CENTER_LEFT, {M_PI / 10., 3 * M_PI / 10, "center_left"}},
    {ZoneType::LEFT, {3 * M_PI / 10, M_PI / 2, "left"}}};

  // Pointer to ros node
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  // ROS publisher for the bumper output
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr bumper_pub_{nullptr};

  // Transformation between robot pose and world
  ignition::math::Matrix4d r_tf_w_;

  // Pointer to the Contact Sensor model
  gazebo::sensors::ContactSensorPtr bumper_{nullptr};

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_{nullptr};

  // HazardDetection message instance
  irobot_create_msgs::msg::HazardDetection msg_;

  // Gazebo transport node
  gazebo::transport::NodePtr gz_node_{nullptr};

  // Gazebo subscriber to pose topic
  gazebo::transport::SubscriberPtr gz_sub_{nullptr};

  // Gazebo transport callback to retrieve the robot's pose
  void GzPoseCallback(ConstPosesStampedPtr & msg);
};

}  // namespace irobot_create_gazebo_plugins

#endif  // IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_BUMPER_HPP_
