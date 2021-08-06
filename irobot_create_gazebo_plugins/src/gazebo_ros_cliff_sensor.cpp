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

#include <irobot_create_gazebo_plugins/gazebo_ros_cliff_sensor.hpp>

namespace irobot_create_gazebo_plugins
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosCliffSensor)

// Load the controller
void GazeboRosCliffSensor::Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
  // Get the world name.
  const std::string world_name_ = parent->WorldName();
  world_ = gazebo::physics::get_world(world_name_);

  cliff_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(parent);
  GZ_ASSERT(
    cliff_sensor_, "[CLIFF PLUGIN] GazeboRosRange controller requires a Ray Sensor as its parent");
  cliff_sensor_->SetActive(true);

  utils::initialize(cliff_detection_threshold_, sdf, "detection_threshold", 0.01);

  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping can be handled.
  ros_node_ = gazebo_ros::Node::Get(sdf);
  // Get QoS profiles
  const gazebo_ros::QoS & qos = ros_node_->get_qos();

  msg_.header.frame_id = gazebo_ros::SensorFrameID(*parent, *sdf);
  max_range_ = cliff_sensor_->RangeMax();
  // Initialize ROS publisher
  pub_ = ros_node_->create_publisher<irobot_create_msgs::msg::HazardDetection>("~/out", rclcpp::SensorDataQoS());

  new_laser_scans_connection_ = cliff_sensor_->LaserShape()->ConnectNewLaserScans(
    std::bind(&GazeboRosCliffSensor::OnNewLaserScans, this));

  RCLCPP_INFO(ros_node_->get_logger(), "Started plugin");
}

// Function is called when the world is resetted
void GazeboRosCliffSensor::Reset() { new_laser_scans_connection_.reset(); }

// Update the plugin
void GazeboRosCliffSensor::OnNewLaserScans()
{
  // Configure an empty message with the timestamp and frame id
  msg_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(world_->SimTime());

  // Find the minimum detected distance
  std::vector<double> ranges;
  cliff_sensor_->Ranges(ranges);
  const double displacement = std::min(utils::FindMinimumRange(ranges), max_range_);
  RCLCPP_DEBUG_STREAM(ros_node_->get_logger(), "Cliff reporting " << displacement << " m");

  if (displacement >= cliff_detection_threshold_) {
    msg_.type = msg_.CLIFF;
    // Publish message
    pub_->publish(msg_);
    RCLCPP_INFO_EXPRESSION(
      ros_node_->get_logger(), true, "Cliff %s ON: %.3f", msg_.header.frame_id.c_str(),
      displacement);
  }
}
}  // namespace irobot_create_gazebo_plugins
