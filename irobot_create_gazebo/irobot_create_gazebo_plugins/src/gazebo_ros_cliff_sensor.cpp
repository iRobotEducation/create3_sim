// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Luis Enrique Chico Capistrano (lchico@irobot.com)

#include "irobot_create_gazebo_plugins/gazebo_ros_cliff_sensor.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "irobot_create_toolbox/math.hpp"

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
  GZ_ASSERT(cliff_sensor_, "GazeboRosCliffSensor controller requires a Ray Sensor as its parent");

  utils::initialize(detection_threshold_, sdf, "detection_threshold", 0.01);
  // Configure our static header message
  utils::initialize(msg_.header.frame_id, sdf, "frame_id", "");

  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping can be handled.
  ros_node_ = gazebo_ros::Node::Get(sdf);

  // Initialize ROS publisher
  pub_ = ros_node_->create_publisher<irobot_create_msgs::msg::HazardDetection>(
    "~/out", rclcpp::SensorDataQoS().reliable());

  new_laser_scans_connection_ = cliff_sensor_->LaserShape()->ConnectNewLaserScans(
    std::bind(&GazeboRosCliffSensor::OnNewLaserScans, this));

  // Configure our static message charasteristics
  max_range_ = cliff_sensor_->RangeMax();

  RCLCPP_INFO(ros_node_->get_logger(), "Started plugin");
}

// Function is called when the world is resetted
void GazeboRosCliffSensor::Reset() {new_laser_scans_connection_.reset();}

// Update the plugin
void GazeboRosCliffSensor::OnNewLaserScans()
{
  // Configure an empty message with the timestamp
  msg_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(world_->SimTime());

  // Find the minimum detected distance
  std::vector<double> ranges;
  cliff_sensor_->Ranges(ranges);
  const double range_detection =
    std::min(irobot_create_toolbox::FindMinimumRange(ranges), max_range_);
  RCLCPP_DEBUG_STREAM(ros_node_->get_logger(), "Cliff reporting " << range_detection << " m");

  if (range_detection >= detection_threshold_) {
    msg_.type = msg_.CLIFF;
    // Publish message
    pub_->publish(msg_);
    RCLCPP_DEBUG_EXPRESSION(
      ros_node_->get_logger(), true, "Cliff %s ON: %.3f", msg_.header.frame_id.c_str(),
      range_detection);
  }
}
}  // namespace irobot_create_gazebo_plugins
