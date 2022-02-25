// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Emiliano Javier Borghi Orue (creativa_eborghi@irobot.com)

#include "irobot_create_gazebo_plugins/gazebo_ros_ir_intensity_sensor.hpp"

#include <algorithm>
#include <vector>

#include "irobot_create_toolbox/math.hpp"

namespace irobot_create_gazebo_plugins
{
// Constructor
GazeboRosIrIntensitySensor::GazeboRosIrIntensitySensor()
: SensorPlugin() {}

// Destructor
GazeboRosIrIntensitySensor::~GazeboRosIrIntensitySensor() {new_laser_scans_connection_.reset();}

// Load the controller
void GazeboRosIrIntensitySensor::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor);
  new_laser_scans_connection_ = parent_sensor_->LaserShape()->ConnectNewLaserScans(
    std::bind(&GazeboRosIrIntensitySensor::OnNewLaserScans, this));

  // Configure the plugin from the SDF file
  ros_node_ = gazebo_ros::Node::Get(sdf);

  // Initialize ROS publishers
  pub_ = ros_node_->create_publisher<irobot_create_msgs::msg::IrIntensity>(
    "~/out", rclcpp::SensorDataQoS().reliable());

  // Configure our static message charasteristics
  msg_.header.frame_id = gazebo_ros::SensorFrameID(*sensor, *sdf);

  max_range_ = parent_sensor_->RangeMax();

  RCLCPP_INFO(ros_node_->get_logger(), "Starting IR Emitter Plugin!");
}

// On each sensor iteration
void GazeboRosIrIntensitySensor::OnNewLaserScans()
{
  msg_.header.stamp =
    gazebo_ros::Convert<builtin_interfaces::msg::Time>(parent_sensor_->LastMeasurementTime());

  // Find the minimum detected distance
  std::vector<double> ranges;
  parent_sensor_->Ranges(ranges);
  const double detection =
    std::min(irobot_create_toolbox::FindMinimumRange(ranges), max_range_);
  RCLCPP_DEBUG_STREAM(ros_node_->get_logger(), "IR reporting " << detection << " m");

  // IR sensor produces an exponential signal that is correlated to the distance,
  // that follows this formula: ir_reading = A exp(-x*B)
  // where:
  // A is a coefficient that depends on the color surface and
  // it can be as high as 3500
  // B is the decay of the signal related to the distance.
  // From the experiments B ~ 26.831568
  const double scaled_detection = 3500 * std::exp(detection * (-2 * M_E / max_range_));

  msg_.value = static_cast<irobot_create_msgs::msg::IrIntensity::_value_type>(scaled_detection);
  // Publish
  pub_->publish(msg_);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosIrIntensitySensor)

}  // namespace irobot_create_gazebo_plugins
