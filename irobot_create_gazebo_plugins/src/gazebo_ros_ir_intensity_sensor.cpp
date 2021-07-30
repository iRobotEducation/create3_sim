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

#include <irobot_create_gazebo_plugins/gazebo_ros_ir_intensity_sensor.hpp>

namespace irobot_create_gazebo_plugins
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosIrIntensitySensor)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosIrIntensitySensor::GazeboRosIrIntensitySensor() : SensorPlugin() {}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIrIntensitySensor::~GazeboRosIrIntensitySensor() { new_laser_scans_connection_.reset(); }

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
    "~/out", rclcpp::SensorDataQoS());

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
  double detection = max_range_;
  std::vector<double> ranges;
  parent_sensor_->Ranges(ranges);
  auto detection_ptr = std::min_element(std::begin(ranges), std::end(ranges));
  if (detection_ptr != std::end(ranges)) detection = *detection_ptr;
  RCLCPP_DEBUG_STREAM(ros_node_->get_logger(), "IR reporting " << detection << " m");

  // IR sensor produces an exponential signal that is corelated to the distance,
  // that follows this formula: ir_reading = A exp(-x*B)
  // where:
  // A is a coefficient that depends on the colour surface and
  // it an be as much as 3500
  // B is the decay of the signal related to the distance.
  // From the experiments B ~ 26.831568
  const double scaled_detection = 3500 * std::exp(detection * (-2 * M_E / max_range_));

  msg_.value = static_cast<irobot_create_msgs::msg::IrIntensity::_value_type>(scaled_detection);
  // Publish
  pub_->publish(msg_);
}

}  // namespace irobot_create_gazebo_plugins
