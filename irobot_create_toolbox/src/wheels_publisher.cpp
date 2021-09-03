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

#include <irobot_create_toolbox/wheels_publisher.hpp>

WheelsPublisher::WheelsPublisher() : rclcpp::Node("wheels_publisher_node")
{
  // Topic parameter to publish angular velocity to
  const rclcpp::ParameterValue vels_topic_param = declare_parameter("velocity_topic");
  // Unset parameters have a type: rclcpp::ParameterType::PARAMETER_NOT_SET
  if (vels_topic_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    throw rclcpp::exceptions::InvalidParameterTypeException(
      "velocity_topic", "Not of type string or was not set");
  }
  const std::string velocity_topic = vels_topic_param.get<std::string>();

  // Topic parameter to publish wheel ticks to
  const rclcpp::ParameterValue ticks_topic_param = declare_parameter("ticks_topic");
  // Unset parameters have a type: rclcpp::ParameterType::PARAMETER_NOT_SET
  if (ticks_topic_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    throw rclcpp::exceptions::InvalidParameterTypeException(
      "ticks_topic", "Not of type string or was not set");
  }
  const std::string ticks_topic = ticks_topic_param.get<std::string>();

  // Publish rate parameter
  const rclcpp::ParameterValue publish_rate_param = declare_parameter("publish_rate");
  // Unset parameters have a type: rclcpp::ParameterType::PARAMETER_NOT_SET
  if (publish_rate_param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw rclcpp::exceptions::InvalidParameterTypeException(
      "publish_rate", "Not of type double or was not set");
  }
  const double publish_rate = publish_rate_param.get<double>();  // Hz

  // Encoder resolution
  const rclcpp::ParameterValue encoder_resolution_param = declare_parameter("encoder_resolution");
  // Unset parameters have a type: rclcpp::ParameterType::PARAMETER_NOT_SET
  if (encoder_resolution_param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw rclcpp::exceptions::InvalidParameterTypeException(
      "encoder_resolution", "Not of type double or was not set");
  }

  encoder_resolution_ = encoder_resolution_param.get<double>();  // Ticks per revolution

  // Wheel radius parameter
  rclcpp::ParameterValue wheel_radius_param = declare_parameter("wheel_radius");
  // Unset parameters have a type: rclcpp::ParameterType::PARAMETER_NOT_SET
  if (wheel_radius_param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw rclcpp::exceptions::InvalidParameterTypeException(
      "wheel_radius", "Not of type double or was not set");
  }

  // Set wheel circumference
  wheel_circumference_ = 2 * M_PI * wheel_radius_param.get<double>();  // wheel radius in meters

  angular_vels_publisher_ = this->create_publisher<irobot_create_msgs::msg::WheelVels>(
    velocity_topic, rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << velocity_topic);
  wheel_ticks_publisher_ = this->create_publisher<irobot_create_msgs::msg::WheelTicks>(
    ticks_topic, rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << ticks_topic);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1 / publish_rate),
    std::bind(&WheelsPublisher::publisher_callback, this));

  subscription_ = this->create_subscription<control_msgs::msg::DynamicJointState>(
    "dynamic_joint_states", rclcpp::SystemDefaultsQoS(),
    [this](const control_msgs::msg::DynamicJointState::SharedPtr msg){
      std::lock_guard<std::mutex> lock{mutex_};
      last_joint_state_ = *msg;
    }
  );
}

void WheelsPublisher::publisher_callback()
{
  {  // Mutex scope
    // Should not proceed if vector is empty
    if (last_joint_state_.joint_names.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock{mutex_};
    // Write WheelVels msg
    angular_vels_msg_.velocity_left = get_dynamic_state_value("left_wheel_joint", "velocity");
    angular_vels_msg_.velocity_right = get_dynamic_state_value("right_wheel_joint", "velocity");

    // Calculate and write WheelTicks msg
    const double left_ticks =
      (get_dynamic_state_value("left_wheel_joint", "position") /
       wheel_circumference_) *
      encoder_resolution_;
    const double right_ticks =
      (get_dynamic_state_value("right_wheel_joint", "position") /
       wheel_circumference_) *
      encoder_resolution_;

    wheel_ticks_msg_.ticks_left = std::round(left_ticks);
    wheel_ticks_msg_.ticks_right = std::round(right_ticks);
  }

  // Publish messages
  angular_vels_publisher_->publish(angular_vels_msg_);
  wheel_ticks_publisher_->publish(wheel_ticks_msg_);
}

int WheelsPublisher::get_joint_index(std::string joint_name){
  for(int k=0 ; k < (int)last_joint_state_.joint_names.size() ; k++){
    if(last_joint_state_.joint_names[k] == joint_name){
      return k;
    }
  }
  // TODO Throw element not found exception
  return -1;
}

int WheelsPublisher::get_interface_index(std::string interface_name, int joint_index){
  for(int k=0 ; k < (int)last_joint_state_.interface_values[joint_index].interface_names.size() ; k++){
    if(last_joint_state_.interface_values[joint_index].interface_names[k] == interface_name){
      return k;
    }
  }
  // TODO Throw element not found exception
  return -1;
}

double WheelsPublisher::get_dynamic_state_value(std::string joint_name, std::string interface_name){
  int joint_index = get_joint_index(joint_name);
  int interface_index = get_interface_index(interface_name, joint_index);

  return last_joint_state_.interface_values[joint_index].values[interface_index];
}
