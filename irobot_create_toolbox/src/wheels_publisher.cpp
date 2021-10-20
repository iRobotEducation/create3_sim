// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "irobot_create_toolbox/wheels_publisher.hpp"

#include <string>
#include <vector>

namespace irobot_create_toolbox
{
WheelsPublisher::WheelsPublisher()
: rclcpp::Node("wheels_publisher_node")
{
  // Topic parameter to publish angular velocity to
  const std::string velocity_topic = declare_and_get_parameter<std::string>("velocity_topic", this);

  // Topic parameter to publish wheel ticks to
  const std::string ticks_topic = declare_and_get_parameter<std::string>("ticks_topic", this);

  // Publish rate parameter
  const double publish_rate = declare_and_get_parameter<double>("publish_rate", this);  // Hz

  // Encoder resolution
  encoder_resolution_ =
    declare_and_get_parameter<double>("encoder_resolution", this);  // Ticks per revolution

  // Set wheel circumference from wheel radius parameter
  wheel_circumference_ =
    2 * M_PI * declare_and_get_parameter<double>("wheel_radius", this);  // wheel radius in meters

  angular_vels_publisher_ = create_publisher<irobot_create_msgs::msg::WheelVels>(
    velocity_topic, rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << velocity_topic);
  wheel_ticks_publisher_ =
    create_publisher<irobot_create_msgs::msg::WheelTicks>(ticks_topic, rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << ticks_topic);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1 / publish_rate),
    std::bind(&WheelsPublisher::publisher_callback, this));

  subscription_ = create_subscription<control_msgs::msg::DynamicJointState>(
    "dynamic_joint_states", rclcpp::SystemDefaultsQoS(),
    [this](const control_msgs::msg::DynamicJointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock{this->mutex_};
      this->last_joint_state_ = *msg;
    });

  // Set header frame_id.
  this->angular_vels_msg_.header.frame_id = "base_link";
  this->wheel_ticks_msg_.header.frame_id = "base_link";
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
      (get_dynamic_state_value("left_wheel_joint", "position") / wheel_circumference_) *
      encoder_resolution_;
    const double right_ticks =
      (get_dynamic_state_value("right_wheel_joint", "position") / wheel_circumference_) *
      encoder_resolution_;

    wheel_ticks_msg_.ticks_left = std::round(left_ticks);
    wheel_ticks_msg_.ticks_right = std::round(right_ticks);
  }

  // Set header timestamp.
  this->angular_vels_msg_.header.stamp = now();
  this->wheel_ticks_msg_.header.stamp = now();

  // Publish messages
  angular_vels_publisher_->publish(angular_vels_msg_);
  wheel_ticks_publisher_->publish(wheel_ticks_msg_);
}

size_t WheelsPublisher::get_joint_index(std::string joint_name)
{
  for (size_t k = 0; k < last_joint_state_.joint_names.size(); k++) {
    if (last_joint_state_.joint_names[k] == joint_name) {
      return k;
    }
  }

  throw std::out_of_range(joint_name + " is not a joint name in joint_names vector");
}

size_t WheelsPublisher::get_interface_index(std::string interface_name, size_t joint_index)
{
  const std::vector<std::string> & interface_names =
    last_joint_state_.interface_values[joint_index].interface_names;
  for (size_t k = 0; k < interface_names.size(); k++) {
    if (interface_names[k] == interface_name) {
      return k;
    }
  }

  throw std::out_of_range(interface_name + " is not an interface name in interface_names vector");
}

double WheelsPublisher::get_dynamic_state_value(std::string joint_name, std::string interface_name)
{
  const size_t joint_index = get_joint_index(joint_name);
  const size_t interface_index = get_interface_index(interface_name, joint_index);

  return last_joint_state_.interface_values[joint_index].values[interface_index];
}

}  // namespace irobot_create_toolbox
