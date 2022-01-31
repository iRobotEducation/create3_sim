/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include <memory>
#include <utility>

#include "irobot_create_ignition_toolbox/sensors/wheel_drop.hpp"

using irobot_create_ignition_toolbox::WheelDrop;

WheelDrop::WheelDrop(std::shared_ptr<rclcpp::Node> & nh)
: nh_(nh),
  detection_threshold_(0.03),
  lower_limit_(detection_threshold_ * 0.75),
  upper_limit_(detection_threshold_ * 0.95),
  joints_{"wheel_drop_left_joint",
    "wheel_drop_right_joint"}
{
  joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&WheelDrop::joint_state_callback, this, std::placeholders::_1));

  hazard_pub_[joints_[0]] = nh_->create_publisher<irobot_create_msgs::msg::HazardDetection>(
    "_internal/wheel_drop/left_wheel/event", rclcpp::SensorDataQoS());
  hazard_pub_[joints_[1]] = nh_->create_publisher<irobot_create_msgs::msg::HazardDetection>(
    "_internal/wheel_drop/right_wheel/event", rclcpp::SensorDataQoS());

  wheeldrop_detected_[joints_[0]] = false;
  wheeldrop_detected_[joints_[1]] = false;
}

void WheelDrop::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
{
  // Get position of each wheeldrop joint
  for (size_t i = 0; i < joint_state_msg->name.size(); i++) {
    for (auto joint : joints_) {
      if (joint_state_msg->name[i] == joint) {
        displacement_[joint] = joint_state_msg->position[i];
      }
    }
  }

  // Check each joint for wheeldrop
  for (auto joint : joints_) {
    if (!wheeldrop_detected_[joint] && (displacement_[joint] >= upper_limit_)) {
      wheeldrop_detected_[joint] = true;
    } else if (wheeldrop_detected_[joint] && (displacement_[joint] <= lower_limit_)) {
      wheeldrop_detected_[joint] = false;
    }

    // Publish if wheeldrop is detected
    if (wheeldrop_detected_[joint]) {
      auto hazard_msg = irobot_create_msgs::msg::HazardDetection();
      hazard_msg.type = irobot_create_msgs::msg::HazardDetection::WHEEL_DROP;
      hazard_msg.header.stamp = nh_->now();
      hazard_msg.header.frame_id = joint;
      hazard_pub_[joint]->publish(std::move(hazard_msg));
    }
  }
}
