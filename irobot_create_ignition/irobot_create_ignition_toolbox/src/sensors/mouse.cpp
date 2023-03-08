/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include <memory>
#include <utility>

#include "irobot_create_ignition_toolbox/sensors/mouse.hpp"

using irobot_create_ignition_toolbox::Mouse;

Mouse::Mouse(std::shared_ptr<rclcpp::Node> & nh)
: nh_(nh),
  integrated_position_{0, 0, 0},
  last_mouse_position_{0, 0, 0}
{
  mouse_pose_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
    "_internal/sim_ground_truth_mouse_pose",
    rclcpp::SensorDataQoS(),
    std::bind(&Mouse::mouse_pose_callback, this, std::placeholders::_1));

  mouse_pub_ = nh_->create_publisher<irobot_create_msgs::msg::Mouse>(
    "mouse", rclcpp::SensorDataQoS());
}

void Mouse::mouse_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  auto mouse_msg = irobot_create_msgs::msg::Mouse();
  mouse_msg.header.stamp = nh_->now();
  mouse_msg.header.frame_id = "mouse_link";

  tf2::Transform mouse_pose;
  tf2::convert(msg->pose.pose, mouse_pose);

  tf2::Vector3 mouse_position = mouse_pose.getOrigin();

  // First iteration
  if (last_mouse_position_.isZero()) {
    last_mouse_position_ = mouse_position;
  }

  const tf2::Vector3 position_displacement = mouse_position - last_mouse_position_;

  integrated_position_ += position_displacement;

  mouse_msg.integrated_x = integrated_position_.getX();
  mouse_msg.integrated_y = integrated_position_.getY();

  mouse_pub_->publish(std::move(mouse_msg));

  // The pose is updated
  if (mouse_msg.integrated_x != 0.0 || mouse_msg.integrated_y != 0.0) {
    last_mouse_position_ = mouse_position;
  }
}
