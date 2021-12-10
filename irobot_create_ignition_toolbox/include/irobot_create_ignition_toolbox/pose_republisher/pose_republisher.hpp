/*
 * Copyright 2021 Clearpath Robotics, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#pragma once

#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "irobot_create_ignition_toolbox/utils.hpp"

namespace irobot_create_ignition_toolbox
{
  
class PoseRepublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  PoseRepublisher();

private:
  void robot_subscriber_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void dock_subscriber_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void joint_state_subscriber_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  // Odometry publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dock_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mouse_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ir_opcode_emitter_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ir_opcode_receiver_publisher_;
  rclcpp::Publisher<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_joint_state_publisher_;

  // TFMessage subscribers
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr robot_subscriber_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dock_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  tf2::Transform last_robot_pose_;
  tf2::Transform last_dock_pose_;

  std::string robot_name_;
  std::string wheel_joints_[2];
};

}  // namespace irobot_create_toolbox
