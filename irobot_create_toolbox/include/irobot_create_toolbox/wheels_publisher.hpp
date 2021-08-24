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

#pragma once

#include <math.h>

#include <cmath>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <irobot_create_msgs/msg/wheel_ticks.hpp>
#include <irobot_create_msgs/msg/wheel_vels.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// TODO set these via parameters
// Set encoder resolution
constexpr double ENCODER_RESOLUTION = 508.8;
// Set wheel radius in meters
constexpr double WHEEL_RADIUS = 0.03575;

// Set wheel circumference
constexpr double WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS;

class WheelsPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  WheelsPublisher();
  void subscription_callback(const control_msgs::msg::DynamicJointState::SharedPtr msg);
  void publisher_callback();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  double last_right_angular_vel_;
  double last_left_angular_vel_;
  double last_right_displacement_;
  double last_left_displacement_;

  irobot_create_msgs::msg::WheelVels angular_vels_msg_;
  irobot_create_msgs::msg::WheelTicks wheel_ticks_msg_;
  std::shared_ptr<rclcpp::Publisher<irobot_create_msgs::msg::WheelVels>> angular_vels_publisher_;
  std::shared_ptr<rclcpp::Publisher<irobot_create_msgs::msg::WheelTicks>> wheel_ticks_publisher_;
  std::shared_ptr<rclcpp::Subscription<control_msgs::msg::DynamicJointState>> subscription_;
  // Mutex
  std::mutex mutex_;
};
