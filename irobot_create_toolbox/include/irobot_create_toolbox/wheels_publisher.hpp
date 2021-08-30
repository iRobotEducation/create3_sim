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

#include <cmath>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <irobot_create_msgs/msg/wheel_ticks.hpp>
#include <irobot_create_msgs/msg/wheel_vels.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class WheelsPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  WheelsPublisher();
  void subscription_callback(const control_msgs::msg::DynamicJointState::SharedPtr msg);
  void publisher_callback();

private:
  // Vector indeces
  enum WheelSide {
    RIGHT = 0,
    LEFT
  };

  enum WheelState{
    VELOCITY = 1,
    DISPLACEMENT
  };

  double encoder_resolution_;
  double wheel_radius_;
  double wheel_circumference_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<control_msgs::msg::InterfaceValue> last_interface_values_;

  irobot_create_msgs::msg::WheelVels angular_vels_msg_;
  irobot_create_msgs::msg::WheelTicks wheel_ticks_msg_;
  rclcpp::Publisher<irobot_create_msgs::msg::WheelVels>::SharedPtr angular_vels_publisher_;
  rclcpp::Publisher<irobot_create_msgs::msg::WheelTicks>::SharedPtr wheel_ticks_publisher_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr subscription_;
  // Mutex
  std::mutex mutex_;
};
