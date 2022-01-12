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

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "std_msgs/msg/int32.hpp"


namespace irobot_create_ignition_toolbox
{

enum Create3Buttons {
  NONE,
  BUTTON_1,
  BUTTON_POWER,
  BUTTON_2
};

class InterfaceButtons : public rclcpp::Node
{
public:
  // Constructor and Destructor
  InterfaceButtons();

private:
  void create3_buttons_callback(const std_msgs::msg::Int32::SharedPtr create3_buttons_msg);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr interface_buttons_sub_;
  rclcpp::Publisher<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr interface_buttons_pub_;

  std::unique_ptr<irobot_create_msgs::msg::InterfaceButtons> interface_buttons_;
};

}  // namespace irobot_create_ignition_toolbox
