/*
 * Copyright 2021 Clearpath Robotics, Inc.
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
