/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include <memory>

#include "irobot_create_ignition_toolbox/sensors/sensors_node.hpp"

using irobot_create_ignition_toolbox::SensorsNode;

SensorsNode::SensorsNode()
: rclcpp::Node("sensors_node")
{
  // Create node handle
  nh_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  // Add sensors
  bumper_ = std::make_unique<Bumper>(nh_);
  cliff_ = std::make_unique<Cliff>(nh_);
  ir_intensity_ = std::make_unique<IrIntensity>(nh_);
  mouse_ = std::make_unique<Mouse>(nh_);
  wheel_drop_ = std::make_unique<WheelDrop>(nh_);
  ir_opcode_ = std::make_unique<IrOpcode>(nh_);
}
