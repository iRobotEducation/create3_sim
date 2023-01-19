/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__SENSORS_NODE_HPP_
#define IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__SENSORS_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "irobot_create_ignition_toolbox/sensors/bumper.hpp"
#include "irobot_create_ignition_toolbox/sensors/cliff.hpp"
#include "irobot_create_ignition_toolbox/sensors/ir_intensity.hpp"
#include "irobot_create_ignition_toolbox/sensors/mouse.hpp"
#include "irobot_create_ignition_toolbox/sensors/wheel_drop.hpp"
#include "irobot_create_ignition_toolbox/sensors/ir_opcode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace irobot_create_ignition_toolbox
{

class SensorsNode : public rclcpp::Node
{
public:
  // Constructor and Destructor
  SensorsNode();

private:
  // Node
  rclcpp::Node::SharedPtr nh_;

  // Sensors
  std::unique_ptr<Bumper> bumper_;
  std::unique_ptr<Cliff> cliff_;
  std::unique_ptr<IrIntensity> ir_intensity_;
  std::unique_ptr<Mouse> mouse_;
  std::unique_ptr<WheelDrop> wheel_drop_;
  std::unique_ptr<IrOpcode> ir_opcode_;
};

}  // namespace irobot_create_ignition_toolbox

#endif  // IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__SENSORS_NODE_HPP_
