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

#include "irobot_create_ignition_toolbox/sensors/bumper.hpp"
#include "irobot_create_ignition_toolbox/sensors/cliff.hpp"
#include "irobot_create_ignition_toolbox/sensors/ir_intensity.hpp"
#include "irobot_create_ignition_toolbox/sensors/mouse.hpp"
#include "irobot_create_ignition_toolbox/sensors/wheel_drop.hpp"
#include "irobot_create_ignition_toolbox/sensors/ir_opcode.hpp"

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
