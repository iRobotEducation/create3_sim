/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include <memory>

#include "irobot_create_ignition_toolbox/sensors/sensors_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<irobot_create_ignition_toolbox::SensorsNode>());
  rclcpp::shutdown();
  return 0;
}
