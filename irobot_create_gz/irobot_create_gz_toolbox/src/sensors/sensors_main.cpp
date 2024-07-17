/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include <memory>

#include "irobot_create_gz_toolbox/sensors/sensors_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<irobot_create_gz_toolbox::SensorsNode>());
  rclcpp::shutdown();
  return 0;
}
