/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include "irobot_create_ignition_toolbox/pose_republisher/pose_republisher.hpp"
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<irobot_create_ignition_toolbox::PoseRepublisher>());
  rclcpp::shutdown();
  return 0;
}
