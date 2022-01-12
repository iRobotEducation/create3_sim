// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Alberto Soragna (asoragna@irobot.com)

#include <memory>

#include "irobot_create_toolbox/motion_control_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<irobot_create_toolbox::MotionControlNode>());
  rclcpp::shutdown();
  return 0;
}
