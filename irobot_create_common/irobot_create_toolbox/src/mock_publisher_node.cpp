// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#include <irobot_create_toolbox/mock_publisher.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<irobot_create_toolbox::MockPublisher>());
  rclcpp::shutdown();
  return 0;
}
