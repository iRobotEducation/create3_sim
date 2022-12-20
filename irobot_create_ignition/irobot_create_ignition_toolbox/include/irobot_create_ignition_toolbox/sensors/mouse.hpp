/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__MOUSE_HPP_
#define IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__MOUSE_HPP_

#include <atomic>
#include <chrono>
#include <memory>

#include "irobot_create_msgs/msg/mouse.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace irobot_create_ignition_toolbox
{

class Mouse
{
public:
  explicit Mouse(std::shared_ptr<rclcpp::Node> & nh);
  virtual ~Mouse() {}

private:
  void robot_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void mouse_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  std::shared_ptr<rclcpp::Node> nh_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mouse_pose_sub_;
  rclcpp::Publisher<irobot_create_msgs::msg::Mouse>::SharedPtr mouse_pub_;

  tf2::Vector3 integrated_position_;
  tf2::Vector3 last_mouse_position_;
};

}  // namespace irobot_create_ignition_toolbox

#endif  // IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__MOUSE_HPP_
