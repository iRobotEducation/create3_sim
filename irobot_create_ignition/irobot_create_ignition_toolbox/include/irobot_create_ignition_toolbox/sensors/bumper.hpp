/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__BUMPER_HPP_
#define IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__BUMPER_HPP_

#include <atomic>
#include <memory>
#include <map>
#include <string>

#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace irobot_create_ignition_toolbox
{

class Bumper
{
public:
  explicit Bumper(std::shared_ptr<rclcpp::Node> & nh);
  virtual ~Bumper() {}

private:
  void bumper_callback(const ros_gz_interfaces::msg::Contacts::SharedPtr bumper_contact_msg);
  void robot_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  std::shared_ptr<rclcpp::Node> nh_;

  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr bumper_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr hazard_pub_;

  tf2::Transform last_robot_pose_;
  std::mutex robot_pose_mutex_;
};

}  // namespace irobot_create_ignition_toolbox

#endif  // IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__BUMPER_HPP_
