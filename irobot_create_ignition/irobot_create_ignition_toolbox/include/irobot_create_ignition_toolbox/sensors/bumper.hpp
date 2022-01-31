/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__BUMPER_HPP_
#define IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__BUMPER_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <map>

#include "nav_msgs/msg/odometry.hpp"
#include "ros_ign_interfaces/msg/contacts.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"


namespace irobot_create_ignition_toolbox
{

class Bumper
{
public:
  explicit Bumper(std::shared_ptr<rclcpp::Node> & nh);
  virtual ~Bumper() {}

private:
// Bumper zones
  enum class ZoneType { RIGHT, CENTER_RIGHT, CENTER, CENTER_LEFT, LEFT };

// Auxiliary data structure to hold bumper zone details
  struct Zone
  {
    double left_limit;
    double right_limit;
    std::string name;
  };

// Data structure to hold the definitions related to bumper zones
  const std::map<ZoneType, Zone> angles_map_ = {
    {ZoneType::RIGHT, {-M_PI / 2, -3 * M_PI / 10, "bump_right"}},
    {ZoneType::CENTER_RIGHT, {-3 * M_PI / 10, -M_PI / 10, "bump_front_right"}},
    {ZoneType::CENTER, {-M_PI / 10, M_PI / 10, "bump_front_center"}},
    {ZoneType::CENTER_LEFT, {M_PI / 10, 3 * M_PI / 10, "bump_front_left"}},
    {ZoneType::LEFT, {3 * M_PI / 10, M_PI / 2, "bump_left"}}
  };

  void bumper_callback(const ros_ign_interfaces::msg::Contacts::SharedPtr bumper_contact_msg);
  void robot_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  std::shared_ptr<rclcpp::Node> nh_;

  rclcpp::Subscription<ros_ign_interfaces::msg::Contacts>::SharedPtr bumper_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr hazard_pub_;

  tf2::Transform last_robot_pose_;
  std::mutex robot_pose_mutex_;
};

}  // namespace irobot_create_ignition_toolbox

#endif  // IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__BUMPER_HPP_
