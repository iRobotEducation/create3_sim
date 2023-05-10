/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include <memory>
#include <utility>

#include "irobot_create_ignition_toolbox/sensors/bumper.hpp"
#include "irobot_create_ignition_toolbox/utils.hpp"
#include "irobot_create_toolbox/math.hpp"
#include "irobot_create_toolbox/polar_coordinates.hpp"
#include "irobot_create_toolbox/sensors/bumpers.hpp"

using irobot_create_ignition_toolbox::Bumper;

Bumper::Bumper(std::shared_ptr<rclcpp::Node> & nh)
: nh_(nh)
{
  bumper_sub_ = nh_->create_subscription<ros_gz_interfaces::msg::Contacts>(
    "bumper_contact",
    rclcpp::SensorDataQoS(),
    std::bind(&Bumper::bumper_callback, this, std::placeholders::_1));

  robot_pose_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
    "sim_ground_truth_pose",
    rclcpp::SensorDataQoS(),
    std::bind(&Bumper::robot_pose_callback, this, std::placeholders::_1));

  hazard_pub_ = nh_->create_publisher<irobot_create_msgs::msg::HazardDetection>(
    "_internal/bumper/event", rclcpp::SensorDataQoS());
}

void Bumper::bumper_callback(const ros_gz_interfaces::msg::Contacts::SharedPtr bumper_contact_msg)
{
  tf2::Transform robot_pose(tf2::Transform::getIdentity());
  {
    const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
    robot_pose = last_robot_pose_;
  }

  // Get rotation of robot in RPY
  auto robot_q = robot_pose.getRotation();
  auto robot_m = tf2::Matrix3x3(robot_q);
  double robot_r, robot_p, robot_y;
  robot_m.getRPY(robot_r, robot_p, robot_y);

  // The bumper may be in contact with multiple objects simultaneously
  for (const auto & contact : bumper_contact_msg->contacts) {
    uint16_t count = 0;
    geometry_msgs::msg::Vector3 average_position;

    for (const auto position : contact.positions) {
      average_position.x += position.x;
      average_position.y += position.y;
      average_position.z += position.z;
      count++;
    }

    if (count != 0) {
      // Get average
      average_position.x /= count;
      average_position.y /= count;
      average_position.z /= count;
    }

    tf2::Transform average_pose;
    average_pose.setOrigin(
      tf2::Vector3(
        average_position.x,
        average_position.y,
        average_position.z));

    tf2::Vector3 contact_point = utils::object_wrt_frame(average_pose, robot_pose);
    ignition::math::Vector2d cartesian_coord = {contact_point[0], contact_point[1]};
    auto azimuth = irobot_create_toolbox::toPolar(cartesian_coord).azimuth;

    // Find contact zone
    const auto iter = std::find_if(
      irobot_create_toolbox::sensors::BUMPER_ZONES_MAP.begin(),
      irobot_create_toolbox::sensors::BUMPER_ZONES_MAP.end(),
      [this, azimuth](const auto & zone) -> bool {
        return irobot_create_toolbox::IsAngleBetween(
          zone.second.left_limit, zone.second.right_limit, azimuth);
      });

    if (iter != irobot_create_toolbox::sensors::BUMPER_ZONES_MAP.end()) {
      auto hazard_msg = irobot_create_msgs::msg::HazardDetection();
      hazard_msg.type = irobot_create_msgs::msg::HazardDetection::BUMP;
      hazard_msg.header.frame_id = iter->second.name;
      hazard_msg.header.stamp = nh_->now();
      hazard_pub_->publish(std::move(hazard_msg));
    }
  }
}

void Bumper::robot_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
  tf2::convert(msg->pose.pose, last_robot_pose_);
}
