/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef IROBOT_CREATE_IGNITION_TOOLBOX__UTILS_HPP_
#define IROBOT_CREATE_IGNITION_TOOLBOX__UTILS_HPP_

#include <atomic>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace irobot_create_ignition_toolbox
{

namespace utils
{

// Get yaw from tf2 Transform
inline double tf2_transform_to_yaw(const tf2::Transform tf)
{
  auto tf_m = tf2::Matrix3x3(tf.getRotation());
  double tf_r, tf_p, tf_y;
  tf_m.getRPY(tf_r, tf_p, tf_y);
  return tf_y;
}

// Get object position wrt frame
inline tf2::Vector3 object_wrt_frame(tf2::Transform object, tf2::Transform frame)
{
  return frame.inverseTimes(object).getOrigin();
}

// Get global transform of a static link connected to base frame
inline tf2::Transform static_link_wrt_global_frame(
  tf2::Transform static_link,
  tf2::Transform base_frame)
{
  tf2::Transform global_pose(tf2::Transform::getIdentity());

  double base_y = tf2_transform_to_yaw(base_frame);

  // Rotate static link frame by the base yaw
  global_pose.getOrigin().setX(
    cos(base_y) * static_link.getOrigin().getX() -
    sin(base_y) * static_link.getOrigin().getY());
  global_pose.getOrigin().setY(
    sin(base_y) * static_link.getOrigin().getX() +
    cos(base_y) * static_link.getOrigin().getY());

  // Static link pose is relative to base pose, so add the base pose to get global pose
  global_pose.setOrigin(global_pose.getOrigin() + base_frame.getOrigin());
  // Get global rotation
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, tf2_transform_to_yaw(global_pose) + tf2_transform_to_yaw(base_frame));
  global_pose.setRotation(rotation);

  return global_pose;
}

inline nav_msgs::msg::Odometry::UniquePtr tf_message_to_odom(
  const tf2_msgs::msg::TFMessage::SharedPtr msg, uint16_t i)
{
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header = msg->transforms[i].header;
  odom_msg->child_frame_id = msg->transforms[i].child_frame_id;

  odom_msg->pose.pose.position.x = msg->transforms[i].transform.translation.x;
  odom_msg->pose.pose.position.y = msg->transforms[i].transform.translation.y;
  odom_msg->pose.pose.position.z = msg->transforms[i].transform.translation.z;

  odom_msg->pose.pose.orientation.x = msg->transforms[i].transform.rotation.x;
  odom_msg->pose.pose.orientation.y = msg->transforms[i].transform.rotation.y;
  odom_msg->pose.pose.orientation.z = msg->transforms[i].transform.rotation.z;
  odom_msg->pose.pose.orientation.w = msg->transforms[i].transform.rotation.w;

  return odom_msg;
}

inline void tf2_transform_to_pose(const tf2::Transform tf, geometry_msgs::msg::Pose & pose)
{
  pose.position.set__x(tf.getOrigin().getX());
  pose.position.set__y(tf.getOrigin().getY());
  pose.position.set__z(tf.getOrigin().getZ());

  pose.orientation.set__x(tf.getRotation().getX());
  pose.orientation.set__y(tf.getRotation().getY());
  pose.orientation.set__z(tf.getRotation().getZ());
  pose.orientation.set__w(tf.getRotation().getW());
}


}  // namespace utils

}  // namespace irobot_create_ignition_toolbox

#endif  // IROBOT_CREATE_IGNITION_TOOLBOX__UTILS_HPP_
