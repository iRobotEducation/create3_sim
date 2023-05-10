/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include <string>
#include <utility>

#include "irobot_create_ignition_toolbox/pose_republisher/pose_republisher.hpp"

using irobot_create_ignition_toolbox::PoseRepublisher;

PoseRepublisher::PoseRepublisher()
: rclcpp::Node("pose_republisher_node"),
  wheel_joints_{"left_wheel_joint",
    "right_wheel_joint"}
{
  robot_name_ =
    this->declare_parameter("robot_name", "create3");
  dock_name_ =
    this->declare_parameter("dock_name", "standard_dock");
  std::string robot_pub_topic =
    this->declare_parameter("robot_publisher_topic", "sim_ground_truth_pose");
  std::string robot_sub_topic =
    this->declare_parameter("robot_subscriber_topic", "sim_ground_truth_pose");
  std::string mouse_pub_topic =
    this->declare_parameter("mouse_publisher_topic", "sim_ground_truth_mouse_pose");

  std::string dock_pub_topic =
    this->declare_parameter("dock_publisher_topic", "sim_ground_truth_dock_pose");
  std::string dock_sub_topic =
    this->declare_parameter("dock_subscriber_topic", "sim_ground_truth_dock_pose");
  std::string ir_emitter_pub_topic =
    this->declare_parameter("ir_emitter_publisher_topic", "sim_ground_truth_ir_emitter_pose");
  std::string ir_receiver_pub_topic =
    this->declare_parameter("ir_receiver_publisher_topic", "sim_ground_truth_ir_receiver_pose");

  robot_subscriber_ = create_subscription<tf2_msgs::msg::TFMessage>(
    robot_sub_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&PoseRepublisher::robot_subscriber_callback, this, std::placeholders::_1));

  robot_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
    robot_pub_topic,
    rclcpp::SensorDataQoS());

  mouse_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
    mouse_pub_topic,
    rclcpp::SensorDataQoS());

  ir_opcode_receiver_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
    ir_receiver_pub_topic,
    rclcpp::SensorDataQoS());

  dock_subscriber_ = create_subscription<tf2_msgs::msg::TFMessage>(
    dock_sub_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&PoseRepublisher::dock_subscriber_callback, this, std::placeholders::_1));

  dock_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
    dock_pub_topic,
    rclcpp::SensorDataQoS());

  ir_opcode_emitter_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
    ir_emitter_pub_topic,
    rclcpp::SensorDataQoS());

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&PoseRepublisher::joint_state_subscriber_callback, this, std::placeholders::_1));

  dynamic_joint_state_publisher_ = create_publisher<control_msgs::msg::DynamicJointState>(
    "dynamic_joint_states",
    rclcpp::SystemDefaultsQoS());
}

void PoseRepublisher::robot_subscriber_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (uint16_t i = 0; i < msg->transforms.size(); i++) {
    // Child frame is model name (i.e. create3)
    if (msg->transforms[i].child_frame_id == robot_name_) {
      auto odom_msg = utils::tf_message_to_odom(msg, i);
      // Save robot pose
      tf2::convert(odom_msg->pose.pose, last_robot_pose_);
      robot_publisher_->publish(std::move(odom_msg));
    } else if (msg->transforms[i].child_frame_id.find("mouse") != std::string::npos) {
      // Send mouse transform
      auto mouse_msg = utils::tf_message_to_odom(msg, i);
      tf2::Transform mouse_pose;
      // Convert pose to tf2 transform
      tf2::convert(mouse_msg->pose.pose, mouse_pose);
      // Get mouse pose with respect to world frame
      tf2::Transform mouse_world_pose = utils::static_link_wrt_global_frame(
        mouse_pose, last_robot_pose_);
      // Convert tf2 transform back to pose
      utils::tf2_transform_to_pose(mouse_world_pose, mouse_msg->pose.pose);
      // Publish
      mouse_publisher_->publish(std::move(mouse_msg));
    } else if (msg->transforms[i].child_frame_id.find("ir_omni") != std::string::npos) {
      // Send IR Opcode Receiver transform
      auto receiver_msg = utils::tf_message_to_odom(msg, i);
      tf2::Transform receiver_pose;
      // Convert pose to tf2 transform
      tf2::convert(receiver_msg->pose.pose, receiver_pose);
      // Get receiver pose with respect to world frame
      tf2::Transform receiver_world_pose = utils::static_link_wrt_global_frame(
        receiver_pose, last_robot_pose_);
      // Convert tf2 transform back to pose
      utils::tf2_transform_to_pose(receiver_world_pose, receiver_msg->pose.pose);
      // Publish
      ir_opcode_receiver_publisher_->publish(std::move(receiver_msg));
    }
  }
}

void PoseRepublisher::dock_subscriber_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (uint16_t i = 0; i < msg->transforms.size(); i++) {
    // Child frame is model name
    if (msg->transforms[i].child_frame_id == dock_name_) {
      auto odom_msg = utils::tf_message_to_odom(msg, i);
      // Save dock pose
      tf2::convert(odom_msg->pose.pose, last_dock_pose_);
      dock_publisher_->publish(std::move(odom_msg));
    } else if (msg->transforms[i].child_frame_id.find("halo_link") != std::string::npos) {
      // Send IR Opcode Emitter transform
      auto emitter_msg = utils::tf_message_to_odom(msg, i);
      tf2::Transform emitter_pose;
      // Convert pose to tf2 transform
      tf2::convert(emitter_msg->pose.pose, emitter_pose);
      // Get emitter pose with respect to world frame
      tf2::Transform emitter_world_pose = utils::static_link_wrt_global_frame(
        emitter_pose, last_dock_pose_);
      // Convert tf2 transform back to pose
      utils::tf2_transform_to_pose(emitter_world_pose, emitter_msg->pose.pose);
      // Publish
      ir_opcode_emitter_publisher_->publish(std::move(emitter_msg));
    }
  }
}

// Publish wheel joint states as dynamic joint state message
void PoseRepublisher::joint_state_subscriber_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  auto dynamic_joint_state_msg = control_msgs::msg::DynamicJointState();
  dynamic_joint_state_msg.header = msg->header;

  for (int i = 0; i < 2; i++) {
    std::string wheel_joint_name = wheel_joints_[i];
    dynamic_joint_state_msg.joint_names.push_back(wheel_joint_name);

    auto interface_value = control_msgs::msg::InterfaceValue();
    interface_value.interface_names.push_back("position");
    interface_value.interface_names.push_back("velocity");

    dynamic_joint_state_msg.interface_values.push_back(interface_value);
    for (size_t j = 0; j < msg->name.size(); j++) {
      if (msg->name[j] == wheel_joint_name) {
        if (msg->position.size() == msg->name.size()) {
          dynamic_joint_state_msg.interface_values[i].values.push_back(msg->position[j]);
        } else {
          dynamic_joint_state_msg.interface_values[i].values.push_back(0.0);
        }

        if (msg->velocity.size() == msg->name.size()) {
          dynamic_joint_state_msg.interface_values[i].values.push_back(msg->velocity[j]);
        } else {
          dynamic_joint_state_msg.interface_values[i].values.push_back(0.0);
        }
      }
    }
  }

  dynamic_joint_state_publisher_->publish(std::move(dynamic_joint_state_msg));
}
