// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Emiliano Javier Borghi Orue (creativa_eborghi@irobot.com)

#include "irobot_create_gazebo_plugins/gazebo_ros_wheel_drop.hpp"

#include <string>

namespace irobot_create_gazebo_plugins
{
void GazeboRosWheelDrop::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  double update_rate{62.0};
  double detection_threshold{0.7};
  std::string joint_name{""};
  utils::initialize(update_rate, sdf, "update_rate", 62.0);
  utils::initialize(detection_threshold, sdf, "detection_threshold", 0.7);
  utils::initialize(joint_name, sdf, "joint_name", "");
  utils::initialize(frame_id_, sdf, "frame_id", "");

  world_ = model->GetWorld();
  GZ_ASSERT(world_, "[WHEEL DROP PLUGIN] Invalid world pointer!");
  joint_ = model->GetJoint(joint_name);
  GZ_ASSERT(joint_, "[WHEEL DROP PLUGIN] Invalid joint pointer!");
  name_ = joint_->GetName();

  // Set bounds for wheel drop detection
  lower_limit_ = detection_threshold * 0.75;
  upper_limit_ = detection_threshold * 0.95;

  // Create a GazeboRos node
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  ros_node_ = gazebo_ros::Node::Get(sdf);
  // Get QoS profiles
  const gazebo_ros::QoS & qos = ros_node_->get_qos();
  // Initialize ROS publisher
  pub_ = ros_node_->create_publisher<irobot_create_msgs::msg::HazardDetection>(
    "~/out", qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable()));

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  update_connection_ =
    gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboRosWheelDrop::OnUpdate, this));

  // Rate enforcer
  rate_enforcer_.load(update_rate);

  // Initialize time
  last_time_ = world_->SimTime();

  RCLCPP_INFO(ros_node_->get_logger(), "Started wheel drop plugin");
}

void GazeboRosWheelDrop::Reset()
{
  wheel_drop_detected_ = false;
  // It's necessary to reset time (it moved backwards)
  last_time_ = world_->SimTime();
}

void GazeboRosWheelDrop::OnUpdate()
{
  const gazebo::common::Time current_time = world_->SimTime();
  const double time_elapsed = (current_time - last_time_).Double();
  if (!rate_enforcer_.shouldUpdate(time_elapsed)) {
    return;
  }

  // Check and update wheeldrop status
  const double displacement{joint_->Position()};
  if ((wheel_drop_detected_ == false) && (displacement >= upper_limit_)) {
    wheel_drop_detected_ = true;
  } else if ((wheel_drop_detected_ == true) && (displacement < lower_limit_)) {
    wheel_drop_detected_ = false;
  }

  // Publish wheeldrop only if it's in the detected state
  if (wheel_drop_detected_) {
    PublishWheeldrop(displacement, current_time);
  }

  last_time_ = current_time;
}

void GazeboRosWheelDrop::PublishWheeldrop(
  const double & displacement, const gazebo::common::Time & current_time)
{
  irobot_create_msgs::msg::HazardDetection msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  msg.header.frame_id = frame_id_;
  msg.type = msg.WHEEL_DROP;
  pub_->publish(msg);
  RCLCPP_DEBUG_EXPRESSION(
    ros_node_->get_logger(), !wheel_drop_detected_, "Wheel drop %s OFF: %.3f",
    name_.c_str(), displacement);
  RCLCPP_DEBUG_EXPRESSION(
    ros_node_->get_logger(), wheel_drop_detected_, "Wheel drop %s ON: %.3f",
    name_.c_str(), displacement);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelDrop)
}  // namespace irobot_create_gazebo_plugins
