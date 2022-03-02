// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "irobot_create_gazebo_plugins/gazebo_ros_optical_mouse.hpp"

#include <string>

namespace irobot_create_gazebo_plugins
{
GazeboRosOpticalMouse::GazeboRosOpticalMouse()
: ModelPlugin() {}

GazeboRosOpticalMouse::~GazeboRosOpticalMouse() {}

void GazeboRosOpticalMouse::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  world_ = model->GetWorld();
  GZ_ASSERT(world_, "World pointer is invalid!");

  std::string link_name{""};
  double update_rate{62.0};

  // Get plugin parameters
  utils::initialize(link_name, sdf, "link_name", "");
  utils::initialize(update_rate, sdf, "update_rate", 62.0);

  // Get link
  link_ = model->GetLink(link_name);
  GZ_ASSERT(link_, "Couldn't find optical mouse link.");

  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  ros_node_ = gazebo_ros::Node::Get(sdf);
  // Initialize ROS publisher
  pub_ = ros_node_->create_publisher<irobot_create_msgs::msg::Mouse>(
    "~/out", rclcpp::SensorDataQoS().reliable());

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosOpticalMouse::OnUpdate, this, std::placeholders::_1));

  // Set message frame_id
  msg_.header.frame_id = link_->GetName();

  // Set frame_id and last_squal default values. These values are not updated by the simulator.
  msg_.frame_id = 0;
  msg_.last_squal = 90;

  // Rate enforcer
  update_rate_enforcer_.load(update_rate);

  // Initialize time and pose markers
  last_time_ = world_->SimTime();
  last_pose_ = link_->WorldPose();
  integrated_position_ = {0, 0, 0};

  RCLCPP_INFO(ros_node_->get_logger(), "Starting optical mouse plugin");
}

// Function is called when the world is reset
void GazeboRosOpticalMouse::Reset()
{
  // Is necessary to reset time (it moved backwards)
  last_time_ = world_->SimTime();
  // Update pose because it was reset
  last_pose_ = link_->WorldPose();
}

void GazeboRosOpticalMouse::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  const gazebo::common::Time current_time = info.simTime;

  if (current_time < last_time_) {
    RCLCPP_WARN(ros_node_->get_logger(), "Negative update time difference detected.");
    last_time_ = current_time;
  }

  const double time_elapsed = (current_time - last_time_).Double();

  // Check if on this iteration corresponds to send the message
  if (!update_rate_enforcer_.shouldUpdate(time_elapsed)) {
    return;
  }

  // Get pose
  const ignition::math::Pose3d current_pose = link_->WorldPose();
  // Pose difference with respect to the last mouse link pose. The result is a Pose from
  // the last pose to the current pose.
  const ignition::math::Vector3d position_displacement = (current_pose - last_pose_).Pos();

  // Configure an empty message with the timestamp
  msg_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);

  integrated_position_ += position_displacement;

  // Calculate displacement for this iteration
  msg_.integrated_x = integrated_position_.X();
  msg_.integrated_y = integrated_position_.Y();
  // Publish message
  pub_->publish(msg_);

  // Update time
  last_time_ = current_time;

  // The pose is updated
  if (msg_.integrated_x != 0.0 || msg_.integrated_y != 0.0) {
    last_pose_ = current_pose;
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosOpticalMouse)
}  // namespace irobot_create_gazebo_plugins
