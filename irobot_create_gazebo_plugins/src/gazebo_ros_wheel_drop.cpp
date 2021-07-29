// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <irobot_create_gazebo_plugins/gazebo_ros_wheel_drop.hpp>

namespace irobot_create_gazebo_plugins
{
void GazeboRosWheelDrop::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  double update_rate{62.0};
  double detection_threshold{0.7};
  std::string joint_name{""};
  utils::initialize(update_rate, sdf, "updateRate", 62.0);
  utils::initialize(detection_threshold, sdf, "detectionThreshold", 0.7);
  utils::initialize(joint_name, sdf, "jointName", "");
  utils::initialize(frame_id_, sdf, "frameId", "");

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
    "~/out", qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS()));

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
  if (!rate_enforcer_.shouldUpdate(time_elapsed)) {return;}

  const double displacement{joint_->Position()};
  if ((wheel_drop_detected_ == false) && (displacement >= upper_limit_)) {
    PublishState(true, displacement, current_time);
  } else if ((wheel_drop_detected_ == true) && (displacement < lower_limit_)) {
    PublishState(false, displacement, current_time);
  }
  last_time_ = current_time;
}

void GazeboRosWheelDrop::PublishState(
  const bool & state, const double & displacement, const gazebo::common::Time & current_time)
{
  wheel_drop_detected_ = state;
  irobot_create_msgs::msg::HazardDetection msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  msg.header.frame_id = frame_id_;
  msg.type = msg.WHEEL_DROP;
  pub_->publish(msg);
  RCLCPP_INFO_EXPRESSION(
    ros_node_->get_logger(), !state, "Wheel drop %s OFF: %.3f", name_.c_str(), displacement);
  RCLCPP_INFO_EXPRESSION(
    ros_node_->get_logger(), state, "Wheel drop %s ON: %.3f", name_.c_str(), displacement);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelDrop)
}  // namespace irobot_create_gazebo_plugins
