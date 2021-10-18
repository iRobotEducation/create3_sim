// Copyright 2021 iRobot, Inc.
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

#include <irobot_create_toolbox/motion_control/docking_behavior.hpp>

#include <memory>

namespace irobot_create_toolbox
{

using namespace std::placeholders;

DockingBehavior::DockingBehavior(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  std::shared_ptr<BehaviorsScheduler> behavior_scheduler)
: logger_(node_logging_interface->get_logger())
{
  behavior_scheduler_ = behavior_scheduler;

  dock_sub_ = rclcpp::create_subscription<irobot_create_msgs::msg::Dock>(
    node_topics_interface,
    "dock",
    rclcpp::SensorDataQoS(),
    std::bind(&DockingBehavior::dock_callback, this, _1));

  odom_sub_ = rclcpp::create_subscription<nav_msgs::msg::Odometry>(
    node_topics_interface,
    "odom",
    rclcpp::SensorDataQoS(),
    std::bind(&DockingBehavior::odom_callback, this, _1));

  docking_action_server_ = rclcpp_action::create_server<irobot_create_msgs::action::DockServo>(
    node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    "dock",
    std::bind(&DockingBehavior::handle_dock_servo_goal, this, _1, _2),
    std::bind(&DockingBehavior::handle_dock_servo_cancel, this, _1),
    std::bind(&DockingBehavior::handle_dock_servo_accepted, this, _1));

  undocking_action_server_ = rclcpp_action::create_server<irobot_create_msgs::action::Undock>(
    node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    "undock",
    std::bind(&DockingBehavior::handle_undock_goal, this, _1, _2),
    std::bind(&DockingBehavior::handle_undock_cancel, this, _1),
    std::bind(&DockingBehavior::handle_undock_accepted, this, _1));
  // Need to get real dock pose into here, math expects orientation to face towards docked robot
  dock_pose_.setIdentity();
  tf2::Quaternion dock_rotation;
  dock_rotation.setRPY(0, 0, M_PI);
  dock_pose_.setRotation(dock_rotation);
}

bool DockingBehavior::docking_behavior_is_done()
{
  return !running_dock_action_;
}

rclcpp_action::GoalResponse DockingBehavior::handle_dock_servo_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const irobot_create_msgs::action::DockServo::Goal> /*goal*/)
{
  RCLCPP_INFO(logger_, "Received new dock servo goal");

  if (!docking_behavior_is_done()) {
    RCLCPP_WARN(logger_, "A docking behavior is already running, reject");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (is_docked_) {
    RCLCPP_WARN(logger_, "Robot already docked, reject");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!sees_dock_) {
    RCLCPP_WARN(logger_, "Robot doesn't see dock, reject");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockingBehavior::handle_dock_servo_cancel(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DockServo>> /*goal_handle*/)
{
  RCLCPP_INFO(logger_, "Received request to cancel dock servo goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DockingBehavior::handle_dock_servo_accepted(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DockServo>> goal_handle)
{
  // Create new Docking state machine
  running_dock_action_ = true;

  // Generate point offset from dock facing dock then point at dock
  SimpleGoalController::CmdPath dock_path;
  tf2::Transform odom_pose;
  {
    const std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_pose = last_odom_pose_;
  }
  // If robot is farther than 0.5 from dock, put offset point 0.5 in front of dock,
  // otherwise put in line with robot's current distance away from the dock
  const tf2::Vector3 & odom_position = odom_pose.getOrigin();
  const tf2::Vector3 & dock_position = dock_pose_.getOrigin();
  double dist_offset = std::hypot(
    dock_position.getX() - odom_position.getX(),
    dock_position.getY() - odom_position.getY());
  if (dist_offset > MAX_DOCK_INTERMEDIATE_GOAL_OFFSET) {
    dist_offset = MAX_DOCK_INTERMEDIATE_GOAL_OFFSET;
  }
  tf2::Transform dock_offset;
  tf2::Quaternion dock_rotation;
  dock_rotation.setRPY(0, 0, M_PI);
  dock_offset.setOrigin(tf2::Vector3(dist_offset, 0, 0));
  dock_offset.setRotation(dock_rotation);
  dock_path.emplace_back(dock_pose_ * dock_offset, 0.02, false);
  tf2::Transform face_dock;
  face_dock.setRotation(dock_rotation);
  dock_path.emplace_back(dock_pose_ * face_dock, 0.01, false);
  goal_controller_.initialize_goal(dock_path, M_PI / 4.0, 0.15);
  // Setup behavior to override other commanded motion
  BehaviorsScheduler::BehaviorsData data;
  data.run_func = std::bind(&DockingBehavior::execute_dock_servo, this, goal_handle);
  data.is_done_func = std::bind(&DockingBehavior::docking_behavior_is_done, this);

  const bool ret = behavior_scheduler_->set_behavior(data);
  if (!ret) {
    // for some reason we couldn't set the new behavior, treat this as a goal being cancelled
    RCLCPP_WARN(logger_, "Dock Servo behavior failed to start");
    auto result = std::make_shared<irobot_create_msgs::action::DockServo::Result>();
    result->is_docked = is_docked_;
    goal_handle->canceled(result);
    running_dock_action_ = false;
  }
}

BehaviorsScheduler::optional_output_t DockingBehavior::execute_dock_servo(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DockServo>> goal_handle)
{
  BehaviorsScheduler::optional_output_t servo_cmd;
  // Handle if goal is cancelling
  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<irobot_create_msgs::action::DockServo::Result>();
    result->is_docked = is_docked_;
    goal_handle->canceled(result);
    goal_controller_.reset();
    running_dock_action_ = false;
    return servo_cmd;
  }

  // Handle if reached dock
  if (is_docked_) {
    auto result = std::make_shared<irobot_create_msgs::action::DockServo::Result>();
    result->is_docked = true;
    RCLCPP_INFO(logger_, "Dock Servo Goal Succeeded");
    goal_handle->succeed(result);
    goal_controller_.reset();
    running_dock_action_ = false;
    return servo_cmd;
  } else {
    // Get next command
    tf2::Transform odom_pose;
    {
      const std::lock_guard<std::mutex> lock(odom_mutex_);
      odom_pose = last_odom_pose_;
    }
    servo_cmd = goal_controller_.get_velocity_for_position(odom_pose);
    if (!servo_cmd) {
      auto result = std::make_shared<irobot_create_msgs::action::DockServo::Result>();
      result->is_docked = false;
      RCLCPP_INFO(logger_, "Dock Servo Goal Aborted");
      goal_handle->abort(result);
      goal_controller_.reset();
      running_dock_action_ = false;
      return servo_cmd;
    }
  }

  // Publish feedback
  auto feedback = std::make_shared<irobot_create_msgs::action::DockServo::Feedback>();
  feedback->sees_dock = sees_dock_;

  goal_handle->publish_feedback(feedback);

  return servo_cmd;
}

rclcpp_action::GoalResponse DockingBehavior::handle_undock_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const irobot_create_msgs::action::Undock::Goal> /*goal*/)
{
  RCLCPP_INFO(logger_, "Received new undock goal");

  if (!docking_behavior_is_done()) {
    RCLCPP_WARN(logger_, "A docking behavior is already running, reject");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!is_docked_) {
    RCLCPP_WARN(logger_, "Robot already undocked, reject");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockingBehavior::handle_undock_cancel(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Undock>> /*goal_handle*/)
{
  RCLCPP_INFO(logger_, "Received request to cancel undock goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DockingBehavior::handle_undock_accepted(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Undock>> goal_handle)
{
  // Create new Docking Action
  running_dock_action_ = true;


  SimpleGoalController::CmdPath undock_path;
  // Generate path with point offset from odom, have robot drive backwards to offset then turn 180
  tf2::Transform odom_pose;
  {
    const std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_pose = last_odom_pose_;
  }
  tf2::Transform dock_offset;
  dock_offset.setOrigin(tf2::Vector3(-UNDOCK_GOAL_OFFSET, 0, 0));
  undock_path.emplace_back(odom_pose * dock_offset, 0.05, true);
  tf2::Transform face_away_dock;
  tf2::Quaternion undock_rotation;
  undock_rotation.setRPY(0, 0, M_PI);
  face_away_dock.setRotation(undock_rotation);
  undock_path.emplace_back(undock_path.back().pose * face_away_dock, 0.05, false);
  goal_controller_.initialize_goal(undock_path, M_PI / 4.0, 0.15);

  BehaviorsScheduler::BehaviorsData data;
  data.run_func = std::bind(&DockingBehavior::execute_undock, this, goal_handle);
  data.is_done_func = std::bind(&DockingBehavior::docking_behavior_is_done, this);

  const bool ret = behavior_scheduler_->set_behavior(data);
  if (!ret) {
    // for some reason we couldn't set the new behavior, treat this as a goal being cancelled
    RCLCPP_WARN(logger_, "Undock behavior failed to start");
    auto result = std::make_shared<irobot_create_msgs::action::Undock::Result>();
    result->is_docked = is_docked_;
    goal_handle->canceled(result);
    goal_controller_.reset();
    running_dock_action_ = false;
  }
}

BehaviorsScheduler::optional_output_t DockingBehavior::execute_undock(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Undock>> goal_handle)
{
  BehaviorsScheduler::optional_output_t servo_cmd;
  // Handle if goal is cancelling
  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<irobot_create_msgs::action::Undock::Result>();
    result->is_docked = is_docked_;
    goal_handle->canceled(result);
    goal_controller_.reset();
    running_dock_action_ = false;
    return BehaviorsScheduler::optional_output_t();
  }
  // Get next command
  tf2::Transform odom_pose;
  {
    const std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_pose = last_odom_pose_;
  }
  servo_cmd = goal_controller_.get_velocity_for_position(odom_pose);

  if (!servo_cmd) {
    auto result = std::make_shared<irobot_create_msgs::action::Undock::Result>();
    result->is_docked = is_docked_;
    if (!is_docked_) {
      RCLCPP_INFO(logger_, "Undock Goal Succeeded");
      goal_handle->succeed(result);
    } else {
      RCLCPP_INFO(logger_, "Undock Goal Aborted");
      goal_handle->abort(result);
    }
    goal_controller_.reset();
    running_dock_action_ = false;
    return BehaviorsScheduler::optional_output_t();
  }

  return servo_cmd;
}

void DockingBehavior::dock_callback(irobot_create_msgs::msg::Dock::ConstSharedPtr msg)
{
  is_docked_ = msg->is_docked;
  sees_dock_ = msg->dock_visible;
}

void DockingBehavior::odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(odom_mutex_);
  tf2::convert(msg->pose.pose, last_odom_pose_);
}

}  // namespace irobot_create_toolbox
