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

#include "irobot_create_toolbox/docking_behavior.hpp"

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
: m_logger(node_logging_interface->get_logger())
{
  m_behavior_scheduler = behavior_scheduler;

  m_dock_sub = rclcpp::create_subscription<irobot_create_msgs::msg::Dock>(
    node_topics_interface,
    "dock",
    rclcpp::SensorDataQoS(),
    std::bind(&DockingBehavior::dock_callback, this, _1));

  m_docking_action_server = rclcpp_action::create_server<irobot_create_msgs::action::DockServo>(
    node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    "dock",
    std::bind(&DockingBehavior::handle_dock_servo_goal, this, _1, _2),
    std::bind(&DockingBehavior::handle_dock_servo_cancel, this, _1),
    std::bind(&DockingBehavior::handle_dock_servo_accepted, this, _1));

  m_undocking_action_server = rclcpp_action::create_server<irobot_create_msgs::action::Undock>(
    node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    "undock",
    std::bind(&DockingBehavior::handle_undock_goal, this, _1, _2),
    std::bind(&DockingBehavior::handle_undock_cancel, this, _1),
    std::bind(&DockingBehavior::handle_undock_accepted, this, _1));
}

bool DockingBehavior::docking_behavior_is_done()
{
  return !m_running_dock_action;
}

rclcpp_action::GoalResponse DockingBehavior::handle_dock_servo_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const irobot_create_msgs::action::DockServo::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(m_logger, "Received new dock servo goal");

  if (!docking_behavior_is_done()) {
    RCLCPP_WARN(m_logger, "A docking behavior is already running, reject");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (m_is_docked) {
    RCLCPP_WARN(m_logger, "Robot already docked, reject");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockingBehavior::handle_dock_servo_cancel(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DockServo>> goal_handle)
{
  RCLCPP_INFO(m_logger, "Received request to cancel dock servo goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DockingBehavior::handle_dock_servo_accepted(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DockServo>> goal_handle)
{
  // Create new Docking state machine
  m_running_dock_action = true;

  BehaviorsScheduler::BehaviorsData data;
  data.run_func = std::bind(&DockingBehavior::execute_dock_servo, this, goal_handle);
  data.is_done_func = std::bind(&DockingBehavior::docking_behavior_is_done, this);

  bool ret = m_behavior_scheduler->set_behavior(data);
  if (!ret) {
    // for some reason we couldn't set the new behavior, treat this as a goal being cancelled
    RCLCPP_WARN(m_logger, "Dock Servo behavior failed to start");
    auto result = std::make_shared<irobot_create_msgs::action::DockServo::Result>();
    result->is_docked = m_is_docked;
    goal_handle->canceled(result);
    m_running_dock_action = false;
  }
}

BehaviorsScheduler::optional_output_t DockingBehavior::execute_dock_servo(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DockServo>> goal_handle)
{
  // Handle if goal is cancelling
  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<irobot_create_msgs::action::DockServo::Result>();
    result->is_docked = m_is_docked;
    goal_handle->canceled(result);
    m_running_dock_action = false;
    return BehaviorsScheduler::optional_output_t();
  }

  // XXX Put drive to dock control code here

  // Handle if protocol completed
  if (m_is_docked) {
    auto result = std::make_shared<irobot_create_msgs::action::DockServo::Result>();
    result->is_docked = true;
    RCLCPP_INFO(m_logger, "Dock Servo Goal Succeeded");
    goal_handle->succeed(result);
    m_running_dock_action = false;
    return BehaviorsScheduler::optional_output_t();
  }

  // Publish feedback
  auto feedback = std::make_shared<irobot_create_msgs::action::DockServo::Feedback>();
  feedback->sees_dock = m_sees_dock;

  goal_handle->publish_feedback(feedback);

  return BehaviorsScheduler::optional_output_t();
}

rclcpp_action::GoalResponse DockingBehavior::handle_undock_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const irobot_create_msgs::action::Undock::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(m_logger, "Received new undock goal");

  if (!docking_behavior_is_done()) {
    RCLCPP_WARN(m_logger, "A docking behavior is already running, reject");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!m_is_docked) {
    RCLCPP_WARN(m_logger, "Robot already undocked, reject");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockingBehavior::handle_undock_cancel(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Undock>> goal_handle)
{
  RCLCPP_INFO(m_logger, "Received request to cancel undock goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DockingBehavior::handle_undock_accepted(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Undock>> goal_handle)
{
  // Create new Docking Action
  m_running_dock_action = true;

  BehaviorsScheduler::BehaviorsData data;
  data.run_func = std::bind(&DockingBehavior::execute_undock, this, goal_handle);
  data.is_done_func = std::bind(&DockingBehavior::docking_behavior_is_done, this);

  bool ret = m_behavior_scheduler->set_behavior(data);
  if (!ret) {
    // for some reason we couldn't set the new behavior, treat this as a goal being cancelled
    RCLCPP_WARN(m_logger, "Undock behavior failed to start");
    auto result = std::make_shared<irobot_create_msgs::action::Undock::Result>();
    result->is_docked = m_is_docked;
    goal_handle->canceled(result);
    m_running_dock_action = false;
  }
}

BehaviorsScheduler::optional_output_t DockingBehavior::execute_undock(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Undock>> goal_handle)
{
  // Handle if goal is cancelling
  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<irobot_create_msgs::action::Undock::Result>();
    result->is_docked = m_is_docked;
    goal_handle->canceled(result);
    m_running_dock_action = false;
    return BehaviorsScheduler::optional_output_t();
  }

  // XXX put undocking code here
  if (!m_is_docked /* && finished undocking action*/) {
    auto result = std::make_shared<irobot_create_msgs::action::Undock::Result>();
    result->is_docked = false;
    RCLCPP_INFO(m_logger, "Undock Goal Succeeded");
    goal_handle->succeed(result);
    m_running_dock_action = false;
    return BehaviorsScheduler::optional_output_t();
  }

  return BehaviorsScheduler::optional_output_t();
}

void DockingBehavior::dock_callback(irobot_create_msgs::msg::Dock::ConstSharedPtr msg)
{
  m_is_docked = msg->is_docked;
  m_sees_dock = msg->dock_visible;
}

}  // namespace irobot_create_toolbox
