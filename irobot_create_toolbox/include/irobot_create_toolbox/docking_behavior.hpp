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

#ifndef IROBOT_CREATE_TOOLBOX__DOCKING_BEHAVIOR_HPP_
#define IROBOT_CREATE_TOOLBOX__DOCKING_BEHAVIOR_HPP_

#include <stdint.h>


#include <irobot_create_msgs/action/dock_servo.hpp>
#include <irobot_create_msgs/action/undock.hpp>
#include <irobot_create_msgs/msg/dock.hpp>
#include <irobot_create_toolbox/behaviors_scheduler.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <memory>

namespace irobot_create_toolbox
{

/**
 * @brief This class allows to create and manage Docking and Undocking action
 * servers.
 */
class DockingBehavior
{
public:
  DockingBehavior(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    std::shared_ptr<BehaviorsScheduler> behavior_scheduler);
  ~DockingBehavior() = default;

private:
  bool docking_behavior_is_done();

  void dock_callback(irobot_create_msgs::msg::Dock::ConstSharedPtr msg);

  rclcpp_action::GoalResponse handle_dock_servo_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const irobot_create_msgs::action::DockServo::Goal> goal);

  rclcpp_action::CancelResponse handle_dock_servo_cancel(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DockServo>> goal_handle);

  void handle_dock_servo_accepted(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DockServo>> goal_handle);

  BehaviorsScheduler::optional_output_t execute_dock_servo(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DockServo>> goal_handle);

  rclcpp_action::GoalResponse handle_undock_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const irobot_create_msgs::action::Undock::Goal> goal);

  rclcpp_action::CancelResponse handle_undock_cancel(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Undock>> goal_handle);

  void handle_undock_accepted(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Undock>> goal_handle);

  BehaviorsScheduler::optional_output_t execute_undock(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Undock>> goal_handle);

  rclcpp_action::Server<irobot_create_msgs::action::DockServo>::SharedPtr m_docking_action_server;
  rclcpp_action::Server<irobot_create_msgs::action::Undock>::SharedPtr m_undocking_action_server;

  rclcpp::Subscription<irobot_create_msgs::msg::Dock>::SharedPtr m_dock_sub;

  rclcpp::Logger m_logger;
  std::shared_ptr<BehaviorsScheduler> m_behavior_scheduler;
  std::atomic<bool> m_is_docked {false};
  std::atomic<bool> m_sees_dock {false};
  std::atomic<bool> m_running_dock_action {false};
};

}  // namespace irobot_create_toolbox
#endif  // IROBOT_CREATE_TOOLBOX__DOCKING_BEHAVIOR_HPP_
