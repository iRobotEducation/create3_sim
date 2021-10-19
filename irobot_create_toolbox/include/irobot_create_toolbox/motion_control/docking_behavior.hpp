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

#ifndef IROBOT_CREATE_TOOLBOX__MOTION_CONTROL__DOCKING_BEHAVIOR_HPP_
#define IROBOT_CREATE_TOOLBOX__MOTION_CONTROL__DOCKING_BEHAVIOR_HPP_

#include <stdint.h>

#include <irobot_create_msgs/action/dock_servo.hpp>
#include <irobot_create_msgs/action/undock.hpp>
#include <irobot_create_msgs/msg/dock.hpp>
#include <irobot_create_toolbox/motion_control/behaviors_scheduler.hpp>
#include <irobot_create_toolbox/motion_control/simple_goal_controller.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

  void calibrate_docked_distance_offset(const tf2::Transform& docked_robot_pose,
          const tf2::Transform& dock_pose);

  void dock_status_callback(irobot_create_msgs::msg::Dock::ConstSharedPtr msg);

  void robot_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void dock_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

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

  rclcpp_action::Server<irobot_create_msgs::action::DockServo>::SharedPtr docking_action_server_;
  rclcpp_action::Server<irobot_create_msgs::action::Undock>::SharedPtr undocking_action_server_;

  rclcpp::Subscription<irobot_create_msgs::msg::Dock>::SharedPtr dock_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dock_pose_sub_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  std::shared_ptr<BehaviorsScheduler> behavior_scheduler_;
  std::atomic<bool> is_docked_ {false};
  std::atomic<bool> sees_dock_ {false};
  std::atomic<bool> running_dock_action_ {false};
  SimpleGoalController goal_controller_;
  std::mutex robot_pose_mutex_;
  tf2::Transform last_robot_pose_;
  std::mutex dock_pose_mutex_;
  tf2::Transform last_dock_pose_;
  rclcpp::Time action_start_time_;
  const rclcpp::Duration max_action_runtime_;
  double last_docked_distance_offset_ {0.0};
  bool calibrated_offset_ {false};
  const double MAX_DOCK_INTERMEDIATE_GOAL_OFFSET {0.5};
  const double UNDOCK_GOAL_OFFSET {0.4};
};

}  // namespace irobot_create_toolbox
#endif  // IROBOT_CREATE_TOOLBOX__MOTION_CONTROL__DOCKING_BEHAVIOR_HPP_
