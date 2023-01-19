// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Justin Kearns (jkearns@irobot.com)

#include <irobot_create_nodes/motion_control/docking_behavior.hpp>

#include <memory>

namespace irobot_create_nodes
{

using namespace std::placeholders;

DockingBehavior::DockingBehavior(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  std::shared_ptr<BehaviorsScheduler> behavior_scheduler)
: clock_(node_clock_interface->get_clock()),
  logger_(node_logging_interface->get_logger()),
  max_action_runtime_(rclcpp::Duration(std::chrono::seconds(30)))
{
  behavior_scheduler_ = behavior_scheduler;
  last_feedback_time_ = clock_->now();

  dock_status_sub_ = rclcpp::create_subscription<irobot_create_msgs::msg::DockStatus>(
    node_topics_interface,
    "dock_status",
    rclcpp::SensorDataQoS(),
    std::bind(&DockingBehavior::dock_status_callback, this, _1));

  robot_pose_sub_ = rclcpp::create_subscription<nav_msgs::msg::Odometry>(
    node_topics_interface,
    "sim_ground_truth_pose",
    rclcpp::SensorDataQoS(),
    std::bind(&DockingBehavior::robot_pose_callback, this, _1));

  dock_pose_sub_ = rclcpp::create_subscription<nav_msgs::msg::Odometry>(
    node_topics_interface,
    "sim_ground_truth_dock_pose",
    rclcpp::SensorDataQoS(),
    std::bind(&DockingBehavior::dock_pose_callback, this, _1));

  docking_action_server_ = rclcpp_action::create_server<irobot_create_msgs::action::Dock>(
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
  // Give poses default value, will be over-written by subscriptions
  last_robot_pose_.setIdentity();
  last_dock_pose_.setIdentity();
  tf2::Quaternion dock_rotation;
  dock_rotation.setRPY(0, 0, M_PI);
  last_dock_pose_.setRotation(dock_rotation);
  // Set number from observation, but will repopulate on undock with calibrated value
  last_docked_distance_offset_ = 0.157;
  action_start_time_ = clock_->now();
}

bool DockingBehavior::docking_behavior_is_done()
{
  return !running_dock_action_;
}

rclcpp_action::GoalResponse DockingBehavior::handle_dock_servo_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const irobot_create_msgs::action::Dock::Goal>/*goal*/)
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
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Dock>>/*goal_handle*/)
{
  RCLCPP_INFO(logger_, "Received request to cancel dock servo goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DockingBehavior::handle_dock_servo_accepted(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Dock>> goal_handle)
{
  // Create new Docking state machine
  running_dock_action_ = true;
  action_start_time_ = clock_->now();

  // Generate point offset from dock facing dock then point at dock
  SimpleGoalController::CmdPath dock_path;
  tf2::Transform robot_pose(tf2::Transform::getIdentity());
  {
    const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
    robot_pose = last_robot_pose_;
  }
  tf2::Transform dock_pose(tf2::Transform::getIdentity());
  {
    const std::lock_guard<std::mutex> lock(dock_pose_mutex_);
    dock_pose = last_dock_pose_;
  }
  // If robot is farther than 0.5 from dock, put offset point 0.5 in front of dock,
  // otherwise put in line with robot's current distance away from the dock
  const tf2::Vector3 & robot_position = robot_pose.getOrigin();
  const tf2::Vector3 & dock_position = dock_pose.getOrigin();
  double dist_offset = std::hypot(
    dock_position.getX() - robot_position.getX(),
    dock_position.getY() - robot_position.getY());
  const double max_goal_offset = MAX_DOCK_INTERMEDIATE_GOAL_OFFSET + last_docked_distance_offset_;
  if (dist_offset > max_goal_offset) {
    dist_offset = max_goal_offset;
  }
  tf2::Transform dock_offset(tf2::Transform::getIdentity());
  tf2::Quaternion dock_rotation;
  dock_rotation.setRPY(0, 0, M_PI);
  dock_offset.setOrigin(tf2::Vector3(dist_offset, 0, 0));
  dock_offset.setRotation(dock_rotation);
  dock_path.emplace_back(dock_pose * dock_offset, 0.02, false);
  dock_offset.setIdentity();
  dock_offset.setOrigin(tf2::Vector3(last_docked_distance_offset_, 0, 0));
  tf2::Transform face_dock(tf2::Transform::getIdentity());
  face_dock.setRotation(dock_rotation);
  dock_path.emplace_back(dock_pose * dock_offset * face_dock, 0.005, false);
  goal_controller_.initialize_goal(dock_path, M_PI / 4.0, 0.15);
  // Setup behavior to override other commanded motion
  BehaviorsScheduler::BehaviorsData data;
  data.run_func = std::bind(&DockingBehavior::execute_dock_servo, this, goal_handle, _1);
  data.is_done_func = std::bind(&DockingBehavior::docking_behavior_is_done, this);
  data.stop_on_new_behavior = true;
  data.apply_backup_limits = false;

  const bool ret = behavior_scheduler_->set_behavior(data);
  if (!ret) {
    // for some reason we couldn't set the new behavior, treat this as a goal being cancelled
    RCLCPP_WARN(logger_, "Dock Servo behavior failed to start");
    auto result = std::make_shared<irobot_create_msgs::action::Dock::Result>();
    result->is_docked = is_docked_;
    goal_handle->abort(result);
    running_dock_action_ = false;
  }
  last_feedback_time_ = clock_->now();
}

BehaviorsScheduler::optional_output_t DockingBehavior::execute_dock_servo(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Dock>> goal_handle,
  const RobotState & /*current_state*/)
{
  BehaviorsScheduler::optional_output_t servo_cmd;
  // Handle if goal is cancelling
  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<irobot_create_msgs::action::Dock::Result>();
    result->is_docked = is_docked_;
    goal_handle->canceled(result);
    goal_controller_.reset();
    running_dock_action_ = false;
    return servo_cmd;
  }

  bool exceeded_runtime = false;
  if (clock_->now() - action_start_time_ > max_action_runtime_) {
    RCLCPP_INFO(logger_, "Dock Servo Goal Exceeded Runtime");
    exceeded_runtime = true;
  }
  // Get next command
  tf2::Transform robot_pose(tf2::Transform::getIdentity());
  {
    const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
    robot_pose = last_robot_pose_;
  }
  servo_cmd = goal_controller_.get_velocity_for_position(robot_pose);
  if (!servo_cmd || exceeded_runtime) {
    auto result = std::make_shared<irobot_create_msgs::action::Dock::Result>();
    if (is_docked_) {
      result->is_docked = true;
      RCLCPP_INFO(logger_, "Dock Servo Goal Succeeded");
      goal_handle->succeed(result);
    } else {
      result->is_docked = false;
      RCLCPP_INFO(logger_, "Dock Servo Goal Aborted");
      goal_handle->abort(result);
    }
    goal_controller_.reset();
    running_dock_action_ = false;
    return servo_cmd;
  }

  rclcpp::Time current_time = clock_->now();
  auto time_since_feedback = current_time - last_feedback_time_;
  if (time_since_feedback > report_feedback_interval_) {
    // Publish feedback
    auto feedback = std::make_shared<irobot_create_msgs::action::Dock::Feedback>();
    feedback->sees_dock = sees_dock_;
    goal_handle->publish_feedback(feedback);
    last_feedback_time_ = current_time;
  }

  return servo_cmd;
}

rclcpp_action::GoalResponse DockingBehavior::handle_undock_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const irobot_create_msgs::action::Undock::Goal>/*goal*/)
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
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Undock>>/*goal_handle*/)
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
  action_start_time_ = clock_->now();

  SimpleGoalController::CmdPath undock_path;
  // Generate path with point offset from robot pose,
  // have robot drive backwards to offset then turn 180
  tf2::Transform robot_pose(tf2::Transform::getIdentity());
  {
    const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
    robot_pose = last_robot_pose_;
  }
  if (!calibrated_offset_) {
    tf2::Transform dock_pose(tf2::Transform::getIdentity());
    {
      const std::lock_guard<std::mutex> lock(dock_pose_mutex_);
      dock_pose = last_dock_pose_;
    }
    calibrate_docked_distance_offset(robot_pose, dock_pose);
  }
  tf2::Transform dock_offset(tf2::Transform::getIdentity());
  dock_offset.setOrigin(tf2::Vector3(-UNDOCK_GOAL_OFFSET, 0, 0));
  tf2::Transform undock_offset = robot_pose * dock_offset;
  undock_path.emplace_back(undock_offset, 0.05, true);
  tf2::Transform face_away_dock(tf2::Transform::getIdentity());
  tf2::Quaternion undock_rotation;
  undock_rotation.setRPY(0, 0, M_PI);
  face_away_dock.setRotation(undock_rotation);
  tf2::Transform undocked_goal = undock_offset * face_away_dock;
  undock_path.emplace_back(undocked_goal, 0.05, false);
  goal_controller_.initialize_goal(undock_path, M_PI / 4.0, 0.15);

  BehaviorsScheduler::BehaviorsData data;
  data.run_func = std::bind(&DockingBehavior::execute_undock, this, goal_handle, _1);
  data.is_done_func = std::bind(&DockingBehavior::docking_behavior_is_done, this);
  data.stop_on_new_behavior = true;
  data.apply_backup_limits = false;

  const bool ret = behavior_scheduler_->set_behavior(data);
  if (!ret) {
    // for some reason we couldn't set the new behavior, treat this as a goal being cancelled
    RCLCPP_WARN(logger_, "Undock behavior failed to start");
    auto result = std::make_shared<irobot_create_msgs::action::Undock::Result>();
    result->is_docked = is_docked_;
    goal_handle->abort(result);
    goal_controller_.reset();
    running_dock_action_ = false;
  }
  last_feedback_time_ = clock_->now();
}

BehaviorsScheduler::optional_output_t DockingBehavior::execute_undock(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::Undock>> goal_handle,
  const RobotState & /*current_state*/)
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
  tf2::Transform robot_pose(tf2::Transform::getIdentity());
  {
    const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
    robot_pose = last_robot_pose_;
  }
  servo_cmd = goal_controller_.get_velocity_for_position(robot_pose);

  bool exceeded_runtime = false;
  if (clock_->now() - action_start_time_ > max_action_runtime_) {
    RCLCPP_INFO(logger_, "Undock Goal Exceeded Runtime");
    exceeded_runtime = true;
  }

  if (!servo_cmd || exceeded_runtime) {
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

void DockingBehavior::dock_status_callback(irobot_create_msgs::msg::DockStatus::ConstSharedPtr msg)
{
  is_docked_ = msg->is_docked;
  sees_dock_ = msg->dock_visible;
}

void DockingBehavior::robot_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
  tf2::convert(msg->pose.pose, last_robot_pose_);
}

void DockingBehavior::dock_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(dock_pose_mutex_);
  tf2::convert(msg->pose.pose, last_dock_pose_);
}

void DockingBehavior::calibrate_docked_distance_offset(
  const tf2::Transform & docked_robot_pose,
  const tf2::Transform & dock_pose)
{
  tf2::Vector3 pos_diff = docked_robot_pose.getOrigin() - dock_pose.getOrigin();
  last_docked_distance_offset_ = std::hypot(pos_diff.getX(), pos_diff.getY());
  calibrated_offset_ = true;
  RCLCPP_DEBUG(logger_, "Setting robot dock offset to %f", last_docked_distance_offset_);
}

}  // namespace irobot_create_nodes
