// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Justin Kearns (jkearns@irobot.com)

#include <irobot_create_nodes/motion_control/wall_follow_behavior.hpp>

#include <memory>
#include <string>
#include <vector>

namespace irobot_create_nodes
{

using namespace std::placeholders;

WallFollowBehavior::WallFollowBehavior(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  std::shared_ptr<BehaviorsScheduler> behavior_scheduler)
:   logger_{node_logging_interface->get_logger()},
  clock_{node_clock_interface->get_clock()},
  wf_end_duration_{std::chrono::milliseconds(0)}
{
  wf_running_ = false;
  behavior_scheduler_ = behavior_scheduler;
  last_feedback_time_ = clock_->now();

  wf_state_mgr_ = std::make_shared<WallFollowStateManager>(logger_.get_child("state_manager"));

  wall_follow_action_server_ = rclcpp_action::create_server<irobot_create_msgs::action::WallFollow>(
    node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    "wall_follow",
    std::bind(&WallFollowBehavior::handle_wall_follow_goal, this, _1, _2),
    std::bind(&WallFollowBehavior::handle_wall_follow_cancel, this, _1),
    std::bind(&WallFollowBehavior::handle_wall_follow_accepted, this, _1));

  ir_intensity_sub_ = rclcpp::create_subscription<irobot_create_msgs::msg::IrIntensityVector>(
    node_topics_interface,
    "ir_intensity",
    rclcpp::SensorDataQoS(),
    std::bind(&WallFollowBehavior::ir_intensity_callback, this, _1));
}

bool WallFollowBehavior::wall_follow_behavior_is_done()
{
  return !wf_running_;
}

rclcpp_action::GoalResponse WallFollowBehavior::handle_wall_follow_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const irobot_create_msgs::action::WallFollow::Goal>/*goal*/)
{
  RCLCPP_INFO(logger_, "Received new wall follow goal");

  if (wf_running_) {
    RCLCPP_WARN(logger_, "Wall follow is already running, reject");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WallFollowBehavior::handle_wall_follow_cancel(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::WallFollow>>/*goal_handle*/)
{
  RCLCPP_INFO(logger_, "Received request to cancel wall follow goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WallFollowBehavior::handle_wall_follow_accepted(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::WallFollow>> goal_handle)
{
  // Start wall follow

  if (goal_handle) {
    auto goal = goal_handle->get_goal();
    if (goal) {
      int8_t wf_side = 0;
      if (goal->follow_side == irobot_create_msgs::action::WallFollow::Goal::FOLLOW_RIGHT ||
        goal->follow_side == irobot_create_msgs::action::WallFollow::Goal::FOLLOW_LEFT)
      {
        wf_side = goal->follow_side;
      } else {
        // Return error if not a valid side
        RCLCPP_WARN(logger_, "Invalid follow_side value %d", goal->follow_side);
        auto result = std::make_shared<irobot_create_msgs::action::WallFollow::Result>();
        result->runtime = rclcpp::Duration::from_nanoseconds(0);
        goal_handle->abort(result);
        return;
      }
      wf_state_mgr_->initialize(wf_side);
      RCLCPP_INFO(
        logger_, "Starting wall follow goal with side %s",
        wf_side == irobot_create_msgs::action::WallFollow::Goal::FOLLOW_RIGHT ?
        "RIGHT" : "LEFT");
      wf_end_duration_ = rclcpp::Duration(goal->max_runtime);
      wf_start_time_ = clock_->now();
      wf_engaged_ = false;

      BehaviorsScheduler::BehaviorsData data;
      data.run_func = std::bind(&WallFollowBehavior::execute_wall_follow, this, goal_handle, _1);
      data.is_done_func = std::bind(&WallFollowBehavior::wall_follow_behavior_is_done, this);
      data.stop_on_new_behavior = false;
      data.apply_backup_limits = false;

      const bool ret = behavior_scheduler_->set_behavior(data);
      if (!ret) {
        // for some reason we couldn't set the new behavior, treat this as a goal being cancelled
        RCLCPP_WARN(logger_, "Wall follow behavior failed to start");
        auto result = std::make_shared<irobot_create_msgs::action::WallFollow::Result>();
        result->runtime = rclcpp::Duration::from_nanoseconds(0);
        goal_handle->abort(result);
        return;
      }
      wf_running_ = true;
    } else {
      RCLCPP_WARN(logger_, "Goal inside goal_handle is null");
      auto result = std::make_shared<irobot_create_msgs::action::WallFollow::Result>();
      result->runtime = rclcpp::Duration::from_nanoseconds(0);
      goal_handle->abort(result);
    }
  } else {
    RCLCPP_WARN(logger_, "goal_handle is null, don't execute");
    return;
  }
  last_feedback_time_ = clock_->now();
}

BehaviorsScheduler::optional_output_t WallFollowBehavior::execute_wall_follow(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::WallFollow>>
  goal_handle,
  const RobotState & current_state)
{
  // Handle if goal is cancelling
  rclcpp::Duration wf_runtime = clock_->now() - wf_start_time_;
  if (goal_handle->is_canceling()) {
    RCLCPP_INFO(logger_, "Wall follow canceled");
    wf_running_ = false;
    auto result = std::make_shared<irobot_create_msgs::action::WallFollow::Result>();
    result->runtime = wf_runtime;
    goal_handle->canceled(result);
    return BehaviorsScheduler::optional_output_t();
  }

  if (wf_runtime > wf_end_duration_) {
    RCLCPP_INFO(logger_, "Wall follow hit max_runtime, succeeded");
    wf_running_ = false;
    auto result = std::make_shared<irobot_create_msgs::action::WallFollow::Result>();
    result->runtime = wf_runtime;
    goal_handle->succeed(result);
    return BehaviorsScheduler::optional_output_t();
  }
  // Get next servo command based on bump and IR sensors
  BehaviorsScheduler::optional_output_t next_cmd = get_next_servo_cmd(current_state);
  rclcpp::Time current_time = clock_->now();
  auto time_since_feedback = current_time - last_feedback_time_;
  if (time_since_feedback > report_feedback_interval_) {
    // Publish feedback
    auto feedback = std::make_shared<irobot_create_msgs::action::WallFollow::Feedback>();
    feedback->engaged = wf_state_mgr_->is_engaged();
    goal_handle->publish_feedback(feedback);
    last_feedback_time_ = current_time;
  }

  return next_cmd;
}

void WallFollowBehavior::ir_intensity_callback(
  irobot_create_msgs::msg::IrIntensityVector::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(sensor_mutex_);
  last_ir_intensity_ = *msg;
}

BehaviorsScheduler::optional_output_t WallFollowBehavior::get_next_servo_cmd(
  const RobotState & current_state)
{
  WFVelocityCommand wf_vel_cmd;
  std::vector<std::string> active_hazard_frames;
  for (const auto & hazard : current_state.hazards.detections) {
    switch (hazard.type) {
      case irobot_create_msgs::msg::HazardDetection::BUMP:
      case irobot_create_msgs::msg::HazardDetection::CLIFF:
      case irobot_create_msgs::msg::HazardDetection::STALL:
      case irobot_create_msgs::msg::HazardDetection::WHEEL_DROP:
        {
          active_hazard_frames.emplace_back(hazard.header.frame_id);
          break;
        }
      case irobot_create_msgs::msg::HazardDetection::BACKUP_LIMIT:
      case irobot_create_msgs::msg::HazardDetection::OBJECT_PROXIMITY:
        {
          // This doesn't impact wall follow trajectory
          break;
        }
    }
  }
  {
    const std::lock_guard<std::mutex> lock(sensor_mutex_);
    wf_state_mgr_->get_next_velocity(
      current_state.pose, last_ir_intensity_,
      active_hazard_frames, wf_vel_cmd);
  }
  BehaviorsScheduler::optional_output_t servo_cmd = geometry_msgs::msg::Twist();
  servo_cmd->linear.x = wf_vel_cmd.translate;
  servo_cmd->angular.z = wf_vel_cmd.rotate;
  return servo_cmd;
}

}  // namespace irobot_create_nodes
