// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Justin Kearns (jkearns@irobot.com)

#ifndef IROBOT_CREATE_NODES__MOTION_CONTROL__DRIVE_GOAL_BEHAVIORS_HPP_
#define IROBOT_CREATE_NODES__MOTION_CONTROL__DRIVE_GOAL_BEHAVIORS_HPP_

#include <stdint.h>

#include <atomic>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_nodes/motion_control/behaviors_scheduler.hpp"
#include "irobot_create_msgs/action/drive_arc.hpp"
#include "irobot_create_msgs/action/drive_distance.hpp"
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace irobot_create_nodes
{

geometry_msgs::msg::Twist get_default_velocity_cmd();
geometry_msgs::msg::PoseStamped get_current_pose_stamped(
  const rclcpp::Time & current_time,
  const tf2::Transform & pose);
/**
 * @brief This class is a common base class that manages an action server for drive goals.
 */
template<typename T>
class DriveGoalBaseBehavior
{
public:
  DriveGoalBaseBehavior(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    std::shared_ptr<BehaviorsScheduler> behavior_scheduler,
    const std::string & server_name)
  : logger_(node_logging_interface->get_logger()),
    clock_(node_clock_interface->get_clock()),
    drive_goal_running_(false),
    server_name_(server_name)
  {
    behavior_scheduler_ = behavior_scheduler;
    last_feedback_time_ = clock_->now();

    if (!server_name.empty()) {
      drive_goal_action_server_ = rclcpp_action::create_server<T>(
        node_base_interface,
        node_clock_interface,
        node_logging_interface,
        node_waitables_interface,
        server_name,
        std::bind(
          &DriveGoalBaseBehavior<T>::handle_drive_goal_goal, this,
          std::placeholders::_1, std::placeholders::_2),
        std::bind(
          &DriveGoalBaseBehavior<T>::handle_drive_goal_cancel, this,
          std::placeholders::_1),
        std::bind(
          &DriveGoalBaseBehavior<T>::handle_drive_goal_accepted, this,
          std::placeholders::_1));
    }
  }
  virtual ~DriveGoalBaseBehavior() = default;

  virtual void initialize_goal(const typename T::Goal & goal) = 0;
  virtual bool iterate_on_goal(
    const tf2::Transform & current_pose,
    BehaviorsScheduler::optional_output_t & output) = 0;
  virtual std::shared_ptr<typename T::Feedback> get_feedback(
    const rclcpp::Duration & time_since_feedback) = 0;

protected:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

private:
  bool drive_goal_behavior_is_done()
  {
    return !drive_goal_running_;
  }

  rclcpp_action::GoalResponse handle_drive_goal_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const typename T::Goal>/*goal*/)
  {
    RCLCPP_INFO(logger_, "Received new %s goal", server_name_.c_str());

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_drive_goal_cancel(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<T>>/*goal_handle*/)
  {
    RCLCPP_INFO(logger_, "Received request to cancel %s goal", server_name_.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void abort_drive_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle)
  {
    auto result = std::make_shared<typename T::Result>();
    result->pose = current_pose_;
    goal_handle->abort(result);
  }

  void cleanup_drive_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle)
  {
    if (goal_handle) {
      RCLCPP_INFO(logger_, "Aborting %s goal: new drive goal received", server_name_.c_str());
      drive_goal_running_ = false;
      abort_drive_goal(goal_handle);
    } else {
      RCLCPP_WARN(logger_, "Failed to cleanup %s goal: goal_handle is null", server_name_.c_str());
    }
  }

  void handle_drive_goal_accepted(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<T>> goal_handle)
  {
    if (goal_handle) {
      auto goal = goal_handle->get_goal();
      if (goal) {
        initialize_goal(*goal);
        drive_goal_running_ = true;
      } else {
        drive_goal_running_ = false;
        RCLCPP_WARN(logger_, "Goal inside goal_handle is null");
        abort_drive_goal(goal_handle);
      }
    } else {
      RCLCPP_WARN(logger_, "goal_handle is null, don't execute");
      return;
    }

    BehaviorsScheduler::BehaviorsData data;
    data.run_func = std::bind(
      &DriveGoalBaseBehavior<T>::execute_drive_goal, this,
      goal_handle, std::placeholders::_1);
    data.is_done_func = std::bind(&DriveGoalBaseBehavior<T>::drive_goal_behavior_is_done, this);
    data.cleanup_func = std::bind(&DriveGoalBaseBehavior<T>::cleanup_drive_goal, this, goal_handle);
    data.stop_on_new_behavior = true;
    data.apply_backup_limits = true;

    bool ret = behavior_scheduler_->set_behavior(data);
    if (!ret) {
      // for some reason we couldn't set the new behavior, treat this as a goal being cancelled
      drive_goal_running_ = false;
      RCLCPP_WARN(logger_, "%s behavior failed to start", server_name_.c_str());
      abort_drive_goal(goal_handle);
      return;
    }
    last_feedback_time_ = clock_->now();
  }

  BehaviorsScheduler::optional_output_t execute_drive_goal(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<T>> goal_handle,
    const RobotState & current_state)
  {
    rclcpp::Time current_time = clock_->now();
    current_pose_ = get_current_pose_stamped(current_time, current_state.pose);
    // Handle if goal is cancelling
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(logger_, "%s canceled", server_name_.c_str());
      drive_goal_running_ = false;
      auto result = std::make_shared<typename T::Result>();
      result->pose = current_pose_;
      goal_handle->canceled(result);
      return BehaviorsScheduler::optional_output_t();
    }

    // Check failure conditions
    bool fail_condition = false;
    for (const auto & hazard : current_state.hazards.detections) {
      if (hazard.type == irobot_create_msgs::msg::HazardDetection::CLIFF) {
        RCLCPP_INFO(logger_, "%s encountered cliff, abort", server_name_.c_str());
        fail_condition = true;
      } else if (hazard.type == irobot_create_msgs::msg::HazardDetection::STALL) {
        RCLCPP_INFO(logger_, "%s encountered stall, abort", server_name_.c_str());
        fail_condition = true;
      }
    }

    if (fail_condition) {
      drive_goal_running_ = false;
      abort_drive_goal(goal_handle);
      return BehaviorsScheduler::optional_output_t();
    }

    // Check termination condition, otherwise send arc velocity
    BehaviorsScheduler::optional_output_t output;
    bool succeeded = iterate_on_goal(current_state.pose, output);
    if (succeeded) {
      drive_goal_running_ = false;
      auto result = std::make_shared<typename T::Result>();
      result->pose = current_pose_;
      goal_handle->succeed(result);
      return output;
    }
    auto time_since_feedback = current_time - last_feedback_time_;
    std::shared_ptr<typename T::Feedback> feedback = get_feedback(time_since_feedback);
    if (feedback) {
      goal_handle->publish_feedback(feedback);
      last_feedback_time_ = current_time;
    }
    return output;
  }

  typename rclcpp_action::Server<T>::SharedPtr drive_goal_action_server_;

  std::atomic<bool> drive_goal_running_;
  const std::string server_name_;
  rclcpp::Time last_feedback_time_;
  std::shared_ptr<BehaviorsScheduler> behavior_scheduler_;
  geometry_msgs::msg::PoseStamped current_pose_ {};
};

/**
 * @brief This class manages the DriveArc action server.
 */
class DriveArcBehavior : public DriveGoalBaseBehavior<irobot_create_msgs::action::DriveArc>
{
public:
  DriveArcBehavior(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    std::shared_ptr<BehaviorsScheduler> behavior_scheduler,
    float servo_speed,
    const std::string & server_name);

  void initialize_goal(const irobot_create_msgs::action::DriveArc::Goal & goal) override;
  bool iterate_on_goal(
    const tf2::Transform & current_pose,
    BehaviorsScheduler::optional_output_t & output) override;
  std::shared_ptr<irobot_create_msgs::action::DriveArc::Feedback> get_feedback(
    const rclcpp::Duration & time_since_feedback) override;

private:
  std::mutex drive_arc_params_mutex_;
  float last_angle_travel_position_;
  float remain_angle_travel_;
  int8_t start_sign_;
  std::atomic<bool> first_iter_;
  geometry_msgs::msg::Twist arc_velocity_cmd_;
  float translate_speed_;
  const float slow_angle_dist_ {0.4};
  const float converge_angle_dist_ {M_PI / 360.0f};
  const float min_percent_ {0.2};
  const float min_controllable_ {0.01};
  const rclcpp::Duration report_feedback_interval_ {std::chrono::seconds(3)};
};

/**
 * @brief This class manages the DriveDistance action server.
 */
class DriveDistanceBehavior
  : public DriveGoalBaseBehavior<irobot_create_msgs::action::DriveDistance>
{
public:
  DriveDistanceBehavior(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    std::shared_ptr<BehaviorsScheduler> behavior_scheduler,
    float servo_speed,
    const std::string & server_name);

  void initialize_goal(const irobot_create_msgs::action::DriveDistance::Goal & goal) override;
  bool iterate_on_goal(
    const tf2::Transform & current_pose,
    BehaviorsScheduler::optional_output_t & output) override;
  std::shared_ptr<irobot_create_msgs::action::DriveDistance::Feedback> get_feedback(
    const rclcpp::Duration & time_since_feedback) override;

private:
  std::mutex drive_distance_params_mutex_;
  float travel_distance_sq_;
  float remaining_travel_;
  float goal_travel_;
  tf2::Vector3 start_position_;
  std::atomic<bool> first_iter_;
  geometry_msgs::msg::Twist drive_velocity_cmd_;
  float translate_speed_;
  const float slow_translate_dist_ {0.1};
  const float converge_translate_dist_ {0.005};
  const float min_translate_vel_ {0.05};
  const rclcpp::Duration report_feedback_interval_ {std::chrono::seconds(3)};
};

/**
 * @brief This class manages the RotateAngle action server.
 */
class RotateAngleBehavior : public DriveGoalBaseBehavior<irobot_create_msgs::action::RotateAngle>
{
public:
  RotateAngleBehavior(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    std::shared_ptr<BehaviorsScheduler> behavior_scheduler,
    float max_rot_speed_radps,
    const std::string & server_name);

  void initialize_goal(const irobot_create_msgs::action::RotateAngle::Goal & goal) override;
  bool iterate_on_goal(
    const tf2::Transform & current_pose,
    BehaviorsScheduler::optional_output_t & output) override;
  std::shared_ptr<irobot_create_msgs::action::RotateAngle::Feedback> get_feedback(
    const rclcpp::Duration & time_since_feedback) override;

private:
  std::mutex rotate_angle_params_mutex_;
  float last_angle_travel_position_;
  float remain_angle_travel_;
  int8_t start_sign_;
  std::atomic<bool> first_iter_;
  geometry_msgs::msg::Twist rotate_velocity_cmd_;
  float servo_speed_;
  float max_rot_speed_radps_;
  const float slow_angle_dist_ {0.6};
  const float converge_angle_dist_ {M_PI / 360.0f};
  const float min_angular_vel_ {0.1};
  const rclcpp::Duration report_feedback_interval_ {std::chrono::seconds(3)};
};
/**
 * @brief This class manages the NavigateToPosition action server.
 */
class NavigateToPositionBehavior
  : public DriveGoalBaseBehavior<irobot_create_msgs::action::NavigateToPosition>
{
public:
  NavigateToPositionBehavior(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    std::shared_ptr<BehaviorsScheduler> behavior_scheduler,
    float servo_speed,
    float max_rot_speed_radps,
    const std::string & server_name);

  void initialize_goal(const irobot_create_msgs::action::NavigateToPosition::Goal & goal) override;
  bool iterate_on_goal(
    const tf2::Transform & current_pose,
    BehaviorsScheduler::optional_output_t & output) override;
  std::shared_ptr<irobot_create_msgs::action::NavigateToPosition::Feedback> get_feedback(
    const rclcpp::Duration & time_since_feedback) override;

private:
  double angle_to_goal(const tf2::Transform & start_pose, const tf2::Transform & goal_pose);

  std::mutex navigate_to_position_params_mutex_;
  tf2::Transform goal_pose_;
  float last_angle_travel_position_;
  float remain_angle_travel_;
  int8_t start_sign_;
  std::atomic<bool> first_iter_;
  geometry_msgs::msg::Twist rotate_velocity_cmd_;
  float servo_speed_;
  float max_rot_speed_radps_;
  const float apply_ang_correction_thresh_ {0.05};
  RotateAngleBehavior rotate_behavior_;
  DriveDistanceBehavior translate_behavior_;
  irobot_create_msgs::action::NavigateToPosition::Goal nav_goal_;
  irobot_create_msgs::action::NavigateToPosition::Feedback nav_state_;
  int8_t last_reported_nav_state_;
};

}  // namespace irobot_create_nodes
#endif  // IROBOT_CREATE_NODES__MOTION_CONTROL__DRIVE_GOAL_BEHAVIORS_HPP_
