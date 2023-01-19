// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Justin Kearns (jkearns@irobot.com)

#include <algorithm>
#include <memory>
#include <string>

#include "angles/angles.h"
#include "irobot_create_nodes/motion_control/drive_goal_behaviors.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace irobot_create_nodes
{
//// Helper functions ////
geometry_msgs::msg::Twist get_default_velocity_cmd()
{
  geometry_msgs::msg::Twist default_cmd;
  default_cmd.linear.x = 0;
  default_cmd.linear.y = 0;
  default_cmd.linear.z = 0;
  default_cmd.angular.x = 0;
  default_cmd.angular.y = 0;
  default_cmd.angular.z = 0;
  return default_cmd;
}
geometry_msgs::msg::PoseStamped get_current_pose_stamped(
  const rclcpp::Time & current_time,
  const tf2::Transform & pose)
{
  geometry_msgs::msg::PoseStamped result_pose;
  result_pose.header.stamp = current_time;
  result_pose.header.frame_id = "odom";
  const tf2::Vector3 & robot_position = pose.getOrigin();
  result_pose.pose.position.x = robot_position.getX();
  result_pose.pose.position.y = robot_position.getY();
  result_pose.pose.position.z = robot_position.getZ();
  const tf2::Quaternion & pose_quat = pose.getRotation();
  result_pose.pose.orientation.w = pose_quat.getW();
  result_pose.pose.orientation.x = pose_quat.getX();
  result_pose.pose.orientation.y = pose_quat.getY();
  result_pose.pose.orientation.z = pose_quat.getZ();
  return result_pose;
}
//// Implementation for Drive Arc action server ////

DriveArcBehavior::DriveArcBehavior(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  std::shared_ptr<BehaviorsScheduler> behavior_scheduler,
  float servo_speed,
  const std::string & server_name)
: DriveGoalBaseBehavior<irobot_create_msgs::action::DriveArc>(node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    behavior_scheduler,
    server_name),
  first_iter_(true),
  translate_speed_(servo_speed)
{
}

void DriveArcBehavior::initialize_goal(const irobot_create_msgs::action::DriveArc::Goal & goal)
{
  first_iter_ = true;
  float max_speed = std::min(translate_speed_, std::abs(goal.max_translation_speed));
  RCLCPP_INFO(
    logger_, "DriveArc with radius %f, angle %f, max_speed %f",
    goal.radius, goal.angle, max_speed);

  const std::lock_guard<std::mutex> lock(drive_arc_params_mutex_);
  arc_velocity_cmd_ = get_default_velocity_cmd();
  arc_velocity_cmd_.linear.x = max_speed;
  arc_velocity_cmd_.angular.z = std::copysign(max_speed / std::abs(goal.radius), goal.angle);
  if (goal.translate_direction == irobot_create_msgs::action::DriveArc::Goal::TRANSLATE_BACKWARD) {
    arc_velocity_cmd_.linear.x *= -1.0f;
  }
  remain_angle_travel_ = goal.angle;
  start_sign_ = std::copysign(1, remain_angle_travel_);
}

bool DriveArcBehavior::iterate_on_goal(
  const tf2::Transform & current_pose,
  BehaviorsScheduler::optional_output_t & output)
{
  double current_angle = tf2::getYaw(current_pose.getRotation());
  if (first_iter_) {
    first_iter_ = false;
    const std::lock_guard<std::mutex> lock(drive_arc_params_mutex_);
    last_angle_travel_position_ = current_angle;
  }

  // Check termination condition, otherwise send arc velocity
  {
    const std::lock_guard<std::mutex> lock(drive_arc_params_mutex_);
    remain_angle_travel_ -=
      angles::shortest_angular_distance(last_angle_travel_position_, current_angle);
    last_angle_travel_position_ = current_angle;
    int8_t current_sign = std::copysign(1, remain_angle_travel_);
    if (current_sign != start_sign_) {
      RCLCPP_INFO(logger_, "Drive Arc traveled commanded angle, succeeded");
      return true;
    } else {
      float abs_remain_angle_travel = std::abs(remain_angle_travel_);
      output = arc_velocity_cmd_;
      if (abs_remain_angle_travel < slow_angle_dist_) {
        if (abs_remain_angle_travel <= converge_angle_dist_) {
          RCLCPP_INFO(logger_, "Drive Arc traveled commanded angle, succeeded");
          return true;
        }
        float scale = abs_remain_angle_travel / slow_angle_dist_;
        scale = std::max(scale, min_percent_);
        output->linear.x = arc_velocity_cmd_.linear.x * scale;
        output->angular.z = arc_velocity_cmd_.angular.z * scale;
        float abs_linear = std::abs(output->linear.x);
        if (abs_linear < min_controllable_) {
          float up_scale = min_controllable_ / abs_linear;
          output->linear.x *= up_scale;
          output->angular.z *= up_scale;
        }
      }
    }
  }
  return false;
}

std::shared_ptr<irobot_create_msgs::action::DriveArc::Feedback> DriveArcBehavior::get_feedback(
  const rclcpp::Duration & time_since_feedback)
{
  if (time_since_feedback > report_feedback_interval_) {
    auto feedback = std::make_shared<irobot_create_msgs::action::DriveArc::Feedback>();
    const std::lock_guard<std::mutex> lock(drive_arc_params_mutex_);
    feedback->remaining_angle_travel = remain_angle_travel_;
    return feedback;
  }
  return nullptr;
}

//// Implementation for Drive Distance action server ////

DriveDistanceBehavior::DriveDistanceBehavior(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  std::shared_ptr<BehaviorsScheduler> behavior_scheduler,
  float servo_speed,
  const std::string & server_name)
:  DriveGoalBaseBehavior<irobot_create_msgs::action::DriveDistance>(node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    behavior_scheduler,
    server_name),
  first_iter_(true),
  translate_speed_(servo_speed)
{
}

void DriveDistanceBehavior::initialize_goal(
  const irobot_create_msgs::action::DriveDistance::Goal & goal)
{
  first_iter_ = true;
  const std::lock_guard<std::mutex> lock(drive_distance_params_mutex_);
  drive_velocity_cmd_ = get_default_velocity_cmd();
  goal_travel_ = goal.distance;
  travel_distance_sq_ = goal_travel_ * goal_travel_;
  remaining_travel_ = goal_travel_;
  float max_speed = std::min(translate_speed_, goal.max_translation_speed);
  RCLCPP_INFO(logger_, "DriveDistance with distance %f, max_speed %f", goal.distance, max_speed);
  drive_velocity_cmd_.linear.x = std::copysign(max_speed, goal_travel_);
}

bool DriveDistanceBehavior::iterate_on_goal(
  const tf2::Transform & current_pose,
  BehaviorsScheduler::optional_output_t & output)
{
  tf2::Vector3 current_position = current_pose.getOrigin();
  if (first_iter_) {
    first_iter_ = false;
    const std::lock_guard<std::mutex> lock(drive_distance_params_mutex_);
    start_position_ = current_position;
  }
  {
    const std::lock_guard<std::mutex> lock(drive_distance_params_mutex_);

    tf2::Vector3 position_diff = current_position - start_position_;
    double odom_dist_sq = tf2::tf2Dot(position_diff, position_diff);
    if (odom_dist_sq >= travel_distance_sq_) {
      RCLCPP_INFO(logger_, "Drive Distance traveled commanded distance, succeeded");
      return true;
    } else {
      remaining_travel_ = goal_travel_ - std::sqrt(odom_dist_sq);
      output = drive_velocity_cmd_;
      float abs_remaining = std::abs(remaining_travel_);
      if (abs_remaining <= slow_translate_dist_) {
        if (abs_remaining <= converge_translate_dist_) {
          RCLCPP_INFO(logger_, "Drive Distance traveled commanded distance, succeeded");
          return true;
        }
        abs_remaining = std::max(abs_remaining, min_translate_vel_);
        float remain_vel = std::copysign(abs_remaining, goal_travel_);
        if (std::abs(remain_vel) < std::abs(output->linear.x)) {
          output->linear.x = remain_vel;
        }
      }
    }
  }
  return false;
}

std::shared_ptr<irobot_create_msgs::action::DriveDistance::Feedback> DriveDistanceBehavior::
get_feedback(const rclcpp::Duration & time_since_feedback)
{
  if (time_since_feedback > report_feedback_interval_) {
    auto feedback = std::make_shared<irobot_create_msgs::action::DriveDistance::Feedback>();
    const std::lock_guard<std::mutex> lock(drive_distance_params_mutex_);
    feedback->remaining_travel_distance = remaining_travel_;
    return feedback;
  }
  return nullptr;
}

//// Implementation for Rotate Angle action server ////

RotateAngleBehavior::RotateAngleBehavior(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  std::shared_ptr<BehaviorsScheduler> behavior_scheduler,
  float max_rot_speed_radps,
  const std::string & server_name)
: DriveGoalBaseBehavior<irobot_create_msgs::action::RotateAngle>(node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    behavior_scheduler,
    server_name),
  first_iter_(true),
  max_rot_speed_radps_(max_rot_speed_radps)
{
}

void RotateAngleBehavior::initialize_goal(
  const irobot_create_msgs::action::RotateAngle::Goal & goal)
{
  first_iter_ = true;
  const std::lock_guard<std::mutex> lock(rotate_angle_params_mutex_);
  rotate_velocity_cmd_ = get_default_velocity_cmd();
  remain_angle_travel_ = goal.angle;
  start_sign_ = std::copysign(1, remain_angle_travel_);
  float max_speed = std::min(max_rot_speed_radps_, goal.max_rotation_speed);
  RCLCPP_INFO(logger_, "RotateAngle with angle %f, max_speed %f", goal.angle, max_speed);
  rotate_velocity_cmd_.angular.z = std::copysign(max_speed, remain_angle_travel_);
}

bool RotateAngleBehavior::iterate_on_goal(
  const tf2::Transform & current_pose,
  BehaviorsScheduler::optional_output_t & output)
{
  double current_angle = tf2::getYaw(current_pose.getRotation());
  if (first_iter_) {
    first_iter_ = false;
    const std::lock_guard<std::mutex> lock(rotate_angle_params_mutex_);
    last_angle_travel_position_ = current_angle;
  }
  const std::lock_guard<std::mutex> lock(rotate_angle_params_mutex_);

  remain_angle_travel_ -=
    angles::shortest_angular_distance(last_angle_travel_position_, current_angle);
  last_angle_travel_position_ = current_angle;
  int8_t current_sign = std::copysign(1, remain_angle_travel_);
  if (current_sign != start_sign_) {
    RCLCPP_INFO(logger_, "Rotate Angle traveled commanded angle, succeeded");
    return true;
  } else {
    float abs_remain_angle_travel = std::abs(remain_angle_travel_);
    output = rotate_velocity_cmd_;
    if (abs_remain_angle_travel < slow_angle_dist_) {
      if (abs_remain_angle_travel < converge_angle_dist_) {
        RCLCPP_INFO(logger_, "Rotate Angle traveled commanded angle, succeeded");
        return true;
      }
      double slow_speed = std::max(abs_remain_angle_travel, min_angular_vel_);
      if (slow_speed < std::abs(output->angular.z)) {
        output->angular.z = std::copysign(slow_speed, remain_angle_travel_);
      }
    }
  }
  return false;
}

std::shared_ptr<irobot_create_msgs::action::RotateAngle::Feedback>
RotateAngleBehavior::get_feedback(const rclcpp::Duration & time_since_feedback)
{
  if (time_since_feedback > report_feedback_interval_) {
    auto feedback = std::make_shared<irobot_create_msgs::action::RotateAngle::Feedback>();
    const std::lock_guard<std::mutex> lock(rotate_angle_params_mutex_);
    feedback->remaining_angle_travel = remain_angle_travel_;
    return feedback;
  }
  return nullptr;
}

//// Implementation for Navigate To Position action server ////

NavigateToPositionBehavior::NavigateToPositionBehavior(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  std::shared_ptr<BehaviorsScheduler> behavior_scheduler,
  float servo_speed,
  float max_rot_speed_radps,
  const std::string & server_name)
: DriveGoalBaseBehavior<irobot_create_msgs::action::NavigateToPosition>(node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    behavior_scheduler,
    server_name),
  first_iter_(true),
  servo_speed_(servo_speed),
  max_rot_speed_radps_(max_rot_speed_radps),
  rotate_behavior_(node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    behavior_scheduler,
    max_rot_speed_radps,
    std::string()),
  translate_behavior_(node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    behavior_scheduler,
    servo_speed,
    std::string())
{
}

void NavigateToPositionBehavior::initialize_goal(
  const irobot_create_msgs::action::NavigateToPosition::Goal & goal)
{
  first_iter_ = true;
  const std::lock_guard<std::mutex> lock(navigate_to_position_params_mutex_);
  nav_goal_ = goal;
  last_reported_nav_state_ = 0;
}

bool NavigateToPositionBehavior::iterate_on_goal(
  const tf2::Transform & current_pose,
  BehaviorsScheduler::optional_output_t & output)
{
  tf2::Vector3 current_position = current_pose.getOrigin();
  if (first_iter_) {
    first_iter_ = false;
    const std::lock_guard<std::mutex> lock(navigate_to_position_params_mutex_);
    nav_state_.navigate_state =
      irobot_create_msgs::action::NavigateToPosition::Feedback::ROTATING_TO_GOAL_POSITION;

    tf2::Quaternion goal_quat;
    tf2::Vector3 goal_position = {nav_goal_.goal_pose.pose.position.x,
      nav_goal_.goal_pose.pose.position.y,
      nav_goal_.goal_pose.pose.position.z};
    tf2::fromMsg(nav_goal_.goal_pose.pose.orientation, goal_quat);
    goal_pose_ = tf2::Transform(goal_quat, goal_position);
    RCLCPP_INFO(
      logger_, "NavigateToPosition with position x %f, y %f, theta %f",
      goal_position.getX(), goal_position.getY(), tf2::getYaw(goal_quat));
    irobot_create_msgs::action::RotateAngle::Goal rotate_goal;
    rotate_goal.angle = angle_to_goal(current_pose, goal_pose_);
    rotate_goal.max_rotation_speed = nav_goal_.max_rotation_speed;
    rotate_behavior_.initialize_goal(rotate_goal);
  }
  const std::lock_guard<std::mutex> lock(navigate_to_position_params_mutex_);
  bool rotate_to_position =
    (nav_state_.navigate_state ==
    irobot_create_msgs::action::NavigateToPosition::Feedback::ROTATING_TO_GOAL_POSITION);
  bool rotate_to_goal_heading =
    (nav_state_.navigate_state ==
    irobot_create_msgs::action::NavigateToPosition::Feedback::ROTATING_TO_GOAL_ORIENTATION);
  bool drive_to_goal =
    (nav_state_.navigate_state ==
    irobot_create_msgs::action::NavigateToPosition::Feedback::DRIVING_TO_GOAL_POSITION);
  if (rotate_to_position || rotate_to_goal_heading) {
    if (rotate_behavior_.iterate_on_goal(current_pose, output)) {
      if (rotate_to_position) {
        // Transition to drive
        nav_state_.navigate_state =
          irobot_create_msgs::action::NavigateToPosition::Feedback::DRIVING_TO_GOAL_POSITION;
        tf2::Vector3 goal_position = goal_pose_.getOrigin();
        tf2::Vector3 pos_diff = goal_position - current_position;
        float goal_dist = std::hypot(pos_diff.getX(), pos_diff.getY());
        irobot_create_msgs::action::DriveDistance::Goal distance_goal;
        distance_goal.distance = goal_dist;
        distance_goal.max_translation_speed = nav_goal_.max_translation_speed;
        translate_behavior_.initialize_goal(distance_goal);
        drive_to_goal = true;
      } else {
        // We are finished
        return true;
      }
    }
  }
  if (drive_to_goal) {
    if (translate_behavior_.iterate_on_goal(current_pose, output)) {
      if (nav_goal_.achieve_goal_heading) {
        nav_state_.navigate_state =
          irobot_create_msgs::action::NavigateToPosition::Feedback::ROTATING_TO_GOAL_ORIENTATION;
        irobot_create_msgs::action::RotateAngle::Goal rotate_goal;
        rotate_goal.angle = angles::shortest_angular_distance(
          tf2::getYaw(current_pose.getRotation()),
          tf2::getYaw(goal_pose_.getRotation()));
        rotate_goal.max_rotation_speed = nav_goal_.max_rotation_speed;
        rotate_behavior_.initialize_goal(rotate_goal);
        return rotate_behavior_.iterate_on_goal(current_pose, output);
      } else {
        // We are finished
        return true;
      }
    } else {
      // Look for deviation from goal while driving straight and apply small correction
      float goal_ang_diff = angle_to_goal(current_pose, goal_pose_);
      if (std::abs(goal_ang_diff) > apply_ang_correction_thresh_) {
        output->angular.z += goal_ang_diff;
      }
    }
  }
  return false;
}

std::shared_ptr<irobot_create_msgs::action::NavigateToPosition::Feedback> NavigateToPositionBehavior
::get_feedback(const rclcpp::Duration & time_since_feedback)
{
  const std::lock_guard<std::mutex> lock(navigate_to_position_params_mutex_);
  bool has_feedback = false;
  bool rotate_to_position =
    (nav_state_.navigate_state ==
    irobot_create_msgs::action::NavigateToPosition::Feedback::ROTATING_TO_GOAL_POSITION);
  bool rotate_to_goal_heading =
    (nav_state_.navigate_state ==
    irobot_create_msgs::action::NavigateToPosition::Feedback::ROTATING_TO_GOAL_ORIENTATION);
  bool drive_to_goal =
    (nav_state_.navigate_state ==
    irobot_create_msgs::action::NavigateToPosition::Feedback::DRIVING_TO_GOAL_POSITION);
  std::shared_ptr<irobot_create_msgs::action::NavigateToPosition::Feedback> feedback;
  if (rotate_to_position || rotate_to_goal_heading) {
    std::shared_ptr<irobot_create_msgs::action::RotateAngle::Feedback> rotate_feedback =
      rotate_behavior_.get_feedback(time_since_feedback);
    if (rotate_feedback) {
      feedback = std::make_shared<irobot_create_msgs::action::NavigateToPosition::Feedback>(
        nav_state_);
      feedback->remaining_angle_travel = rotate_feedback->remaining_angle_travel;
      has_feedback = true;
    }
  } else if (drive_to_goal) {
    std::shared_ptr<irobot_create_msgs::action::DriveDistance::Feedback> translate_feedback =
      translate_behavior_.get_feedback(time_since_feedback);
    if (translate_feedback) {
      feedback = std::make_shared<irobot_create_msgs::action::NavigateToPosition::Feedback>(
        nav_state_);
      feedback->remaining_travel_distance = translate_feedback->remaining_travel_distance;
      has_feedback = true;
    }
  }
  if ((last_reported_nav_state_ != nav_state_.navigate_state) || has_feedback) {
    last_reported_nav_state_ = nav_state_.navigate_state;
    if (!feedback) {
      feedback = std::make_shared<irobot_create_msgs::action::NavigateToPosition::Feedback>(
        nav_state_);
    }
    feedback->navigate_state = nav_state_.navigate_state;
  }
  return feedback;
}

double NavigateToPositionBehavior::angle_to_goal(
  const tf2::Transform & start_pose,
  const tf2::Transform & goal_pose)
{
  const tf2::Vector3 & start_position = start_pose.getOrigin();
  const tf2::Vector3 & goal_position = goal_pose.getOrigin();
  return angles::shortest_angular_distance(
    tf2::getYaw(start_pose.getRotation()),
    std::atan2(
      goal_position.getY() - start_position.getY(),
      goal_position.getX() - start_position.getX()));
}

}  // namespace irobot_create_nodes
