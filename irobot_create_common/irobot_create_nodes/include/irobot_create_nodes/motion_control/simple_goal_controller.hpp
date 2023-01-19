// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Justin Kearns (jkearns@irobot.com)

#ifndef IROBOT_CREATE_NODES__MOTION_CONTROL__SIMPLE_GOAL_CONTROLLER_HPP_
#define IROBOT_CREATE_NODES__MOTION_CONTROL__SIMPLE_GOAL_CONTROLLER_HPP_

#include <deque>
#include <functional>
#include <mutex>
#include <vector>

#include "angles/angles.h"
#include "boost/optional.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_nodes/motion_control/behaviors_scheduler.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace irobot_create_nodes
{

/**
 * @brief This class provides an API to give velocity commands given a goal and position.
 */
class SimpleGoalController
{
public:
  SimpleGoalController()
  {
  }

  /// \brief Structure to keep information for each point in commanded path
  //  including pose with position and orientation of point
  //  radius that is considered close enough to achieving the point
  //  drive_backwards whether the robot should drive backwards towards the point (for undocking)
  struct CmdPathPoint
  {
    CmdPathPoint(tf2::Transform p, float r, bool db)
    : pose(p), radius(r), drive_backwards(db) {}
    tf2::Transform pose;
    float radius;
    bool drive_backwards;
  };
  using CmdPath = std::vector<CmdPathPoint>;

  /// \brief Set goal path for controller along with max rotation and translation speed
  void initialize_goal(const CmdPath & cmd_path, double max_rotation, double max_translation)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    // Convert path points to goal points
    goal_points_.clear();
    goal_points_.resize(cmd_path.size());
    for (size_t i = 0; i < cmd_path.size(); ++i) {
      GoalPoint & gp = goal_points_[i];
      const tf2::Vector3 & pt_position = cmd_path[i].pose.getOrigin();
      gp.x = pt_position.getX();
      gp.y = pt_position.getY();
      gp.theta = tf2::getYaw(cmd_path[i].pose.getRotation());
      gp.radius = cmd_path[i].radius;
      gp.drive_backwards = cmd_path[i].drive_backwards;
    }
    navigate_state_ = NavigateStates::ANGLE_TO_GOAL;
    max_rotation_ = max_rotation;
    max_translation_ = max_translation;
  }

  /// \brief Clear goal
  void reset()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    goal_points_.clear();
  }

  // \brief Generate velocity based on current position and next goal point looking for convergence
  // with goal point based on radius.
  // \return empty optional if no goal or velocity command to get to next goal point
  BehaviorsScheduler::optional_output_t get_velocity_for_position(
    const tf2::Transform & current_pose)
  {
    BehaviorsScheduler::optional_output_t servo_vel;
    const std::lock_guard<std::mutex> lock(mutex_);
    if (goal_points_.size() == 0) {
      return servo_vel;
    }
    double current_angle = tf2::getYaw(current_pose.getRotation());
    const tf2::Vector3 & current_position = current_pose.getOrigin();
    // Generate velocity based on current position and next goal point looking for convergence
    // with goal point based on radius.
    switch (navigate_state_) {
      case NavigateStates::ANGLE_TO_GOAL:
        {
          const GoalPoint & gp = goal_points_.front();
          double dist_to_goal = std::hypot(
            gp.x - current_position.getX(),
            gp.y - current_position.getY());
          if (dist_to_goal <= gp.radius) {
            servo_vel = geometry_msgs::msg::Twist();
            navigate_state_ = NavigateStates::GO_TO_GOAL_POSITION;
          } else {
            double ang = diff_angle(gp, current_position, current_angle);
            if (gp.drive_backwards) {
              // Angle is 180 from travel direction
              ang = angles::normalize_angle(ang + M_PI);
            }
            bound_rotation(ang);
            servo_vel = geometry_msgs::msg::Twist();
            if (std::abs(ang) < TO_GOAL_ANGLE_CONVERGED) {
              navigate_state_ = NavigateStates::GO_TO_GOAL_POSITION;
            } else {
              servo_vel->angular.z = ang;
            }
          }
          break;
        }
      case NavigateStates::GO_TO_GOAL_POSITION:
        {
          const GoalPoint & gp = goal_points_.front();
          double dist_to_goal = std::hypot(
            gp.x - current_position.getX(),
            gp.y - current_position.getY());
          double ang = diff_angle(gp, current_position, current_angle);
          double abs_ang = std::abs(ang);
          if (gp.drive_backwards) {
            // Angle is 180 from travel direction
            abs_ang = angles::normalize_angle(abs_ang + M_PI);
          }
          servo_vel = geometry_msgs::msg::Twist();
          // If robot is close enough to goal, move to final stage
          if (dist_to_goal < goal_points_.front().radius) {
            navigate_state_ = NavigateStates::GOAL_ANGLE;
            // If robot angle has deviated too much from path, reset
          } else if (abs_ang > GO_TO_GOAL_ANGLE_TOO_FAR) {
            navigate_state_ = NavigateStates::ANGLE_TO_GOAL;
            // If niether of above conditions met, drive towards goal
          } else {
            double translate_velocity = dist_to_goal;
            if (translate_velocity > max_translation_) {
              translate_velocity = max_translation_;
            }
            if (gp.drive_backwards) {
              translate_velocity *= -1;
            }
            servo_vel->linear.x = translate_velocity;
            if (abs_ang > GO_TO_GOAL_APPLY_ROTATION_ANGLE) {
              servo_vel->angular.z = ang;
            }
          }
          break;
        }
      case NavigateStates::GOAL_ANGLE:
        {
          double ang =
            angles::shortest_angular_distance(current_angle, goal_points_.front().theta);
          bound_rotation(ang);
          if (std::abs(ang) > GOAL_ANGLE_CONVERGED) {
            servo_vel = geometry_msgs::msg::Twist();
            servo_vel->angular.z = ang;
          } else {
            goal_points_.pop_front();
            if (goal_points_.size() > 0) {
              servo_vel = geometry_msgs::msg::Twist();
              navigate_state_ = NavigateStates::ANGLE_TO_GOAL;
            }
          }
          break;
        }
    }
    return servo_vel;
  }

private:
  enum class NavigateStates
  {
    ANGLE_TO_GOAL,
    GO_TO_GOAL_POSITION,
    GOAL_ANGLE,
  };

  struct GoalPoint
  {
    double x;
    double y;
    double theta;
    float radius;
    bool drive_backwards;
  };

  void bound_rotation(double & rotation_velocity)
  {
    double abs_rot = std::abs(rotation_velocity);
    if (abs_rot > max_rotation_) {
      rotation_velocity = std::copysign(max_rotation_, rotation_velocity);
    } else if (abs_rot < MIN_ROTATION && abs_rot > 0.01) {
      // min speed if desire small non zero velocity
      rotation_velocity = std::copysign(MIN_ROTATION, rotation_velocity);
    }
  }

  double diff_angle(const GoalPoint & goal_pt, const tf2::Vector3 & cur_position, double cur_angle)
  {
    return angles::shortest_angular_distance(
      cur_angle, std::atan2(
        goal_pt.y - cur_position.getY(),
        goal_pt.x - cur_position.getX()));
  }

  std::mutex mutex_;
  std::deque<GoalPoint> goal_points_;
  NavigateStates navigate_state_;
  double max_rotation_;
  double max_translation_;
  const double MIN_ROTATION {0.1};
  const double TO_GOAL_ANGLE_CONVERGED {0.03};
  const double GO_TO_GOAL_ANGLE_TOO_FAR {M_PI / 16.0};
  const double GO_TO_GOAL_APPLY_ROTATION_ANGLE {0.02};
  const double GOAL_ANGLE_CONVERGED {0.02};
};

}  // namespace irobot_create_nodes
#endif   // IROBOT_CREATE_NODES__MOTION_CONTROL__SIMPLE_GOAL_CONTROLLER_HPP_
