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

#ifndef IROBOT_CREATE_TOOLBOX__MOTION_CONTROL__SIMPLE_GOAL_CONTROLLER_HPP_
#define IROBOT_CREATE_TOOLBOX__MOTION_CONTROL__SIMPLE_GOAL_CONTROLLER_HPP_

#include <angles/angles.h>
#include <boost/optional.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <irobot_create_toolbox/motion_control/behaviors_scheduler.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <deque>
#include <functional>
#include <mutex>
#include <vector>

namespace irobot_create_toolbox
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
    const std::lock_guard<std::mutex> lock(m_mutex);
    // Convert path points to goal points
    m_goal_points.clear();
    m_goal_points.resize(cmd_path.size());
    for (size_t i = 0; i < cmd_path.size(); ++i) {
      GoalPoint & gp = m_goal_points[i];
      const tf2::Vector3 & pt_position = cmd_path[i].pose.getOrigin();
      gp.x = pt_position.getX();
      gp.y = pt_position.getY();
      gp.theta = tf2::getYaw(cmd_path[i].pose.getRotation());
      gp.radius = cmd_path[i].radius;
      gp.drive_backwards = cmd_path[i].drive_backwards;
    }
    m_navigate_state = NavigateStates::ANGLE_TO_GOAL;
    m_max_rotation = max_rotation;
    m_max_translation = max_translation;
  }

  /// \brief Clear goal
  void reset()
  {
    const std::lock_guard<std::mutex> lock(m_mutex);
    m_goal_points.clear();
  }

  // \brief Generate velocity based on current position and next goal point looking for convergence
  // with goal point based on radius.
  // \return empty optional if no goal or velocity command to get to next goal point
  BehaviorsScheduler::optional_output_t get_velocity_for_position(
    const tf2::Transform & current_pose)
  {
    BehaviorsScheduler::optional_output_t servo_vel;
    const std::lock_guard<std::mutex> lock(m_mutex);
    if (m_goal_points.size() == 0) {
      return servo_vel;
    }
    double current_angle = tf2::getYaw(current_pose.getRotation());
    const tf2::Vector3 & current_position = current_pose.getOrigin();
    // Generate velocity based on current position and next goal point looking for convergence
    // with goal point based on radius.
    switch (m_navigate_state) {
      case NavigateStates::ANGLE_TO_GOAL:
        {
          const GoalPoint & gp = m_goal_points.front();
          double dist_to_goal = std::hypot(
            gp.x - current_position.getX(),
            gp.y - current_position.getY());
          if (dist_to_goal <= gp.radius) {
            servo_vel = geometry_msgs::msg::Twist();
            m_navigate_state = NavigateStates::GO_TO_GOAL_POSITION;
          } else {
            double ang = diff_angle(gp, current_position, current_angle);
            if (gp.drive_backwards) {
              // Angle is 180 from travel direction
              ang = angles::normalize_angle(ang + M_PI);
            }
            bound_rotation(ang);
            servo_vel = geometry_msgs::msg::Twist();
            if (std::abs(ang) < 0.03f) {
              m_navigate_state = NavigateStates::GO_TO_GOAL_POSITION;
            } else {
              servo_vel->angular.z = ang;
            }
          }
          break;
        }
      case NavigateStates::GO_TO_GOAL_POSITION:
        {
          const GoalPoint & gp = m_goal_points.front();
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
          if (dist_to_goal < m_goal_points.front().radius) {
            m_navigate_state = NavigateStates::GOAL_ANGLE;
            // If robot angle has deviated too much from path, reset
          } else if (abs_ang > M_PI / 16.0) {
            m_navigate_state = NavigateStates::ANGLE_TO_GOAL;
            // If niether of above conditions met, drive towards goal
          } else {
            double translate_velocity = dist_to_goal;
            if (translate_velocity > m_max_translation) {
              translate_velocity = m_max_translation;
            }
            if (gp.drive_backwards) {
              translate_velocity *= -1;
            }
            servo_vel->linear.x = translate_velocity;
            if (abs_ang > 0.02) {
              servo_vel->angular.z = ang;
            }
          }
          break;
        }
      case NavigateStates::GOAL_ANGLE:
        {
          double ang =
            angles::shortest_angular_distance(current_angle, m_goal_points.front().theta);
          bound_rotation(ang);
          if (std::abs(ang) > 0.02f) {
            servo_vel = geometry_msgs::msg::Twist();
            servo_vel->angular.z = ang;
          } else {
            m_goal_points.pop_front();
            if (m_goal_points.size() > 0) {
              servo_vel = geometry_msgs::msg::Twist();
              m_navigate_state = NavigateStates::ANGLE_TO_GOAL;
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
    if (abs_rot > m_max_rotation) {
      rotation_velocity = std::copysign(m_max_rotation, rotation_velocity);
    } else if (abs_rot < 0.1 && abs_rot > 0.01) {
      // min speed if desire small non zero velocity
      rotation_velocity = std::copysign(0.1, rotation_velocity);
    }
  }

  double diff_angle(const GoalPoint & goal_pt, const tf2::Vector3 & cur_position, double cur_angle)
  {
    return angles::shortest_angular_distance(
      cur_angle, std::atan2(
        goal_pt.y - cur_position.getY(),
        goal_pt.x - cur_position.getX()));
  }

  std::mutex m_mutex;
  std::deque<GoalPoint> m_goal_points;
  NavigateStates m_navigate_state;
  double m_max_rotation;
  double m_max_translation;
};

}  // namespace irobot_create_toolbox
#endif   // IROBOT_CREATE_TOOLBOX__MOTION_CONTROL__SIMPLE_GOAL_CONTROLLER_HPP_
