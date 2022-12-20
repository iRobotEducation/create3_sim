// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Justin Kearns (jkearns@irobot.com)

#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "irobot_create_nodes/motion_control/wall_follow_states.hpp"

namespace irobot_create_nodes
{
void WallFollowStateManager::initialize(int8_t follow_side)
{
  follow_side_ = follow_side;
  wall_engaged_ = false;
  current_follow_mode_ = nullptr;
}

bool WallFollowStateManager::get_next_velocity(
  const tf2::Transform & robot_pose,
  const irobot_create_msgs::msg::IrIntensityVector & ir_intensity,
  const std::vector<std::string> & active_hazard_frames,
  WFVelocityCommand & vel_cmd)
{
  if (!current_follow_mode_) {
    // Setup a spiral behavior to find the wall
    current_follow_mode_ = std::make_shared<SpiralToEngageWall>(robot_pose, follow_side_);
    RCLCPP_DEBUG(logger_, "Start Wall Follow with Spiral To Engage");
  }
  bool still_running = current_follow_mode_->get_next_velocity(
    robot_pose, ir_intensity,
    active_hazard_frames, vel_cmd);
  if (!still_running) {
    switch (current_follow_mode_->get_id()) {
      case WallFollowStateID::SPIRAL:
        {
          // Wall Engaged
          wall_engaged_ = true;
          // Exits from bump, transition to obstacle in front case
          current_follow_mode_ = std::make_shared<ObstacleInFront>(robot_pose, follow_side_);
          RCLCPP_DEBUG(logger_, "Wall Follow transition to Obstacle In Front");
          break;
        }
      case WallFollowStateID::ALIGNED_SERVO:
        {
          // Either exited from losing wall or obstacles in front
          if (current_follow_mode_->is_engaged()) {
            // Must be obstacles in front
            current_follow_mode_ = std::make_shared<ObstacleInFront>(robot_pose, follow_side_);
            RCLCPP_DEBUG(logger_, "Wall Follow transition to Obstacle In Front");
          } else {
            // Must have lost wall
            wall_engaged_ = false;
            current_follow_mode_ = std::make_shared<SpiralToEngageWall>(robot_pose, follow_side_);
            RCLCPP_DEBUG(logger_, "Wall Follow transition to Spiral To Engage");
          }
          break;
        }
      case WallFollowStateID::OBSTACLE_IN_FRONT:
        {
          // Either exited from losing wall or being aligned
          if (current_follow_mode_->is_engaged()) {
            // Must be aligned
            current_follow_mode_ = std::make_shared<AlignedIRServo>(
              ir_intensity.header.stamp,
              follow_side_);
            RCLCPP_DEBUG(logger_, "Wall Follow transition to Aligned Servo");
          } else {
            // Must have lost wall
            wall_engaged_ = false;
            current_follow_mode_ = std::make_shared<SpiralToEngageWall>(robot_pose, follow_side_);
            RCLCPP_DEBUG(logger_, "Wall Follow transition to Spiral To Engage");
          }
          break;
        }
      case WallFollowStateID::NONE:
        {
          // This behavior won't be instantiated by state machine
          RCLCPP_ERROR(logger_, "Wall Follow State machine has NONE behavior");
          break;
        }
    }
  }
  // This manager never stops, must be managed externally
  return true;
}

bool SpiralToEngageWall::get_next_velocity(
  const tf2::Transform & robot_pose,
  const irobot_create_msgs::msg::IrIntensityVector & /*ir_intensity*/,
  const std::vector<std::string> & active_hazard_frames,
  WFVelocityCommand & vel_cmd)
{
  // See if we have encountered an obstruction
  if (active_hazard_frames.size() > 0) {
    // Return that we are no longer running
    found_obstruction_ = true;
    return false;
  }
  double current_angle = tf2::getYaw(robot_pose.getRotation());
  double diff_from_start = std::abs(
    angles::shortest_angular_distance(current_angle, start_orientation_));
  if (check_orientation_converge_) {
    if (diff_from_start < converged_angle_threshold_) {
      spiral_rot_divisor_ += 1.0;
      check_orientation_converge_ = false;
    }
  } else if (diff_from_start > M_PI_2) {
    check_orientation_converge_ = true;
  }
  vel_cmd.translate = spiral_trans_vel_;
  vel_cmd.rotate = follow_side_ * spiral_start_rot_vel_ / spiral_rot_divisor_;
  return true;
}

bool AlignedIRServo::get_next_velocity(
  const tf2::Transform & /*robot_pose*/,
  const irobot_create_msgs::msg::IrIntensityVector & ir_intensity,
  const std::vector<std::string> & active_hazard_frames,
  WFVelocityCommand & vel_cmd)
{
  // See if we have encountered an obstruction
  if (active_hazard_frames.size() > 0) {
    // Return that we are no longer running
    found_obstruction_ = true;
    return false;
  }
  rclcpp::Time sensor_time(ir_intensity.header.stamp);
  for (const auto & ir_val : ir_intensity.readings) {
    if (ir_val.header.frame_id == follow_frame_) {
      if (ir_val.value < min_sees_wall_ir_intensity_) {
        // No longer sees wall
        if (sensor_time - last_seen_obs_time_ <
          drive_after_lost_time_)
        {
          vel_cmd.translate = align_trans_vel_;
          return true;
        } else {
          found_obstruction_ = false;
          return false;
        }
      } else {
        last_seen_obs_time_ = sensor_time;
        // Servo to try to get desired ir_intensity
        int16_t ir_diff = sees_wall_desired_ir_intensity_ - ir_val.value;
        vel_cmd.translate = align_trans_vel_;
        vel_cmd.rotate = follow_side_ * ir_diff * ir_diff_to_rotate_scale_;
        if (std::abs(vel_cmd.rotate) > max_rotate_) {
          vel_cmd.rotate = std::copysign(max_rotate_, vel_cmd.rotate);
        }
        return true;
      }
    }
  }
  // Shouldn't not find frame, but if so, same as not seeing wall
  found_obstruction_ = false;
  return false;
}

bool ObstacleInFront::get_next_velocity(
  const tf2::Transform & robot_pose,
  const irobot_create_msgs::msg::IrIntensityVector & ir_intensity,
  const std::vector<std::string> & active_hazard_frames,
  WFVelocityCommand & vel_cmd)
{
  // Summarize IR sensors
  auto obstacle_in_sensors = [this](const std::vector<std::string> & sensors,
      const irobot_create_msgs::msg::IrIntensityVector & ir_intensity,
      int16_t min_obs_threshold, int16_t & obs_val) -> bool {
      for (const auto & frame : sensors) {
        auto frame_it = std::find_if(
          ir_intensity.readings.begin(),
          ir_intensity.readings.end(),
          [&](const irobot_create_msgs::msg::IrIntensity & ir) {
            return ir.header.frame_id == frame;
          });
        if (frame_it != ir_intensity.readings.end()) {
          if (frame_it->value >= min_obs_threshold) {
            // sensor sees obstacle
            obs_val = frame_it->value;
            return true;
          }
        }
      }
      return false;
    };
  int16_t front_val = 0;
  bool obstacle_in_front = obstacle_in_sensors(
    front_ir_frames_, ir_intensity,
    min_sees_wall_front_ir_intensity_, front_val);
  int16_t opp_val = 0;
  bool obstacle_opposite = obstacle_in_sensors(
    opposite_ir_frames_, ir_intensity,
    min_sees_wall_front_ir_intensity_, opp_val);
  int16_t front_side_val = 0;
  bool obstacle_on_front_side = obstacle_in_sensors(
    front_side_ir_frames_, ir_intensity,
    min_sees_wall_side_ir_intensity_, front_side_val);
  int16_t side_val = 0;
  bool obstacle_on_side = obstacle_in_sensors(
    side_ir_frames_, ir_intensity,
    min_sees_wall_side_ir_intensity_, side_val);
  bool hazard = false;
  if (active_hazard_frames.size() > 0) {
    hazard = true;
    obstacle_in_front = true;
  }
  vel_cmd.translate = 0.0;
  vel_cmd.rotate = 0.0;
  if (obstacle_in_front || obstacle_opposite) {
    if (hazard) {
      // Need to backup
      vel_cmd.translate = backup_translate_;
      if (!obstacle_on_side) {
        // Rotate away while backing up
        vel_cmd.rotate = follow_side_ * backup_rotate_;
      }
    } else {
      vel_cmd.rotate = follow_side_ * rotate_in_place_;
    }
  } else if (obstacle_on_front_side && (front_side_val > side_val)) {
    vel_cmd.translate = upcoming_translate_;
    vel_cmd.rotate = follow_side_ * upcoming_rotate_;
  } else {
    double current_angle = tf2::getYaw(robot_pose.getRotation());
    double ang_traversed = std::abs(
      angles::shortest_angular_distance(current_angle, start_pose_angle_));
    const tf2::Vector3 & current_position = robot_pose.getOrigin();
    double dist_traversed = std::hypot(
      start_pose_position_.getX() - current_position.getX(),
      start_pose_position_.getY() - current_position.getY());
    bool traversed_enough = (ang_traversed > min_angular_traversal_ ||
      dist_traversed > min_linear_traversal_);
    // See if we have traversed enough to exit
    if (traversed_enough) {
      if (obstacle_on_side) {
        // We are aligned, let another behavior take over
        aligned_ = true;
        return false;
      } else {
        // No obstacles around, let another behavior take over
        aligned_ = false;
        return false;
      }
    } else {
      // Haven't traversed enough, but no obstacles in front,
      // drive forward a bit
      vel_cmd.translate = upcoming_translate_;
      if (obstacle_on_side) {
        vel_cmd.rotate = follow_side_ * upcoming_rotate_;
      }
    }
  }
  return true;
}
}  // namespace irobot_create_nodes
