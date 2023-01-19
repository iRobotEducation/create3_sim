// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Justin Kearns (jkearns@irobot.com)

#ifndef IROBOT_CREATE_NODES__MOTION_CONTROL__WALL_FOLLOW_STATES_HPP_
#define IROBOT_CREATE_NODES__MOTION_CONTROL__WALL_FOLLOW_STATES_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "irobot_create_msgs/action/wall_follow.hpp"
#include "irobot_create_msgs/msg/ir_intensity_vector.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace irobot_create_nodes
{

struct WFVelocityCommand
{
  double translate;
  double rotate;
};
enum class WallFollowStateID
{
  NONE,
  SPIRAL,
  ALIGNED_SERVO,
  OBSTACLE_IN_FRONT,
};
/// \brief Base Class for Wall Follow States in State Machine
class WallFollowState
{
public:
  virtual bool get_next_velocity(
    const tf2::Transform & robot_pose,
    const irobot_create_msgs::msg::IrIntensityVector & ir_intensity,
    const std::vector<std::string> & active_hazard_frames,
    WFVelocityCommand & vel_cmd) = 0;
  virtual WallFollowStateID get_id() = 0;
  virtual bool is_engaged() = 0;
};

/// \brief Execute Spiral Pattern increasing in size to engage wall
class SpiralToEngageWall : public WallFollowState
{
public:
  SpiralToEngageWall(const tf2::Transform & robot_pose, int8_t follow_side)
  : start_orientation_{tf2::getYaw(robot_pose.getRotation())},
    follow_side_{follow_side} {}

  bool get_next_velocity(
    const tf2::Transform & robot_pose,
    const irobot_create_msgs::msg::IrIntensityVector & ir_intensity,
    const std::vector<std::string> & active_hazard_frames,
    WFVelocityCommand & vel_cmd) override;
  WallFollowStateID get_id() override {return WallFollowStateID::SPIRAL;}
  bool is_engaged() override {return found_obstruction_;}

private:
  const double start_orientation_;
  const int8_t follow_side_;
  std::atomic<bool> found_obstruction_ {false};
  const double spiral_trans_vel_ {0.15};
  const double spiral_start_rot_vel_ {M_PI / 4.0};
  const double converged_angle_threshold_ {M_PI / 60.0};
  unsigned int spiral_rot_divisor_ {1};
  bool check_orientation_converge_ {false};
};
/// \brief Servo off of side IR sensor to maintain desired value
class AlignedIRServo : public WallFollowState
{
public:
  AlignedIRServo(const rclcpp::Time start_time, int8_t follow_side)
  : last_seen_obs_time_{start_time},
    follow_side_{follow_side},
    follow_frame_{follow_side == irobot_create_msgs::action::WallFollow::Goal::FOLLOW_LEFT ?
      "ir_intensity_side_left" : "ir_intensity_right"}
  {}

  bool get_next_velocity(
    const tf2::Transform & robot_pose,
    const irobot_create_msgs::msg::IrIntensityVector & ir_intensity,
    const std::vector<std::string> & active_hazard_frames,
    WFVelocityCommand & vel_cmd) override;
  WallFollowStateID get_id() override {return WallFollowStateID::ALIGNED_SERVO;}
  bool is_engaged() override {return found_obstruction_;}

private:
  rclcpp::Time last_seen_obs_time_;
  const int8_t follow_side_;
  const std::string follow_frame_;
  std::atomic<bool> found_obstruction_ {false};
  const double align_trans_vel_ {0.15};
  const int16_t min_sees_wall_ir_intensity_ {50};
  const int16_t sees_wall_desired_ir_intensity_ {700};
  const double ir_diff_to_rotate_scale_ {0.001};
  const double max_rotate_ {M_PI / 8.0};
  const rclcpp::Duration drive_after_lost_time_ {std::chrono::seconds(1)};
};

/// \brief Rotate away from obstacle in front of robot
class ObstacleInFront : public WallFollowState
{
public:
  ObstacleInFront(const tf2::Transform & robot_pose, int8_t follow_side)
  : start_pose_angle_{tf2::getYaw(robot_pose.getRotation())},
    start_pose_position_{robot_pose.getOrigin()},
    follow_side_{follow_side},
    side_ir_frames_{(follow_side == irobot_create_msgs::action::WallFollow::Goal::FOLLOW_LEFT) ?
      std::vector<std::string>({"ir_intensity_side_left"}) :
      std::vector<std::string>({"ir_intensity_right"})},
    front_side_ir_frames_{(follow_side ==
      irobot_create_msgs::action::WallFollow::Goal::FOLLOW_LEFT) ?
      std::vector<std::string>({"ir_intensity_front_left", "ir_intensity_left"}) :
      std::vector<std::string>({"ir_intensity_front_right"})},
    opposite_ir_frames_{(follow_side == irobot_create_msgs::action::WallFollow::Goal::FOLLOW_LEFT) ?
      std::vector<std::string>({"ir_intensity_front_right"}) :
      std::vector<std::string>({"ir_intensity_front_left", "ir_intensity_left"})}
  {}

  bool get_next_velocity(
    const tf2::Transform & robot_pose,
    const irobot_create_msgs::msg::IrIntensityVector & ir_intensity,
    const std::vector<std::string> & active_hazard_frames,
    WFVelocityCommand & vel_cmd) override;
  WallFollowStateID get_id() override {return WallFollowStateID::OBSTACLE_IN_FRONT;}
  bool is_engaged() override {return aligned_;}

private:
  double start_pose_angle_;
  tf2::Vector3 start_pose_position_;
  const int8_t follow_side_;
  std::atomic<bool> aligned_ {false};
  const double backup_translate_ {-0.1};
  const double backup_rotate_ {-0.1};    // When multiplied by RIGHT (-1) this will turn positive
  const double rotate_in_place_ {-M_PI / 8.0};  // When multiplied by RIGHT this will turn positive
  const double upcoming_translate_ {0.05};
  const double upcoming_rotate_ {-M_PI / 4.0};
  const std::vector<std::string> front_ir_frames_ {"ir_intensity_front_center_left",
    "ir_intensity_front_center_right", "ir_intensity_front_left"};
  const std::vector<std::string> side_ir_frames_;
  const std::vector<std::string> front_side_ir_frames_;
  const std::vector<std::string> opposite_ir_frames_;
  const int16_t min_sees_wall_front_ir_intensity_ {20};
  const int16_t min_sees_wall_side_ir_intensity_ {20};
  const double min_linear_traversal_ {0.2};
  const double min_angular_traversal_ {M_PI / 8.0};
};

/// \brief Manage state machine for transition between wall follow behaviors
class WallFollowStateManager : public WallFollowState
{
public:
  explicit WallFollowStateManager(const rclcpp::Logger & logger)
  : logger_{logger}
  {}

  void initialize(int8_t follow_side);

  bool get_next_velocity(
    const tf2::Transform & robot_pose,
    const irobot_create_msgs::msg::IrIntensityVector & ir_intensity,
    const std::vector<std::string> & active_hazard_frames,
    WFVelocityCommand & vel_cmd) override;
  WallFollowStateID get_id() override {return WallFollowStateID::NONE;}
  bool is_engaged() override {return wall_engaged_;}

private:
  const std::string odom_frame_ {"odom"};
  std::string base_frame_ {"base_link"};
  rclcpp::Logger logger_;
  std::atomic<int8_t> follow_side_{0};
  std::atomic<bool> wall_engaged_{false};
  std::shared_ptr<WallFollowState> current_follow_mode_ {nullptr};
};
}  // namespace irobot_create_nodes

#endif  // IROBOT_CREATE_NODES__MOTION_CONTROL__WALL_FOLLOW_STATES_HPP_
