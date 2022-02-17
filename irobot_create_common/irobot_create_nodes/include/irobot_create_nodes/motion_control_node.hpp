// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Alberto Soragna (asoragna@irobot.com)

#ifndef IROBOT_CREATE_NODES__MOTION_CONTROL_NODE_HPP_
#define IROBOT_CREATE_NODES__MOTION_CONTROL_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/msg/kidnap_status.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "irobot_create_msgs/msg/wheel_status.hpp"
#include "irobot_create_msgs/srv/e_stop.hpp"
#include "irobot_create_msgs/srv/robot_power.hpp"
#include "irobot_create_nodes/motion_control/docking_behavior.hpp"
#include "irobot_create_nodes/motion_control/drive_goal_behaviors.hpp"
#include "irobot_create_nodes/motion_control/reflex_behavior.hpp"
#include "irobot_create_nodes/motion_control/wall_follow_behavior.hpp"
#include "rclcpp/rclcpp.hpp"

namespace irobot_create_nodes
{

class MotionControlNode : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit MotionControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// \brief Function to centralize velocity command for system
  void control_robot();
  /// \brief Publish limit if backup buffer low
  void check_backup_buffer();
  /// \brief Clear any cached teleop command
  void reset_last_teleop_cmd();

  /// \brief Helper function to declare ROS 2 safety parameters
  void declare_safety_parameters();

  /// \brief Helper function to validate changes to parameters
  rcl_interfaces::msg::SetParametersResult set_parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);
  /// \brief Callback for e_stop service
  void e_stop_request(
    const irobot_create_msgs::srv::EStop::Request::SharedPtr request,
    irobot_create_msgs::srv::EStop::Response::SharedPtr response);
  /// \brief Callback for power_off service
  void power_off_request(
    const irobot_create_msgs::srv::RobotPower::Request::SharedPtr request,
    irobot_create_msgs::srv::RobotPower::Response::SharedPtr response);

  enum class SafetyOverrideMode
  {
    NONE,
    BACKUP_ONLY,
    FULL
  };
  const std::map<std::string, SafetyOverrideMode> safety_to_str_ {
    {"none", SafetyOverrideMode::NONE},
    {"backup_only", SafetyOverrideMode::BACKUP_ONLY},
    {"full", SafetyOverrideMode::FULL}
  };
  /// \brief Name of parameter for setting system safety mode,
  //     which dictates cliff safety and max speed
  const std::string safety_override_param_name_{"safety_override"};
  /// \brief Name of parameter for reporting system's max speed,
  //     this will be changed by system based on safety_override setting
  const std::string max_speed_param_name_{"max_speed"};

  bool set_safety_mode(const std::string & safety_mode);
  /// \brief Storage for custom parameter validation callbacks
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_{
    nullptr};
  /// \brief Callback for current hazards seen by robot
  void hazard_vector_callback(irobot_create_msgs::msg::HazardDetectionVector::ConstSharedPtr msg);

  /// \brief Callback for new velocity commands
  void commanded_velocity_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg);

  /// \brief Callback for robot odometry
  void robot_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /// \brief Callback for kidnap status
  void kidnap_callback(irobot_create_msgs::msg::KidnapStatus::ConstSharedPtr msg);

  /// \brief Given command, bound by max speed, checking each wheel
  void bound_command_by_limits(geometry_msgs::msg::Twist & cmd);

  /// These robot services live in ui-mgr node on robot, but here for convenience
  enum ResponseStatus : bool
  {
    SUCCESS = true,
    FAILURE = false,
  };
  /// \brief Service to turn off robot power, this won't do anything in sim
  rclcpp::Service<irobot_create_msgs::srv::RobotPower>::SharedPtr power_server_;
  /// \brief Service to e_stop robot, this will prevent robot from commanding velocity
  rclcpp::Service<irobot_create_msgs::srv::EStop>::SharedPtr e_stop_server_;

  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr
    hazard_detection_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_pose_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::KidnapStatus>::SharedPtr kidnap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr backup_limit_hazard_pub_;
  rclcpp::Publisher<irobot_create_msgs::msg::WheelStatus>::SharedPtr wheel_status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_ {nullptr};
  rclcpp::TimerBase::SharedPtr backup_limit_timer_ {nullptr};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ {nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};

  std::shared_ptr<BehaviorsScheduler> scheduler_ {nullptr};
  std::shared_ptr<DockingBehavior> docking_behavior_ {nullptr};
  std::shared_ptr<DriveArcBehavior> drive_arc_behavior_;
  std::shared_ptr<DriveDistanceBehavior> drive_distance_behavior_;
  std::shared_ptr<NavigateToPositionBehavior> navigate_to_position_behavior_;
  std::shared_ptr<ReflexBehavior> reflex_behavior_ {nullptr};
  std::shared_ptr<RotateAngleBehavior> rotate_angle_behavior_;
  std::shared_ptr<WallFollowBehavior> wall_follow_behavior_ {nullptr};

  std::mutex mutex_;
  geometry_msgs::msg::Twist last_teleop_cmd_;
  rclcpp::Time last_teleop_ts_;
  rclcpp::Duration wheels_stop_threshold_;
  std::atomic<bool> allow_speed_param_change_ {false};
  std::atomic<SafetyOverrideMode> safety_override_mode_ {SafetyOverrideMode::NONE};
  const double GYRO_MAX_ROTATE_SPEED_RAD_S {1.9};
  const double SAFETY_ON_MAX_SPEED {0.306};
  const double SAFETY_OFF_MAX_SPEED {0.46};
  double max_speed_ {SAFETY_ON_MAX_SPEED};
  const double wheel_base_ {0.233};
  double backup_buffer_ {0.0};
  std::mutex robot_pose_mutex_;
  tf2::Transform last_backup_pose_;
  std::atomic<bool> last_kidnap_ {false};
  rclcpp::Time auto_override_print_ts_;
  const rclcpp::Duration repeat_print_;
  std::atomic<bool> backup_printed_{false};
  const std::string base_frame_ {"base_link"};
  std::atomic<bool> backup_buffer_low_ {false};
  const double BACKUP_BUFFER_WARN_THRESHOLD {0.05};
  const double BACKUP_BUFFER_STOP_THRESHOLD {0.15};
  std::atomic<bool> e_stop_engaged_ {false};
  std::mutex current_state_mutex_;
  RobotState current_state_;
};

}  // namespace irobot_create_nodes

#endif  // IROBOT_CREATE_NODES__MOTION_CONTROL_NODE_HPP_
