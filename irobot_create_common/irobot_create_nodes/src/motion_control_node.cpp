// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Alberto Soragna (asoragna@irobot.com)

#include "irobot_create_nodes/motion_control_node.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace irobot_create_nodes
{
using namespace std::placeholders;
using namespace std::chrono_literals;

MotionControlNode::MotionControlNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("motion_control", options),
  wheels_stop_threshold_(0.5s),
  repeat_print_(1s)
{
  // Declare ROS 2 parameters for robot safety.
  this->declare_safety_parameters();
  // Setup transform tools for frames at time
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  // Services
  power_server_ = this->create_service<irobot_create_msgs::srv::RobotPower>(
    "robot_power",
    std::bind(
      &MotionControlNode::power_off_request, this, std::placeholders::_1,
      std::placeholders::_2));
  e_stop_server_ = this->create_service<irobot_create_msgs::srv::EStop>(
    "e_stop",
    std::bind(
      &MotionControlNode::e_stop_request, this, std::placeholders::_1,
      std::placeholders::_2));

  // Create behaviors scheduler
  scheduler_ = std::make_shared<BehaviorsScheduler>();
  // Create Docking Behavior manager
  docking_behavior_ = std::make_shared<DockingBehavior>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_topics_interface(),
    this->get_node_waitables_interface(),
    scheduler_);
  // Create Reflex Behavior manager
  reflex_behavior_ = std::make_shared<ReflexBehavior>(
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    tf_buffer_,
    scheduler_);
  // Create Wall Follow Behavior manager
  wall_follow_behavior_ = std::make_shared<WallFollowBehavior>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_topics_interface(),
    this->get_node_waitables_interface(),
    scheduler_);
  // Create Drive Goals behaviors
  drive_arc_behavior_ = std::make_shared<DriveArcBehavior>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    scheduler_,
    SAFETY_ON_MAX_SPEED,
    "drive_arc");

  drive_distance_behavior_ = std::make_shared<DriveDistanceBehavior>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    scheduler_,
    SAFETY_ON_MAX_SPEED,
    "drive_distance");

  navigate_to_position_behavior_ = std::make_shared<NavigateToPositionBehavior>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    scheduler_,
    SAFETY_ON_MAX_SPEED,
    GYRO_MAX_ROTATE_SPEED_RAD_S,
    "navigate_to_position");

  rotate_angle_behavior_ = std::make_shared<RotateAngleBehavior>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    scheduler_,
    GYRO_MAX_ROTATE_SPEED_RAD_S,
    "rotate_angle");

  hazard_detection_sub_ =
    this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
    "hazard_detection",
    rclcpp::SensorDataQoS(),
    std::bind(&MotionControlNode::hazard_vector_callback, this, _1));

  // Create subscription to let other applications drive the robot
  teleop_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS(),
    std::bind(&MotionControlNode::commanded_velocity_callback, this, _1));

  odom_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&MotionControlNode::robot_pose_callback, this, _1));

  kidnap_sub_ = this->create_subscription<irobot_create_msgs::msg::KidnapStatus>(
    "kidnap_status", rclcpp::SensorDataQoS(),
    std::bind(&MotionControlNode::kidnap_callback, this, _1));

  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "diffdrive_controller/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS());

  backup_limit_hazard_pub_ = this->create_publisher<irobot_create_msgs::msg::HazardDetection>(
    "_internal/backup_limit", rclcpp::SensorDataQoS().reliable());

  wheel_status_pub_ = this->create_publisher<irobot_create_msgs::msg::WheelStatus>(
    "wheel_status", rclcpp::SensorDataQoS().reliable());
  // Register a callback to handle parameter changes
  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MotionControlNode::set_parameters_callback, this, _1));

  auto_override_print_ts_ = this->now();

  current_state_.pose.setIdentity();
  last_backup_pose_.setIdentity();
  last_teleop_ts_ = this->now();
  // Create timer to periodically execute behaviors and control the robot
  constexpr auto control_period = std::chrono::duration<double>(1.0 / 40.0);
  control_timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration(control_period),
    std::bind(&MotionControlNode::control_robot, this));
  constexpr auto backup_pub_period = std::chrono::duration<double>(1.0 / 62.0);
  backup_limit_timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration(backup_pub_period),
    std::bind(&MotionControlNode::check_backup_buffer, this));
}

void MotionControlNode::declare_safety_parameters()
{
  std::stringstream long_description_string;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = false;
  long_description_string <<
    "Maximum speed of the system in m/s, updated by robot based on safety_override mode. " <<
    "Cannot be updated externally.";
  descriptor.description = long_description_string.str();
  double default_speed = SAFETY_ON_MAX_SPEED;
  auto speed = this->declare_parameter<double>(max_speed_param_name_, default_speed, descriptor);
  if (speed != default_speed) {
    RCLCPP_WARN(
      this->get_logger(), "%s %s \'%s\' parameter",
      "Ignoring user set max speed as parameter is for reporting purposes only.",
      "Max speed is only changed by updating the",
      safety_override_param_name_.c_str());
  }

  long_description_string.str("");
  long_description_string.clear();
  descriptor.read_only = false;
  long_description_string <<
    "Mode to override safety options {\"none\"(default), " <<
    "\"backup_only\"(disable backup limits, no cliff safety driving backwards), " <<
    "\"full\"(disables cliffs completely and allows for higher max drive speed " <<
    "(0.46m/s vs 0.306m/s in other modes))}";
  descriptor.description = long_description_string.str();
  auto val = this->declare_parameter<std::string>(
    safety_override_param_name_, "none",
    descriptor);
  if (val != "none") {
    set_safety_mode(val);
  }
}

rcl_interfaces::msg::SetParametersResult MotionControlNode::set_parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // This is just a stub implementation. Reflexes are permanently disabled
  // and user can't enable them. We reject all changes.
  // See https://github.com/iRobotEducation/create3_sim/issues/65
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = false;

  for (const rclcpp::Parameter & parameter : parameters) {
    if (parameter.get_name() == safety_override_param_name_) {
      // Handle new value for parameters
      result.successful = set_safety_mode(parameter.get_value<std::string>());
      if (!result.successful) {
        result.reason = "Failed to set safety mode, see rosout for more detail";
      }
    } else if (parameter.get_name() == max_speed_param_name_) {
      result.successful = allow_speed_param_change_;
      if (!result.successful) {
        result.reason = "parameter \'" + parameter.get_name() +
          "\' cannot be set externally. Only updated from change in \'" +
          safety_override_param_name_ + "\' parameter";
      }
    }
  }

  return result;
}

void MotionControlNode::control_robot()
{
  if (max_speed_ != this->get_parameter(max_speed_param_name_).get_value<double>()) {
    allow_speed_param_change_ = true;
    this->set_parameter(rclcpp::Parameter(max_speed_param_name_, max_speed_));
    allow_speed_param_change_ = false;
    RCLCPP_INFO(this->get_logger(), "Robot max speed is now %f m/s", max_speed_);
  }
  // Handle behaviors
  BehaviorsScheduler::optional_output_t command;
  bool apply_backup_limits = true;
  if (scheduler_->has_behavior()) {
    apply_backup_limits = scheduler_->apply_backup_limits();
    const std::lock_guard<std::mutex> lock(current_state_mutex_);
    command = scheduler_->run_behavior(current_state_);
    // Reset last teleoperation command if we are executing a behavior
    this->reset_last_teleop_cmd();
  } else {
    // If we don't have any behavior, let's drive the robot using teleoperation command
    // After wheels_stop_threshold_ has passed without receiving new velocity commands
    // we stop the wheels.
    rclcpp::Duration time_diff(rclcpp::Duration::max());
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      time_diff = (this->now() - last_teleop_ts_);
    }
    if (time_diff > wheels_stop_threshold_) {
      this->reset_last_teleop_cmd();
    } else {
      const std::lock_guard<std::mutex> lock(mutex_);
      command = last_teleop_cmd_;
    }
  }

  // Create a null command if we don't have anything.
  // We also disable reflexes because the robot is in an idle state.
  if (!command) {
    command = geometry_msgs::msg::Twist();
  } else if (apply_backup_limits) {
    bound_command_by_limits(*command);
  }
  {
    const std::lock_guard<std::mutex> lock(current_state_mutex_);
    // Update backup buffer
    tf2::Transform diff_tf = last_backup_pose_.inverseTimes(current_state_.pose);
    backup_buffer_ += diff_tf.getOrigin().getX();
    backup_buffer_ = std::min(backup_buffer_, BACKUP_BUFFER_STOP_THRESHOLD);
    last_backup_pose_ = current_state_.pose;
  }
  backup_buffer_low_ = (safety_override_mode_ == SafetyOverrideMode::NONE) &&
    (backup_buffer_ <= BACKUP_BUFFER_WARN_THRESHOLD);
  if (backup_printed_ && !backup_buffer_low_) {
    backup_printed_ = false;
  }
  auto cmd_out_msg = std::make_unique<geometry_msgs::msg::Twist>();
  if (!e_stop_engaged_) {
    *cmd_out_msg = *command;
  }
  cmd_vel_out_pub_->publish(std::move(cmd_out_msg));
  auto wheel_status_msg = std::make_unique<irobot_create_msgs::msg::WheelStatus>();
  wheel_status_msg->header.stamp = this->now();
  wheel_status_msg->header.frame_id = base_frame_;
  wheel_status_msg->wheels_enabled = !e_stop_engaged_;
  wheel_status_pub_->publish(std::move(wheel_status_msg));
}

void MotionControlNode::check_backup_buffer()
{
  if (backup_buffer_low_) {
    auto backup_limit_msg = std::make_unique<irobot_create_msgs::msg::HazardDetection>();
    backup_limit_msg->header.frame_id = base_frame_;
    backup_limit_msg->header.stamp = this->now();
    backup_limit_msg->type = irobot_create_msgs::msg::HazardDetection::BACKUP_LIMIT;
    backup_limit_hazard_pub_->publish(std::move(backup_limit_msg));
  }
}

bool MotionControlNode::set_safety_mode(const std::string & safety_mode)
{
  std::map<std::string, SafetyOverrideMode>::const_iterator safety_mode_it = safety_to_str_.find(
    safety_mode);
  if (safety_mode_it != safety_to_str_.end()) {
    safety_override_mode_ = safety_mode_it->second;
    switch (safety_override_mode_) {
      case SafetyOverrideMode::NONE:
      case SafetyOverrideMode::BACKUP_ONLY:
        max_speed_ = SAFETY_ON_MAX_SPEED;
        break;
      case SafetyOverrideMode::FULL:
        max_speed_ = SAFETY_OFF_MAX_SPEED;
        break;
    }
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Tried to set invalid safety mode %s, options are {\'none\',\'backup_only\',\'full\'}",
      safety_mode.c_str());
    return false;
  }
  return true;
}

void MotionControlNode::hazard_vector_callback(
  irobot_create_msgs::msg::HazardDetectionVector::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(current_state_mutex_);
  current_state_.hazards = *msg;
  reflex_behavior_->update_hazards(current_state_);
}

void MotionControlNode::commanded_velocity_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  if (scheduler_->has_behavior()) {
    const auto time_now = this->now();
    if (time_now - auto_override_print_ts_ > repeat_print_) {
      auto_override_print_ts_ = time_now;
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring velocities commanded while an autonomous behavior is running!");
    }
    return;
  }

  const std::lock_guard<std::mutex> lock(mutex_);

  last_teleop_cmd_ = *msg;
  last_teleop_ts_ = this->now();
}

void MotionControlNode::robot_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(current_state_mutex_);
  tf2::convert(msg->pose.pose, current_state_.pose);
}

void MotionControlNode::kidnap_callback(irobot_create_msgs::msg::KidnapStatus::ConstSharedPtr msg)
{
  if (!msg->is_kidnapped && last_kidnap_) {
    backup_buffer_ = 0.0;
  }
  last_kidnap_ = msg->is_kidnapped;
}

void MotionControlNode::reset_last_teleop_cmd()
{
  const std::lock_guard<std::mutex> lock(mutex_);

  last_teleop_cmd_ = get_default_velocity_cmd();
  last_teleop_ts_ = this->now();
}

void MotionControlNode::bound_command_by_limits(geometry_msgs::msg::Twist & cmd)
{
  if (std::abs(cmd.angular.z) > GYRO_MAX_ROTATE_SPEED_RAD_S) {
    cmd.angular.z = std::copysign(GYRO_MAX_ROTATE_SPEED_RAD_S, cmd.angular.z);
  }
  if (safety_override_mode_ == SafetyOverrideMode::NONE &&
    backup_buffer_ <= 0.0 &&
    cmd.linear.x < 0.0)
  {
    // Robot has run out of room to backup
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    const auto time_now = this->now();
    if (!backup_printed_) {
      backup_printed_ = true;
      RCLCPP_WARN(
        this->get_logger(),
        "Reached backup limit! Stop Driving robot backward or disable from %s parameter!",
        safety_override_param_name_.c_str());
    }
  } else {
    double left_vel = cmd.linear.x - cmd.angular.z * wheel_base_ / 2.0;
    double right_vel = cmd.angular.z * wheel_base_ + left_vel;
    const double max_vel = std::max(std::abs(left_vel), std::abs(right_vel));
    if (max_vel > 0 && max_vel > max_speed_) {
      const double scale = max_speed_ / max_vel;
      // Scale velocity to bring them in limits
      left_vel *= scale;
      right_vel *= scale;
      // Convert back to cartesian
      cmd.linear.x = (left_vel + right_vel) / 2.0;
      cmd.angular.z = (right_vel - left_vel) / wheel_base_;
    }
  }
}

void MotionControlNode::e_stop_request(
  const irobot_create_msgs::srv::EStop::Request::SharedPtr request,
  irobot_create_msgs::srv::EStop::Response::SharedPtr response)
{
  auto set_response_w_msg = [this, response](const std::string msg, ResponseStatus result) {
      response->message = msg;
      response->success = static_cast<bool>(result);
      const std::string log_prefix = "E-Stop request:";
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "%s %s", log_prefix.c_str(), response->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s %s", log_prefix.c_str(), response->message.c_str());
      }
    };

  if (request->e_stop_on) {
    e_stop_engaged_ = true;
    set_response_w_msg("Set system E-Stop on, cutting motor power", SUCCESS);
  } else {
    e_stop_engaged_ = false;
    set_response_w_msg("Set system E-Stop off, enabling motor power", SUCCESS);
  }
}

void MotionControlNode::power_off_request(
  const irobot_create_msgs::srv::RobotPower::Request::SharedPtr request,
  irobot_create_msgs::srv::RobotPower::Response::SharedPtr response)
{
  auto set_response_w_msg = [this, response](const std::string msg, ResponseStatus result) {
      response->message = msg;
      response->success = static_cast<bool>(result);
      const std::string log_prefix = "Power off request:";
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "%s %s", log_prefix.c_str(), response->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s %s", log_prefix.c_str(), response->message.c_str());
      }
    };

  (void)request;
  set_response_w_msg("Set system power off failed (not supported in sim)", FAILURE);
}

}  // namespace irobot_create_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(irobot_create_nodes::MotionControlNode)
