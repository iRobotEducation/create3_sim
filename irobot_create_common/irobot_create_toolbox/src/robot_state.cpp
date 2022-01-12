// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#include <irobot_create_toolbox/robot_state.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace irobot_create_toolbox
{
RobotState::RobotState()
: rclcpp::Node("robot_state")
{
  transitioned_to_docked_ = this->now();
  transitioned_to_undocked_ = transitioned_to_docked_;
  transitioned_to_stopped_ = transitioned_to_docked_;
  transitioned_to_drive_ = transitioned_to_docked_;

  // Topic parameter to publish battery state to
  battery_state_publisher_topic_ =
    declare_and_get_parameter<std::string>("battery_state_topic", this);
  // Topic parameter to publish stop status to
  stop_status_publisher_topic_ = declare_and_get_parameter<std::string>("stop_status_topic", this);

  // Subscriber topics
  dock_subscription_topic_ = declare_and_get_parameter<std::string>("dock_topic", this);
  wheel_vels_subscription_topic_ = declare_and_get_parameter<std::string>("wheel_vels_topic", this);

  // Publish rate parameters
  const double battery_state_publish_rate =
    declare_and_get_parameter<double>("battery_state_publish_rate", this);  // Hz

  // Sets velocity tolerances
  linear_velocity_tolerance = declare_and_get_parameter<float>("linear_velocity_tolerance", this);
  angular_velocity_tolerance = declare_and_get_parameter<float>("angular_velocity_tolerance", this);

  // Define battery state publisher
  battery_state_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
    battery_state_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << battery_state_publisher_topic_);

  // Define battery parameters
  battery_full_charge_percentage =
    declare_and_get_parameter<float>("full_charge_percentage", this);
  battery_high_percentage_limit =
    declare_and_get_parameter<float>("battery_high_percentage", this);

  // Subscription to the hazard detection vector
  dock_subscription_ = create_subscription<irobot_create_msgs::msg::Dock>(
    dock_subscription_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RobotState::dock_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << dock_subscription_topic_);

  // Define stop status publisher
  stop_status_publisher_ = create_publisher<irobot_create_msgs::msg::StopStatus>(
    stop_status_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << stop_status_publisher_topic_);

  // Subscription to the stop status
  stop_status_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    wheel_vels_subscription_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RobotState::stop_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << wheel_vels_subscription_topic_);

  // Set stop status header
  stop_status_msg_.header.frame_id = base_frame_;

  undocked_charge_limit =
    declare_and_get_parameter<float>("undocked_charge_limit", this);
  battery_state_timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration(std::chrono::duration<double>(1 / battery_state_publish_rate)), [this]() {
      // Set header timestamp.
      rclcpp::Time current_time = now();
      this->battery_state_msg_.header.stamp = current_time;

      if (is_docked_) {
        this->battery_state_msg_.percentage = get_docked_charge_percentage(current_time);
        if (this->battery_state_msg_.percentage == battery_full_charge_percentage) {
          this->battery_state_msg_.current = full_charge_current_;
        } else {
          this->battery_state_msg_.current = charge_current_;
        }
      } else {
        this->battery_state_msg_.percentage = get_undocked_charge_percentage(current_time);
        if (is_stopped_) {
          this->battery_state_msg_.current = idle_current_;
        } else {
          this->battery_state_msg_.current = drive_current_;
        }
      }
      // Approximate voltage from extrapolation of observed data
      double voltage_scale_factor = battery_voltage_range_middle_;
      if (this->battery_state_msg_.percentage > battery_high_percentage_limit) {
        voltage_scale_factor = battery_voltage_range_high_;
      }
      this->battery_state_msg_.voltage = full_batter_state_voltage_ - (voltage_scale_factor *
      (battery_full_charge_percentage - this->battery_state_msg_.percentage));
      this->battery_state_msg_.charge = this->battery_state_msg_.capacity *
      this->battery_state_msg_.percentage;

      // Publish topics
      this->battery_state_publisher_->publish(this->battery_state_msg_);
    });

  // The battery percentage goes from zero to one, one meaning that the battery is full.
  battery_state_msg_.percentage = 1;
  // Set battery state header
  battery_state_msg_.header.frame_id = base_frame_;
  battery_state_msg_.capacity = battery_capacity_;
  battery_state_msg_.design_capacity = battery_capacity_;
  battery_state_msg_.present = true;
  battery_state_msg_.temperature = battery_default_temp_;
}

double RobotState::get_docked_charge_percentage(const rclcpp::Time & at_time)
{
  const std::lock_guard<std::mutex> lock(battery_charge_timings_mutex_);
  rclcpp::Duration time_docked = at_time - transitioned_to_docked_;
  // current charge is last charged + charging rate * time
  double docked_charge = last_docked_charge_percentage_ +
    (time_docked.seconds() * charge_rate_percent_per_second_);
  docked_charge = std::min(docked_charge, 1.0);
  return docked_charge;
}

double RobotState::get_undocked_charge_percentage(const rclcpp::Time & at_time)
{
  const std::lock_guard<std::mutex> lock(battery_charge_timings_mutex_);
  double drain_percentage = 0.0;
  if (is_stopped_) {
    rclcpp::Duration time_stopped = at_time - transitioned_to_stopped_;
    drain_percentage += time_stopped.seconds() * idle_drain_percentage_per_second;
  } else {
    rclcpp::Duration time_moving = at_time - transitioned_to_drive_;
    drain_percentage += time_moving.seconds() * driving_drain_percentage_per_second;
  }
  drain_percentage += off_dock_drive_time_.seconds() * driving_drain_percentage_per_second;
  drain_percentage += off_dock_idle_time_.seconds() * idle_drain_percentage_per_second;
  double undocked_charge = last_docked_charge_percentage_ - drain_percentage;
  if (undocked_charge < undocked_charge_limit) {
    // Battery will never let itself get to 0
    undocked_charge = undocked_charge_limit;
  }
  return undocked_charge;
}

void RobotState::dock_callback(irobot_create_msgs::msg::Dock::SharedPtr msg)
{
  if (!is_docked_ && msg->is_docked) {
    rclcpp::Time current_time = this->now();
    // Get charge percentage we are entering dock with
    last_docked_charge_percentage_ = get_undocked_charge_percentage(current_time);
    const std::lock_guard<std::mutex> lock(battery_charge_timings_mutex_);
    transitioned_to_docked_ = current_time;
  } else if (is_docked_ && !msg->is_docked) {
    // Get charge percentage we are leaving dock with
    rclcpp::Time current_time = this->now();
    last_docked_charge_percentage_ = get_docked_charge_percentage(current_time);
    // Reset timers
    const std::lock_guard<std::mutex> lock(battery_charge_timings_mutex_);
    transitioned_to_undocked_ = current_time;
    off_dock_drive_time_ = rclcpp::Duration(std::chrono::seconds(0));
    off_dock_idle_time_ = rclcpp::Duration(std::chrono::seconds(0));
  }
  is_docked_ = msg->is_docked;
}

void RobotState::stop_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double linear_velocity = msg->twist.twist.linear.x;
  const double angular_velocity = msg->twist.twist.angular.z;

  const bool cond_linear_velocity = abs(linear_velocity) < linear_velocity_tolerance;
  const bool cond_angular_velocity = abs(angular_velocity) < angular_velocity_tolerance;

  // Set header timestamp.
  stop_status_msg_.header.stamp = now();
  // Set stop status. The robot is stopped when both linear and angular velocity
  // are almost zero.
  stop_status_msg_.is_stopped = cond_linear_velocity && cond_angular_velocity;
  if (!is_stopped_ && stop_status_msg_.is_stopped) {
    const std::lock_guard<std::mutex> lock(battery_charge_timings_mutex_);
    transitioned_to_stopped_ = this->now();
    // book drive time
    off_dock_drive_time_ = off_dock_drive_time_ + transitioned_to_stopped_ -
      transitioned_to_drive_;
  } else if (is_stopped_ && !stop_status_msg_.is_stopped) {
    const std::lock_guard<std::mutex> lock(battery_charge_timings_mutex_);
    transitioned_to_drive_ = this->now();
    // book stopped time
    off_dock_drive_time_ = off_dock_drive_time_ + transitioned_to_drive_ -
      transitioned_to_stopped_;
  }
  is_stopped_ = stop_status_msg_.is_stopped;

  // Publish topics
  stop_status_publisher_->publish(stop_status_msg_);
}
}  // namespace irobot_create_toolbox
