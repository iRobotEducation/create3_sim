// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#ifndef IROBOT_CREATE_NODES__ROBOT_STATE_HPP_
#define IROBOT_CREATE_NODES__ROBOT_STATE_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/stop_status.hpp"
#include "irobot_create_msgs/msg/wheel_vels.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace irobot_create_nodes
{

class RobotState : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit RobotState(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callback functions
  void dock_callback(irobot_create_msgs::msg::DockStatus::SharedPtr msg);
  void stop_callback(nav_msgs::msg::Odometry::SharedPtr msg);

  double get_docked_charge_percentage(const rclcpp::Time & at_time);
  double get_undocked_charge_percentage(const rclcpp::Time & at_time);

  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr battery_state_timer_;
  rclcpp::TimerBase::SharedPtr stop_status_timer_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_{nullptr};
  rclcpp::Publisher<irobot_create_msgs::msg::StopStatus>::SharedPtr stop_status_publisher_{nullptr};

  // Subscribers
  rclcpp::Subscription<irobot_create_msgs::msg::DockStatus>::SharedPtr dock_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr stop_status_subscription_;

  // Topic to publish battery state to
  std::string battery_state_publisher_topic_;
  // Topic to publish stop status to
  std::string stop_status_publisher_topic_;

  // Topic to subscribe to dock
  std::string dock_subscription_topic_;
  // Topic to subscribe to wheel vels vector
  std::string wheel_vels_subscription_topic_;

  // Message to store the stop status
  irobot_create_msgs::msg::StopStatus stop_status_msg_;
  // Message to store the battery state
  sensor_msgs::msg::BatteryState battery_state_msg_;

  double linear_velocity_tolerance{std::numeric_limits<double>::max()};
  double angular_velocity_tolerance{std::numeric_limits<double>::max()};
  std::atomic<bool> is_stopped_{true};
  std::atomic<bool> is_docked_{false};
  // Fields to help populate battery_state
  const double idle_current_ {-0.404};
  const double drive_current_ {-0.526};
  const double charge_current_ {0.9};
  const double full_charge_current_ {-0.15};
  const double full_batter_state_voltage_ {16.474};
  std::mutex battery_charge_timings_mutex_;
  rclcpp::Time transitioned_to_stopped_;
  rclcpp::Time transitioned_to_drive_;
  rclcpp::Time transitioned_to_undocked_;
  rclcpp::Time transitioned_to_docked_;
  rclcpp::Duration off_dock_drive_time_ {std::chrono::seconds(0)};
  rclcpp::Duration off_dock_idle_time_ {std::chrono::seconds(0)};
  std::atomic<double> last_docked_charge_percentage_ {1.0};
  const double charge_rate_percent_per_second_ {0.00012658291225191914};
  const double driving_drain_percentage_per_second {0.00008875};
  const double idle_drain_percentage_per_second {0.00005634};
  const double battery_voltage_range_high_ {6};
  const double battery_voltage_range_middle_ {3.27};
  const double battery_capacity_ {2.046};
  const double battery_default_temp_ {27.0};

  double battery_full_charge_percentage;
  double battery_high_percentage_limit;
  double undocked_charge_limit;

  const std::string base_frame_ {"base_link"};
};

}  // namespace irobot_create_nodes

#endif  // IROBOT_CREATE_NODES__ROBOT_STATE_HPP_
