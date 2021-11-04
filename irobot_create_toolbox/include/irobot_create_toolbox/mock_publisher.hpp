// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_
#define IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_

#include <irobot_create_msgs/msg/button.hpp>
#include <irobot_create_msgs/msg/dock.hpp>
#include <irobot_create_msgs/msg/hazard_detection.hpp>
#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <irobot_create_msgs/msg/interface_buttons.hpp>
#include <irobot_create_msgs/msg/kidnap_status.hpp>
#include <irobot_create_msgs/msg/led_color.hpp>
#include <irobot_create_msgs/msg/lightring_leds.hpp>
#include <irobot_create_msgs/msg/slip_status.hpp>
#include <irobot_create_msgs/msg/stop_status.hpp>
#include <irobot_create_msgs/msg/wheel_vels.hpp>
#include <irobot_create_toolbox/parameter_helper.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>


namespace irobot_create_toolbox
{
class MockPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  MockPublisher();

  // Callback functions
  void dock_callback(irobot_create_msgs::msg::Dock::SharedPtr msg);
  void kidnap_callback(irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg);
  void stop_callback(nav_msgs::msg::Odometry::SharedPtr msg);
  void lightring_callback(irobot_create_msgs::msg::LightringLeds::SharedPtr msg);

protected:
  double get_docked_charge_percentage(const rclcpp::Time & at_time);
  double get_undocked_charge_percentage(const rclcpp::Time & at_time);
  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr buttons_timer_;
  rclcpp::TimerBase::SharedPtr slip_status_timer_;
  rclcpp::TimerBase::SharedPtr kidnap_status_timer_;
  rclcpp::TimerBase::SharedPtr battery_state_timer_;
  rclcpp::TimerBase::SharedPtr stop_status_timer_;

  // Publishers
  std::shared_ptr<
    rclcpp::Publisher<irobot_create_msgs::msg::InterfaceButtons>> buttons_publisher_{nullptr};
  rclcpp::Publisher<irobot_create_msgs::msg::SlipStatus>::SharedPtr slip_status_publisher_{nullptr};
  rclcpp::Publisher<irobot_create_msgs::msg::KidnapStatus>::SharedPtr
    kidnap_status_publisher_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_{nullptr};
  rclcpp::Publisher<irobot_create_msgs::msg::StopStatus>::SharedPtr stop_status_publisher_{nullptr};

  // Subscribers
  rclcpp::Subscription<irobot_create_msgs::msg::Dock>::SharedPtr dock_subscription_;
  rclcpp::Subscription<
    irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr kidnap_status_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr stop_status_subscription_;
  rclcpp::Subscription<irobot_create_msgs::msg::LightringLeds>::SharedPtr lightring_subscription_;


  // Topic to publish interface buttons to
  std::string buttons_publisher_topic_;
  // Topic to publish slip status to
  std::string slip_status_publisher_topic_;
  // Topic to publish kidnap status to
  std::string kidnap_status_publisher_topic_;
  // Topic to publish battery state to
  std::string battery_state_publisher_topic_;
  // Topic to publish stop status to
  std::string stop_status_publisher_topic_;

  // Topic to subscribe to dock
  std::string dock_subscription_topic_;
  // Topic to subscribe to hazard detection vector
  std::string hazard_subscription_topic_;
  // Topic to subscribe to wheel vels vector
  std::string wheel_vels_subscription_topic_;
  // Topic to subscribe to light ring vector
  std::string lightring_subscription_topic_;

  // Message to store the interface buttons
  irobot_create_msgs::msg::InterfaceButtons buttons_msg_;
  // Message to store the slip status
  irobot_create_msgs::msg::SlipStatus slip_status_msg_;
  // Message to store the kidnap status
  irobot_create_msgs::msg::KidnapStatus kidnap_status_msg_;
  // Message to store the battery state
  sensor_msgs::msg::BatteryState battery_state_msg_;
  // Message to store the stop status
  irobot_create_msgs::msg::StopStatus stop_status_msg_;

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
  const std::string base_frame_ {"base_link"};
};

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_
