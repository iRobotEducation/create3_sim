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
//
// @author Lola Segura (lsegura@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_
#define IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_

#include <irobot_create_msgs/msg/button.hpp>
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
  void kidnap_callback(irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg);
  void stop_callback(nav_msgs::msg::Odometry::SharedPtr msg);
  void lightring_callback(irobot_create_msgs::msg::LightringLeds::SharedPtr msg);

protected:
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

  float linear_velocity_tolerance{0};
  float angular_velocity_tolerance{0};
};

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_
