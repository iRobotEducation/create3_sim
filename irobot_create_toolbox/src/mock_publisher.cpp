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

#include <irobot_create_toolbox/mock_publisher.hpp>

#include <string>
#include <vector>

namespace irobot_create_toolbox
{
MockPublisher::MockPublisher()
: rclcpp::Node("mock_publisher_node")
{
  // Topic parameter to publish buttons to
  buttons_publisher_topic_ = declare_and_get_parameter<std::string>("button_topic", this);
  // Topic parameter to publish slip status to
  slip_status_publisher_topic_ = declare_and_get_parameter<std::string>("slip_status_topic", this);
  // Topic parameter to publish kidnap status to
  kidnap_status_publisher_topic_ =
    declare_and_get_parameter<std::string>("kidnap_status_topic", this);
  // Topic parameter to publish battery state to
  battery_state_publisher_topic_ =
    declare_and_get_parameter<std::string>("battery_state_topic", this);
  // Topic parameter to publish stop status to
  stop_status_publisher_topic_ = declare_and_get_parameter<std::string>("stop_status_topic", this);

  // Subscriber topics
  hazard_subscription_topic_ = declare_and_get_parameter<std::string>("hazard_topic", this);
  wheel_vels_subscription_topic_ = declare_and_get_parameter<std::string>("wheel_vels_topic", this);
  lightring_subscription_topic_ = declare_and_get_parameter<std::string>("lightring_topic", this);

  // Publish rate parameters
  const double buttons_publish_rate =
    declare_and_get_parameter<double>("buttons_publish_rate", this);  // Hz
  const double slip_status_publish_rate =
    declare_and_get_parameter<double>("slip_status_publish_rate", this);  // Hz
  const double battery_state_publish_rate =
    declare_and_get_parameter<double>("battery_state_publish_rate", this);  // Hz

  // Sets the velocity tolerance
  linear_velocity_tolerance = declare_and_get_parameter<float>("linear_velocity_tolerance", this);
  angular_velocity_tolerance = declare_and_get_parameter<float>("angular_velocity_tolerance", this);

  // Define buttons publisher
  buttons_publisher_ = create_publisher<irobot_create_msgs::msg::InterfaceButtons>(
    buttons_publisher_topic_, rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised mocked topic: " << buttons_publisher_topic_);

  // Define slip status publisher
  slip_status_publisher_ = create_publisher<irobot_create_msgs::msg::SlipStatus>(
    slip_status_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised mocked topic: " << slip_status_publisher_topic_);

  // Define kidnap status publisher
  kidnap_status_publisher_ = create_publisher<irobot_create_msgs::msg::KidnapStatus>(
    kidnap_status_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised mocked topic: " << kidnap_status_publisher_topic_);

  // Define battery state publisher
  battery_state_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
    battery_state_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised mocked topic: " << battery_state_publisher_topic_);

  // Define stop status publisher
  stop_status_publisher_ = create_publisher<irobot_create_msgs::msg::StopStatus>(
    stop_status_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised mocked topic: " << stop_status_publisher_topic_);

  // Subscription to the hazard detection vector
  kidnap_status_subscription_ = create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
    hazard_subscription_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MockPublisher::kidnap_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << hazard_subscription_topic_);

  // Set kidnap status header
  this->kidnap_status_msg_.header.frame_id = "base_link";

  // Subscription to the stop status
  stop_status_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    wheel_vels_subscription_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MockPublisher::stop_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << wheel_vels_subscription_topic_);

  // Set stop status header
  this->stop_status_msg_.header.frame_id = "base_link";

  // Subscription to the lightring leds
  lightring_subscription_ = create_subscription<irobot_create_msgs::msg::LightringLeds>(
    lightring_subscription_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MockPublisher::lightring_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << lightring_subscription_topic_);

  buttons_timer_ = create_wall_timer(
    std::chrono::duration<double>(1 / buttons_publish_rate), [this]() {
      // Set header timestamp.
      this->buttons_msg_.header.stamp = now();

      this->buttons_msg_.button_1.header.stamp = now();
      this->buttons_msg_.button_power.header.stamp = now();
      this->buttons_msg_.button_2.header.stamp = now();

      // Publish topics
      this->buttons_publisher_->publish(this->buttons_msg_);
    });

  // Set buttons header
  this->buttons_msg_.header.frame_id = "base_link";
  this->buttons_msg_.button_1.header.frame_id = "button_1";
  this->buttons_msg_.button_power.header.frame_id = "button_power";
  this->buttons_msg_.button_2.header.frame_id = "button_2";


  slip_status_timer_ = create_wall_timer(
    std::chrono::duration<double>(1 / slip_status_publish_rate), [this]() {
      // Set header timestamp.
      this->slip_status_msg_.header.stamp = now();

      // Publish topics
      this->slip_status_publisher_->publish(this->slip_status_msg_);
    });

  // Set slip status header
  this->slip_status_msg_.header.frame_id = "base_link";
  // Set slip status status
  this->slip_status_msg_.is_slipping = false;

  battery_state_timer_ = create_wall_timer(
    std::chrono::duration<double>(1 / battery_state_publish_rate), [this]() {
      // Set header timestamp.
      this->battery_state_msg_.header.stamp = now();

      // The battery percentage goes from zero to one, one meaning that the battery is full.
      this->battery_state_msg_.percentage = 1;

      // Publish topics
      this->battery_state_publisher_->publish(this->battery_state_msg_);
    });

  // Set battery state header
  this->battery_state_msg_.header.frame_id = "base_link";
}

void MockPublisher::kidnap_callback(irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg)
{
  std::vector<irobot_create_msgs::msg::HazardDetection> hazard_vector = msg->detections;

  bool wheel_drop_left = false;
  bool wheel_drop_right = false;

  for (const auto & detection : hazard_vector) {
    if (detection.header.frame_id == "wheel_drop_left") {
      wheel_drop_left = true;
    } else if (detection.header.frame_id == "wheel_drop_right") {
      wheel_drop_right = true;
    }
  }

  // Set header timestamp.
  kidnap_status_msg_.header.stamp = now();
  // Set kidnap status. The robot is kidnapped when both wheel drops are activated
  kidnap_status_msg_.is_kidnapped = wheel_drop_left && wheel_drop_right;
  // Publish topics
  kidnap_status_publisher_->publish(this->kidnap_status_msg_);
}

void MockPublisher::stop_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto linear_velocity = msg->twist.twist.linear;
  const auto angular_velocity = msg->twist.twist.angular;

  const bool cond_linear_velocity = abs(linear_velocity.x) < linear_velocity_tolerance;
  const bool cond_angular_velocity = abs(angular_velocity.z) < angular_velocity_tolerance;

  // Set header timestamp.
  stop_status_msg_.header.stamp = now();
  // Set stop status. The robot is stopped when both linear and angular velocity
  // are almost zero.
  stop_status_msg_.is_stopped = cond_linear_velocity && cond_angular_velocity;

  // Publish topics
  stop_status_publisher_->publish(this->stop_status_msg_);
}

void MockPublisher::lightring_callback(irobot_create_msgs::msg::LightringLeds::SharedPtr /*msg*/)
{
  RCLCPP_INFO(get_logger(),
    "Lightring message received but it is not implemented in simulation");
}
}  // namespace irobot_create_toolbox
