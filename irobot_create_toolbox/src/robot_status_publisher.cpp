// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#include <irobot_create_toolbox/robot_status_publisher.hpp>

#include <string>
#include <vector>

namespace irobot_create_toolbox
{
RobotStatus::RobotStatus()
: rclcpp::Node("robot_status_publisher_node")
{
  // Topic parameter to publish kidnap status to
  kidnap_status_publisher_topic_ =
    declare_and_get_parameter<std::string>("kidnap_status_topic", this);
  // Topic parameter to publish stop status to
  stop_status_publisher_topic_ = declare_and_get_parameter<std::string>("stop_status_topic", this);

  // Subscriber topics
  hazard_subscription_topic_ = declare_and_get_parameter<std::string>("hazard_topic", this);
  wheel_vels_subscription_topic_ = declare_and_get_parameter<std::string>("wheel_vels_topic", this);

  // Sets velocity tolerances
  linear_velocity_tolerance = declare_and_get_parameter<float>("linear_velocity_tolerance", this);
  angular_velocity_tolerance = declare_and_get_parameter<float>("angular_velocity_tolerance", this);

  // Define kidnap status publisher
  kidnap_status_publisher_ = create_publisher<irobot_create_msgs::msg::KidnapStatus>(
    kidnap_status_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised mocked topic: " << kidnap_status_publisher_topic_);

  // Subscription to the hazard detection vector
  kidnap_status_subscription_ = create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
    hazard_subscription_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RobotStatus::kidnap_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << hazard_subscription_topic_);

  // Set kidnap status header
  kidnap_status_msg_.header.frame_id = "base_link";

  // Define stop status publisher
  stop_status_publisher_ = create_publisher<irobot_create_msgs::msg::StopStatus>(
    stop_status_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised mocked topic: " << stop_status_publisher_topic_);

  // Subscription to the stop status
  stop_status_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    wheel_vels_subscription_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RobotStatus::stop_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << wheel_vels_subscription_topic_);

  // Set stop status header
  stop_status_msg_.header.frame_id = "base_link";
}

void RobotStatus::kidnap_callback(irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg)
{
  auto hazard_vector = msg->detections;

  bool wheel_drop_left = false;
  bool wheel_drop_right = false;

  for (const auto & detection : hazard_vector) {
    if (detection.header.frame_id == "wheel_drop_left") {
      wheel_drop_left = true;
    } else if (detection.header.frame_id == "wheel_drop_right") {
      wheel_drop_right = true;
    }
  }

  bool cliff_side_left = false;
  bool cliff_side_right = false;
  bool cliff_front_left = false;
  bool cliff_front_right = false;
  for (const auto & detection : hazard_vector) {
    if (detection.header.frame_id == "cliff_side_left") {
      cliff_side_left = true;
    } else if (detection.header.frame_id == "cliff_side_right") {
      cliff_side_right = true;
    } else if (detection.header.frame_id == "cliff_front_left") {
      cliff_front_left = true;
    } else if (detection.header.frame_id == "cliff_front_right") {
      cliff_front_right = true;
    }
  }

  // Set header timestamp.
  kidnap_status_msg_.header.stamp = now();
  // Set kidnap status. The robot is kidnapped when both wheel drops are activated
  kidnap_status_msg_.is_kidnapped = wheel_drop_left && wheel_drop_right &&
    cliff_side_left && cliff_side_right &&
    cliff_front_left && cliff_front_right;
  // Publish topics
  kidnap_status_publisher_->publish(kidnap_status_msg_);
}

void RobotStatus::stop_callback(nav_msgs::msg::Odometry::SharedPtr msg)
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

  // Publish topics
  stop_status_publisher_->publish(stop_status_msg_);
}
}  // namespace irobot_create_toolbox
