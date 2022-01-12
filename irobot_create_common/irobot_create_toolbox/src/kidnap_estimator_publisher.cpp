// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#include <irobot_create_toolbox/kidnap_estimator_publisher.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace irobot_create_toolbox
{
KidnapEstimator::KidnapEstimator()
: rclcpp::Node("kidnap_estimator")
{
  // Topic parameter to publish kidnap status to
  kidnap_status_publisher_topic_ =
    declare_and_get_parameter<std::string>("kidnap_status_topic", this);

  // Subscriber topics
  hazard_subscription_topic_ = declare_and_get_parameter<std::string>("hazard_topic", this);

  // Define kidnap status publisher
  kidnap_status_publisher_ = create_publisher<irobot_create_msgs::msg::KidnapStatus>(
    kidnap_status_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << kidnap_status_publisher_topic_);

  // Subscription to the hazard detection vector
  kidnap_status_subscription_ = create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
    hazard_subscription_topic_, rclcpp::SensorDataQoS(),
    std::bind(&KidnapEstimator::kidnap_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << hazard_subscription_topic_);

  // Set kidnap status header
  kidnap_status_msg_.header.frame_id = base_frame_;
}

void KidnapEstimator::kidnap_callback(irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg)
{
  auto hazard_vector = msg->detections;
  auto wheel_drop_count = std::count_if(
    hazard_vector.begin(), hazard_vector.end(), [](auto hazard_vector) {
      return hazard_vector.header.frame_id == "wheel_drop_left" ||
      hazard_vector.header.frame_id == "wheel_drop_right";
    });

  auto cliff_sensor_count = std::count_if(
    hazard_vector.begin(), hazard_vector.end(), [](auto hazard_vector) {
      return hazard_vector.header.frame_id == "cliff_side_left" ||
      hazard_vector.header.frame_id == "cliff_side_right" ||
      hazard_vector.header.frame_id == "cliff_front_left" ||
      hazard_vector.header.frame_id == "cliff_front_right";
    });

  // Set header timestamp.
  kidnap_status_msg_.header.stamp = now();
  // Set kidnap status. The robot is kidnapped when both wheel drops
  // and four cliff sensors are triggered.
  kidnap_status_msg_.is_kidnapped = wheel_drop_count >= min_wheel_drop_count_ &&
    cliff_sensor_count >= min_cliff_sensor_count_;

  // Publish topics
  kidnap_status_publisher_->publish(kidnap_status_msg_);
}
}  // namespace irobot_create_toolbox
