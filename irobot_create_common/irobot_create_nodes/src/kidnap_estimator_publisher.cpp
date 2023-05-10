// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#include "irobot_create_nodes/kidnap_estimator_publisher.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace irobot_create_nodes
{
KidnapEstimator::KidnapEstimator(const rclcpp::NodeOptions & options)
: rclcpp::Node("kidnap_estimator", options)
{
  // Topic parameter to publish kidnap status to
  kidnap_status_publisher_topic_ =
    this->declare_parameter("kidnap_status_topic", "kidnap_status");

  // Subscriber topics
  hazard_subscription_topic_ =
    this->declare_parameter("hazard_topic", "hazard_detection");

  // Define kidnap status publisher
  kidnap_status_publisher_ = create_publisher<irobot_create_msgs::msg::KidnapStatus>(
    kidnap_status_publisher_topic_, rclcpp::SensorDataQoS().reliable());
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
  const auto hazard_vector = msg->detections;
  const std::size_t wheel_drop_count = std::count_if(
    hazard_vector.begin(), hazard_vector.end(), [](auto hazard_vector) {
      return hazard_vector.header.frame_id == "wheel_drop_left" ||
      hazard_vector.header.frame_id == "wheel_drop_right";
    });

  const std::size_t cliff_sensor_count = std::count_if(
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

}  // namespace irobot_create_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(irobot_create_nodes::KidnapEstimator)
