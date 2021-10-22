// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "irobot_create_toolbox/hazards_vector_publisher.hpp"

#include <string>
#include <vector>

namespace irobot_create_toolbox
{
HazardsVectorPublisher::HazardsVectorPublisher()
: rclcpp::Node("hazard_detection_vector_node")
{
  // Topic parameter to publish hazards vector to
  publisher_topic_ = declare_and_get_parameter<std::string>("publisher_topic", this);

  // Subscription topics parameter
  subscription_topics_ =
    declare_and_get_parameter<std::vector<std::string>>("subscription_topics", this);

  // Publish rate parameter
  const double publish_rate = declare_and_get_parameter<double>("publish_rate", this);  // Hz

  publisher_ = create_publisher<irobot_create_msgs::msg::HazardDetectionVector>(
    publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << publisher_topic_);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1 / publish_rate), [this]() {
      std::lock_guard<std::mutex> lock{this->mutex_};

      // Set header timestamp.
      this->msg_.header.stamp = now();

      // Publish detected vector.
      this->publisher_->publish(this->msg_);
      this->msg_.detections.clear();
    });

  // Set header frame_id.
  this->msg_.header.frame_id = "base_link";

  // Create subscriptions
  for (std::string topic : subscription_topics_) {
    subs_vector_.push_back(
      (create_subscription<irobot_create_msgs::msg::HazardDetection>(
        topic, rclcpp::SensorDataQoS(),
        [this](const irobot_create_msgs::msg::HazardDetection::SharedPtr msg) {
          std::lock_guard<std::mutex> lock{this->mutex_};
          this->msg_.detections.push_back(*msg);
        })));
    RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << topic);
  }
}

}  // namespace irobot_create_toolbox
