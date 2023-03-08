// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "irobot_create_nodes/ir_intensity_vector_publisher.hpp"

#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace irobot_create_nodes
{

IrIntensityVectorPublisher::IrIntensityVectorPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("ir_intensity_readings_vector_node", options)
{
  // Topic parameter to publish IR intensity vector to
  publisher_topic_ =
    this->declare_parameter("publisher_topic", "ir_intensity");

  // Subscription topics parameter
  subscription_topics_ =
    this->declare_parameter("subscription_topics", std::vector<std::string>());

  // Publish rate parameter in Hz
  const double publish_rate =
    this->declare_parameter("publish_rate", 62.0);

  publisher_ = create_publisher<irobot_create_msgs::msg::IrIntensityVector>(
    publisher_topic_, rclcpp::SensorDataQoS().reliable());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << publisher_topic_);

  timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration(std::chrono::duration<double>(1 / publish_rate)), [this]() {
      std::lock_guard<std::mutex> lock{this->mutex_};

      // Set header timestamp.
      this->msg_.header.stamp = now();
      this->msg_.readings.clear();
      this->msg_.readings.reserve(this->ir_intensities_.size());
      for (const auto & ir_msgs : this->ir_intensities_) {
        this->msg_.readings.emplace_back(ir_msgs.second);
      }

      // Publish detected vector.
      this->publisher_->publish(this->msg_);
    });

  // Set header frame_id.
  this->msg_.header.frame_id = "base_link";

  // Create subscriptions
  for (std::string topic : subscription_topics_) {
    subs_vector_.push_back(
      (create_subscription<irobot_create_msgs::msg::IrIntensity>(
        topic, rclcpp::SensorDataQoS(),
        [this](const irobot_create_msgs::msg::IrIntensity::SharedPtr msg) {
          std::lock_guard<std::mutex> lock{this->mutex_};
          this->ir_intensities_[msg->header.frame_id] = *msg;
        })));
    RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << topic);
  }
}

}  // namespace irobot_create_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(irobot_create_nodes::IrIntensityVectorPublisher)
