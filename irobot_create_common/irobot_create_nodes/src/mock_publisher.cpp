// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#include "irobot_create_nodes/mock_publisher.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace irobot_create_nodes
{
using namespace std::placeholders;

MockPublisher::MockPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("mock_publisher_node", options)
{
  // Topic parameter to publish slip status to
  slip_status_publisher_topic_ =
    this->declare_parameter("slip_status_topic", "slip_status");

  // Publish rate parameters in Hz
  const double slip_status_publish_rate =
    this->declare_parameter("slip_status_publish_rate", 62.0);

  // Define slip status publisher
  slip_status_publisher_ = create_publisher<irobot_create_msgs::msg::SlipStatus>(
    slip_status_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised mocked topic: " << slip_status_publisher_topic_);

  slip_status_timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration(std::chrono::duration<double>(1 / slip_status_publish_rate)), [this]() {
      // Set header timestamp.
      this->slip_status_msg_.header.stamp = now();

      // Publish topics
      this->slip_status_publisher_->publish(this->slip_status_msg_);
    });

  // Set slip status header
  slip_status_msg_.header.frame_id = base_frame_;
  // Set slip status status
  slip_status_msg_.is_slipping = false;
}

}  // namespace irobot_create_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(irobot_create_nodes::MockPublisher)
