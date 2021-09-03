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
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include <irobot_create_toolbox/ir_intensity_vector_publisher.hpp>

IrIntensityVectorPublisher::IrIntensityVectorPublisher()
: rclcpp::Node("ir_intensity_readings_vector_node")
{
  // Topic parameter to publish IR intensity vector to
  rclcpp::ParameterValue publisher_topic_param = declare_parameter("publisher_topic");
  // Unset parameters have a type: rclcpp::ParameterType::PARAMETER_NOT_SET
  if (publisher_topic_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    throw rclcpp::exceptions::InvalidParameterTypeException(
      "publisher_topic", "Not of type string or was not set");
  }
  publisher_topic_ = publisher_topic_param.get<std::string>();

  // Subscription topics parameter
  rclcpp::ParameterValue subscription_topics_param = declare_parameter("subscription_topics");
  // Unset parameters have a type: rclcpp::ParameterType::PARAMETER_NOT_SET
  if (subscription_topics_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    throw rclcpp::exceptions::InvalidParameterTypeException(
      "subscription_topics", "Not of type string array or was not set");
  }
  subscription_topics_ = subscription_topics_param.get<std::vector<std::string>>();

  // Publish rate parameter
  rclcpp::ParameterValue publish_rate_param = declare_parameter("publish_rate");
  // Unset parameters have a type: rclcpp::ParameterType::PARAMETER_NOT_SET
  if (publish_rate_param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw rclcpp::exceptions::InvalidParameterTypeException(
      "publish_rate", "Not of type double or was not set");
  }
  double publish_rate = publish_rate_param.get<double>();  // Hz

  publisher_ = create_publisher<irobot_create_msgs::msg::IrIntensityVector>(
    publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << publisher_topic_);

  timer_ = create_wall_timer(std::chrono::duration<double>(1 / publish_rate), [this]() {
    std::lock_guard<std::mutex> lock{mutex_};

    // Publish detected vector.
    publisher_->publish(this->msg_);
    msg_.readings.clear();
  });

  // Create subscriptions
  for (std::string topic : subscription_topics_) {
    subs_vector_.push_back((create_subscription<irobot_create_msgs::msg::IrIntensity>(
      topic, rclcpp::SensorDataQoS(),
      [this](const irobot_create_msgs::msg::IrIntensity::SharedPtr msg) {
        std::lock_guard<std::mutex> lock{mutex_};
        msg_.readings.push_back(*msg);
      })));
    RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << topic);
  }
}
