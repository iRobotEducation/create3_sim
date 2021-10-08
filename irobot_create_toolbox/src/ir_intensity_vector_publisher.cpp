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

#include <string>
#include <vector>

#include <irobot_create_toolbox/ir_intensity_vector_publisher.hpp>

namespace irobot_create_toolbox
{
IrIntensityVectorPublisher::IrIntensityVectorPublisher()
: rclcpp::Node("ir_intensity_readings_vector_node")
{
  // Topic parameter to publish IR intensity vector to
  publisher_topic_ = declare_and_get_parameter<std::string>("publisher_topic", this);

  // Subscription topics parameter
  subscription_topics_ =
    declare_and_get_parameter<std::vector<std::string>>("subscription_topics", this);

  // Publish rate parameter
  const double publish_rate = declare_and_get_parameter<double>("publish_rate", this);  // Hz

  publisher_ = create_publisher<irobot_create_msgs::msg::IrIntensityVector>(
    publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << publisher_topic_);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1 / publish_rate), [this]() {
      std::lock_guard<std::mutex> lock{this->mutex_};

      // Set header timestamp.
      this->msg_.header.stamp = now();

      // Publish detected vector.
      this->publisher_->publish(this->msg_);
      this->msg_.readings.clear();
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
          this->msg_.readings.push_back(*msg);
        })));
    RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << topic);
  }
}

}  // namespace irobot_create_toolbox
