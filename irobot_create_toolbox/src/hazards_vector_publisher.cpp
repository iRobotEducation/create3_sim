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

#include <irobot_create_toolbox/hazards_vector_publisher.hpp>

HazardsVectorPublisher::HazardsVectorPublisher() : rclcpp::Node("hazard_detection_vector_node")
{
  // Topic to publish hazards vector to
  publisher_topic_ = declare_parameter("publisher_topic").get<std::string>();

  // Subscription topics
  subscription_topics_ = declare_parameter("subscription_topics").get<std::vector<std::string>>();

  publisher_ = create_publisher<irobot_create_msgs::msg::HazardDetectionVector>(publisher_topic_, rclcpp::SensorDataQoS());

  const double frequency{62.0};  // Hz
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1 / frequency),
    std::bind(&HazardsVectorPublisher::publisher_callback, this));

  // Create subscriptions
  for(std::string topic : subscription_topics_) subs_vector_.push_back((create_subscription<irobot_create_msgs::msg::HazardDetection>(topic, rclcpp::SensorDataQoS(), std::bind(&HazardsVectorPublisher::subscription_callback, this, std::placeholders::_1))));
}

void HazardsVectorPublisher::subscription_callback(
  const std::shared_ptr<irobot_create_msgs::msg::HazardDetection> msg)
{
  {  // Limit the scope of the mutex for good practice.
    std::lock_guard<std::mutex> lock{mutex_};
    msg_.detections.push_back(*msg);
  }
}

void HazardsVectorPublisher::publisher_callback() {
  {  // Limit the scope of the mutex for good practice.
    std::lock_guard<std::mutex> lock{mutex_};

    // Publish detected vector.
    publisher_->publish(this->msg_);
    msg_.detections.clear();
  }
}
