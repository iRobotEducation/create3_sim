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

#pragma once

#include <irobot_create_msgs/msg/hazard_detection.hpp>
#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class HazardsVectorPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  HazardsVectorPublisher();

private:
  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr timer_;

  // Detection vector publisher
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr publisher_;

  // Vector of subscriptions
  using HazardVectorSubscriptionPtr =
    std::vector<rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr>;
  HazardVectorSubscriptionPtr subs_vector_;

  // Mutex to protect access to subs_vector_ from different threads
  std::mutex mutex_;

  // Topic to publish hazards vector to
  std::string publisher_topic_;

  // Topics from where hazard messages will be received from
  std::vector<std::string> subscription_topics_;

  // Message containing a vector to store detected hazards
  irobot_create_msgs::msg::HazardDetectionVector msg_;
};
