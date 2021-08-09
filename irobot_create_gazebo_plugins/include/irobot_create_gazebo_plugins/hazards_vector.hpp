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

#include <algorithm>
#include <chrono>
#include <functional>
#include <irobot_create_msgs/msg/hazard_detection.hpp>
#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class HazardsVector : public rclcpp::Node
{
public:
  /// Constructor

  HazardsVector();

private:
  void subscriber_callback(irobot_create_msgs::msg::HazardDetection::SharedPtr msg);
  void publish_timer_callback();

  // Publish aggregated hazard detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr timer_;

  // Hazard detection vector publisher
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr publisher_;

  // Bumper subscriptions
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr bumper_sub_;

  // Cliff subscriptions
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_front_left_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_front_right_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_side_left_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_side_right_sub_;

  // Wheeldrop subscriptions
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr
    wheel_drop_left_wheel_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr
    wheel_drop_right_wheel_sub_;

  // Vector holding hazard detections per iteration
  std::vector<irobot_create_msgs::msg::HazardDetection> msgs_;

  // Mutex
  std::mutex mutex_;
};
