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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <irobot_create_msgs/msg/hazard_detection.hpp>

class HazardsVector : public rclcpp::Node
{
public:
  HazardsVector();

private:
  void subscriber_callback(const irobot_create_msgs::msg::HazardDetection::SharedPtr msg) const;
  void publish_timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr publisher_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_front_left_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_front_right_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_side_left_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_side_right_sub_;

  size_t count_;
};
