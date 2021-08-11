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
// #include <irobot_create_msgs/msg/hazard_detection.hpp>
// #include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

template<typename T, typename V>
class VectorPublisher : public rclcpp::Node
{
public:
  /// Constructor
  VectorPublisher();

private:
  void subscription_callback(std::shared_ptr<T> msg);
  void publisher_callback();

  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr timer_;

  // Detection vector publisher
  std::shared_ptr<rclcpp::Publisher<V>> publisher_;

  // Vector of subscriptions
  std::vector<std::shared_ptr<rclcpp::Subscription<T>>>
    subs_vector_;

  // Detections message
  V msg_;

  // Mutex
  std::mutex mutex_;
};

#include <irobot_create_toolbox/vector_publisher.tpp>
