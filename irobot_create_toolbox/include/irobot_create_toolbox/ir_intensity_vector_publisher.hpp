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

#include <irobot_create_msgs/msg/ir_intensity.hpp>
#include <irobot_create_msgs/msg/ir_intensity_vector.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class IrIntensityVectorPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  IrIntensityVectorPublisher();

  /// \brief Callback to be called upon receiving a message
  void subscription_callback(const std::shared_ptr<irobot_create_msgs::msg::IrIntensity> msg);

  /// \brief Callback to be called periodically to publish the vector message
  void publisher_callback();

protected:
  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr timer_;

  // Detection vector publisher
  std::shared_ptr<rclcpp::Publisher<irobot_create_msgs::msg::IrIntensityVector>> publisher_;

  // Vector of subscriptions
  std::vector<std::shared_ptr<rclcpp::Subscription<irobot_create_msgs::msg::IrIntensity>>>
    subs_vector_;

  // Mutex to protect access to subs_vector_ from different threads
  std::mutex mutex_;

  // Topic to publish IR intensity vector to
  std::string publisher_topic_;

  // Topics from where IR intensity messages will be received from
  std::vector<std::string> subscription_topics_;

  // Message containing a vector to store IR intensity reasings
  irobot_create_msgs::msg::IrIntensityVector msg_;
};
