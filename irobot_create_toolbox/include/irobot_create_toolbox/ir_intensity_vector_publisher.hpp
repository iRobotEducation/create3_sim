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

#ifndef IROBOT_CREATE_TOOLBOX__IR_INTENSITY_VECTOR_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <irobot_create_msgs/msg/ir_intensity.hpp>
#include <irobot_create_msgs/msg/ir_intensity_vector.hpp>
#include <irobot_create_toolbox/parameter_helper.hpp>
#include <rclcpp/rclcpp.hpp>

namespace irobot_create_toolbox
{
class IrIntensityVectorPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  IrIntensityVectorPublisher();

protected:
  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr timer_;

  // Detection vector publisher
  std::shared_ptr<rclcpp::Publisher<irobot_create_msgs::msg::IrIntensityVector>> publisher_;

  // Vector of subscriptions
  using IrIntensityVectorSubscriptionPtr =
    std::vector<rclcpp::Subscription<irobot_create_msgs::msg::IrIntensity>::SharedPtr>;
  IrIntensityVectorSubscriptionPtr subs_vector_;

  // Mutex to protect access to subs_vector_ from different threads
  std::mutex mutex_;

  // Topic to publish IR intensity vector to
  std::string publisher_topic_;

  // Topics from where IR intensity messages will be received from
  std::vector<std::string> subscription_topics_;

  // Message containing a vector to store IR intensity reasings
  irobot_create_msgs::msg::IrIntensityVector msg_;
};

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__IR_INTENSITY_VECTOR_PUBLISHER_HPP_
