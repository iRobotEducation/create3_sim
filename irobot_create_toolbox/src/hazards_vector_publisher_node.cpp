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

#include <irobot_create_msgs/msg/hazard_detection.hpp>
#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <irobot_create_toolbox/hazards_vector_publisher.hpp>
#include <irobot_create_toolbox/vector_publisher.hpp>

int main(int argc, char * argv[])
{
  // Topic to publish hazards vector to.
  std::string publisher_topic = "hazard_detection";
  // Vector of subscription topics to listen for messages.
  std::vector<std::string> subscription_topics;

  // Bumper Subscription
  subscription_topics.push_back("/bumper/event");

  // Cliff Subscriptions
  subscription_topics.push_back("/cliff_front_left/event");
  subscription_topics.push_back("/cliff_front_right/event");
  subscription_topics.push_back("/cliff_side_left/event");
  subscription_topics.push_back("/cliff_side_right/event");

  // Wheeldrop Subscriptions
  subscription_topics.push_back("/wheel_drop/left_wheel/event");
  subscription_topics.push_back("/wheel_drop/right_wheel/event");

  using HazardDetectionVectorPublisher = VectorPublisher<
    irobot_create_msgs::msg::HazardDetection, irobot_create_msgs::msg::HazardDetectionVector,
    HazardsVectorPublisher>;

  rclcpp::init(argc, argv);
  // Instantiate a VectorPublisher node specialized to handle hazards detections and pass that
  // node to spin().
  rclcpp::spin(
    std::make_shared<HazardDetectionVectorPublisher>(publisher_topic, subscription_topics));
  rclcpp::shutdown();
  return 0;
}
