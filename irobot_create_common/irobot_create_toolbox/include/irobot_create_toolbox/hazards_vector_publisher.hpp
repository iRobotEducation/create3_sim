// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__HAZARDS_VECTOR_PUBLISHER_HPP_
#define IROBOT_CREATE_TOOLBOX__HAZARDS_VECTOR_PUBLISHER_HPP_

#include <irobot_create_msgs/msg/hazard_detection.hpp>
#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <irobot_create_toolbox/parameter_helper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace irobot_create_toolbox
{
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

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__HAZARDS_VECTOR_PUBLISHER_HPP_
