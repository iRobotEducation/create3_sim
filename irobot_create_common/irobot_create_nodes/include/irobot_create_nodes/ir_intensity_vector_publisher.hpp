// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef IROBOT_CREATE_NODES__IR_INTENSITY_VECTOR_PUBLISHER_HPP_
#define IROBOT_CREATE_NODES__IR_INTENSITY_VECTOR_PUBLISHER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "irobot_create_msgs/msg/ir_intensity.hpp"
#include "irobot_create_msgs/msg/ir_intensity_vector.hpp"
#include "rclcpp/rclcpp.hpp"

namespace irobot_create_nodes
{

class IrIntensityVectorPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit IrIntensityVectorPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
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

  // Cache of last message for each frame
  // Each IR reading is from a separate publication so there is potential
  // for a race condition where frames could not come in, or come in twice
  // between vector publications.  The cache will keep the most recent
  // reading from each frame and the publication will make a vector of
  // each frame's most recent received reading from the map of values
  std::map<std::string, irobot_create_msgs::msg::IrIntensity> ir_intensities_;
};

}  // namespace irobot_create_nodes

#endif  // IROBOT_CREATE_NODES__IR_INTENSITY_VECTOR_PUBLISHER_HPP_
