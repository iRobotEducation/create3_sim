// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#ifndef IROBOT_CREATE_NODES__KIDNAP_ESTIMATOR_PUBLISHER_HPP_
#define IROBOT_CREATE_NODES__KIDNAP_ESTIMATOR_PUBLISHER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/kidnap_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace irobot_create_nodes
{

class KidnapEstimator : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit KidnapEstimator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// \brief Callback function
  void kidnap_callback(irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg);

  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr kidnap_status_timer_;

  // Publisher
  rclcpp::Publisher<irobot_create_msgs::msg::KidnapStatus>::SharedPtr
    kidnap_status_publisher_{nullptr};

  // Subscriber
  rclcpp::Subscription<
    irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr kidnap_status_subscription_;

  // Topic to publish kidnap status to
  std::string kidnap_status_publisher_topic_;

  // Topic to subscribe to hazard detection vector
  std::string hazard_subscription_topic_;

  // Message to store the kidnap status
  irobot_create_msgs::msg::KidnapStatus kidnap_status_msg_;

  const std::size_t min_wheel_drop_count_{2};
  const std::size_t min_cliff_sensor_count_{4};

  const std::string base_frame_ {"base_link"};
};

}  // namespace irobot_create_nodes

#endif  // IROBOT_CREATE_NODES__KIDNAP_ESTIMATOR_PUBLISHER_HPP_
