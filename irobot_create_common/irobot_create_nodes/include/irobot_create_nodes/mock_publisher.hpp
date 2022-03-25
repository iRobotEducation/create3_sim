// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#ifndef IROBOT_CREATE_NODES__MOCK_PUBLISHER_HPP_
#define IROBOT_CREATE_NODES__MOCK_PUBLISHER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "irobot_create_msgs/msg/slip_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace irobot_create_nodes
{

class MockPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit MockPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr slip_status_timer_;

  // Publishers
  rclcpp::Publisher<irobot_create_msgs::msg::SlipStatus>::SharedPtr slip_status_publisher_{nullptr};

  // Topic to publish slip status to
  std::string slip_status_publisher_topic_;

  // Message to store the slip status
  irobot_create_msgs::msg::SlipStatus slip_status_msg_;

  const std::string base_frame_ {"base_link"};
};

}  // namespace irobot_create_nodes

#endif  // IROBOT_CREATE_NODES__MOCK_PUBLISHER_HPP_
