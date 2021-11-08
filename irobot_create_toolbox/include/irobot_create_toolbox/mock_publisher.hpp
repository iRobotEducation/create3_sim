// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_
#define IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_

#include <irobot_create_msgs/msg/button.hpp>
#include <irobot_create_msgs/msg/interface_buttons.hpp>
#include <irobot_create_msgs/msg/led_color.hpp>
#include <irobot_create_msgs/msg/lightring_leds.hpp>
#include <irobot_create_msgs/msg/slip_status.hpp>
#include <irobot_create_toolbox/parameter_helper.hpp>

#include <rclcpp/rclcpp.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>


namespace irobot_create_toolbox
{
class MockPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  MockPublisher();

  // Callback functions
  void lightring_callback(irobot_create_msgs::msg::LightringLeds::SharedPtr msg);

protected:
  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr buttons_timer_;
  rclcpp::TimerBase::SharedPtr slip_status_timer_;

  // Publishers
  std::shared_ptr<
    rclcpp::Publisher<irobot_create_msgs::msg::InterfaceButtons>> buttons_publisher_{nullptr};
  rclcpp::Publisher<irobot_create_msgs::msg::SlipStatus>::SharedPtr slip_status_publisher_{nullptr};

  // Subscribers
  rclcpp::Subscription<irobot_create_msgs::msg::LightringLeds>::SharedPtr lightring_subscription_;

  // Topic to publish interface buttons to
  std::string buttons_publisher_topic_;
  // Topic to publish slip status to
  std::string slip_status_publisher_topic_;

  // Topic to subscribe to light ring vector
  std::string lightring_subscription_topic_;

  // Message to store the interface buttons
  irobot_create_msgs::msg::InterfaceButtons buttons_msg_;
  // Message to store the slip status
  irobot_create_msgs::msg::SlipStatus slip_status_msg_;

  const std::string base_frame_ {"base_link"};
};

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_
