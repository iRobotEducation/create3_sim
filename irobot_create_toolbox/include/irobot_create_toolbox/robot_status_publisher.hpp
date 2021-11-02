// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__ROBOT_STATUS_PUBLISHER_HPP_
#define IROBOT_CREATE_TOOLBOX__ROBOT_STATUS_PUBLISHER_HPP_

#include <irobot_create_msgs/msg/hazard_detection.hpp>
#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <irobot_create_msgs/msg/kidnap_status.hpp>
#include <irobot_create_msgs/msg/stop_status.hpp>
#include <irobot_create_msgs/msg/wheel_vels.hpp>
#include <irobot_create_toolbox/parameter_helper.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>


namespace irobot_create_toolbox
{
class RobotStatus : public rclcpp::Node
{
public:
  /// \brief Constructor
  RobotStatus();

  // Callback functions
  void kidnap_callback(irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg);
  void stop_callback(nav_msgs::msg::Odometry::SharedPtr msg);

protected:
  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr kidnap_status_timer_;
  rclcpp::TimerBase::SharedPtr stop_status_timer_;

  // Publishers
  rclcpp::Publisher<irobot_create_msgs::msg::KidnapStatus>::SharedPtr
    kidnap_status_publisher_{nullptr};
  rclcpp::Publisher<irobot_create_msgs::msg::StopStatus>::SharedPtr stop_status_publisher_{nullptr};

  // Subscribers
  rclcpp::Subscription<
    irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr kidnap_status_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr stop_status_subscription_;

  // Topic to publish kidnap status to
  std::string kidnap_status_publisher_topic_;
  // Topic to publish stop status to
  std::string stop_status_publisher_topic_;

  // Topic to subscribe to hazard detection vector
  std::string hazard_subscription_topic_;
  // Topic to subscribe to wheel vels vector
  std::string wheel_vels_subscription_topic_;

  // Message to store the kidnap status
  irobot_create_msgs::msg::KidnapStatus kidnap_status_msg_;
  // Message to store the stop status
  irobot_create_msgs::msg::StopStatus stop_status_msg_;

  double linear_velocity_tolerance{std::numeric_limits<double>::max()};
  double angular_velocity_tolerance{std::numeric_limits<double>::max()};
};

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__ROBOT_STATUS_PUBLISHER_HPP_
