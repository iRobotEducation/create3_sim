/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__WHEEL_DROP_HPP_
#define IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__WHEEL_DROP_HPP_

#include <map>
#include <memory>
#include <string>

#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace irobot_create_ignition_toolbox
{

class WheelDrop
{
public:
  explicit WheelDrop(std::shared_ptr<rclcpp::Node> & nh);
  virtual ~WheelDrop() {}

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg);

  std::shared_ptr<rclcpp::Node> nh_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::map<std::string,
    rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr> hazard_pub_;

  double detection_threshold_;
  double lower_limit_, upper_limit_;
  std::string joints_[2];
  std::map<std::string, bool> wheeldrop_detected_;
  std::map<std::string, double> displacement_;
};

}  // namespace irobot_create_ignition_toolbox

#endif  // IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__WHEEL_DROP_HPP_
