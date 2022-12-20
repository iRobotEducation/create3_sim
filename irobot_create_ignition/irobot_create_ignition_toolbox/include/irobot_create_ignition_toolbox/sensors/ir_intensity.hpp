/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__IR_INTENSITY_HPP_
#define IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__IR_INTENSITY_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "irobot_create_msgs/msg/ir_intensity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace irobot_create_ignition_toolbox
{

class IrIntensity
{
public:
  explicit IrIntensity(std::shared_ptr<rclcpp::Node> & nh);
  virtual ~IrIntensity() {}

private:
  void ir_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr ir_msg);

  std::shared_ptr<rclcpp::Node> nh_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> ir_scan_sub_;
  std::map<std::string,
    rclcpp::Publisher<irobot_create_msgs::msg::IrIntensity>::SharedPtr> ir_intensity_pub_;

  std::string ir_intensity_sensors_[7];
};

}  // namespace irobot_create_ignition_toolbox

#endif  // IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__IR_INTENSITY_HPP_
