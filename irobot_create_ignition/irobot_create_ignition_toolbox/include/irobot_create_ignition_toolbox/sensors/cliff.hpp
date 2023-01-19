/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__CLIFF_HPP_
#define IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__CLIFF_HPP_

#include <memory>
#include <map>
#include <string>
#include <vector>

#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace irobot_create_ignition_toolbox
{

class Cliff
{
public:
  explicit Cliff(std::shared_ptr<rclcpp::Node> & nh);
  virtual ~Cliff() {}

  enum CliffSensors
  {
    FRONT_LEFT,
    FRONT_RIGHT,
    SIDE_LEFT,
    SIDE_RIGHT
  };

private:
  void cliff_callback(const sensor_msgs::msg::LaserScan::SharedPtr cliff_msg);

  std::shared_ptr<rclcpp::Node> nh_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> cliff_sub_;
  std::map<std::string,
    rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr> hazard_pub_;

  std::string cliff_sensors_[4];
};

}  // namespace irobot_create_ignition_toolbox

#endif  // IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__CLIFF_HPP_
