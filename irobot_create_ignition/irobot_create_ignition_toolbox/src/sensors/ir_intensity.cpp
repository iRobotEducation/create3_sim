/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "irobot_create_ignition_toolbox/sensors/ir_intensity.hpp"
#include "irobot_create_toolbox/math.hpp"

using irobot_create_ignition_toolbox::IrIntensity;

IrIntensity::IrIntensity(std::shared_ptr<rclcpp::Node> & nh)
: nh_(nh),
  ir_intensity_sensors_{
    "front_center_left",
    "front_center_right",
    "front_left",
    "front_right",
    "left",
    "right",
    "side_left"
  }
{
  auto ir_scan_sub_topics =
    nh_->declare_parameter("ir_scan_subscription_topics", std::vector<std::string>());

  auto ir_intensity_pub_topics =
    nh_->declare_parameter("ir_intensity_publish_topics", std::vector<std::string>());

  for (const std::string & topic : ir_scan_sub_topics) {
    ir_scan_sub_.push_back(
      nh_->create_subscription<sensor_msgs::msg::LaserScan>(
        topic,
        rclcpp::SensorDataQoS(),
        std::bind(&IrIntensity::ir_scan_callback, this, std::placeholders::_1)));
  }

  for (const std::string & topic : ir_intensity_pub_topics) {
    for (const std::string & sensor : ir_intensity_sensors_) {
      if (topic.find(sensor) != std::string::npos) {
        ir_intensity_pub_[sensor] = nh_->create_publisher<
          irobot_create_msgs::msg::IrIntensity>(
          topic,
          rclcpp::SensorDataQoS());
      }
    }
  }
}

void IrIntensity::ir_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr ir_msg)
{
  auto ir_intensity_msg = irobot_create_msgs::msg::IrIntensity();

  const double detection =
    std::min(irobot_create_toolbox::FindMinimumRange(ir_msg->ranges), ir_msg->range_max);

  // IR sensor produces an exponential signal that is correlated to the distance,
  // that follows this formula: ir_reading = A exp(-x*B)
  // where:
  // A is a coefficient that depends on the color surface and
  // it can be as high as 3500
  // B is the decay of the signal related to the distance.
  // From the experiments B ~ 26.831568
  const double scaled_detection = 3500 * std::exp(detection * (-2 * M_E / ir_msg->range_max));
  ir_intensity_msg.value = static_cast<irobot_create_msgs::msg::IrIntensity::_value_type>(
    scaled_detection);

  // Publish to appropriate topic
  for (const std::string & sensor : ir_intensity_sensors_) {
    if (ir_msg->header.frame_id.find(sensor) != std::string::npos) {
      ir_intensity_pub_[sensor]->publish(ir_intensity_msg);
    }
  }
}
