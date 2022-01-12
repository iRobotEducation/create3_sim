/*
 * Copyright 2021 Clearpath Robotics, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "irobot_create_msgs/msg/ir_intensity.hpp"

namespace irobot_create_ignition_toolbox
{

class IrIntensity
{

public:
    explicit IrIntensity(std::shared_ptr<rclcpp::Node> & nh);
    virtual ~IrIntensity() {}

private:
    void ir_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr ir_msg);
    float find_minimum_range(std::vector<float> ranges);
    
    std::shared_ptr<rclcpp::Node> nh_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> ir_scan_sub_;
    std::map<std::string, rclcpp::Publisher<irobot_create_msgs::msg::IrIntensity>::SharedPtr> ir_intensity_pub_;

    std::string ir_intensity_sensors_[7];
};

} // namespace irobot_create_ignition_toolbox