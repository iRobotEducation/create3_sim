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
#include "irobot_create_msgs/msg/hazard_detection.hpp"

namespace irobot_create_ignition_toolbox
{

class Cliff
{

public:
    explicit Cliff(std::shared_ptr<rclcpp::Node> & nh);
    virtual ~Cliff() {}

    enum CliffSensors {
        FRONT_LEFT,
        FRONT_RIGHT,
        SIDE_LEFT,
        SIDE_RIGHT
    };

private:
    void cliff_callback(const sensor_msgs::msg::LaserScan::SharedPtr cliff_msg);
    
    std::shared_ptr<rclcpp::Node> nh_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> cliff_sub_;
    std::map<std::string, rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr> hazard_pub_;

    std::string cliff_sensors_[4];
};

} // namespace irobot_create_ignition_toolbox
