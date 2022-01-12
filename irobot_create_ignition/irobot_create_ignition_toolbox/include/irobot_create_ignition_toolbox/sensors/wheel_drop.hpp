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

#include "sensor_msgs/msg/joint_state.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"

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
    std::map<std::string, rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr> hazard_pub_;

    double detection_threshold_;
    double lower_limit_, upper_limit_;
    std::string joints_[2];
    std::map<std::string, bool> wheeldrop_detected_;
    std::map<std::string, double> displacement_;
};

} // namespace irobot_create_ignition_toolbox