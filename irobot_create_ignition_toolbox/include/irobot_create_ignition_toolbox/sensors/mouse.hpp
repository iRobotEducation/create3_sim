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

#include <atomic>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "irobot_create_msgs/msg/mouse.hpp"

namespace irobot_create_ignition_toolbox
{

class Mouse
{

public:
    explicit Mouse(std::shared_ptr<rclcpp::Node> & nh);
    virtual ~Mouse() {}

private:
    void robot_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void mouse_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    
    std::shared_ptr<rclcpp::Node> nh_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mouse_pose_sub_;
    rclcpp::Publisher<irobot_create_msgs::msg::Mouse>::SharedPtr mouse_pub_;

    tf2::Vector3 integrated_position_;
    tf2::Vector3 last_mouse_position_;
};

} // namespace irobot_create_ignition_toolbox