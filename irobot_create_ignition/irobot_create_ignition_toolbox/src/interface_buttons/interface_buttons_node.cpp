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

#include "irobot_create_ignition_toolbox/interface_buttons/interface_buttons_node.hpp"

using namespace irobot_create_ignition_toolbox;

InterfaceButtons::InterfaceButtons() : rclcpp::Node("sensors_node")
{
    interface_buttons_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "create3/buttons",
        rclcpp::SensorDataQoS(),
        std::bind(&InterfaceButtons::create3_buttons_callback, this, std::placeholders::_1));

    interface_buttons_pub_ = this->create_publisher<irobot_create_msgs::msg::InterfaceButtons>(
        "interface_buttons", 
        rclcpp::SensorDataQoS());

    interface_buttons_ = std::make_unique<irobot_create_msgs::msg::InterfaceButtons>();
}

// Convert Int32 message to InterfaceButtons message
void InterfaceButtons::create3_buttons_callback(const std_msgs::msg::Int32::SharedPtr create3_buttons_msg)
{
    // User control not allowed when in STOP or RECOIL response
    switch (create3_buttons_msg->data)
    {
        // All buttons released
        case Create3Buttons::NONE:
        {
            if (interface_buttons_->button_1.is_pressed)
            {
                interface_buttons_->button_1.last_pressed_duration = 
                    this->get_clock()->now() - 
                    interface_buttons_->button_1.last_start_pressed_time;
                interface_buttons_->button_1.is_pressed = false;
            }
            if (interface_buttons_->button_power.is_pressed)
            {
                interface_buttons_->button_power.last_pressed_duration = 
                    this->get_clock()->now() - 
                    interface_buttons_->button_power.last_start_pressed_time;
                interface_buttons_->button_power.is_pressed = false;
            }
            if (interface_buttons_->button_2.is_pressed)
            {
                interface_buttons_->button_2.last_pressed_duration = 
                    this->get_clock()->now() - 
                    interface_buttons_->button_2.last_start_pressed_time;
                interface_buttons_->button_2.is_pressed = false;
            }
            break;
        }
        case Create3Buttons::BUTTON_1:
        {
            interface_buttons_->button_1.is_pressed = true;
            interface_buttons_->button_1.last_start_pressed_time = this->get_clock()->now();
            break;
        }
        case Create3Buttons::BUTTON_POWER:
        {
            interface_buttons_->button_power.is_pressed = true;
            interface_buttons_->button_power.last_start_pressed_time = this->get_clock()->now();
            break;
        }
        case Create3Buttons::BUTTON_2:
        {
            interface_buttons_->button_2.is_pressed = true;
            interface_buttons_->button_2.last_start_pressed_time = this->get_clock()->now();
            break;
        }
        default:
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid create3 button %d", create3_buttons_msg->data);
            break;
        }
    }
    // Make copy of interface_buttons_ to publish
    auto interface_buttons_msg = *interface_buttons_;
    interface_buttons_pub_->publish(interface_buttons_msg);
}
