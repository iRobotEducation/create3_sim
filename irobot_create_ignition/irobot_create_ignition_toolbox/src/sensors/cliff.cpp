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

#include "irobot_create_ignition_toolbox/sensors/cliff.hpp"
#include "irobot_create_toolbox/parameter_helper.hpp"

using namespace irobot_create_ignition_toolbox;

Cliff::Cliff(std::shared_ptr<rclcpp::Node> &nh) : nh_(nh),
                                                  cliff_sensors_{
                                                      "front_left",
                                                      "front_right",
                                                      "side_left",
                                                      "side_right"
                                                  }
{
    auto cliff_sub_topics = irobot_create_toolbox::declare_and_get_parameter<std::vector<std::string>>("cliff_subscription_topics", nh_.get());
    auto cliff_pub_topics = irobot_create_toolbox::declare_and_get_parameter<std::vector<std::string>>("cliff_publish_topics", nh_.get());

    for (std::string topic : cliff_sub_topics) 
    {
        cliff_sub_.push_back(nh_->create_subscription<sensor_msgs::msg::LaserScan>(
                                topic,
                                rclcpp::SensorDataQoS(),
                                std::bind(&Cliff::cliff_callback, this, std::placeholders::_1)));
    }

    for (std::string topic : cliff_pub_topics)
    {
        for (const std::string sensor : cliff_sensors_)
        {
            if (topic.find(sensor) != std::string::npos)
            {
                hazard_pub_[sensor] = nh_->create_publisher<irobot_create_msgs::msg::HazardDetection>(
                                                topic,
                                                rclcpp::SensorDataQoS());
            }
        }
    }
}

void Cliff::cliff_callback(const sensor_msgs::msg::LaserScan::SharedPtr cliff_msg)
{
    if (cliff_msg->ranges[0] > 0.03)
    {
        auto hazard_msg = irobot_create_msgs::msg::HazardDetection();
        hazard_msg.type = irobot_create_msgs::msg::HazardDetection::CLIFF;
        // TODO: Add tf for sensor frames
        hazard_msg.header.frame_id = "base_link";

        // Publish to appropriate topic
        for (const std::string sensor : cliff_sensors_)
        {
            if (cliff_msg->header.frame_id.find(sensor) != std::string::npos)
            {
                hazard_pub_[sensor]->publish(hazard_msg);
            }
        }
    }
}
