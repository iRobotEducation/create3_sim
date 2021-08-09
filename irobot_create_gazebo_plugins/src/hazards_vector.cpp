// Copyright 2021 iRobot, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <irobot_create_msgs/msg/hazard_detection.hpp>

class HazardsVector : public rclcpp::Node
{
public:
  HazardsVector() : Node("hazards_vector"), count_(0)
  {
    publisher_ = this->create_publisher<irobot_create_msgs::msg::HazardDetection>("hazard_detection", rclcpp::SensorDataQoS());
    float freq = 62.0F; // Hz
    timer_ = this->create_wall_timer(std::chrono::duration<float>(1 / freq), std::bind(&HazardsVector::publish_timer_callback, this));

    // Cliff Subscriber topics
    cliff_front_left_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetection>(
      "/cliff_front_left/event", rclcpp::SensorDataQoS(), std::bind(&HazardsVector::subscriber_callback, this, std::placeholders::_1));
    cliff_front_right_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetection>(
      "/cliff_front_right/event", rclcpp::SensorDataQoS(), std::bind(&HazardsVector::subscriber_callback, this, std::placeholders::_1));
    cliff_side_left_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetection>(
      "/cliff_side_left/event", rclcpp::SensorDataQoS(), std::bind(&HazardsVector::subscriber_callback, this, std::placeholders::_1));
    cliff_side_right_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetection>(
      "/cliff_side_right/event", rclcpp::SensorDataQoS(), std::bind(&HazardsVector::subscriber_callback, this, std::placeholders::_1));

  }

private:
  void subscriber_callback(const irobot_create_msgs::msg::HazardDetection::SharedPtr msg) const
  {
    std::cout << "I heard: " <<  msg->header.frame_id << std::endl;
  }
  void publish_timer_callback()
  {
    auto message = irobot_create_msgs::msg::HazardDetection();
    // RCLCPP_INFO(this->get_logger(), "Publishing now");
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr publisher_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_front_left_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_front_right_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_side_left_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetection>::SharedPtr cliff_side_right_sub_;

      // /cliff_front_right/event
    // /cliff_side_left/event
    // /cliff_side_right/event
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HazardsVector>());
  rclcpp::shutdown();
  return 0;
}