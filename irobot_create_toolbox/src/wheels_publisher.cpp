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

#include <irobot_create_toolbox/wheels_publisher.hpp>

WheelsPublisher::WheelsPublisher() : rclcpp::Node("wheels_publisher_node")
{
  angular_vels_publisher_ = this->create_publisher<irobot_create_msgs::msg::WheelVels>("wheel_vels", rclcpp::SystemDefaultsQoS());
  wheel_ticks_publisher_ = this->create_publisher<irobot_create_msgs::msg::WheelTicks>("wheel_ticks", rclcpp::SystemDefaultsQoS());

  const double frequency{62.0};  // Hz
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1 / frequency),
    std::bind(&WheelsPublisher::publisher_callback, this));

  subscription_ = this->create_subscription<control_msgs::msg::DynamicJointState>("dynamic_joint_states", rclcpp::SystemDefaultsQoS(), std::bind(&WheelsPublisher::subscription_callback, this, std::placeholders::_1));
}

void WheelsPublisher::subscription_callback(const control_msgs::msg::DynamicJointState::SharedPtr msg){
  RCLCPP_DEBUG(this->get_logger(), "Updating wheel interface variables");

  {  // Limit the scope of the mutex for good practice.
    std::lock_guard<std::mutex> lock{mutex_};
    // TODO: Make sure the indexes below are not error prone, if they are then find a way to always obtain
    // the desired wheel independent of the indexes.
    last_right_angular_vel_ = msg->interface_values[0].values[1];  // index [0].[1] refers to the right wheel angular velocity
    last_left_angular_vel_ = msg->interface_values[1].values[1];  // index [1].[1] refers to the left wheel angular velocity
  }
}

void WheelsPublisher::publisher_callback(){
  RCLCPP_INFO(this->get_logger(), "publisher is working");
  {  // Limit the scope of the mutex for good practice.
    std::lock_guard<std::mutex> lock{mutex_};
    std::cout << "left wheel angular vel: " << last_left_angular_vel_ << std::endl;
    std::cout << "right wheel angular vel: " << last_right_angular_vel_ << std::endl;

    // Publish WheelVels
    angular_vels_msg_.velocity_left = last_left_angular_vel_;
    angular_vels_msg_.velocity_right = last_right_angular_vel_;
    angular_vels_publisher_->publish(angular_vels_msg_);
  }
}
