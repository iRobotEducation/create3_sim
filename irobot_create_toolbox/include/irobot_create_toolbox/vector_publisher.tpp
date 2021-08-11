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

template<class T, class V>
VectorPublisher<T, V>::VectorPublisher() : Node("vector_publisher")
{
  publisher_ = this->create_publisher<V>(
    "hazard_detection", rclcpp::SensorDataQoS());

  const float frequency{62.0};  // Hz
  timer_ = this->create_wall_timer(
    std::chrono::duration<float>(1 / frequency),
    std::bind(&VectorPublisher::publisher_callback, this));

  // Bumper Subscription
  subs_vector_.push_back(this->create_subscription<T>(
    "/bumper/event", rclcpp::SensorDataQoS(),
    std::bind(&VectorPublisher::subscription_callback, this, std::placeholders::_1)));

  // Cliff Subscriptions
  subs_vector_.push_back(this->create_subscription<T>(
    "/cliff_front_left/event", rclcpp::SensorDataQoS(),
    std::bind(&VectorPublisher::subscription_callback, this, std::placeholders::_1)));
  subs_vector_.push_back(this->create_subscription<T>(
    "/cliff_front_right/event", rclcpp::SensorDataQoS(),
    std::bind(&VectorPublisher::subscription_callback, this, std::placeholders::_1)));
  subs_vector_.push_back(this->create_subscription<T>(
    "/cliff_side_left/event", rclcpp::SensorDataQoS(),
    std::bind(&VectorPublisher::subscription_callback, this, std::placeholders::_1)));
  subs_vector_.push_back(this->create_subscription<T>(
    "/cliff_side_right/event", rclcpp::SensorDataQoS(),
    std::bind(&VectorPublisher::subscription_callback, this, std::placeholders::_1)));

  // Wheeldrop Subscriptions
  subs_vector_.push_back(this->create_subscription<T>(
    "/wheel_drop/left_wheel/event", rclcpp::SensorDataQoS(),
    std::bind(&VectorPublisher::subscription_callback, this, std::placeholders::_1)));
  subs_vector_.push_back(this->create_subscription<T>(
    "/wheel_drop/right_wheel/event", rclcpp::SensorDataQoS(),
    std::bind(&VectorPublisher::subscription_callback, this, std::placeholders::_1)));
}

template<class T, class V>
void VectorPublisher<T, V>::subscription_callback(std::shared_ptr<T> msg)
{
  std::lock_guard<std::mutex> lock{mutex_};

  // Add message to vector.
  msg_.detections.push_back(*msg);
}

template<class T, class V>
void VectorPublisher<T, V>::publisher_callback()
{
  std::lock_guard<std::mutex> lock{mutex_};

  // Publish detected vector.
  publisher_->publish(msg_);

  // Clear the vector now that it was published.
  msg_.detections.clear();
}
