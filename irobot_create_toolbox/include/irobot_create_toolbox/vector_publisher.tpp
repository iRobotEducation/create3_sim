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

/*
* vector_publisher.tpp is the template definition of vector_publisher.hpp. This class implements the policy based design to allow
* publishing agregator messages of type Base. In order for this class to work the Base class needs to derive from rclcpp::Node.
*/

template<class Msg, class VectorMsg, class Base>
VectorPublisher<Msg, VectorMsg, Base>::VectorPublisher() : Base()
{
  publisher_ = ((rclcpp::Node* )this)->create_publisher<VectorMsg>(this->publisher_topic_, rclcpp::SensorDataQoS());

  const double frequency{62.0};  // Hz
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1 / frequency),
    std::bind(&VectorPublisher::publisher_callback, this));

  // Create subscriptions
  for(std::string topic : this->subscription_topics_) subs_vector_.push_back(((rclcpp::Node* )this)->create_subscription<Msg>(topic, rclcpp::SensorDataQoS(), std::bind(&VectorPublisher::subscription_callback, this, std::placeholders::_1)));
}

template<class Msg, class VectorMsg, class Base>
void VectorPublisher<Msg, VectorMsg, Base>::subscription_callback(const std::shared_ptr<Msg> msg)
{
  {  // Limit the scope of the mutex for good practice.
    std::lock_guard<std::mutex> lock{mutex_};
    this->add_msg(msg);
  }
}

template<class Msg, class VectorMsg, class Base>
void VectorPublisher<Msg, VectorMsg, Base>::publisher_callback()
{
  {  // Limit the scope of the mutex for good practice.
    std::lock_guard<std::mutex> lock{mutex_};

    // Publish detected vector.
    publisher_->publish(this->msg_);
    this->clear_msgs();
  }
}
