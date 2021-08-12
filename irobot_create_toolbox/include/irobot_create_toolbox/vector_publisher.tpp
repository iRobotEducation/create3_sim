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
* vector_publisher.tpp the template definition of vector_publisher.hpp. This class implements the policy based design to allow
* publishing agregator messages of type Base. In order for this class to work the Base class needs to derive from rclcpp::Node.
*/

template<class T, class V, class Base>
VectorPublisher<T, V, Base>::VectorPublisher(std::string publisher_topic, std::vector<std::string> subscription_topics) : Base()
{
  publisher_ = ((rclcpp::Node* )this)->create_publisher<V>(publisher_topic, rclcpp::SensorDataQoS());

  std::cout << subscription_topics[0] << std::endl;

  const double frequency{62.0};  // Hz
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1 / frequency),
    std::bind(&VectorPublisher::publisher_callback, this));


  // Create subscriptions
  for(std::string topic : subscription_topics) subs_vector_.push_back(((rclcpp::Node* )this)->create_subscription<T>(topic, rclcpp::SensorDataQoS(), std::bind(&VectorPublisher::subscription_callback, this, std::placeholders::_1)));
}

template<class T, class V, class Base>
void VectorPublisher<T, V, Base>::subscription_callback(std::shared_ptr<T> msg)
{
  std::lock_guard<std::mutex> lock{mutex_};

  this->add_msg(msg);
}

template<class T, class V, class Base>
void VectorPublisher<T, V, Base>::publisher_callback()
{
  std::lock_guard<std::mutex> lock{mutex_};

  // Publish detected vector.
  publisher_->publish(this->msg_);

  this->clear_msgs();
}
