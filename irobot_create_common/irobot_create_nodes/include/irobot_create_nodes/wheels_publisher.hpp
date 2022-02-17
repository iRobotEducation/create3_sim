// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef IROBOT_CREATE_NODES__WHEELS_PUBLISHER_HPP_
#define IROBOT_CREATE_NODES__WHEELS_PUBLISHER_HPP_

#include <stdexcept>
#include <string>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "irobot_create_msgs/msg/wheel_ticks.hpp"
#include "irobot_create_msgs/msg/wheel_vels.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace irobot_create_nodes
{

class WheelsPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit WheelsPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// \brief Callback to be called periodically to publish the vector message
  void publisher_callback();
  // Get vector index based on joint name.
  size_t get_joint_index(std::string joint_name);
  // Get vector index based on interface name.
  size_t get_interface_index(std::string interface_name, size_t joint_index);
  // Retrieve dynamic state values from last_joint_state_.
  // This is necessary because the indeces are not fixed but the names are.
  double get_dynamic_state_value(std::string joint_name, std::string interface_name);

  // Encoder parameters
  double encoder_resolution_;
  double wheel_circumference_;

  // Handling wheel ticks and wheel velocity messages
  rclcpp::TimerBase::SharedPtr timer_;

  /*
  * This variable holds the last message sent by the topic /dynamic_joint_states which
  * is published by the diff_drive_controller. This variable holds information about the
  * wheels' effort, velocity and position in a vector but the order in which these appear
  * in the vector is not guaranteed, which is why we rely on the get_dynamic_state_value()
  * method to retrieve information from the vectors.
  */
  control_msgs::msg::DynamicJointState last_joint_state_;
  irobot_create_msgs::msg::WheelVels angular_vels_msg_;
  irobot_create_msgs::msg::WheelTicks wheel_ticks_msg_;
  rclcpp::Publisher<irobot_create_msgs::msg::WheelVels>::SharedPtr angular_vels_publisher_;
  rclcpp::Publisher<irobot_create_msgs::msg::WheelTicks>::SharedPtr wheel_ticks_publisher_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr subscription_;

  // Mutex
  std::mutex mutex_;
};

}  // namespace irobot_create_nodes

#endif  // IROBOT_CREATE_NODES__WHEELS_PUBLISHER_HPP_
