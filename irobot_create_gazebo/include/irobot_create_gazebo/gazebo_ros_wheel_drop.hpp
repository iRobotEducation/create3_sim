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
// @author Emiliano Javier Borghi Orue
// Contact: creativa_eborghi@irobot.com

#ifndef IROBOT_CREATE_GAZEBO__GAZEBO_ROS_WHEEL_DROP_HPP_
#define IROBOT_CREATE_GAZEBO__GAZEBO_ROS_WHEEL_DROP_HPP_

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <irobot_create_gazebo/gazebo_ros_helpers.hpp>
#include <irobot_create_msgs/msg/hazard_detection.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace irobot_create_gazebo
{
/// Example ROS-powered Gazebo plugin with some useful boilerplate.
/// \details This is a `ModelPlugin`, but it could be any supported Gazebo plugin type, such as
/// System, Visual, GUI, World, Sensor, etc.
class GazeboRosWheelDrop : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosWheelDrop() = default;

  /// Destructor
  virtual ~GazeboRosWheelDrop() = default;

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  /// Reset variables on Reset event
  void Reset() override;

protected:
  /// Optional callback to be called at every simulation iteration.
  virtual void OnUpdate();

private:
  /// Publish state
  void PublishState(
    const bool & state, const double & range, const gazebo::common::Time & common_time);

  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_{nullptr};

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// ROS Publisher for wheel drop message
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr pub_{nullptr};

  /// Flag to detect sensor
  bool wheel_drop_detected_{false};

  /// Detection limits
  double lower_limit_;
  double upper_limit_;

  /// Last time
  gazebo::common::Time last_time_;

  /// Helper class to enforce a specific update rate
  utils::UpdateRateEnforcer rate_enforcer_;

  /// Joint name
  std::string name_{""};

  /// Frame ID
  std::string frame_id_{""};

  /// Joint pointer
  gazebo::physics::JointPtr joint_{nullptr};

  /// World pointer
  gazebo::physics::WorldPtr world_{nullptr};
};
}  // namespace irobot_create_gazebo

#endif  // IROBOT_CREATE_GAZEBO__GAZEBO_ROS_WHEEL_DROP_HPP_
