// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_IMU_HPP_
#define IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_IMU_HPP_

#include "gazebo/common/Assert.hh"
#include "gazebo/plugins/ImuSensorPlugin.hh"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo_ros/utils.hpp"
#include "irobot_create_gazebo_plugins/gazebo_ros_helpers.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace irobot_create_gazebo_plugins
{
/// Plugin to attach to a gazebo IMU sensor and publish ROS message as output
class GazeboRosImu : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  GazeboRosImu();
  /// Destructor.
  virtual ~GazeboRosImu();

protected:
  /// Called by Gazebo when the plugin is loaded.
  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publisher for imu message
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  /// IMU message modified each update
  sensor_msgs::msg::Imu::SharedPtr msg_;
  /// IMU sensor this plugin is attached to
  gazebo::sensors::ImuSensorPtr sensor_;
  // The gravity vector
  ignition::math::Vector3d gravity_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;

  /// Publish latest imu data to ROS
  void OnUpdate();
};

}  // namespace irobot_create_gazebo_plugins

#endif  // IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_IMU_HPP_
