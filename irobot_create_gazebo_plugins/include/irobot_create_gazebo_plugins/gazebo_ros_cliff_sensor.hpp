// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Luis Enrique Chico Capistrano (lchico@irobot.com)

#ifndef IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_CLIFF_SENSOR_HPP_
#define IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_CLIFF_SENSOR_HPP_

#include <limits>

#include "gazebo/common/Assert.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/node.hpp"
#include "gazebo_ros/utils.hpp"
#include "irobot_create_gazebo_plugins/gazebo_ros_helpers.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "rclcpp/node.hpp"

namespace irobot_create_gazebo_plugins
{
class GazeboRosCliffSensor : public gazebo::SensorPlugin
{
public:
  /// \brief Constructor
  GazeboRosCliffSensor() = default;
  /// \brief Destructor
  ~GazeboRosCliffSensor() = default;
  /// \brief Called when plugin is loaded
  /// \param parent Pointer to the parent sensor to which the plugin is attached
  /// \param sdf Take in SDF root element
  void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf);
  /// \brief Reset variables on Reset event
  void Reset() override;

protected:
  /// \brief Update the controller
  virtual void OnNewLaserScans();

private:
  // Pointer to left cliff sensor
  gazebo::sensors::RaySensorPtr cliff_sensor_{nullptr};
  // Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};
  // Measured distance in meter for detecting a cliff
  double detection_threshold_;
  // Cliff message modified each update
  irobot_create_msgs::msg::HazardDetection msg_;
  // Set bound for cliff  detection
  double max_range_{std::numeric_limits<double>::max()};
  // World pointer
  gazebo::physics::WorldPtr world_{nullptr};
  // Publish for cliff message
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr pub_{nullptr};
  // The connection tied to update the laser scans OnNewLaserScans()
  gazebo::event::ConnectionPtr new_laser_scans_connection_{nullptr};
};

}  // namespace irobot_create_gazebo_plugins

#endif  // IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_CLIFF_SENSOR_HPP_
