#pragma once

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <irobot_create_msgs/msg/ir_intensity.hpp>

namespace irobot_gazebo_plugins
{
class GazeboRosIrIntensitySensor : public gazebo::SensorPlugin
{
public:
  /// \brief Constructor
  GazeboRosIrIntensitySensor();

  /// \brief Destructor
  ~GazeboRosIrIntensitySensor();

  /// \brief Load the plugin
  /// \param parent Pointer to the parent sensor to which the plugin is attached
  /// \param sdf Take in SDF root element
  void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf) override;

  /// \brief Update callback
  virtual void OnNewLaserScans();

private:
  // pointer to ros node
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  // The connection tied to RayPlugin::OnNewLaserScans()
  gazebo::event::ConnectionPtr new_laser_scans_connection_{nullptr};

  // The parent sensor
  gazebo::sensors::RaySensorPtr parent_sensor_{nullptr};

  // ROS receiver messages publisher
  rclcpp::Publisher<irobot_create_msgs::msg::IrIntensity>::SharedPtr pub_{nullptr};

  // Sensor Emitter description message
  irobot_create_msgs::msg::IrIntensity msg_;
};
}  // namespace irobot_gazebo_plugins
