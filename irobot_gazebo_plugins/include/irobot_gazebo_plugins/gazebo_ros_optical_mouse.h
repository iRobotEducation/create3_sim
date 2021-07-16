// Copyright 2021 iRobot, Inc.

#pragma once

#include <memory>

#include <gazebo/common/Plugin.hh>

#include <irobot_create_msgs/msg/mouse.hpp>
#include <irobot_gazebo_plugins/gazebo_ros_helpers.h>

static constexpr double M_TO_INCHES = 100/2.54;

namespace irobot_gazebo_plugins
{
/*! \brief Plugin to control odometry based on the optical mouse sensor.
  *
  *  ROS Publishers
  *  - /optical_mouse/data: Odometry data.
  *
  */
class GazeboRosOpticalMouse : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosOpticalMouse();

  /// Destructor
  virtual ~GazeboRosOpticalMouse();

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  /// Function is called when the world is reset
  void Reset() override;

protected:
  /// Optional callback to be called at every simulation iteration.
  void OnUpdate(const gazebo::common::UpdateInfo& info);

private:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_{ nullptr };

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_{ nullptr };

  /// Link pointer to sensor
  gazebo::physics::LinkPtr link_{ nullptr };

  /// World pointer
  gazebo::physics::WorldPtr world_{ nullptr };

  /// Topic name
  const std::string topic_name_{ "mouse" };

  /// Sensor resolution
  double resolution_ = 0;

  /// Sensor rotation
  double sensor_rotation_;

  /// ROS publisher
  rclcpp::Publisher<irobot_create_msgs::msg::Mouse>::SharedPtr pub_{ nullptr };

  /// Last time the sensor was updated
  gazebo::common::Time last_time_;

  /// Position of the sensor for previous iteration
  ignition::math::Vector3d last_position_;

  /// Helper class to enforce a specific update rate
  UpdateRateEnforcer update_rate_enforcer_;
};
}  // namespace irobot_gazebo_plugins