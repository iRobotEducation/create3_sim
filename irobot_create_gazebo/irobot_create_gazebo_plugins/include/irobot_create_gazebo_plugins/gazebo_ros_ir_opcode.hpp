// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_IR_OPCODE_HPP_
#define IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_IR_OPCODE_HPP_

#include <cmath>
#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <irobot_create_gazebo_plugins/docking_manager.hpp>
#include <irobot_create_gazebo_plugins/gazebo_ros_helpers.hpp>
#include <irobot_create_msgs/msg/ir_opcode.hpp>

namespace irobot_create_gazebo_plugins
{
class GazeboRosIrOpcode : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosIrOpcode();

  /// Destructor
  virtual ~GazeboRosIrOpcode();

  /// Gazebo calls this when the plugin is loaded.
  /// @param model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// @param sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  /// Callback to be called at every simulation iteration.
  /// @param info Object containing the world name, sim time and the real time of simulation.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

private:
  // Robot receiver parameters
  // Index 0: Sensor 0, the omnidirectional receiver
  // Index 1: Sensor 1, is the forward facing receiver
  struct SensorParams
  {
    double fov;
    double range;
  };
  std::array<SensorParams, 2> sensors_;

  // Dock emitter parameters
  const double DOCK_BUOYS_FOV_ = 50 * M_PI / 180;  // Convert to radians
  const double DOCK_BUOY_FOV_RATIO_ = 0.6;  // Red Buoy is 0.6 times the total fov, Same for green.
  const double DOCK_BUOYS_RANGE_ = 1.0;
  const double DOCK_HALO_RANGE_ = 0.6096;

  /// World pointer
  gazebo::physics::WorldPtr world_{nullptr};

  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_{nullptr};

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// ROS IrOpcode message
  irobot_create_msgs::msg::IrOpcode msg_;

  /// ROS publisher
  rclcpp::Publisher<irobot_create_msgs::msg::IrOpcode>::SharedPtr pub_{nullptr};

  /// Last time the msg was published
  gazebo::common::Time last_time_;

  /// Helper class to enforce a specific update rate
  utils::UpdateRateEnforcer update_rate_enforcer_;

  // DockingManager
  std::shared_ptr<DockingManager> dock_manager_{nullptr};

  // Check dock visibility and return the associated opcode
  int CheckBuoysDetection(const double fov, const double range);
  int CheckForceFieldDetection(const double fov, const double range);

  // Publish the detected opcodes for Sensor 0 and Sensor 1
  void PublishSensors(const std::array<int, 2> detected_opcodes);
};
}  // namespace irobot_create_gazebo_plugins

#endif  // IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_IR_OPCODE_HPP_
