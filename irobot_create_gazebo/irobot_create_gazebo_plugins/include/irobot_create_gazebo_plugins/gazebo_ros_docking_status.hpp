// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_DOCKING_STATUS_HPP_
#define IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_DOCKING_STATUS_HPP_

#include <cmath>
#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <irobot_create_gazebo_plugins/docking_manager.hpp>
#include <irobot_create_gazebo_plugins/gazebo_ros_helpers.hpp>
#include <irobot_create_msgs/msg/dock_status.hpp>
#include <irobot_create_msgs/msg/ir_opcode.hpp>

namespace irobot_create_gazebo_plugins
{
class GazeboRosDockingStatus : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosDockingStatus();

  /// Destructor
  virtual ~GazeboRosDockingStatus() = default;

  /// Gazebo calls this when the plugin is loaded.
  /// @param model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// @param sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  /// Callback to be called at every simulation iteration.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// Callback to be called every time a new IROpcode message is received
  void IrOpcodeCb(const irobot_create_msgs::msg::IrOpcode::SharedPtr msg);

private:
  // Docked thresholds
  const double DOCKED_DISTANCE{0.075};  // Max distance in meters.
  const double DOCKED_YAW{M_PI / 30.0};       // Max Yaw between dock and robot in radians.

  // Current dock status
  bool is_dock_visible_{false};
  bool is_docked_{false};

  // IROpcode check rate
  double opcode_check_rate_{20};  // In Hertz, this is lower than IROpcode publish rate.

  // Mutex to protect variables written from different threads
  std::mutex mutex_;

  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_{nullptr};

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// ROS Dock message
  irobot_create_msgs::msg::DockStatus msg_;

  /// ROS publisher
  rclcpp::Publisher<irobot_create_msgs::msg::DockStatus>::SharedPtr pub_{nullptr};

  /// ROS Subscription
  rclcpp::Subscription<irobot_create_msgs::msg::IrOpcode>::SharedPtr sub_{nullptr};

  /// World pointer
  gazebo::physics::WorldPtr world_{nullptr};

  // DockingManager used to determine if robot is docked
  std::shared_ptr<DockingManager> dock_manager_;

  /// Last time the status was published
  gazebo::common::Time last_pub_time_;

  /// Last time the visible status was updated
  gazebo::common::Time last_visible_update_time_;

  /// Helper class to enforce a specific update rate
  utils::UpdateRateEnforcer update_rate_enforcer_;
};
}  // namespace irobot_create_gazebo_plugins

#endif  // IROBOT_CREATE_GAZEBO_PLUGINS__GAZEBO_ROS_DOCKING_STATUS_HPP_
