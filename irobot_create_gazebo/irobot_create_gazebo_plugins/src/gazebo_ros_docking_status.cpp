// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include <irobot_create_gazebo_plugins/gazebo_ros_docking_status.hpp>

#include <memory>
#include <string>

#include "irobot_create_toolbox/polar_coordinates.hpp"

namespace irobot_create_gazebo_plugins
{
GazeboRosDockingStatus::GazeboRosDockingStatus()
: ModelPlugin() {}

void GazeboRosDockingStatus::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  ros_node_ = gazebo_ros::Node::Get(sdf);

  world_ = model->GetWorld();
  GZ_ASSERT(world_, "World pointer is invalid!");

  double update_rate{1.0};
  std::string robot_model_name{""};
  std::string receiver_link_name{""};
  std::string dock_model_name{""};
  std::string emitter_link_name{""};

  // Get plugin parameters
  utils::initialize(update_rate, sdf, "update_rate", 1.0);
  utils::initialize(robot_model_name, sdf, "robot_model_name", "");
  utils::initialize(receiver_link_name, sdf, "receiver_link_name", "");
  utils::initialize(dock_model_name, sdf, "dock_model_name", "");
  utils::initialize(emitter_link_name, sdf, "emitter_link_name", "");

  // Used to determine if robot is docked or not
  dock_manager_ = std::make_shared<DockingManager>(
    world_, robot_model_name, receiver_link_name,
    dock_model_name, emitter_link_name);

  // Initialize ROS publisher
  pub_ = ros_node_->create_publisher<irobot_create_msgs::msg::DockStatus>(
    "~/out", rclcpp::SensorDataQoS().reliable());

  sub_ = ros_node_->create_subscription<irobot_create_msgs::msg::IrOpcode>(
    "ir_opcode", rclcpp::SensorDataQoS(),
    std::bind(&GazeboRosDockingStatus::IrOpcodeCb, this, std::placeholders::_1));

  // Set message frame_id
  msg_.header.frame_id = "base_link";

  // Rate enforcer
  update_rate_enforcer_.load(update_rate);

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosDockingStatus::OnUpdate, this, std::placeholders::_1));

  last_pub_time_ = world_->SimTime();
  last_visible_update_time_ = world_->SimTime();

  RCLCPP_INFO(ros_node_->get_logger(), "Starting ir opcode plugin");
}

void GazeboRosDockingStatus::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  // Check that robot and dock models are spawned
  if (!dock_manager_->AreModelsReady()) {
    RCLCPP_WARN_ONCE(ros_node_->get_logger(), "standard_dock model is not ready yet");
    return;
  }

  const gazebo::common::Time current_time = info.simTime;

  // Check elapsed time since last publish
  const double pub_time_elapsed = (current_time - last_pub_time_).Double();

  // Check elapsed time since last dock visible status update
  const double visible_update_time_elapsed = (current_time - last_visible_update_time_).Double();


  {  // This is the lock guard scope to protect is_dock_visible_ and last_visible_update_time_
    std::lock_guard<std::mutex> lock{mutex_};

    // If opcode not received after the publish period, dock isn't visible
    const double opcode_check_period = 1 / opcode_check_rate_;
    if (visible_update_time_elapsed > opcode_check_period) {
      is_dock_visible_ = false;
      last_visible_update_time_ = current_time;
    }

    // Check if docked now
    irobot_create_toolbox::PolarCoordinate receiver_wrt_emitter_polar =
      dock_manager_->ReceiverCartesianPointToEmitterPolarPoint({0.0, 0.0});
    irobot_create_toolbox::PolarCoordinate emitter_wrt_receiver_polar =
      dock_manager_->EmitterCartesianPointToReceiverPolarPoint({0.0, 0.0});

    is_docked_ = receiver_wrt_emitter_polar.radius < DOCKED_DISTANCE &&
      abs(receiver_wrt_emitter_polar.azimuth) < DOCKED_YAW &&
      abs(emitter_wrt_receiver_polar.azimuth) < DOCKED_YAW;

    // Check for docking status changes since last publish
    if (is_docked_ != msg_.is_docked) {
      RCLCPP_DEBUG_EXPRESSION(
        ros_node_->get_logger(), is_docked_ != msg_.is_docked,
        "is_docked_ status change occurred...");
    }
    if (is_dock_visible_ != msg_.dock_visible) {
      RCLCPP_DEBUG_EXPRESSION(
        ros_node_->get_logger(), is_dock_visible_ != msg_.dock_visible,
        "is_dock_visible_ status change occurred...");
    }

    // Publish if docking status changed or if it is time to do so according to publish rate
    const bool is_changed = is_docked_ != msg_.is_docked || is_dock_visible_ != msg_.dock_visible;
    if (is_changed || update_rate_enforcer_.shouldUpdate(pub_time_elapsed)) {
      // Reset time
      last_pub_time_ = current_time;
      msg_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
      msg_.dock_visible = is_dock_visible_;
      msg_.is_docked = is_docked_;
      pub_->publish(msg_);
      return;
    }
  }
}

void GazeboRosDockingStatus::IrOpcodeCb(const irobot_create_msgs::msg::IrOpcode::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock{mutex_};

  // If IROpcode received isn't Virtual Wall, it means the dock is visible
  is_dock_visible_ = msg->opcode != irobot_create_msgs::msg::IrOpcode::CODE_IR_VIRTUAL_WALL;
  last_visible_update_time_ = world_->SimTime();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosDockingStatus)
}  // namespace irobot_create_gazebo_plugins
