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

#include <irobot_create_gazebo_plugins/gazebo_ros_ir_opcode.hpp>

#include <string>
#include <memory>

namespace irobot_create_gazebo_plugins
{
GazeboRosIrOpcode::GazeboRosIrOpcode()
: ModelPlugin() {}

GazeboRosIrOpcode::~GazeboRosIrOpcode() {}

void GazeboRosIrOpcode::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Pass SDF parameters so common options like namespace and remapping
  // can be handled.
  ros_node_ = gazebo_ros::Node::Get(sdf);

  world_ = model->GetWorld();
  GZ_ASSERT(world_, "World pointer is invalid!");

  double update_rate{31.0};
  std::string link_name{""};
  double sensor_0_fov{1.0};
  double sensor_0_range{1.0};
  double sensor_1_fov{1.0};
  double sensor_1_range{1.0};

  // Get plugin parameters
  utils::initialize(update_rate, sdf, "update_rate", 31.0);
  utils::initialize(link_name, sdf, "link_name", "");
  utils::initialize(sensor_0_fov, sdf, "sensor_0_fov", 1.0);
  utils::initialize(sensor_0_range, sdf, "sensor_0_range", 1.0);
  utils::initialize(sensor_1_fov, sdf, "sensor_1_fov", 1.0);
  utils::initialize(sensor_1_range, sdf, "sensor_1_range", 1.0);

  // Fill the SensorParams array for Sensor 0 and Sensor 1
  sensors_[0] = {irobot_create_msgs::msg::IrOpcode::SENSOR_OMNI, sensor_0_fov, sensor_0_range};
  sensors_[1] = {irobot_create_msgs::msg::IrOpcode::SENSOR_DIRECTIONAL_FRONT, sensor_1_fov,
    sensor_1_range};

  dock_manager_ptr_ = std::make_shared<DockingManager>(world_, "create3", "standard_dock");

  // Initialize ROS publisher
  pub_ = ros_node_->create_publisher<irobot_create_msgs::msg::IrOpcode>(
    "~/out", rclcpp::SensorDataQoS());

  // Set message frame_id
  msg_.header.frame_id = link_name;

  // Rate enforcer
  update_rate_enforcer_.load(update_rate);

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosIrOpcode::OnUpdate, this, std::placeholders::_1));

  RCLCPP_INFO(ros_node_->get_logger(), "Starting ir opcode plugin");
}

void GazeboRosIrOpcode::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  const gazebo::common::Time current_time = info.simTime;

  if (current_time < last_time_) {
    RCLCPP_WARN(ros_node_->get_logger(), "Negative update time difference detected.");
    last_time_ = current_time;
  }

  const double time_elapsed = (current_time - last_time_).Double();

  // Check if on this iteration corresponds to send the message
  if (!update_rate_enforcer_.shouldUpdate(time_elapsed)) {return;}

  // Update time
  last_time_ = current_time;

  if (!dock_manager_ptr_->checkIfModelsReady()) {
    RCLCPP_DEBUG(ros_node_->get_logger(), "standard_dock model is not ready yet");
    return;
  }

  // Array to hold detected opcodes from force field
  const int detected_forcefield_opcodes[] = {CheckForceFieldDetection(
      sensors_[0].fov,
      sensors_[0].range), CheckForceFieldDetection(sensors_[1].fov, sensors_[1].range)};

  // Array to hold detected opcodes from buoys
  const int detected_buoys_opcodes[] = {CheckBuoysDetection(
      sensors_[0].fov,
      sensors_[0].range), CheckBuoysDetection(sensors_[1].fov, sensors_[1].range)};

  // Check and publish Sensors
  PublishSensors(detected_forcefield_opcodes);
  PublishSensors(detected_buoys_opcodes);
}

int GazeboRosIrOpcode::CheckBuoysDetection(const double fov, const double range)
{
  // Get the origin of the receiver in a polar point WRT the emitter
  const PolarCoordinate receiver_wrt_emitter_polar =
    dock_manager_ptr_->receiverWRTEmitterPolarPoint({0.0, 0.0});

  // Get the origin of the emitter in a polar point WRT the receiver
  const PolarCoordinate emitter_wrt_receiver_polar =
    dock_manager_ptr_->emitterWRTReceiverPolarPoint({0.0, 0.0});

  bool receiver_sees_emitter = false;
  bool in_front_of_buoys = false;
  bool buoys_in_range = false;
  bool red_emitter_sees_receiver = false;
  bool green_emitter_sees_receiver = false;
  int detected_opcode = 0;

  // Check emitter fov
  if (emitter_wrt_receiver_polar.azimuth > -fov / 2 &&
    emitter_wrt_receiver_polar.azimuth < fov / 2)
  {
    receiver_sees_emitter = true;
  }

  // Check that receiver is in front of emitter
  if (receiver_wrt_emitter_polar.azimuth > -M_PI / 2 &&
    receiver_wrt_emitter_polar.azimuth < M_PI / 2)
  {
    in_front_of_buoys = true;
  }

  // Check if buoys are in range
  if (emitter_wrt_receiver_polar.radius < range + DOCK_BUOYS_RANGE_) {
    buoys_in_range = true;
  }

  // Check if receiver within red fov
  // Red fov angle is DOCK_BUOY_FOV_RATIO_*DOCK_BUOYS_FOV_ but is offset from DOCK_BUOYS_FOV_/2
  // pointing towards the left of the dock
  if (receiver_wrt_emitter_polar.azimuth < DOCK_BUOYS_FOV_ / 2 &&
    receiver_wrt_emitter_polar.azimuth >
    DOCK_BUOYS_FOV_ / 2 - DOCK_BUOY_FOV_RATIO_ * DOCK_BUOYS_FOV_)
  {
    red_emitter_sees_receiver = true;
  }

  // Check if receiver within green fov
  // Green fov angle is DOCK_BUOY_FOV_RATIO_*DOCK_BUOYS_FOV_ but is offset from -DOCK_BUOYS_FOV_/2
  // pointing towards the right of the dock
  if (receiver_wrt_emitter_polar.azimuth > -DOCK_BUOYS_FOV_ / 2 &&
    receiver_wrt_emitter_polar.azimuth <
    DOCK_BUOY_FOV_RATIO_ * DOCK_BUOYS_FOV_ - DOCK_BUOYS_FOV_ / 2)
  {
    green_emitter_sees_receiver = true;
  }

  if (buoys_in_range && in_front_of_buoys && receiver_sees_emitter) {
    if (green_emitter_sees_receiver) {
      detected_opcode |= irobot_create_msgs::msg::IrOpcode::CODE_IR_BUOY_GREEN;
    }
    if (red_emitter_sees_receiver) {
      detected_opcode |= irobot_create_msgs::msg::IrOpcode::CODE_IR_BUOY_RED;
    }
  }

  return detected_opcode;
}

int GazeboRosIrOpcode::CheckForceFieldDetection(const double fov, const double range)
{
  // Get the origin of the emitter in a polar point WRT the receiver
  const PolarCoordinate emitter_wrt_receiver_polar =
    dock_manager_ptr_->emitterWRTReceiverPolarPoint({0.0, 0.0});

  bool force_field_in_range = false;
  bool receiver_sees_emitter = false;
  int detected_opcode = 0;

  // Check emitter range
  if (emitter_wrt_receiver_polar.radius < range + DOCK_HALO_RANGE_) {
    force_field_in_range = true;
  }

  // Check emitter fov
  if (
    emitter_wrt_receiver_polar.azimuth > -fov / 2 && emitter_wrt_receiver_polar.azimuth < fov / 2)
  {
    receiver_sees_emitter = true;
  }

  if (force_field_in_range && receiver_sees_emitter) {
    detected_opcode |= irobot_create_msgs::msg::IrOpcode::CODE_IR_FORCE_FIELD;
  }

  return detected_opcode;
}


void GazeboRosIrOpcode::PublishSensors(const int detected_opcodes[2])
{
  // First for sensor 0 then sensor 1
  for (int k = 0; k < 2; k++) {
    const int detected_opcode = detected_opcodes[k];
    if (detected_opcode > 0) {
      // Fill msg for this iteration
      msg_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(world_->SimTime());
      msg_.opcode = detected_opcode;
      msg_.sensor = k;
      // Publish message
      pub_->publish(msg_);
    }
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosIrOpcode)
}  // namespace irobot_create_gazebo_plugins