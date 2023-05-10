/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include <memory>
#include <utility>

#include "irobot_create_ignition_toolbox/sensors/ir_opcode.hpp"
#include "irobot_create_toolbox/polar_coordinates.hpp"

using irobot_create_ignition_toolbox::IrOpcode;

IrOpcode::IrOpcode(std::shared_ptr<rclcpp::Node> & nh)
: nh_(nh)
{
  emitter_pose_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
    "_internal/sim_ground_truth_ir_emitter_pose",
    rclcpp::SensorDataQoS(),
    std::bind(&IrOpcode::emitter_pose_callback, this, std::placeholders::_1));

  receiver_pose_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
    "_internal/sim_ground_truth_ir_receiver_pose",
    rclcpp::SensorDataQoS(),
    std::bind(&IrOpcode::receiver_pose_callback, this, std::placeholders::_1));

  ir_opcode_pub_ = nh_->create_publisher<irobot_create_msgs::msg::IrOpcode>(
    "ir_opcode",
    rclcpp::SensorDataQoS());

  dock_pub_ = nh_->create_publisher<irobot_create_msgs::msg::DockStatus>(
    "dock_status",
    rclcpp::SensorDataQoS());

  auto sensor_0_fov =
    nh_->declare_parameter("ir_opcode_sensor_0_fov", 3.839724);
  auto sensor_0_range =
    nh_->declare_parameter("ir_opcode_sensor_0_range", 0.1);
  auto sensor_1_fov =
    nh_->declare_parameter("ir_opcode_sensor_1_fov", 1.570796);
  auto sensor_1_range =
    nh_->declare_parameter("ir_opcode_sensor_1_range", 0.5);

  sensors_[irobot_create_msgs::msg::IrOpcode::SENSOR_OMNI] = {
    sensor_0_fov, sensor_0_range};
  sensors_[irobot_create_msgs::msg::IrOpcode::SENSOR_DIRECTIONAL_FRONT] = {
    sensor_1_fov, sensor_1_range};

  ir_opcode_timer_ = rclcpp::create_timer(
    nh_,
    nh_->get_clock(),
    rclcpp::Duration(std::chrono::duration<double>(1 / 62.0)),
    [this]() -> void
    {
      // Get detected opcodes from force field
      detected_forcefield_opcodes_ = {
        CheckForceFieldDetection(
          sensors_[irobot_create_msgs::msg::IrOpcode::SENSOR_OMNI].fov,
          sensors_[irobot_create_msgs::msg::IrOpcode::SENSOR_OMNI].range),
        CheckForceFieldDetection(
          sensors_[irobot_create_msgs::msg::IrOpcode::SENSOR_DIRECTIONAL_FRONT].fov,
          sensors_[irobot_create_msgs::msg::IrOpcode::SENSOR_DIRECTIONAL_FRONT].range)
      };

      // Get detected opcodes from buoys
      detected_buoys_opcodes_ = {
        CheckBuoysDetection(
          sensors_[irobot_create_msgs::msg::IrOpcode::SENSOR_OMNI].fov,
          sensors_[irobot_create_msgs::msg::IrOpcode::SENSOR_OMNI].range),
        CheckBuoysDetection(
          sensors_[irobot_create_msgs::msg::IrOpcode::SENSOR_DIRECTIONAL_FRONT].fov,
          sensors_[irobot_create_msgs::msg::IrOpcode::SENSOR_DIRECTIONAL_FRONT].range)
      };

      PublishSensors(detected_forcefield_opcodes_);
      PublishSensors(detected_buoys_opcodes_);
    });

  dock_status_timer_ = rclcpp::create_timer(
    nh_,
    nh_->get_clock(),
    rclcpp::Duration(std::chrono::duration<double>(1 / 20.0)),
    [this]() -> void
    {
      irobot_create_toolbox::PolarCoordinate receiver_wrt_emitter_polar =
      ReceiverCartesianPointToEmitterPolarPoint(tf2::Vector3(0.0, 0.0, 0.0));

      irobot_create_toolbox::PolarCoordinate emitter_wrt_receiver_polar =
      EmitterCartesianPointToReceiverPolarPoint(tf2::Vector3(0.0, 0.0, 0.0));

      is_docked_ = receiver_wrt_emitter_polar.radius < DOCKED_DISTANCE &&
      std::abs(emitter_wrt_receiver_polar.azimuth) < DOCKED_YAW &&
      std::abs(receiver_wrt_emitter_polar.azimuth) < DOCKED_YAW;

      is_dock_visible_ = detected_buoys_opcodes_[
        irobot_create_msgs::msg::IrOpcode::SENSOR_OMNI] !=
      irobot_create_msgs::msg::IrOpcode::CODE_IR_VIRTUAL_WALL &&
      detected_buoys_opcodes_[
        irobot_create_msgs::msg::IrOpcode::SENSOR_DIRECTIONAL_FRONT] !=
      irobot_create_msgs::msg::IrOpcode::CODE_IR_VIRTUAL_WALL;

      auto dock_msg = irobot_create_msgs::msg::DockStatus();
      dock_msg.header.stamp = nh_->now();
      dock_msg.is_docked = is_docked_;
      dock_msg.dock_visible = is_dock_visible_;
      dock_pub_->publish(std::move(dock_msg));
    });
}

irobot_create_toolbox::PolarCoordinate
IrOpcode::EmitterCartesianPointToReceiverPolarPoint(const tf2::Vector3 & emitter_point)
{
  tf2::Transform emitter_pose;
  {
    const std::lock_guard<std::mutex> lock(emitter_pose_mutex_);
    emitter_pose = last_emitter_pose_;
  }
  tf2::Transform receiver_pose;
  {
    const std::lock_guard<std::mutex> lock(receiver_pose_mutex_);
    receiver_pose = last_receiver_pose_;
  }

  tf2::Vector3 emitter_wrt_receiver_pose = irobot_create_ignition_toolbox::utils::object_wrt_frame(
    emitter_pose, receiver_pose);
  tf2::Vector3 emitter_wrt_receiver_point = emitter_wrt_receiver_pose + emitter_point;
  ignition::math::Vector2d cartesian_coord =
  {emitter_wrt_receiver_point[0], emitter_wrt_receiver_point[1]};
  return irobot_create_toolbox::toPolar(cartesian_coord);
}

irobot_create_toolbox::PolarCoordinate
IrOpcode::ReceiverCartesianPointToEmitterPolarPoint(const tf2::Vector3 & receiver_point)
{
  tf2::Transform emitter_pose;
  {
    const std::lock_guard<std::mutex> lock(emitter_pose_mutex_);
    emitter_pose = last_emitter_pose_;
  }
  tf2::Transform receiver_pose;
  {
    const std::lock_guard<std::mutex> lock(receiver_pose_mutex_);
    receiver_pose = last_receiver_pose_;
  }

  // Pose of receiver relative to the emitter
  tf2::Vector3 receiver_wrt_emitter_pose = irobot_create_ignition_toolbox::utils::object_wrt_frame(
    receiver_pose, emitter_pose);
  tf2::Vector3 receiver_wrt_emitter_point = receiver_wrt_emitter_pose + receiver_point;
  ignition::math::Vector2d cartesian_coord =
  {receiver_wrt_emitter_point[0], receiver_wrt_emitter_point[1]};
  return irobot_create_toolbox::toPolar(cartesian_coord);
}

int IrOpcode::CheckBuoysDetection(const double fov, const double range)
{
  // Get the origin of the receiver as a polar point WRT the emitter
  const irobot_create_toolbox::PolarCoordinate receiver_wrt_emitter_polar =
    ReceiverCartesianPointToEmitterPolarPoint(tf2::Vector3(0.0, 0.0, 0.0));

  // Get the origin of the emitter as a polar point WRT the receiver
  const irobot_create_toolbox::PolarCoordinate emitter_wrt_receiver_polar =
    EmitterCartesianPointToReceiverPolarPoint(tf2::Vector3(0.0, 0.0, 0.0));

  bool receiver_sees_emitter = false;
  bool in_front_of_buoys = false;
  bool buoys_in_range = false;
  bool red_emitter_sees_receiver = false;
  bool green_emitter_sees_receiver = false;
  int detected_opcode = 0;

  // Check emitter fov
  if (
    emitter_wrt_receiver_polar.azimuth > -fov / 2 && emitter_wrt_receiver_polar.azimuth < fov / 2)
  {
    receiver_sees_emitter = true;
  }

  // Check that receiver is in front of emitter
  if (
    receiver_wrt_emitter_polar.azimuth > -M_PI / 2 &&
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
  if (
    receiver_wrt_emitter_polar.azimuth < DOCK_BUOYS_FOV_ / 2 &&
    receiver_wrt_emitter_polar.azimuth >
    DOCK_BUOYS_FOV_ / 2 - DOCK_BUOY_FOV_RATIO_ * DOCK_BUOYS_FOV_)
  {
    red_emitter_sees_receiver = true;
  }

  // Check if receiver within green fov
  // Green fov angle is DOCK_BUOY_FOV_RATIO_*DOCK_BUOYS_FOV_ but is offset from -DOCK_BUOYS_FOV_/2
  // pointing towards the right of the dock
  if (
    receiver_wrt_emitter_polar.azimuth > -DOCK_BUOYS_FOV_ / 2 &&
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

int IrOpcode::CheckForceFieldDetection(const double fov, const double range)
{
  // Get the origin of the emitter as a polar point WRT the receiver
  const irobot_create_toolbox::PolarCoordinate emitter_wrt_receiver_polar =
    EmitterCartesianPointToReceiverPolarPoint(tf2::Vector3{0.0, 0.0, 0.0});

  bool force_field_in_range = false;
  bool receiver_sees_emitter = false;
  int detected_opcode = 0;

  // Check emitter range
  if (emitter_wrt_receiver_polar.radius < range + DOCK_HALO_RANGE_) {
    force_field_in_range = true;
  }

  // Check emitter fov
  if (emitter_wrt_receiver_polar.azimuth > -fov / 2 &&
    emitter_wrt_receiver_polar.azimuth < fov / 2)
  {
    receiver_sees_emitter = true;
  }

  if (force_field_in_range && receiver_sees_emitter) {
    detected_opcode |= irobot_create_msgs::msg::IrOpcode::CODE_IR_FORCE_FIELD;
  }

  return detected_opcode;
}

void IrOpcode::PublishSensors(const std::array<int, 2> detected_opcodes)
{
  // First for sensor 0 then sensor 1
  for (size_t k = 0; k < detected_opcodes.size(); k++) {
    const int detected_opcode = detected_opcodes[k];
    if (detected_opcode > 0) {
      // Fill msg for this iteration
      auto msg_ = irobot_create_msgs::msg::IrOpcode();
      msg_.header.stamp = nh_->now();
      msg_.header.frame_id = "ir_opcode_receiver_link";
      msg_.opcode = detected_opcode;
      msg_.sensor = k;
      // Publish message
      ir_opcode_pub_->publish(std::move(msg_));
    }
  }
}

void IrOpcode::emitter_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(emitter_pose_mutex_);
  tf2::convert(msg->pose.pose, last_emitter_pose_);
}

void IrOpcode::receiver_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(receiver_pose_mutex_);
  tf2::convert(msg->pose.pose, last_receiver_pose_);
}
