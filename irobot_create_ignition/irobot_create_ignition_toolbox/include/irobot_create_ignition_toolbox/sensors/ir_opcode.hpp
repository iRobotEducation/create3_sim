/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <atomic>

#include "irobot_create_msgs/msg/ir_opcode.hpp"
#include "irobot_create_msgs/msg/dock.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include "irobot_create_ignition_toolbox/utils.hpp"

namespace irobot_create_ignition_toolbox
{

class IrOpcode
{

public:
    explicit IrOpcode(std::shared_ptr<rclcpp::Node> & nh);
    virtual ~IrOpcode() {}

private:
    // Robot receiver parameters
    // Index 0: Sensor 0, the omnidirectional receiver
    // Index 1: Sensor 1, is the forward facing receiver
    struct SensorParams
    {
      double fov;
      double range;
    };

    // Dock emitter parameters
    const double DOCK_BUOYS_FOV_ = 50 * M_PI / 180;  // Convert to radians
    const double DOCK_BUOY_FOV_RATIO_ = 0.6;  // Red Buoy is 0.6 times the total fov, Same for green.
    const double DOCK_BUOYS_RANGE_ = 1.0;
    const double DOCK_HALO_RANGE_ = 0.6096;

    // Docked thresholds
    const double DOCKED_DISTANCE = 0.075;  // Max distance in meters.
    const double DOCKED_YAW = M_PI / 30.0;       // Max Yaw between dock and robot in radians.

    // Convert cartesian point to polar point
    utils::PolarCoordinate EmitterCartesianPointToReceiverPolarPoint(const tf2::Vector3 & emitter_point);
    utils::PolarCoordinate ReceiverCartesianPointToEmitterPolarPoint(const tf2::Vector3 & receiver_point);

    // Check dock visibility and return the associated opcode
    int CheckBuoysDetection(const double fov, const double range);
    int CheckForceFieldDetection(const double fov, const double range);

    // Publish sensors
    void PublishSensors(const std::array<int, 2> detected_opcodes);

    // Subscription callbacks
    void emitter_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void receiver_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void robot_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void dock_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

    std::shared_ptr<rclcpp::Node> nh_;

    rclcpp::TimerBase::SharedPtr ir_opcode_timer_;
    rclcpp::TimerBase::SharedPtr dock_status_timer_;

    rclcpp::Publisher<irobot_create_msgs::msg::IrOpcode>::SharedPtr ir_opcode_pub_;
    rclcpp::Publisher<irobot_create_msgs::msg::Dock>::SharedPtr dock_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr emitter_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr receiver_pose_sub_;

    std::array<SensorParams, 2> sensors_;
    std::array<int, 2> detected_forcefield_opcodes_;
    std::array<int, 2> detected_buoys_opcodes_;

    tf2::Transform last_emitter_pose_;
    tf2::Transform last_receiver_pose_;

    std::mutex emitter_pose_mutex_;
    std::mutex receiver_pose_mutex_;

    bool is_docked_ = false;
    bool is_dock_visible_ = false;
};

} // namespace irobot_create_ignition_toolbox