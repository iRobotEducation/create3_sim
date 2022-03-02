// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "irobot_create_gazebo_plugins/gazebo_ros_imu.hpp"

#include <memory>

namespace irobot_create_gazebo_plugins
{
GazeboRosImu::GazeboRosImu() {}

GazeboRosImu::~GazeboRosImu() {}

void GazeboRosImu::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  GZ_ASSERT(sensor, "Parent is not an imu sensor. Exiting.");

  ros_node_ = gazebo_ros::Node::Get(sdf);

  sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensor);
  sensor_->SetWorldToReferenceOrientation(ignition::math::Quaterniond::Identity);

  gravity_ = gazebo::physics::get_world(sensor_->WorldName())->Gravity();

  pub_ = ros_node_->create_publisher<sensor_msgs::msg::Imu>(
    "~/out", rclcpp::SensorDataQoS().reliable());

  // Get frame for message
  msg_ = std::make_shared<sensor_msgs::msg::Imu>();
  msg_->header.frame_id = gazebo_ros::SensorFrameID(*sensor, *sdf);

  sensor_update_event_ = sensor_->ConnectUpdated(std::bind(&GazeboRosImu::OnUpdate, this));
}

void GazeboRosImu::OnUpdate()
{
  // Calculate gravity w.r.t. IMU frame
  const ignition::math::Matrix4d imu_tf_w{sensor_->Orientation()};
  const ignition::math::Vector3d gravity_imu{imu_tf_w.Inverse() * gravity_};

  // Remove gravity component from IMU reading
  const ignition::math::Vector3d no_gravity_acceleration{
    sensor_->LinearAcceleration() + gravity_imu};

  // Fill message with latest sensor data
  msg_->header.stamp =
    gazebo_ros::Convert<builtin_interfaces::msg::Time>(sensor_->LastUpdateTime());
  msg_->orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(sensor_->Orientation());
  msg_->angular_velocity =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(sensor_->AngularVelocity());
  msg_->linear_acceleration =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(no_gravity_acceleration);

  // Publish message
  pub_->publish(*msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosImu)

}  // namespace irobot_create_gazebo_plugins
