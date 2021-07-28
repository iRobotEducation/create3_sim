// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>

#include <irobot_create_gazebo_plugins/gazebo_ros_optical_mouse.hpp>

namespace irobot_create_gazebo_plugins
{
GazeboRosOpticalMouse::GazeboRosOpticalMouse()
  : ModelPlugin()
{
}

GazeboRosOpticalMouse::~GazeboRosOpticalMouse()
{
}

void GazeboRosOpticalMouse::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  world_ = model->GetWorld();
  GZ_ASSERT(world_, "World pointer is invalid!");

  std::string link_name;
  double update_rate;
  double gaussian_mean;
  double gaussian_var;
  double sensor_rotation;
  int seed;
  srand(time(0));

  // Get plugin parameters
  utils::initialize(link_name, sdf, "link", "base_link");
  utils::initialize(update_rate, sdf, "update_rate", 100.0);
  utils::initialize(resolution_, sdf, "resolution", 125.0);
  utils::initialize(sensor_rotation, sdf, "sensor_rotation", 0.0);
  utils::initialize(gaussian_mean, sdf, "gaussian_mean", 0.0);
  utils::initialize(gaussian_var, sdf, "gaussian_var", 0.0);
  utils::initialize(seed, sdf, "mouse_seed", rand());

  // Get link
  link_ = model->GetLink(link_name);
  GZ_ASSERT(link_, "Couldn't find optical mouse link.");

  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  ros_node_ = gazebo_ros::Node::Get(sdf);
  // Get QoS profiles
  const gazebo_ros::QoS& qos = ros_node_->get_qos();

  // Initialize ROS publisher
  pub_ = ros_node_->create_publisher<irobot_create_msgs::msg::Mouse>(
      topic_name_, qos.get_publisher_qos(topic_name_, rclcpp::SensorDataQoS()));

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosOpticalMouse::OnUpdate, this, std::placeholders::_1));

  // Rate enforcer
  update_rate_enforcer_.load(update_rate);

  // Initialize time and position markers
  last_time_  = world_->SimTime();
  last_position_ = link_->WorldPose().Pos();
  integrated_position_ = {0, 0, 0};
  sensor_rotation_ = ignition::math::Quaterniond(ignition::math::Vector3d::UnitZ, sensor_rotation);

  RCLCPP_INFO(ros_node_->get_logger(), "Starting optical mouse plugin");
}

// Function is called when the world is reset
void GazeboRosOpticalMouse::Reset() {
  // Is necessary to reset time (it moved backwards)
  last_time_  = world_->SimTime();
  // Update position because it was reset
  last_position_ = link_->WorldPose().Pos();
}

void GazeboRosOpticalMouse::OnUpdate(const gazebo::common::UpdateInfo& info)
{
  const gazebo::common::Time current_time = info.simTime;

  if (current_time < last_time_) {
    RCLCPP_WARN(ros_node_->get_logger(), "Negative update time difference detected.");
    last_time_ = current_time;
  }

  const double time_elapsed = (current_time - last_time_).Double();

  // Check if on this iteration corresponds to send the message
  if (update_rate_enforcer_.shouldUpdate(time_elapsed))
  {
    // Get position
    const ignition::math::Vector3d position = link_->WorldPose().Pos();
    // Position difference with respect to the robot frame
    const ignition::math::Vector3d& delta_distance = sensor_rotation_.RotateVector(position - last_position_);

    // configure an empty message with the timestamp
    irobot_create_msgs::msg::Mouse msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);

    integrated_position_.X(integrated_position_.X() + delta_distance.X());
    integrated_position_.Y(integrated_position_.Y() + delta_distance.Y());

    // Calculate displacement for this iteration
    msg.integrated_x = integrated_position_.X();
    msg.integrated_y = integrated_position_.Y();
    // Publish message
    pub_->publish(msg);

    // Update time and position markers
    last_time_ = current_time;

    // The position is updated according to the resolution of the sensor (i.e. snapped to discrete grid)
    if (msg.integrated_x != 0 || msg.integrated_y != 0) {
      last_position_ = position;
    }
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosOpticalMouse)
}  // namespace irobot_create_gazebo_plugins
