// Copyright 2021 iRobot, Inc.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <irobot_gazebo_plugins/gazebo_ros_optical_mouse.h>

namespace irobot_gazebo_plugins
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
  int seed;
  srand(time(0));

  // Get plugin parameters
  utils::initialize(link_name, sdf, "link", "base_link");
  utils::initialize(update_rate, sdf, "update_rate", 100.0);
  utils::initialize(resolution_, sdf, "resolution", 125.0);
  utils::initialize(sensor_rotation_, sdf, "sensor_rotation", 0.0);
  utils::initialize(gaussian_mean, sdf, "gaussian_mean", 0.0);
  utils::initialize(gaussian_var, sdf, "gaussian_var", 0.0);
  utils::initialize(seed, sdf, "mouse_seed", rand());

  // Get link
  link_ = model->GetLink(link_name);
  GZ_ASSERT(link_, "Couldn't find optical mouse link.");

  // instance of class std::normal_distribution with specific mean and stddev
  d_ = std::normal_distribution<double>(gaussian_mean, gaussian_var);
  // Initialize mt19337 generator with a known seed
  gen_.seed(seed);

  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  ros_node_ = gazebo_ros::Node::Get(sdf);
  // Get QoS profiles
  const gazebo_ros::QoS& qos = ros_node_->get_qos();

  // Initialize ROS publisher
  pub_ = ros_node_->create_publisher<irobot_gazebo_msgs::msg::OpticalMouse>(
      topic_name_, qos.get_publisher_qos("optical_mouse/data", rclcpp::SensorDataQoS()));

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosOpticalMouse::OnUpdate, this, std::placeholders::_1));

  // Rate enforcer
  update_rate_enforcer_.load(update_rate);

  // Initialize time and position markers
  last_time_  = world_->SimTime();
  last_position_ = link_->WorldPose().Pos();

  RCLCPP_INFO(ros_node_->get_logger(), "Starting optical mouse plugin");
}

// Function is called when the world is reset
void GazeboRosOpticalMouse::Reset() {
  // Is necessary to reset time (it moved backwards)
  last_time_  = world_->SimTime();
  // Update position because it was reset
  last_position_ = link_->WorldPose().Pos();
}

inline double GazeboRosOpticalMouse::MetersToCounts(const double distance)
{
  return std::round(distance * M_TO_INCHES * resolution_);
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
    const ignition::math::Vector3d& delta_distance = position - last_position_;

    // configure an empty message with the timestamp
    irobot_gazebo_msgs::msg::OpticalMouse msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);

    double dx_m = 0;
    double dy_m = 0;

    // Calculate displacement for this iteration
    dx_m = std::cos(sensor_rotation_) * delta_distance.X() - std::sin(sensor_rotation_) * delta_distance.Y();
    dy_m = std::sin(sensor_rotation_) * delta_distance.X() + std::cos(sensor_rotation_) * delta_distance.Y();

    // Convert distantance from original meters to CPI (counts per inch) and add gaussian noise
    msg.dx = MetersToCounts(dx_m + d_(gen_));
    msg.dy = MetersToCounts(dy_m + d_(gen_));

    // Publish message
    pub_->publish(msg);

    // Update time and position markers
    last_time_ = current_time;

    // The position is updated according to the resolution of the sensor (i.e. snapped to discrete grid)
    if (msg.dx != 0 || msg.dy != 0) {
      last_position_ = position;
    }
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosOpticalMouse)
}  // namespace irobot_gazebo_plugins
