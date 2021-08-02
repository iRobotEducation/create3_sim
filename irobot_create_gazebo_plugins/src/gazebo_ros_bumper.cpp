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
// @author Alexis Pojomovsky (apojomovsky@irobot.com)

#include <irobot_create_gazebo_plugins/gazebo_ros_bumper.hpp>

namespace irobot_gazebo_plugins
{
void GazeboRosBumper::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  bumper_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(sensor);
  if (!bumper_) {
    gzerr << "Bumper Contact Plugin requires a Contact Sensor.\n";
    return;
  }
  rosnode_ = gazebo_ros::Node::Get(sdf);
  const std::string bumper_topic_name = "/bumper";
  const gazebo_ros::QoS & qos = rosnode_->get_qos();
  bumper_pub_ = rosnode_->create_publisher<irobot_create_msgs::msg::HazardDetection>(
    bumper_topic_name, qos.get_publisher_qos(bumper_topic_name, rclcpp::SensorDataQoS()));

  // Listen to the update event.
  update_connection_ = bumper_->ConnectUpdated(boost::bind(&GazeboRosBumper::OnUpdate, this));

  // Initialize gazebo node and subscribe to pose topic
  gz_node_.reset(new gazebo::transport::Node());
  gz_node_->Init();
  gz_sub_ = gz_node_->Subscribe("~/pose/local/info", &GazeboRosBumper::GzPoseCallback, this);

  // Make sure the parent sensor is active.
  bumper_->SetActive(true);
  RCLCPP_INFO(rosnode_->get_logger(), "Bumper plugin loaded correctly");
}

void GazeboRosBumper::OnUpdate()
{
  // Get all contacts from bumper
  gazebo::msgs::Contacts contacts;
  contacts = bumper_->Contacts();

  for (int i = 0; i < contacts.contact_size(); ++i) {
    // Get collision point in world reference frame
    const ignition::math::Vector3d c_vec =
      gazebo::msgs::ConvertIgn(contacts.contact(i).position(0));
    // Get collision w.r.t. robot frame
    if (r_tf_w_ == ignition::math::Matrix4d::Zero) {
      RCLCPP_WARN_STREAM(rosnode_->get_logger(), "Global pose callback is not being invoked");
      return;
    }
    const ignition::math::Vector3d r_vec = r_tf_w_.Inverse() * c_vec;
    const double relative_contact_angle_xy = std::atan2(r_vec.Y(), r_vec.X());
    // Check what zone of the bumper has bumped
    // Only publish if the bump event corresponds to one of the zones
    // "released" events are not publsihed.
    for (auto & bumper_zone : bumper_angles_map) {
      if (IsAngleBetween(
            relative_contact_angle_xy, bumper_zone.second.left_limit,
            bumper_zone.second.right_limit)) {
        // Fill and publish a message
        msg_.header.frame_id = bumper_zone.second.name;
        msg_.type = irobot_create_msgs::msg::HazardDetection::BUMP;
        bumper_pub_->publish(msg_);
        break;
      }
    }
  }
}

void GazeboRosBumper::GzPoseCallback(ConstPosesStampedPtr & msg)
{
  // Proactively update the message's timestamp
  msg_.header.stamp =
    gazebo_ros::Convert<builtin_interfaces::msg::Time>(bumper_->LastMeasurementTime());

  // Iterate over the pose's msg
  for (auto & pose : msg->pose()) {
    // Stop when the create3 is found, since it's the one in world coordinates.
    if (pose.name() == "create3") {
      // Update the global pose object
      r_tf_w_ = ignition::math::Matrix4d(ignition::math::Pose3d(
        pose.position().x(), pose.position().y(), pose.position().z(), pose.orientation().w(),
        pose.orientation().x(), pose.orientation().y(), pose.orientation().z()));
      return;
    }
  }
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosBumper)

}  // namespace irobot_gazebo_plugins
