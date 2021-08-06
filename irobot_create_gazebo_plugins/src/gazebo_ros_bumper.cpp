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

namespace irobot_create_gazebo_plugins
{
void GazeboRosBumper::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  bumper_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(sensor);
  GZ_ASSERT(bumper_, "Bumper Contact Plugin requires a Contact Sensor");
  ros_node_ = gazebo_ros::Node::Get(sdf);
  bumper_pub_ = ros_node_->create_publisher<irobot_create_msgs::msg::HazardDetection>(
    "~/out", rclcpp::SensorDataQoS());

  // Listen to the update event.
  update_connection_ = bumper_->ConnectUpdated(boost::bind(&GazeboRosBumper::OnUpdate, this));

  // Initialize gazebo node and subscribe to pose topic
  gz_node_.reset(new gazebo::transport::Node());
  gz_node_->Init();
  gz_sub_ = gz_node_->Subscribe("~/pose/local/info", &GazeboRosBumper::GzPoseCallback, this);

  // Set the hazard type a single time.
  msg_.type = irobot_create_msgs::msg::HazardDetection::BUMP;

  // Make sure the parent sensor is active.
  bumper_->SetActive(true);
  RCLCPP_INFO(ros_node_->get_logger(), "Bumper plugin loaded correctly");
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
      RCLCPP_WARN_STREAM(ros_node_->get_logger(), "Global pose callback is not being invoked");
      return;
    }
    const ignition::math::Vector3d r_vec = r_tf_w_.Inverse() * c_vec;
    const double relative_contact_angle_xy = std::atan2(r_vec.Y(), r_vec.X());
    // Check what zone of the bumper has hit an object
    // Only publish if the bump event corresponds to one of the zones
    // "released" events are not published.
    const auto iter = std::find_if(
      angles_map_.begin(), angles_map_.end(),
      [relative_contact_angle_xy](const auto & zone) -> bool {
        return utils::IsAngleBetween(
          zone.second.left_limit, zone.second.right_limit, relative_contact_angle_xy);
      });
    if (iter == angles_map_.end()) {
      return;
    } else {
      msg_.header.frame_id = iter->second.name;
      bumper_pub_->publish(msg_);
      return;
    }
  }
}

void GazeboRosBumper::GzPoseCallback(ConstPosesStampedPtr & msg)
{
  // Proactively update the message's timestamp
  msg_.header.stamp =
    gazebo_ros::Convert<builtin_interfaces::msg::Time>(bumper_->LastMeasurementTime());
  auto & poses = msg->pose();
  // Find in the message's vector a pose element corresponding to the mobile base's absolute pose
  // identified under the "create3" name.
  const auto i = std::find_if(
    poses.begin(), poses.end(), [](const auto & pose) -> bool { return pose.name() == "create3"; });
  // If not matches are found, return immediately.
  if (i == poses.end()) {
    return;
    //  Otherwise, update global pose with the new value.
  } else {
    r_tf_w_ = ignition::math::Matrix4d(ignition::math::Pose3d(
      i->position().x(), i->position().y(), i->position().z(), i->orientation().w(),
      i->orientation().x(), i->orientation().y(), i->orientation().z()));
    return;
  }
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosBumper)

}  // namespace irobot_create_gazebo_plugins
