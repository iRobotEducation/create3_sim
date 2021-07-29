#include <irobot_create_gazebo_plugins/gazebo_ros_ir_intensity_sensor.hpp>


namespace irobot_gazebo_plugins
{
  // Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosIrIntensitySensor)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosIrIntensitySensor::GazeboRosIrIntensitySensor()
  : SensorPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIrIntensitySensor::~GazeboRosIrIntensitySensor()
{
  new_laser_scans_connection_.reset();
}

// Load the controller
void GazeboRosIrIntensitySensor::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor);
  new_laser_scans_connection_ =
    parent_sensor_->LaserShape()->ConnectNewLaserScans(
      std::bind(&GazeboRosIrIntensitySensor::OnNewLaserScans, this));

  // Configure the plugin from the SDF file
  ros_node_ = gazebo_ros::Node::Get(sdf);

  // Initialize ROS publishers
  pub_ = ros_node_->create_publisher<irobot_create_msgs::msg::IrIntensity>(
    "~/out", rclcpp::SensorDataQoS());

  // Configure our static message charasteristics
  msg_.header.frame_id = gazebo_ros::SensorFrameID(*sensor, *sdf);

  RCLCPP_INFO(ros_node_->get_logger(), "Starting IR Emitter Plugin!");
}

// On each sensor iteration
void GazeboRosIrIntensitySensor::OnNewLaserScans()
{
  msg_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    parent_sensor_->LastMeasurementTime());
  // Set range to the minimum of the ray ranges
  // For single rays, this will just be the range of the ray
  msg_.value = parent_sensor_->RangeMax();
  std::vector<double> ranges;
  parent_sensor_->Ranges(ranges);
  for (const double & range : ranges) {
    if (range < msg_.value) {
      msg_.value = range;
    }
  }
  // Publish
  pub_->publish(msg_);
}

}  // namespace irobot_gazebo_plugins
