// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Justin Kearns (jkearns@irobot.com)

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_nodes/motion_control/reflex_behavior.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace irobot_create_nodes
{

using namespace std::placeholders;

ReflexBehavior::ReflexBehavior(
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<BehaviorsScheduler> behavior_scheduler)
: clock_{node_clock_interface->get_clock()},
  tf_buffer_{tf_buffer},
  logger_{node_logging_interface->get_logger()},
  max_reflex_runtime_{rclcpp::Duration(std::chrono::seconds(10))},
  max_continuous_reflex_runtime_{rclcpp::Duration(std::chrono::seconds(30))},
  moving_check_duration_{rclcpp::Duration(std::chrono::seconds(1))}
{
  behavior_scheduler_ = behavior_scheduler;

  // Declare ROS 2 parameters and setup the system according to default values
  this->declare_parameters(node_parameters_interface);

  // Register a callback to handle parameter changes
  params_callback_handle_ = node_parameters_interface->add_on_set_parameters_callback(
    std::bind(&ReflexBehavior::set_parameters_callback, this, _1));

  // Give poses default value, will be over-written by subscriptions
  reflex_start_pose_.setIdentity();
  last_robot_pose_.setIdentity();
  reflex_start_time_ = clock_->now();
  last_moving_check_time_ = clock_->now();
}

bool ReflexBehavior::reflex_behavior_is_done()
{
  return !running_reflex_;
}

void ReflexBehavior::declare_parameters(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = false;
  rclcpp::ParameterValue ret;

  // Declare parameters to enable or disable each of the supported reflexes
  for (const auto & reflex_hazard : reflex_names_to_hazard_) {
    uint8_t hazard_type = reflex_hazard.second;
    bool default_enabled = hazard_type != std::numeric_limits<uint8_t>::max();
    const std::string reflex_param_name = std::string("reflexes.") + reflex_hazard.first;
    ret =
      node_parameters_interface->declare_parameter(
      reflex_param_name, rclcpp::ParameterValue(
        default_enabled), descriptor);
    if (!default_enabled && ret.get<bool>()) {
      RCLCPP_ERROR(
        logger_, "Trying to enable reflex: '%s'. This is not supported yet.",
        reflex_hazard.first.c_str());
      throw std::runtime_error("User tried to enable reflexes. This are not supported yet.");
    }
    this->enable_reflex(hazard_type, ret.get<bool>());
  }

  // Declare parameter to enable or disable the reflex arbiter
  ret = node_parameters_interface->declare_parameter(
    reflex_enabled_param_name_, rclcpp::ParameterValue(
      true), descriptor);
  reflexes_enabled_ = ret.get<bool>();
}

rcl_interfaces::msg::SetParametersResult ReflexBehavior::set_parameters_callback(
  std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  bool updated_param = false;

  for (const rclcpp::Parameter & parameter : parameters) {
    bool invalid_delete = false;
    rclcpp::ParameterType parameter_type = parameter.get_type();
    if (rclcpp::ParameterType::PARAMETER_NOT_SET == parameter_type) {
      invalid_delete = true;
    }
    // Check if string starts with reflexes.
    const std::string & param_name = parameter.get_name();
    if (param_name.rfind("reflexes.", 0) == 0) {
      // Get string after reflexes.
      std::size_t dot_pos = param_name.find(".");
      // Get string starting after .
      std::string reflex_name = param_name.substr(dot_pos + 1);
      auto it = reflex_names_to_hazard_.find(reflex_name);
      if (it != reflex_names_to_hazard_.end()) {
        if (!invalid_delete) {
          this->enable_reflex(it->second, parameter.get_value<bool>());
          updated_param = true;
        }
      }
    } else if (parameter.get_name().compare(reflex_enabled_param_name_) == 0) {
      if (!invalid_delete) {
        reflexes_enabled_ = parameter.get_value<bool>();
        updated_param = true;
      }
    }
    if (!updated_param) {
      // not handled by this component
      continue;
    } else if (invalid_delete) {
      RCLCPP_WARN(logger_, "Can't delete parameter '%s'", parameter.get_name().c_str());
      result.successful = false;
      result.reason = "parameter \'" + parameter.get_name() + "\' cannot be deleted";
    }
  }
  return result;
}

void ReflexBehavior::enable_reflex(uint8_t reflex, bool enable)
{
  const std::lock_guard<std::mutex> lock(hazard_reflex_mutex_);
  std::map<uint8_t, bool>::iterator hazard_reflex = hazard_reflex_enabled_.find(reflex);
  if (hazard_reflex != hazard_reflex_enabled_.end()) {
    hazard_reflex->second = enable;
    // Print reflex being enabled/disabled
    for (const auto & name : reflex_names_to_hazard_) {
      RCLCPP_INFO_STREAM_EXPRESSION(
        logger_, name.second == reflex && enable,
        "Enabling " << name.first);
      RCLCPP_INFO_STREAM_EXPRESSION(
        logger_, name.second == reflex && !enable,
        "Disabling " << name.first);
    }
  }
}

BehaviorsScheduler::optional_output_t ReflexBehavior::execute_reflex(
  const RobotState & current_state)
{
  BehaviorsScheduler::optional_output_t servo_cmd;
  enum class DriveAwayDirection
  {
    NO_HAZARD_TO_ESCAPE,
    PURE_BACKUP,
    ARC_CLOCKWISE,
    ARC_COUNTER_CLOCKWISE
  };
  DriveAwayDirection drive_dir = DriveAwayDirection::NO_HAZARD_TO_ESCAPE;
  {
    // Check active reflexes
    const std::lock_guard<std::mutex> lock(hazard_mutex_);
    for (const auto & hazard : last_hazards_) {
      switch (hazard.type) {
        case irobot_create_msgs::msg::HazardDetection::BUMP:
          {
            drive_dir = DriveAwayDirection::PURE_BACKUP;
            break;
          }
        case irobot_create_msgs::msg::HazardDetection::CLIFF:
          {
            boost::optional<tf2::Vector3> hazard_offset =
              get_pose_relative_to_odom(hazard);
            if (hazard_offset) {
              if (hazard_offset->getY() > 0) {
                if (drive_dir == DriveAwayDirection::ARC_COUNTER_CLOCKWISE) {
                  drive_dir = DriveAwayDirection::PURE_BACKUP;
                } else {
                  drive_dir = DriveAwayDirection::ARC_CLOCKWISE;
                }
              } else {
                if (drive_dir == DriveAwayDirection::ARC_CLOCKWISE) {
                  drive_dir = DriveAwayDirection::PURE_BACKUP;
                } else {
                  drive_dir = DriveAwayDirection::ARC_COUNTER_CLOCKWISE;
                }
              }
            }
            break;
          }
        case irobot_create_msgs::msg::HazardDetection::STALL:
          {
            drive_dir = DriveAwayDirection::PURE_BACKUP;
            break;
          }
        case irobot_create_msgs::msg::HazardDetection::WHEEL_DROP:
          {
            drive_dir = DriveAwayDirection::PURE_BACKUP;
            break;
          }
        case irobot_create_msgs::msg::HazardDetection::BACKUP_LIMIT:
        case irobot_create_msgs::msg::HazardDetection::OBJECT_PROXIMITY:
          {
            // We do not trigger a reflex on these types of hazards
            break;
          }
      }
      if (drive_dir == DriveAwayDirection::PURE_BACKUP) {
        // This is overriding escape, so won't be further modified
        break;
      }
    }
  }
  bool finish_reflex = false;
  // Get next command
  // See if robot has done too much without clearing hazard
  double distance_since_hazard = 0.0;
  double distance_since_continuous_hazard = 0.0;
  if (driving_backwards_) {
    tf2::Vector3 delta_pose = current_state.pose.getOrigin() - reflex_start_pose_.getOrigin();
    distance_since_hazard = std::hypot(delta_pose.getX(), delta_pose.getY());
    tf2::Vector3 delta_continuous_pose = current_state.pose.getOrigin() -
      continuous_reflex_start_pose_.getOrigin();
    distance_since_continuous_hazard = std::hypot(
      delta_continuous_pose.getX(), delta_continuous_pose.getY());
  } else {
    tf2::Transform relative_motion = reflex_start_pose_.inverseTimes(current_state.pose);
    if (relative_motion.getOrigin().getX() < 0.0) {
      driving_backwards_ = true;
    }
  }
  const bool exceeded_distance = distance_since_hazard > MAX_REFLEX_DISTANCE ||
    distance_since_continuous_hazard > MAX_CONTINUOUS_REFLEX_DISTANCE;
  rclcpp::Time current_time = clock_->now();
  const bool exceeded_time = (current_time - reflex_start_time_ > max_reflex_runtime_) ||
    (current_time - continuous_reflex_start_time_ > max_continuous_reflex_runtime_);
  const bool cleared_hazed_with_min_travel_achieved =
    drive_dir == DriveAwayDirection::NO_HAZARD_TO_ESCAPE &&
    distance_since_hazard > MIN_REFLEX_DISTANCE;
  if (cleared_hazed_with_min_travel_achieved) {
    finish_reflex = true;
  } else if (exceeded_distance) {
    RCLCPP_WARN(logger_, "Reflex Exceeded Max Travel Distance without clearing hazard");
    finish_reflex = true;
  } else if (exceeded_time) {
    RCLCPP_WARN(logger_, "Reflex Exceeded Runtime without clearing hazard");
    finish_reflex = true;
  }

  if (finish_reflex) {
    running_reflex_ = false;
  } else if (drive_dir != DriveAwayDirection::NO_HAZARD_TO_ESCAPE) {
    // Pick velocity to escape hazards based on hazard types and locations
    servo_cmd = geometry_msgs::msg::Twist();
    switch (drive_dir) {
      case DriveAwayDirection::PURE_BACKUP:
        {
          servo_cmd->linear.x = BACKUP_X_VELOCITY;
          break;
        }
      case DriveAwayDirection::ARC_CLOCKWISE:
        {
          servo_cmd->linear.x = ARC_X_VELOCITY;
          servo_cmd->angular.z = -ARC_ANGULAR_VELOCITY;
          break;
        }
      case DriveAwayDirection::ARC_COUNTER_CLOCKWISE:
        {
          servo_cmd->linear.x = ARC_X_VELOCITY;
          servo_cmd->angular.z = ARC_ANGULAR_VELOCITY;
          break;
        }
      case DriveAwayDirection::NO_HAZARD_TO_ESCAPE:  // this code path is covered above
        {
          break;
        }
    }
  } else {
    // If here, we don't have a hazard, but haven't traveled the min distance
    // keep last command
    servo_cmd = last_reflex_cmd_;
  }
  last_reflex_cmd_ = servo_cmd;

  return servo_cmd;
}

void ReflexBehavior::update_hazards(const RobotState & current_state)
{
  // Update if robot is moving
  rclcpp::Time clock_now = clock_->now();
  if (clock_now - last_moving_check_time_ > moving_check_duration_) {
    last_moving_check_time_ = clock_now;
    tf2::Vector3 last_position = last_robot_pose_.getOrigin();
    tf2::Vector3 current_position = current_state.pose.getOrigin();
    tf2::Vector3 pos_diff = current_position - last_position;
    double pos_dist = std::hypot(pos_diff.getX(), pos_diff.getY());
    double abs_ang_diff = std::abs(
      angles::shortest_angular_distance(
        tf2::getYaw(current_state.pose.getRotation()),
        tf2::getYaw(last_robot_pose_.getRotation())));
    if (pos_dist < NOT_MOVING_DISTANCE &&
      abs_ang_diff < NOT_MOVING_ANGLE)
    {
      moving_ = false;
    } else {
      moving_ = true;
    }
    last_robot_pose_ = current_state.pose;
  }
  std::vector<irobot_create_msgs::msg::HazardDetection> active_hazards;
  for (const auto & hazard : current_state.hazards.detections) {
    const std::lock_guard<std::mutex> lock(hazard_reflex_mutex_);
    // See if reflex for this hazard is enabled
    std::map<uint8_t, bool>::const_iterator hazard_reflex =
      hazard_reflex_enabled_.find(hazard.type);
    if (hazard_reflex == hazard_reflex_enabled_.end()) {
      // hazard doesn't have corresponding reflex
      continue;
    }
    if (hazard_reflex->second) {
      // Reflex is active for this hazard
      active_hazards.emplace_back(hazard);
    }
  }
  if (running_reflex_) {
    // Just cache the data for processing by execute_reflex
    const std::lock_guard<std::mutex> lock(hazard_mutex_);
    last_hazards_ = active_hazards;
  } else {
    // Don't trigger reflex is robot isn't driving
    if (!moving_ || !reflexes_enabled_) {return;}
    // If hazard is triggered for active reflex, start reflex
    if (active_hazards.size() > 0) {
      // See if there is an overlap with last trigger hazards, in which case
      // hazards failed to clear from last reflex behavior
      bool matching_previous_hazard = false;
      for (const auto & last_hazard : last_trigger_hazards_) {
        for (const auto & hazard : active_hazards) {
          if ((last_hazard.type == hazard.type) &&
            (last_hazard.header.frame_id == hazard.header.frame_id))
          {
            matching_previous_hazard = true;
            break;
          }
        }
        if (matching_previous_hazard) {
          break;
        }
      }
      if (!matching_previous_hazard) {
        // new hazards compared to last run
        bool continuous_reflex = last_trigger_hazards_.size() > 0;
        {
          const std::lock_guard<std::mutex> lock(hazard_mutex_);
          last_hazards_ = active_hazards;
          last_trigger_hazards_ = last_hazards_;
        }
        BehaviorsScheduler::BehaviorsData data;
        data.run_func = std::bind(&ReflexBehavior::execute_reflex, this, _1);
        data.is_done_func = std::bind(&ReflexBehavior::reflex_behavior_is_done, this);
        data.stop_on_new_behavior = false;

        running_reflex_ = behavior_scheduler_->set_behavior(data);
        if (running_reflex_) {
          reflex_start_time_ = clock_->now();
          {
            const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
            reflex_start_pose_ = current_state.pose;
          }
          driving_backwards_ = false;
          if (!continuous_reflex) {
            // If starting fresh, reset continuous reflex starting points
            // otherwise, will continue to check continous thresholds against
            // previous start points
            continuous_reflex_start_pose_ = reflex_start_pose_;
            continuous_reflex_start_time_ = reflex_start_time_;
          }
        }
      } else {
        // If rejecting from previous match, then reset continuous start points
        // as no longer running continuously, but keep triggered sensors
        {
          const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
          continuous_reflex_start_pose_ = current_state.pose;
        }
        continuous_reflex_start_time_ = clock_->now();
      }
    } else {
      // If no hazards triggered, clear history to let any hazard trigger reflex
      last_trigger_hazards_.clear();
    }
  }
}

boost::optional<tf2::Vector3> ReflexBehavior::get_pose_relative_to_odom(
  const irobot_create_msgs::msg::HazardDetection & hazard)
{
  try {
    geometry_msgs::msg::TransformStamped odom_hazard =
      tf_buffer_->lookupTransform(
      hazard.header.frame_id, odom_frame_,
      hazard.header.stamp);
    geometry_msgs::msg::TransformStamped odom_now =
      tf_buffer_->lookupTransform(
      base_frame_, odom_frame_,
      rclcpp::Time(0));
    // Get difference between when hazard was observed and current odometry
    tf2::Stamped<tf2::Transform> odom_hazard_tf;
    tf2::fromMsg(odom_hazard, odom_hazard_tf);
    tf2::Stamped<tf2::Transform> odom_now_tf;
    tf2::fromMsg(odom_now, odom_now_tf);
    tf2::Transform diff_tf = odom_now_tf.inverseTimes(odom_hazard_tf);
    return diff_tf.getOrigin();
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(
      logger_,
      "No Transform available Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(
      logger_,
      "Connectivity Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(
      logger_,
      "Extrapolation Error looking up target frame: %s\n", ex.what());
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR(
      logger_,
      "Transform timeout");
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger_, "Failed to transform from %s to %s",
      odom_frame_.c_str(), hazard.header.frame_id.c_str());
  }

  return boost::optional<tf2::Vector3>();
}

}  // namespace irobot_create_nodes
