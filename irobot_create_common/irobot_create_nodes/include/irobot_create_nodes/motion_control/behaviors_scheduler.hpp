// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Alberto Soragna (asoragna@irobot.com)

#ifndef IROBOT_CREATE_NODES__MOTION_CONTROL__BEHAVIORS_SCHEDULER_HPP_
#define IROBOT_CREATE_NODES__MOTION_CONTROL__BEHAVIORS_SCHEDULER_HPP_

#include <atomic>
#include <functional>
#include <mutex>

#include "boost/optional.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "tf2/LinearMath/Transform.h"

namespace irobot_create_nodes
{

struct RobotState
{
  tf2::Transform pose;
  irobot_create_msgs::msg::HazardDetectionVector hazards;
};

/**
 * @brief This class manages the execution of a single behavior at the time.
 */
class BehaviorsScheduler
{
public:
  using optional_output_t = boost::optional<geometry_msgs::msg::Twist>;
  using run_behavior_func_t = std::function<optional_output_t(const RobotState &)>;
  using is_done_func_t = std::function<bool ()>;
  using cleanup_func_t = std::function<void ()>;

  // A behavior implementation needs to provide the following data in order to be used
  struct BehaviorsData
  {
    // This function is executed each iteration to generate new velocity commands
    run_behavior_func_t run_func;
    // This function is executed at the end of each iteration to know if we need to run again
    is_done_func_t is_done_func;
    // This function is executed when a behavior is stopped externally
    cleanup_func_t cleanup_func;
    // Whether a new behavior should stop this behavior (true) or if behavior should prevent
    // new ones from running till finished (false)
    bool stop_on_new_behavior;
    // Whether backup limits should be applied to behavior
    bool apply_backup_limits;
  };

  BehaviorsScheduler()
  {
  }

  bool set_behavior(const BehaviorsData & data)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!data.run_func || !data.is_done_func) {
      return false;
    }

    // If already has behavior, only take new behavior if old behavior is stop_on_new_behavior
    // and new behavior is not
    if (has_behavior_) {
      if (!current_behavior_.stop_on_new_behavior) {
        // Reject new behavior to keep running current
        return false;
      } else if (current_behavior_.cleanup_func) {
        // If behavior has to cleanup before moving onto another, run func
        current_behavior_.cleanup_func();
      }
    }

    has_behavior_ = true;
    current_behavior_ = data;
    return true;
  }

  bool has_behavior()
  {
    return has_behavior_;
  }

  bool apply_backup_limits()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return current_behavior_.apply_backup_limits;
  }

  bool stop_on_new_behavior()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return current_behavior_.stop_on_new_behavior;
  }

  optional_output_t run_behavior(const RobotState & current_state)
  {
    if (!has_behavior_) {
      return optional_output_t();
    }

    std::unique_lock<std::mutex> lock(mutex_);
    optional_output_t output = current_behavior_.run_func(current_state);

    if (current_behavior_.is_done_func()) {
      has_behavior_ = false;
    }

    return output;
  }

private:
  std::mutex mutex_;
  std::atomic<bool> has_behavior_ {false};
  BehaviorsData current_behavior_;
};

}  // namespace irobot_create_nodes
#endif   // IROBOT_CREATE_NODES__MOTION_CONTROL__BEHAVIORS_SCHEDULER_HPP_
