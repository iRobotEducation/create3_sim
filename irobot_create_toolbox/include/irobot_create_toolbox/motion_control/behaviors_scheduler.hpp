// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Alberto Soragna (asoragna@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__MOTION_CONTROL__BEHAVIORS_SCHEDULER_HPP_
#define IROBOT_CREATE_TOOLBOX__MOTION_CONTROL__BEHAVIORS_SCHEDULER_HPP_

#include <boost/optional.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <functional>
#include <mutex>

namespace irobot_create_toolbox
{

/**
 * @brief This class manages the execution of a single behavior at the time.
 */
class BehaviorsScheduler
{
public:
  using optional_output_t = boost::optional<geometry_msgs::msg::Twist>;
  using run_behavior_func_t = std::function<optional_output_t()>;
  using is_done_func_t = std::function<bool ()>;

  // A behavior implementation needs to provide the following data in order to be used
  struct BehaviorsData
  {
    // This function is executed each iteration to generate new velocity commands
    run_behavior_func_t run_func;
    // This function is executed at the end of each iteration to know if we need to run again
    is_done_func_t is_done_func;
  };

  BehaviorsScheduler()
  {
  }

  bool set_behavior(const BehaviorsData & data)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (has_behavior_) {
      return false;
    }

    if (!data.run_func || !data.is_done_func) {
      return false;
    }

    has_behavior_ = true;
    current_behavior_ = data;
    return true;
  }

  bool has_behavior()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return has_behavior_;
  }

  optional_output_t run_behavior()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!has_behavior_) {
      return optional_output_t();
    }

    optional_output_t output = current_behavior_.run_func();

    if (current_behavior_.is_done_func()) {
      has_behavior_ = false;
    }

    return output;
  }

private:
  std::mutex mutex_;
  bool has_behavior_ {false};
  BehaviorsData current_behavior_;
};

}  // namespace irobot_create_toolbox
#endif   // IROBOT_CREATE_TOOLBOX__MOTION_CONTROL__BEHAVIORS_SCHEDULER_HPP_
