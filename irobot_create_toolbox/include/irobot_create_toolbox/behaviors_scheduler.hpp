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

#ifndef IROBOT_CREATE_TOOLBOX__BEHAVIORS_SCHEDULER_HPP_
#define IROBOT_CREATE_TOOLBOX__BEHAVIORS_SCHEDULER_HPP_

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
    m_has_behavior = false;
  }

  bool set_behavior(const BehaviorsData & data)
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    if (m_has_behavior) {
      return false;
    }

    if (!data.run_func || !data.is_done_func) {
      return false;
    }

    m_has_behavior = true;
    m_current_behavior = data;
    return true;
  }

  bool has_behavior()
  {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_has_behavior;
  }

  optional_output_t run_behavior()
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    if (!m_has_behavior) {
      return optional_output_t();
    }

    optional_output_t output = m_current_behavior.run_func();

    if (m_current_behavior.is_done_func()) {
      m_has_behavior = false;
    }

    return output;
  }

private:
  std::mutex m_mutex;
  bool m_has_behavior;
  BehaviorsData m_current_behavior;
};

}  // namespace irobot_create_toolbox
#endif   // IROBOT_CREATE_TOOLBOX__BEHAVIORS_SCHEDULER_HPP_
