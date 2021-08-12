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
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#pragma once

#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <rclcpp/rclcpp.hpp>

class HazardsVectorPublisher
{
public:
  /// Constructor
  explicit HazardsVectorPublisher(std::string name);
  void add_msg(std::shared_ptr<irobot_create_msgs::msg::HazardDetection> msg);
  void clear_msgs();


private:
  irobot_create_msgs::msg::HazardDetectionVector msg_;
};
