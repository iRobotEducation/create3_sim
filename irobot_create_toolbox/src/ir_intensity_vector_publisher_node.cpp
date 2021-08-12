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

#include <irobot_create_msgs/msg/ir_intensity.hpp>
#include <irobot_create_msgs/msg/ir_intensity_vector.hpp>
#include <irobot_create_toolbox/ir_intensity_vector_publisher.hpp>
#include <irobot_create_toolbox/vector_publisher.hpp>

int main(int argc, char * argv[])
{
  using IrIntensityVectorPublisher = VectorPublisher<
    irobot_create_msgs::msg::IrIntensity, irobot_create_msgs::msg::IrIntensityVector,
    IrIntensityVectorPublisher>;

  rclcpp::init(argc, argv);
  // Instantiate a VectorPublisher node specialized to handle IR intensity readings and pass that
  // node to spin().
  rclcpp::spin(std::make_shared<IrIntensityVectorPublisher>());
  rclcpp::shutdown();
  return 0;
}
