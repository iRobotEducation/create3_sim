cmake_minimum_required(VERSION 3.5)

project(irobot_create_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(action_files
  "action/DockServo.action"
  "action/Undock.action"
  "action/WallFollow.action"
)

set(msg_files
  "msg/Button.msg"
  "msg/Dock.msg"
  "msg/HazardDetection.msg"
  "msg/HazardDetectionVector.msg"
  "msg/InterfaceButtons.msg"
  "msg/IrIntensity.msg"
  "msg/IrIntensityVector.msg"
  "msg/IrOpcode.msg"
  "msg/KidnapStatus.msg"
  "msg/LedColor.msg"
  "msg/LightringLeds.msg"
  "msg/Mouse.msg"
  "msg/SlipStatus.msg"
  "msg/StopStatus.msg"
  "msg/WheelTicks.msg"
  "msg/WheelVels.msg"
)

set(srv_files
  "srv/EStop.srv"
  "srv/RobotPower.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES action_msgs builtin_interfaces std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
