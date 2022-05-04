cmake_minimum_required(VERSION 3.5)
project(irobot_create_ignition_toolbox)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(irobot_create_toolbox REQUIRED)
find_package(ros_ign_interfaces QUIET)

if(NOT ros_ign_interfaces_FOUND)
  message(WARNING "ros_ign_interfaces not found. Skipping ${PROJECT_NAME} package.")
  ament_package()
  return()
endif()

# Libraries

set(dependencies
  control_msgs
  geometry_msgs
  sensor_msgs
  irobot_create_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
  ros_ign_interfaces
  irobot_create_toolbox
)

# Pose republisher
add_library(irobot_create_ignition_pose_republisher_lib SHARED)
target_sources(
  irobot_create_ignition_pose_republisher_lib
  PRIVATE
    src/pose_republisher/pose_republisher_node.cpp
)
target_include_directories(irobot_create_ignition_pose_republisher_lib PUBLIC include)
ament_target_dependencies(irobot_create_ignition_pose_republisher_lib
  ${dependencies}
)

# Sensors
add_library(irobot_create_ignition_sensors_lib SHARED)
target_sources(
  irobot_create_ignition_sensors_lib
  PRIVATE
    src/sensors/sensors_node.cpp
    src/sensors/bumper.cpp
    src/sensors/cliff.cpp
    src/sensors/ir_intensity.cpp
    src/sensors/ir_opcode.cpp
    src/sensors/mouse.cpp
    src/sensors/wheel_drop.cpp
)
target_include_directories(irobot_create_ignition_sensors_lib PUBLIC include)
ament_target_dependencies(irobot_create_ignition_sensors_lib
  ${dependencies}
)

# Interface buttons
add_library(irobot_create_ignition_interface_buttons_lib SHARED)
target_sources(
  irobot_create_ignition_interface_buttons_lib
  PRIVATE
    src/interface_buttons/interface_buttons_node.cpp
)
target_include_directories(irobot_create_ignition_interface_buttons_lib PUBLIC include)
ament_target_dependencies(irobot_create_ignition_interface_buttons_lib
  ${dependencies}
)

set(libraries_names
  irobot_create_ignition_pose_republisher_lib
  irobot_create_ignition_sensors_lib
  irobot_create_ignition_interface_buttons_lib
)

# Executables

# Pose republisher node
add_executable(pose_republisher_node)
target_sources(
  pose_republisher_node
  PRIVATE
    src/pose_republisher/pose_republisher_main.cpp
)
target_link_libraries(pose_republisher_node irobot_create_ignition_pose_republisher_lib)

# Sensors node
add_executable(sensors_node)
target_sources(
  sensors_node
  PRIVATE
    src/sensors/sensors_main.cpp
)
target_link_libraries(sensors_node irobot_create_ignition_sensors_lib)

# Interface buttons node
add_executable(interface_buttons_node)
target_sources(
  interface_buttons_node
  PRIVATE
    src/interface_buttons/interface_buttons_main.cpp
)
target_link_libraries(interface_buttons_node irobot_create_ignition_interface_buttons_lib)

set(executables_names
  pose_republisher_node
  sensors_node
  interface_buttons_node
)

# Install

install(TARGETS ${libraries_names}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executables_names}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${libraries_names})
ament_export_dependencies(${dependencies})
ament_package()
