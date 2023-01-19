cmake_minimum_required(VERSION 3.5)

project(irobot_create_nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(irobot_create_toolbox REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)

#### Libraries

set(dependencies
  angles
  control_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  irobot_create_msgs
  irobot_create_toolbox
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2
  tf2_geometry_msgs
  tf2_ros
  Boost
)

# Hazards vector publisher
add_library(hazard_publisher_lib SHARED)
target_sources(
  hazard_publisher_lib
  PRIVATE
    src/hazards_vector_publisher.cpp
)
target_include_directories(hazard_publisher_lib PUBLIC include)
ament_target_dependencies(hazard_publisher_lib
  ${dependencies}
)

rclcpp_components_register_node(hazard_publisher_lib
  PLUGIN "irobot_create_nodes::HazardsVectorPublisher"
  EXECUTABLE hazards_vector_publisher)

# IR intensity vector publisher
add_library(ir_intensity_publisher_lib SHARED)
target_sources(
  ir_intensity_publisher_lib
  PRIVATE
    src/ir_intensity_vector_publisher.cpp
)
target_include_directories(ir_intensity_publisher_lib PUBLIC include)
ament_target_dependencies(ir_intensity_publisher_lib
  ${dependencies}
)

rclcpp_components_register_node(ir_intensity_publisher_lib
  PLUGIN "irobot_create_nodes::IrIntensityVectorPublisher"
  EXECUTABLE ir_intensity_vector_publisher)

# Motion control node
add_library(motion_control_lib SHARED)
target_sources(
  motion_control_lib
  PRIVATE
    src/motion_control_node.cpp
    src/motion_control/docking_behavior.cpp
    src/motion_control/drive_goal_behaviors.cpp
    src/motion_control/reflex_behavior.cpp
    src/motion_control/wall_follow_behavior.cpp
    src/motion_control/wall_follow_states.cpp
)
target_include_directories(motion_control_lib PUBLIC include)
ament_target_dependencies(motion_control_lib
  ${dependencies}
)

rclcpp_components_register_node(motion_control_lib
  PLUGIN "irobot_create_nodes::MotionControlNode"
  EXECUTABLE motion_control)

# Wheel status publisher
add_library(wheel_status_publisher_lib SHARED)
target_sources(
  wheel_status_publisher_lib
  PRIVATE
    src/wheels_publisher.cpp
)
target_include_directories(wheel_status_publisher_lib PUBLIC include)
ament_target_dependencies(wheel_status_publisher_lib
  ${dependencies}
)

rclcpp_components_register_node(wheel_status_publisher_lib
  PLUGIN "irobot_create_nodes::WheelsPublisher"
  EXECUTABLE wheel_status_publisher)

# Robot status publisher
add_library(robot_state_lib SHARED)
target_sources(
  robot_state_lib
  PRIVATE
    src/robot_state.cpp
)
target_include_directories(robot_state_lib PUBLIC include)
ament_target_dependencies(robot_state_lib
  ${dependencies}
)

rclcpp_components_register_node(robot_state_lib
  PLUGIN "irobot_create_nodes::RobotState"
  EXECUTABLE robot_state)

# UI Mgr Node
add_library(ui_mgr_lib SHARED)
target_sources(
  ui_mgr_lib
  PRIVATE
    src/ui_mgr.cpp
)
target_include_directories(ui_mgr_lib PUBLIC include)
ament_target_dependencies(ui_mgr_lib
  ${dependencies}
)

rclcpp_components_register_node(ui_mgr_lib
  PLUGIN "irobot_create_nodes::UIMgr"
  EXECUTABLE ui_mgr)

# Mock publisher
add_library(mock_publisher_lib SHARED)
target_sources(
  mock_publisher_lib
  PRIVATE
    src/mock_publisher.cpp
)
target_include_directories(mock_publisher_lib PUBLIC include)
ament_target_dependencies(mock_publisher_lib
  ${dependencies}
)

rclcpp_components_register_node(mock_publisher_lib
  PLUGIN "irobot_create_nodes::MockPublisher"
  EXECUTABLE mock_publisher)

# Kidnap estimator publisher
add_library(kidnap_estimator_publisher_lib SHARED)
target_sources(
  kidnap_estimator_publisher_lib
  PRIVATE
    src/kidnap_estimator_publisher.cpp
)
target_include_directories(kidnap_estimator_publisher_lib PUBLIC include)
ament_target_dependencies(kidnap_estimator_publisher_lib
  ${dependencies}
)

rclcpp_components_register_node(kidnap_estimator_publisher_lib
  PLUGIN "irobot_create_nodes::KidnapEstimator"
  EXECUTABLE kidnap_estimator_publisher)

set(libraries_names
  hazard_publisher_lib
  ir_intensity_publisher_lib
  motion_control_lib
  wheel_status_publisher_lib
  robot_state_lib
  ui_mgr_lib
  mock_publisher_lib
  kidnap_estimator_publisher_lib
)

#### Install

install(TARGETS ${libraries_names}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${libraries_names})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
