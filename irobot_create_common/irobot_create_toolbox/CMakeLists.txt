cmake_minimum_required(VERSION 3.5)

project(irobot_create_toolbox)

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
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

#### Libraries

set(dependencies
  angles
  control_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  irobot_create_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
)

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

set(libraries_names
  hazard_publisher_lib
  ir_intensity_publisher_lib
  motion_control_lib
  wheel_status_publisher_lib
  robot_state_lib
  mock_publisher_lib
  kidnap_estimator_publisher_lib
)

#### Executables

# Hazards vector publisher
add_executable(hazards_vector_publisher_node)
target_sources(
  hazards_vector_publisher_node
  PRIVATE
    src/hazards_vector_publisher_node.cpp
)
target_link_libraries(hazards_vector_publisher_node hazard_publisher_lib)

# IR intensity vector publisher
add_executable(ir_intensity_vector_publisher_node)
target_sources(
  ir_intensity_vector_publisher_node
  PRIVATE
    src/ir_intensity_vector_publisher_node.cpp
)
target_link_libraries(ir_intensity_vector_publisher_node ir_intensity_publisher_lib)

# Motion control node
add_executable(motion_control)
target_sources(
  motion_control
  PRIVATE
    src/motion_control_main.cpp
)
target_link_libraries(motion_control motion_control_lib)

# Wheel status publisher
add_executable(wheel_status_publisher_node)
target_sources(
  wheel_status_publisher_node
  PRIVATE
    src/wheels_publisher_node.cpp
)
target_link_libraries(wheel_status_publisher_node wheel_status_publisher_lib)

# Robot status publisher
add_executable(robot_state_node)
target_sources(
  robot_state_node
  PRIVATE
    src/robot_state_node.cpp
)
target_link_libraries(robot_state_node robot_state_lib)

# Mock publisher
add_executable(mock_publisher_node)
target_sources(
  mock_publisher_node
  PRIVATE
    src/mock_publisher_node.cpp
)
target_link_libraries(mock_publisher_node mock_publisher_lib)

# Kidnap estimator publisher
add_executable(kidnap_estimator_publisher_node)
target_sources(
  kidnap_estimator_publisher_node
  PRIVATE
    src/kidnap_estimator_publisher_node.cpp
)
target_link_libraries(kidnap_estimator_publisher_node kidnap_estimator_publisher_lib)

set(executables_names
  hazards_vector_publisher_node
  ir_intensity_vector_publisher_node
  motion_control
  wheel_status_publisher_node
  robot_state_node
  mock_publisher_node
  kidnap_estimator_publisher_node
)

#### Install

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

ament_export_include_directories(include)
ament_export_libraries(${libraries_names})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
