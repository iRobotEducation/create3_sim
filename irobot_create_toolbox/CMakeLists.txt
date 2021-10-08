cmake_minimum_required(VERSION 3.5)

project(irobot_create_toolbox)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

ament_auto_add_library(vector_publishers_lib SHARED
  src/hazards_vector_publisher.cpp
  src/ir_intensity_vector_publisher.cpp
)
ament_target_dependencies(vector_publishers_lib
  "irobot_create_msgs"
  "rclcpp"
)

ament_auto_add_library(wheel_status_publisher_lib SHARED
  src/wheels_publisher.cpp
)
ament_target_dependencies(wheel_status_publisher_lib
  "control_msgs"
  "irobot_create_msgs"
  "rclcpp"
)

# Hazards vector
ament_auto_add_executable(hazards_vector_publisher_node
  src/hazards_vector_publisher_node.cpp
)
target_link_libraries(hazards_vector_publisher_node vector_publishers_lib)

# IR intensity vector
ament_auto_add_executable(ir_intensity_vector_publisher_node
  src/ir_intensity_vector_publisher_node.cpp
)
target_link_libraries(ir_intensity_vector_publisher_node vector_publishers_lib)

# Wheel status publisher
ament_auto_add_executable(wheel_status_publisher_node
  src/wheels_publisher_node.cpp
)
target_link_libraries(wheel_status_publisher_node wheel_status_publisher_lib)

install(
  DIRECTORY
    include
  DESTINATION include
)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

# Install CPP libraries and executables
install(TARGETS
  hazards_vector_publisher_node
  ir_intensity_vector_publisher_node
  wheel_status_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/topic_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
