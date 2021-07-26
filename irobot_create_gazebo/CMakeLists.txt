cmake_minimum_required(VERSION 3.5)

project(irobot_create_gazebo)

# Error if Gazebo not present
find_package(gazebo QUIET)
if(NOT gazebo_FOUND)
  message(FATAL_ERROR "Gazebo not found.")
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Add libraries
add_library(gazebo_ros_irobot_helpers SHARED
  src/gazebo_ros_helpers.cpp
)
add_library(gazebo_ros_irobot_wheel_drop SHARED
  src/gazebo_ros_wheel_drop.cpp
)

# Linking
## gazebo_ros_irobot_helpers
target_include_directories(gazebo_ros_irobot_helpers PUBLIC include)
ament_target_dependencies(gazebo_ros_irobot_helpers
  "gazebo_dev"
  "gazebo_ros"
)
ament_export_libraries(gazebo_ros_irobot_helpers)
## gazebo_ros_irobot_wheel_drop
target_include_directories(gazebo_ros_irobot_wheel_drop PUBLIC include)
ament_target_dependencies(gazebo_ros_irobot_wheel_drop
  "gazebo_dev"
  "gazebo_ros"
  "irobot_create_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_irobot_wheel_drop)
target_link_libraries(gazebo_ros_irobot_wheel_drop gazebo_ros_irobot_helpers)

# Exports
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(irobot_create_msgs)
ament_export_dependencies(rclcpp)

install(DIRECTORY include/
        DESTINATION include)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

install(TARGETS
    gazebo_ros_irobot_helpers
    gazebo_ros_irobot_wheel_drop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
