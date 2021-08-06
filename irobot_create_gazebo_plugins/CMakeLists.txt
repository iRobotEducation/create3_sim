cmake_minimum_required(VERSION 3.5)

project(irobot_create_gazebo_plugins)

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

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Add libraries
add_library(gazebo_ros_create_bumper SHARED
  src/gazebo_ros_bumper.cpp
)
add_library(gazebo_ros_create_helpers SHARED
  src/gazebo_ros_helpers.cpp
)
add_library(gazebo_ros_create_ir_intensity_sensor SHARED
  src/gazebo_ros_ir_intensity_sensor.cpp
)
add_library(gazebo_ros_create_optical_mouse SHARED
src/gazebo_ros_optical_mouse.cpp
)
add_library(gazebo_ros_create_wheel_drop SHARED
  src/gazebo_ros_wheel_drop.cpp
)

## gazebo_ros_create_bumper
target_include_directories(gazebo_ros_create_bumper PUBLIC include)
ament_target_dependencies(gazebo_ros_create_bumper
  "gazebo_dev"
  "gazebo_ros"
  "irobot_create_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_create_bumper)
target_link_libraries(gazebo_ros_create_bumper gazebo_ros_create_helpers)
## gazebo_ros_create_helpers
target_include_directories(gazebo_ros_create_helpers PUBLIC include)
ament_target_dependencies(gazebo_ros_create_helpers
  "gazebo_dev"
  "gazebo_ros"
)
ament_export_libraries(gazebo_ros_create_helpers)
## gazebo_ros_create_ir_intensity_sensor
target_include_directories(gazebo_ros_create_ir_intensity_sensor PUBLIC include)
ament_target_dependencies(gazebo_ros_create_ir_intensity_sensor
  "gazebo_dev"
  "gazebo_ros"
  "irobot_create_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_create_ir_intensity_sensor)
target_link_libraries(gazebo_ros_create_ir_intensity_sensor gazebo_ros_create_helpers)
# gazebo_ros_optical_mouse
target_include_directories(gazebo_ros_create_optical_mouse PUBLIC include)
ament_target_dependencies(gazebo_ros_create_optical_mouse
  "gazebo_dev"
  "gazebo_ros"
  "irobot_create_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_create_optical_mouse)
target_link_libraries(gazebo_ros_create_optical_mouse gazebo_ros_create_helpers)
## gazebo_ros_create_wheel_drop
target_include_directories(gazebo_ros_create_wheel_drop PUBLIC include)
ament_target_dependencies(gazebo_ros_create_wheel_drop
  "gazebo_dev"
  "gazebo_ros"
  "irobot_create_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_create_wheel_drop)
target_link_libraries(gazebo_ros_create_wheel_drop gazebo_ros_create_helpers)

# Exports
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(irobot_create_msgs)
ament_export_dependencies(rclcpp)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_ros_create_bumper
    gazebo_ros_create_helpers
    gazebo_ros_create_ir_intensity_sensor
    gazebo_ros_create_optical_mouse
    gazebo_ros_create_wheel_drop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
