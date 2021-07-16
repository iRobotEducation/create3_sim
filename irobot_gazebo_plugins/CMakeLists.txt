cmake_minimum_required(VERSION 3.5)
project(irobot_gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Add libraries
add_library(gazebo_ros_helpers SHARED
  src/gazebo_ros_helpers.cpp
)

add_library(gazebo_ros_irobot_optical_mouse SHARED
  src/gazebo_ros_optical_mouse.cpp
)

# Linking

# gazebo_ros_helpers
target_include_directories(gazebo_ros_helpers PUBLIC include)
ament_target_dependencies(gazebo_ros_helpers
  "gazebo_dev"
  "gazebo_ros"
)
ament_export_libraries(gazebo_ros_helpers)
if(ENABLE_PROFILER)
  target_include_directories(gazebo_ros_helpers PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(gazebo_ros_helpers ${ignition-common3_LIBRARIES})
endif()

# gazebo_ros_optical_mouse
target_include_directories(gazebo_ros_irobot_optical_mouse PUBLIC include)
ament_target_dependencies(gazebo_ros_irobot_optical_mouse
  "gazebo_dev"
  "gazebo_ros"
  "irobot_create_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_irobot_optical_mouse)
target_link_libraries(gazebo_ros_irobot_optical_mouse gazebo_ros_helpers)
if(ENABLE_PROFILER)
  target_include_directories(gazebo_ros_irobot_optical_mouse PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(gazebo_ros_irobot_optical_mouse ${ignition-common3_LIBRARIES})
endif()


# Exports
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)

set(AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
    "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_ros_helpers
    gazebo_ros_irobot_optical_mouse
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
