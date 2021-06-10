cmake_minimum_required(VERSION 3.5)

project(irobot_create_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(gazebo QUIET)
if(NOT gazebo_FOUND)
  message(WARNING "Gazebo not found, proceeding without that simulator.")
  return()
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    meshes
    rviz
    urdf
  DESTINATION
    share/${PROJECT_NAME}/
)

# Update gazebo resources path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/create3.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/create3.sh.in")

ament_package()
