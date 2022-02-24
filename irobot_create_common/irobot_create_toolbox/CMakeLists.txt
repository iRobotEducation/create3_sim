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
find_package(ignition-math6 REQUIRED)
find_package(rclcpp REQUIRED)

#### Libraries

set(dependencies
  ignition-math6
  rclcpp
)

add_library(irobot_create_toolbox SHARED)
target_sources(
  irobot_create_toolbox
  PRIVATE
    src/polar_coordinates.cpp
)
target_include_directories(irobot_create_toolbox PUBLIC include)
ament_target_dependencies(irobot_create_toolbox
  ${dependencies}
)

#### Install

install(TARGETS irobot_create_toolbox
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(irobot_create_toolbox)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
