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
find_package(ament_cmake_python REQUIRED)
find_package(control_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)


#### Libraries

set(dependencies
  control_msgs
  irobot_create_msgs
  rclcpp
)

add_library(hazard_publisher_lib SHARED)
target_sources(
  hazard_publisher_lib
  PRIVATE
    src/hazards_vector_publisher_node.cpp
)
target_include_directories(hazard_publisher_lib PUBLIC include)
ament_target_dependencies(hazard_publisher_lib
  ${dependencies}
)

add_library(ir_intensity_publisher_lib SHARED)
target_sources(
  ir_intensity_publisher_lib
  PRIVATE
    src/ir_intensity_vector_publisher_node.cpp
)
target_include_directories(ir_intensity_publisher_lib PUBLIC include)
ament_target_dependencies(ir_intensity_publisher_lib
  ${dependencies}
)

add_library(wheel_status_publisher_lib SHARED)
target_sources(
  wheel_status_publisher_lib
  PRIVATE
    src/wheels_publisher_node.cpp
)
target_include_directories(wheel_status_publisher_lib PUBLIC include)
ament_target_dependencies(wheel_status_publisher_lib
  ${dependencies}
)

set(libraries_names
  hazard_publisher_lib
  ir_intensity_publisher_lib
  wheel_status_publisher_lib
)

#### Executables

# Hazards vector publisher
add_executable(hazards_vector_publisher)
target_sources(
  hazards_vector_publisher
  PRIVATE
    src/hazards_vector_publisher_main.cpp
)
target_link_libraries(hazards_vector_publisher hazard_publisher_lib)

# IR intensity vector publisher
add_executable(ir_intensity_vector_publisher)
target_sources(
  ir_intensity_vector_publisher
  PRIVATE
    src/ir_intensity_vector_publisher_main.cpp
)
target_link_libraries(ir_intensity_vector_publisher ir_intensity_publisher_lib)

# Wheel status publisher
add_executable(wheel_status_publisher)
target_sources(
  wheel_status_publisher
  PRIVATE
    src/wheels_publisher_main.cpp
)
target_link_libraries(wheel_status_publisher wheel_status_publisher_lib)

set(executables_names
  hazards_vector_publisher
  ir_intensity_vector_publisher
  wheel_status_publisher
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
        DESTINATION include)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/topic_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${libraries_names})
ament_export_dependencies(${dependencies})

ament_package()
