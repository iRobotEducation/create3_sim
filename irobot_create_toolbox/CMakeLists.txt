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
find_package(irobot_create_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Add libraries
add_library(hazards_vector SHARED
  src/hazards_vector.cpp
)

# Add executables
add_executable(hazards_vector_node src/hazards_vector_node.cpp)

# Linking
## hazards_vector
target_include_directories(hazards_vector PUBLIC include)
ament_target_dependencies(hazards_vector
  "irobot_create_msgs"
  "rclcpp"
)

target_link_libraries(hazards_vector_node hazards_vector)

# Exports
ament_export_include_directories(include)
ament_export_dependencies(irobot_create_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(hazards_vector)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
  hazards_vector
  hazards_vector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/topic_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
