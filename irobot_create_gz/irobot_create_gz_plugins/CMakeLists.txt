cmake_minimum_required(VERSION 3.8)
project(irobot_create_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(OpenGL_GL_PREFERENCE LEGACY)

# Find the Ignition gui library
find_package(gz_gui_vendor REQUIRED)
add_subdirectory(Create3Hmi)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
