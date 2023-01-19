cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)

# Find Qt5
find_package(Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
)

qt5_add_resources(resources_rcc Create3Hmi.qrc)

add_library(
  Create3Hmi
  SHARED
    ${headers_MOC}
    Create3Hmi.cc
    ${resources_rcc}
)
include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Qml_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${Qt5QuickControls2_INCLUDE_DIRS}
)
target_link_libraries(
  Create3Hmi
    ${Qt5Core_LIBRARIES}
    ${Qt5Qml_LIBRARIES}
    ${Qt5Quick_LIBRARIES}
    ${Qt5QuickControls2_LIBRARIES}
)
ament_target_dependencies(
  Create3Hmi
    ignition-common4
    ignition-gazebo6
    ignition-plugin1
)

install(
  TARGETS Create3Hmi
  DESTINATION share/${PROJECT_NAME}/lib
)
