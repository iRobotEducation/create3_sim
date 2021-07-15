cmake_minimum_required(VERSION 3.5)
project(irobot_gazebo_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
# Messages
  "msg/Battery.msg"
  "msg/Beacon.msg"
  "msg/Bumper.msg"
  "msg/BumperContact.msg"
  "msg/Cliff.msg"
  "msg/Docking.msg"
  "msg/Duration.msg"
  "msg/FloorSensor.msg"
  "msg/IrRange.msg"
  "msg/IrReceiver.msg"
  "msg/JoystickCommand.msg"
  "msg/MagneticReceiver.msg"
  "msg/Motor.msg"
  "msg/OdoaSim.msg"
  "msg/OpticalMouse.msg"
  "msg/PixartDistance.msg"
  "msg/PosesStamped.msg"
  "msg/RfReceiver.msg"
  "msg/Rtf.msg"
  "msg/SensorEmitterDescriptor.msg"
  "msg/UiEvents.msg"
  "msg/Vector2D.msg"
  "msg/WheelDrop.msg"
  "msg/WheelDropArray.msg"
  "msg/WifiSignal.msg"
#Services
  "srv/CarpetAdder.srv"
  "srv/DoorAngle.srv"
  "srv/DoorState.srv"
  "srv/FastForward.srv"
  "srv/Force.srv"
  "srv/Kidnap.srv"
  "srv/Light.srv"
  "srv/PartitionSpacePoints.srv"
  "srv/Pose.srv"
  "srv/SavePath.srv"
  "srv/Slip.srv"

  DEPENDENCIES geometry_msgs std_msgs)

if(BUILD_TESTING)
  # TODO: Enable linting tests after fix possible errors
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
