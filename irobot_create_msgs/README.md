# iRobot Create Platform interfaces

ROS 2 action, message and service definitions used by the iRobot Create Platform.

For more information about ROS 2 interfaces, see [index.ros2.org](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/)

## Actions (.action)
* [DockServo](action/DockServo.action): Command the robot to dock into its charging station.
* [Undock](action/Undock.action): Command the robot to undock from its charging station.
* [WallFollow](action/WallFollow.action): Command the robot to wall follow on left or right side using bump and IR sensors.

## Messages (.msg)
* [Button](msg/Button.msg): Status for a button.
* [Dock](msg/Dock.msg): Information about the robot sensing the its dock charging station.
* [HazardDetection](msg/HazardDetection.msg): An hazard or obstacle detected by the robot.
* [HazardDetectionVector](msg/HazardDetectionVector.msg): All the hazards and obstacles detected by the robot.
* [InterfaceButtons](msg/InterfaceButtons.msg): Status of the 3 interface buttons on the Create faceplate.
* [IrIntensity](msg/IrIntensity.msg): Reading from an IR intensity sensor.
* [IrIntensityVector](msg/IrIntensityVector.msg): Vector of current IR intensity readings from all sensors.
* [IrOpcode](msg/IrOpcode.msg): Opcode detected by the robot IR receivers.
* [KidnapStatus](msg/KidnapStatus.msg): Whether the robot is currently kidnapped or not.
* [LedColor](msg/LedColor.msg): RGB values for an LED.
* [LightringLeds](msg/LightringLeds.msg): Command RGB values of 6 lightring lights.
* [Mouse](msg/Mouse.msg): Reading from a mouse sensor.
* [SlipStatus](msg/SlipStatus.msg): Whether the robot is currently slipping or not.
* [StopStatus](msg/StopStatus.msg): Whether the robot is currently stopped or not.
* [WheelTicks](msg/WheelTicks.msg): Reading from the robot two wheels encoders.
* [WheelVels](msg/WheelVels.msg): Indication about the robot two wheels current speed.

## Services (.srv)
* [EStop](srv/EStop.srv): Set system EStop on or off, cutting motor power when on and enabling motor power when off.
* [RobotPower](srv/RobotPower.srv): Power off robot.

## To be implemented
* AudioNoteCmd
* AudioFileCmd
