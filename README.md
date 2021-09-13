# iRobot Create 3

[![Linter](https://github.com/iRobotSTEM/create3_sim/actions/workflows/lint.yml/badge.svg)](https://github.com/iRobotSTEM/create3_sim/actions/workflows/lint.yml) [![Testing](https://github.com/iRobotSTEM/create3_sim/actions/workflows/ci.yml/badge.svg)](https://github.com/iRobotSTEM/create3_sim/actions/workflows/ci.yml) [![License](https://img.shields.io/github/license/iRobotSTEM/create3_sim)](https://github.com/iRobotSTEM/create3_sim/blob/master/LICENSE)

This is a [ROS 2](https://docs.ros.org/en/foxy/index.html) simulation stack for the [iRobot Create 3]() robot.

## Prerequisites

1. Ros 2 ([foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) or [galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)): it's recommended to install the desktop version of the distribution of your choosing, this will also install RViz 2. Bare in mind that if another version is installed, some dependencies may be missing.
2. [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu)
3. [RViz2](https://github.com/ros2/rviz): this is included as part of the rosdep dependecies.

## Build

- Create a workspace if you don't already have one:

```bash
mkdir -p ~/colcon_ws/src
```

- Clone this repository into the src directory from above.

- Navigate to the workspace and install ros2 dependencies with:

```bash
cd ~/colcon_ws
rosdep install --from-path src -yi
```

- Build the workspace with:

```bash
colcon build --symlink-install
source install/local_setup.bash
```

## Examples

Create 3 can be spawned in an empty world in Gazebo and RViz with

```bash
ros2 launch irobot_create_gazebo create3.launch.py
```

## [Contributions](CONTRIBUTING.md)