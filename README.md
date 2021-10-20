# iRobot® Create® 3 Simulator

[![Testing](https://github.com/iRobotSTEM/create3_sim/actions/workflows/ci.yml/badge.svg)](https://github.com/iRobotSTEM/create3_sim/actions/workflows/ci.yml) [![License](https://img.shields.io/github/license/iRobotEducation/create3_sim)](https://github.com/iRobotEducation/create3_sim/blob/main/LICENSE)

This is a ROS 2 simulation stack for the [iRobot® Create® 3](https://edu.irobot.com/create3) robot.

## Prerequisites

Required dependencies:

1. [Ros 2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html): it's recommended to install the desktop version of the distribution of your choosing, this will also install RViz 2. Bare in mind that if another version is installed, some dependencies may be missing.
2. [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu)
3. [RViz2](https://github.com/ros2/rviz): this is included as part of the rosdep dependecies.
4. ROS 2 dev tools:
    - [colcon-common-extensions](https://pypi.org/project/colcon-common-extensions/)
    - [rosdep](https://pypi.org/project/rosdep/): Used to install dependencies when building from sources
    - [vcs](https://pypi.org/project/vcstool/): Automates cloning of git repositories declared on a YAML file.

Optional dependencies:

1. [xterm](https://manpages.ubuntu.com/manpages/xenial/man1/xterm.1.html): Used to teleoperate the robot with the keyboard.

## Build

- Create a workspace if you don't already have one:

```bash
mkdir -p ~/create3_ws/src
```

- Clone this repository into the src directory from above.

- Use `vcs` to clone additional dependencies into the workspace:

```bash
vcs import ~/create3_ws/src/ < ~/create3_ws/src/create3_sim/dependencies.repos
```

- Navigate to the workspace and install ROS 2 dependencies with:

```bash
cd ~/create3_ws
rosdep install --from-path src -yi
```

- Build the workspace with:

```bash
colcon build --symlink-install
source install/local_setup.bash
```

## Examples

#### Empty world

Create® 3 can be spawned in an empty world in Gazebo and monitored through RViz with

```bash
ros2 launch irobot_create_gazebo create3.launch.py
```

#### AWS house

Create® 3 can be spawned in the AWS small house in Gazebo and monitored through RViz.
This requires that the package `aws_robomaker_small_house_world` is available.

If you need it, you can build `aws_robomaker_small_house_world` in your ROS 2 workspace by doing:
```bash
vcs import ~/create3_ws/src/ < ~/create3_ws/src/create3_sim/demo.repos
cd ~/create3_ws
colcon build --symlink-install
source install/local_setup.bash
```

Then you can run:

```bash
ros2 launch irobot_create_gazebo aws_small.launch.py
```
