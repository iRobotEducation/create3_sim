# iRobot® Create® 3 Ignition Simulator

[![Testing](https://github.com/iRobotSTEM/create3_sim/actions/workflows/ci.yml/badge.svg)](https://github.com/iRobotSTEM/create3_sim/actions/workflows/ci.yml) [![License](https://img.shields.io/github/license/iRobotEducation/create3_sim)](https://github.com/iRobotEducation/create3_sim/blob/main/LICENSE)

This is a ROS 2 Ignition Gazebo simulation stack for the [iRobot® Create® 3](https://edu.irobot.com/create3) robot.

Have a look at the [Create® 3 documentation](https://iroboteducation.github.io/create3_docs/) for more details on the ROS 2 interfaces exposed by the robot.

## Prerequisites

Required dependencies:

1. [ROS 2 galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
2. ROS 2 dev tools:
    - [colcon-common-extensions](https://pypi.org/project/colcon-common-extensions/)
    - [rosdep](https://pypi.org/project/rosdep/): Used to install dependencies when building from sources
    - [vcs](https://pypi.org/project/vcstool/): Automates cloning of git repositories declared on a YAML file.
3. Ignition Gazebo Edifice:
    - [From Source (Recommended)](https://ignitionrobotics.org/docs/edifice/install_ubuntu_src)
    - [Debian](https://ignitionrobotics.org/docs/edifice/install_ubuntu)

## Build

- Create a workspace if you don't already have one:

```bash
mkdir -p ~/create3_ws/src
```

- Clone this repository into the src directory from above.

- Use `vcs` to clone additional dependencies into the workspace:

```bash
vcs import ~/create3_ws/src/ < ~/create3_ws/src/create3_sim/irobot_create_ignition/dependencies.repos
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

#### Depot world

Create® 3 can be spawned in an empty world in Gazebo and monitored through RViz with

```bash
ros2 launch irobot_create_ignition_bringup create3_ignition.launch.py
```
