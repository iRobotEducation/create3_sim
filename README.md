# iRobot® Create® 3 Simulator

[![Testing](https://github.com/iRobotSTEM/create3_sim/actions/workflows/ci.yml/badge.svg)](https://github.com/iRobotSTEM/create3_sim/actions/workflows/ci.yml) [![License](https://img.shields.io/github/license/iRobotEducation/create3_sim)](https://github.com/iRobotEducation/create3_sim/blob/main/LICENSE)

This is a ROS 2 simulation stack for the [iRobot® Create® 3](https://edu.irobot.com/create3) robot.
Only Gazebo Harmonic is supported.

Have a look at the [Create® 3 documentation](https://iroboteducation.github.io/create3_docs/) for more details on the ROS 2 interfaces exposed by the robot.

## Prerequisites

Required dependencies:

1. [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
2. ROS 2 dev tools:
    - [colcon-common-extensions](https://pypi.org/project/colcon-common-extensions/)
    - [rosdep](https://pypi.org/project/rosdep/): Used to install dependencies when building from sources
    - [vcs](https://pypi.org/project/vcstool/): Automates cloning of git repositories declared on a YAML file.

Besides the aforementioned dependencies you will also need Gazebo Harmonic.

#### Gazebo Harmonic

```bash
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ros-gz
```

## Build

- Create a workspace if you don't already have one:

```bash
mkdir -p ~/create3_ws/src
```

- Clone this repository into the src directory from above.
- Clone the [`irobot_create_msgs` repository](https://github.com/iRobotEducation/irobot_create_msgs) into the workspace

- Navigate to the workspace and install ROS 2 dependencies with:

```bash
cd ~/create3_ws
sudo apt-get update
rosdep install --from-path src -yi
```

- Build the workspace with:

```bash
colcon build --symlink-install
source install/local_setup.bash
```

## Run

#### Gazebo Harmonic

Create® 3 can be spawned in a demo world in Gazebo and monitored through RViz with

```bash
ros2 launch irobot_create_gz_bringup create3_gz.launch.py
```

The spawn point can be changed with the `x`, `y`, `z` and `yaw` launch arguments:

```bash
ros2 launch irobot_create_gz_bringup create3_gz.launch.py x:=1.0 y:=0.5 yaw:=1.5707
```

##### Namespacing

A namespace can be applied to the robot using the `namespace` launch argument:

```bash
ros2 launch irobot_create_gz_bringup create3_gz.launch.py namespace:=my_robot
```

Multiple robots can be spawned with unique namespaces:

```bash
ros2 launch irobot_create_gz_bringup create3_gz.launch.py namespace:=robot1
ros2 launch irobot_create_gz_bringup create3_spawn.launch.py namespace:=robot2 x:=1.0
```

> :warning: `create3_gz.launch.py` should only be used once as it launches the Ignition simulator itself. Additional robots should be spawned with `create3_spawn.launch.py`. Namespaces and spawn points should be unique for each robot.

## Package layout

This repository contains packages for both the Classic and Ignition Gazebo simulators:

- `irobot_create_common` Packages common to both Classic and Ignition
    - `irobot_create_common_bringup` Launch files and configurations
    - `irobot_create_control` Launch control nodes
    - `irobot_create_description`  URDF and mesh files describing the robot
    - `irobot_create_nodes` Nodes for simulating robot topics and motion control
    - `irobot_create_toolbox` Tools and helpers for creating nodes and plugins

- `irobot_create_gz` Packages used for the Gazebo Harmonic Simulator
    - `irobot_create_gz_bringup` Launch files and configurations
    - `irobot_create_gz_plugins` GUI plugins
    - `irobot_create_gz_sim`  Metapackage
    - `irobot_create_gz_toolbox` Sensor and interface nodes
