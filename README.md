# iRobot® Create® 3 Simulator

[![Testing](https://github.com/iRobotSTEM/create3_sim/actions/workflows/ci.yml/badge.svg)](https://github.com/iRobotSTEM/create3_sim/actions/workflows/ci.yml) [![License](https://img.shields.io/github/license/iRobotEducation/create3_sim)](https://github.com/iRobotEducation/create3_sim/blob/main/LICENSE)

This is a ROS 2 simulation stack for the [iRobot® Create® 3](https://edu.irobot.com/create3) robot.
Both Ignition Gazebo and Classic Gazebo are supported.

Have a look at the [Create® 3 documentation](https://iroboteducation.github.io/create3_docs/) for more details on the ROS 2 interfaces exposed by the robot.

## Prerequisites

Required dependencies:

1. [ROS 2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. ROS 2 dev tools:
    - [colcon-common-extensions](https://pypi.org/project/colcon-common-extensions/)
    - [rosdep](https://pypi.org/project/rosdep/): Used to install dependencies when building from sources
    - [vcs](https://pypi.org/project/vcstool/): Automates cloning of git repositories declared on a YAML file.

Besides the aforementioned dependencies you will also need at least one among Ignition Gazebo and Classic Gazebo

#### Classic Gazebo

Install [Gazebo 11](http://gazebosim.org/tutorials?tut=install_ubuntu)

#### Ignition Fortress

```bash
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```

## Build

- Create a workspace if you don't already have one:

```bash
mkdir -p ~/create3_ws/src
```

- Clone this repository into the src directory from above.

- Navigate to the workspace and install ROS 2 dependencies with:

```bash
cd ~/create3_ws
sudo apt-get update
rosdep install --from-path src -yi
```

- Build the workspace with:

```bash
export IGNITION_VERSION=fortress
colcon build --symlink-install
source install/local_setup.bash
```

## Run

#### Classic Gazebo

##### Empty world

Create® 3 can be spawned in an empty world in Gazebo and monitored through RViz with:

```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py
```

The spawn point can be changed with the `x`, `y`, `z` and `yaw` launch arguments:

```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py x:=1.0 y:=0.5 yaw:=1.5707
```

##### Namespacing

A namespace can be applied to the robot using the `namespace` launch argument:

```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py namespace:=my_robot
```

Multiple robots can be spawned with unique namespaces:

```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py namespace:=robot1
ros2 launch irobot_create_gazebo_bringup create3_spawn.launch.py namespace:=robot2 x:=1.0
```

> :warning: `create3_gazebo.launch.py` should only be used once as it launches the Gazebo simulator itself. Additional robots should be spawned with `create3_spawn.launch.py`. Namespaces and spawn points should be unique for each robot.

##### AWS house

Create® 3 can be spawned in the AWS small house in Gazebo and monitored through RViz.
This requires that the package `aws_robomaker_small_house_world` is available.

If you need it, you can build `aws_robomaker_small_house_world` in your ROS 2 workspace by doing:
```bash
vcs import ~/create3_ws/src/ < ~/create3_ws/src/create3_sim/irobot_create_gazebo/demo.repos
cd ~/create3_ws
colcon build --symlink-install
source install/local_setup.bash
```

Then you can run:

```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo_aws_small.launch.py
```

#### Ignition Gazebo

Create® 3 can be spawned in a demo world in Ignition and monitored through RViz with

```bash
ros2 launch irobot_create_ignition_bringup create3_ignition.launch.py
```

The spawn point can be changed with the `x`, `y`, `z` and `yaw` launch arguments:

```bash
ros2 launch irobot_create_ignition_bringup create3_ignition.launch.py x:=1.0 y:=0.5 yaw:=1.5707
```

##### Namespacing

A namespace can be applied to the robot using the `namespace` launch argument:

```bash
ros2 launch irobot_create_ignition_bringup create3_ignition.launch.py namespace:=my_robot
```

Multiple robots can be spawned with unique namespaces:

```bash
ros2 launch irobot_create_ignition_bringup create3_ignition.launch.py namespace:=robot1
ros2 launch irobot_create_ignition_bringup create3_spawn.launch.py namespace:=robot2 x:=1.0
```

> :warning: `create3_ignition.launch.py` should only be used once as it launches the Ignition simulator itself. Additional robots should be spawned with `create3_spawn.launch.py`. Namespaces and spawn points should be unique for each robot.

## Package layout

This repository contains packages for both the Classic and Ignition Gazebo simulators:

- `irobot_create_common` Packages common to both Classic and Ignition
    - `irobot_create_common_bringup` Launch files and configurations
    - `irobot_create_control` Launch control nodes
    - `irobot_create_description`  URDF and mesh files describing the robot
    - `irobot_create_nodes` Nodes for simulating robot topics and motion control
    - `irobot_create_toolbox` Tools and helpers for creating nodes and plugins

- `irobot_create_gazebo` Packages used for the Classic Gazebo Simulator
    - `irobot_create_gazebo_bringup` Launch files and configurations
    - `irobot_create_gazebo_plugins` Sensor plugins
    - `irobot_create_gazebo_sim`  Metapackage

- `irobot_create_ignition` Packages used for the Ignition Gazebo Simulator
    - `irobot_create_ignition_bringup` Launch files and configurations
    - `irobot_create_ignition_plugins` GUI plugins
    - `irobot_create_ignition_sim`  Metapackage
    - `irobot_create_ignition_toolbox` Sensor and interface nodes
