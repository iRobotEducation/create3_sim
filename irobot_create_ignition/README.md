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
### Ignition Edifice

#### From Source (Recommended)
- Create a workspace
```bash
mkdir -p ~/ignition_ws/src
```

- Use `vcs` to clone Ignition Edifice dependencies into the workspace:
```bash
cd ~/ignition_ws
wget https://raw.githubusercontent.com/iRobotEducation/create3_sim/ignition/irobot_create_ignition/ignition_edifice.repos
vcs import ~/ignition_ws/src < ~/ignition_ws/ignition_edifice.repos
```

- Install additional dependencies:
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ')
```

- Build the Ignition libraries:
```bash
cd ~/ignition_ws
source /opt/ros/galactic/setup.bash
colcon build --merge-install
```

 #### Binary
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-edifice
```

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

- Source the Ignition workspace if installed from source:
```bash
source ~/ignition_ws/install/setup.bash
```

- Build the workspace with:

```bash
colcon build --symlink-install
source install/local_setup.bash
```

## Examples

#### Depot world

Create® 3 can be spawned in a demo world in Ignition and monitored through RViz with

```bash
ros2 launch irobot_create_ignition_bringup create3_ignition.launch.py
```
