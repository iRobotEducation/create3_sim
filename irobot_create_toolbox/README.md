# iRobot® Create® 3 toolbox

This package defines a set of extra tools that one might find handy when developing its own robot application.

## Using the teleop_joystick launchfile
As part of the Create3 toolbox we are offering a launchfile with a ready-to-go configuration for the awesome [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy) package.

It can be used by invoking the following command:

```bash
ros2 launch irobot_create_toolbox teleop_joystick.launch.py
```

This will default to an xbox 360 controller, but can be easily overriden using the `joy_config` launchfile argument for any of the supported platforms. As of time of writing this entry, these are:
- Logitech Attack3 (`atk3`)
- Logitech Extreme 3D Pro (`xd3`)
- PS3 (`ps3` or `ps3-holonomic`)
- Xbox 360 (`xbox`)

Example for a PS3 controller:

```bash
ros2 launch irobot_create_toolbox teleop_joystick.launch.py joy_config:=ps3
```

Also, it's possible to select the specific device to use with the `joy_dev` argument. It can be used as follows:

```bash
ros2 launch irobot_create_toolbox teleop_joystick.launch.py joy_dev:=/dev/input/js1
```