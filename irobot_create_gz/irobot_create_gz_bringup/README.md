# turtlebot4_gz

To launch the simulation, run
```bash
ros2 launch irobot_create_gz_bringup create3_gz.launch.py
```

Wait for the simulation environment to load completely, then press the orange `play` button in the
lower-left corner to start the simulation.  The robot starts docked on its charger.

## Worlds

The default simulation world is the `depot` environment. To chage worlds, use the `world`
argument.  Supported worlds are:
- `depot` (default)
- `maze`

## Robot spawn location

By default the robot spawns at the origin.  To change the robot's spawn location, use the
`x`, `y`, `z` and `yaw` arguments.  Note that changing `z` may result in the robot spawning
in the air or below the ground plane, depending in the value used.