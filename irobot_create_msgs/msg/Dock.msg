# This message contains information about whether the robot is docked or sees the dock in its sensors.

std_msgs/Header header  # Header stamp is when dock info was queried.
bool dock_visible       # Whether robot sees dock in its sensors used for docking
bool is_docked          # Whether the robot is docked
