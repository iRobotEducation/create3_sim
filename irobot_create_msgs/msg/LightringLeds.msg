# This message sets the RGB values of the lightright LEDs
std_msgs/Header header
# If you look at the lightring as an analog clock with 12 at the front
# of the robot
# led index 0 is 4 o'clock,
# led index 1 is 2 o'clock,
# led index 2 is 12 o'clock,
# led index 3 is 10 o'clock,
# led index 4 is 8 o'clock,
# led index 5 is 6 o'clock
irobot_create_msgs/LedColor[6] leds
# Whether to override system's default light patterns with given command
# Set to false to return control of lights to system
bool override_system
