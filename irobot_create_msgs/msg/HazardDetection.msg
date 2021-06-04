# This message describes a detected hazard. The frame ID allows to retrieve the location of the sensor that made the detection.

uint8 NONE=0
uint8 BUMP=1
uint8 CLIFF=2
uint8 STALL=3
uint8 WHEEL_DROP=4
uint8 OBJECT_PROXIMITY=5

std_msgs/Header header
uint8 type
