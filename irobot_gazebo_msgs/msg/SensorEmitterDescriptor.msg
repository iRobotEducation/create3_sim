# IR emitter descriptor
# Describes an IR emitter characteristics and its position
# Name of the emitter
string name
# Type of device associated (for example if it is a wall barrier or dock)
string device_type
# Associated code that it is emitting
uint16 code
# Aperture of the emission (from 0 to 2 PI)
float32 aperture
# Range of the device
float32 range
# Position and orientation
geometry_msgs/Pose pose
