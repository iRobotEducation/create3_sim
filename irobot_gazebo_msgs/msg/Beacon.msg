# Beacon message
std_msgs/Header header
# Beacon ID
uint8 id
# Shock: Beacon moved. In general it is 0 but if the beacon moves, it will become 1.
uint8 shock
# Angle in radians between the vector that joins the robot's antenna and the beacon antenna, and the normal plane of robot's antenna.
float64 elevation
# Angle in radians of the beacon. 0 = a normal vertical beacon.  pi/2 = horizontal beacon lying on the ground
float64 pitch
# Range: Distance from 1cm of the top of the beacon antenna to one centimeter to the top of the robot's antenna.
float64 range
# Double that described the angle from the beacon's antenna to the robot's antenna with respect to the robot reference(axle x positive)
float64 azimuth
# ASCII character, generally it is  alater between A and H.
uint8 label
# Value of the battery charge.
float64 battery
# Position of the beacon relative to the world frame
geometry_msgs/Point position
