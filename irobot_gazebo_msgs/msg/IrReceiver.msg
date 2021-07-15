# IR receiver message
# Publish the code of the received emitter
std_msgs/Header header
# Name of the emitter ray detected for debugging purposes
string name
# Associated code that it is receiving
uint16 code
# Distance data in meters
float32 distance
