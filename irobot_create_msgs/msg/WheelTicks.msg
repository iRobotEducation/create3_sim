# This message contains wheel ticks raw measurements.
# It can be used both for publishing integrated as well as absolute encoder readings.

std_msgs/Header header      # Header stamp should be acquisition time of measure.
int32 ticks_left            # Count of encoder ticks for left wheel.
int32 ticks_right           # Count of encoder ticks for right wheel.
