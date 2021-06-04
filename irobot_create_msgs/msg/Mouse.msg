# This message contains a mouse raw measurement.

std_msgs/Header header    # Header stamp should be acquisition time of mouse measure.
float32 integrated_x      # Motions in the X dimension in meters.
float32 integrated_y      # Motions in the Y dimension in meters.
uint32 frame_id           # Value incremented every time mouse stops tracking
uint8 last_squal          # Surface quality level. It's proportional to the number of valid features visible by the sensor.
