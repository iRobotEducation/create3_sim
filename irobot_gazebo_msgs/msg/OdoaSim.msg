# ODOA simulator message
#
# Number of detected elements (necessary for json decoding)
int16 elements_number

# bool to indicate whether dealing with hazard or obstacle (using int16 because of ros bridge issues)
int16[] is_hazard

# max and min corners of the obstacle's bounding box
geometry_msgs/Point[] bb_min
geometry_msgs/Point[] bb_max