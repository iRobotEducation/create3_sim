# Real Time Factor message definition
#
# The rtf is the RTF that is displayed on the Gazebo UI. It is actually
# different from the RTF API that Gazebo exposes (which is literally
# sim_time/real_time). Using that API is not ideal because that does not
# account for sudden changes in RTF because it uses overall sim_time/real_time
# so we are using the same algorithm Gazebo UI's RTF uses, which is very sensitive
# to quick realtime changes.
std_msgs/Header header
float32 rtf
