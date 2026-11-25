# Filtered palm pose of the selected hand in base axes. The origin is arbitrary; only differences
# are used.

string LEFT="left"
string RIGHT="right"

# frame_id is leap_teleop_frame.
std_msgs/Header header
string side
geometry_msgs/Pose pose
uint32 sequence
