# Filtered palm pose of one hand in the operator frame (x forward, y left, z up). The origin is
# arbitrary; only differences are used. Neither position nor orientation is scaled.

string LEFT="left"
string RIGHT="right"

# frame_id is leap_operator.
std_msgs/Header header
string side
geometry_msgs/Pose pose
uint32 sequence
