# Filtered palm pose of the selected hand in base axes. The origin is arbitrary; only differences
# are used. Position is already multiplied by the tracker's translation_scale; orientation is
# unscaled.

string LEFT="left"
string RIGHT="right"

# frame_id is leap_teleop_frame.
std_msgs/Header header
string side
geometry_msgs/Pose pose
# Tracker's rotation_scale. The supervisor multiplies the rotation since engagement by it.
float64 rotation_scale
uint32 sequence
