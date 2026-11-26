# Filtered state of one tracked hand, as used for teleoperation.

# frame_id is leap_operator_frame.
std_msgs/Header header
# filtered palm pose in the operator frame (x forward, y left, z up). The origin is
# arbitrary; only differences are used. Neither position nor orientation is scaled.
geometry_msgs/Pose pose
# filtered pinch closure, 0 open to 1 closed
float32 closure
