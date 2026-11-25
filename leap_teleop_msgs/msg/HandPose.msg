# Filtered palm pose of one hand in the operator frame (x forward, y left, z up). The origin is
# arbitrary; only differences are used. Neither position nor orientation is scaled. The hand is
# given by the topic, hand_pose/left or hand_pose/right.

# frame_id is leap_operator_frame.
std_msgs/Header header
geometry_msgs/Pose pose
