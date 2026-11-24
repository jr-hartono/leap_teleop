# Hand pose after yaw, gains, filtering and deadband, in base axes.
# The origin of this frame is arbitrary: only differences of pose are used.

# Hand identifiers used by side fields.
uint8 LEFT=1
uint8 RIGHT=2

# Stamp is the hand frame arrival time; frame_id is leap_teleop/hand.
std_msgs/Header header

# LEFT or RIGHT, owned by the tracker.
uint8 side

# Pose in metres and radians, in the operator frame.
geometry_msgs/Pose pose

# Increasing counter, one per published target.
uint32 sequence
