# One hand as reported by the tracking service.
# Constants are shared by Hand, HandPose and TeleopDelta.

# Hand identifiers used by side fields.
uint8 LEFT=1
uint8 RIGHT=2

# LEFT or RIGHT.
uint8 side

# Cumulative visibility reported by the device, in microseconds.
uint64 visible_time_us

# Palm frame in the device frame, in metres. The quaternion is normalised.
geometry_msgs/Pose palm
