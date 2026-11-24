# All hands in one tracking frame, published for diagnostics.
# Kept publishing at 10 Hz with is_tracking false after device loss, so a dead
# device is distinguishable from an empty field of view and from a dead tracker.

# Stamp is arrival time on the ROS clock; frame_id is leap_device_frame.
std_msgs/Header header

# Device frame identifier, diagnostic only.
uint64 tracking_frame_id

# Tracking service clock in microseconds. Not a ROS time.
uint64 device_timestamp_us

# Estimated device frame rate in Hz. Zero before estimation.
float32 framerate

# False when the device or tracking service is unavailable.
bool is_tracking

# Hands present in this frame. An empty array is valid.
Hand[] hands
