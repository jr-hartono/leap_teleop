# All tracked hands, for diagnostics. Published at 10 Hz with is_tracking false when the device
# or service is gone, so a dead device differs from an empty view.

# Stamp is arrival time on the ROS clock; frame_id is leap_device_frame.
std_msgs/Header header
uint64 tracking_frame_id
uint64 device_timestamp_us
float32 framerate
bool is_tracking
Hand[] hands
