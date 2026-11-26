# All tracked hands, as reported by ultraleap-hand-tracking-service

# Stamp is arrival time on the ROS clock; frame_id is leap_device_frame.
std_msgs/Header header
uint64 tracking_frame_id
uint64 device_timestamp_us
float32 framerate
bool is_tracking
Hand[] hands

# tracking mode the service reports
string UNKNOWN="unknown"
string DESKTOP="desktop"
string HEAD_MOUNTED="head_mounted"
string tracking_mode
