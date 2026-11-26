# One hand as reported by the tracking service.

string LEFT="left"
string RIGHT="right"

string side
uint64 visible_time_us
geometry_msgs/Pose pose  # palm pose in leap_device_frame
float32 pinch_strength  # 0 to 1, thumb against any finger
float32 pinch_distance  # mm, index fingertip to thumb
float32 grab_strength  # 0 to 1, four fingers curled
