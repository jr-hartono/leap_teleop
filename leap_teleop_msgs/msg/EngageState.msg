# Deadman key state, published by leap_teleop_engage.
# Each message is valid for supervisor.engage_timeout.

# Time of the key state read, on the ROS clock.
builtin_interfaces/Time stamp

# True while the deadman key is held.
bool engaged

# Input backend, always "evdev" in v0.1.
string source
