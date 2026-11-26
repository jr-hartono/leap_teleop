# Mapper state, for diagnostics.

string IDLE="idle"
string ENGAGED="engaged"
string HOLD="hold"

std_msgs/Header header
string state
string reason
bool tracking_ok
bool hand_present
bool engaged
