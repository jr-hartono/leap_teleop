# Pose change since engagement, the input of every robot bridge. Translation is added to the
# latched tool position; rotation is composed in base axes about the tool origin.

string LEFT="left"
string RIGHT="right"

string IDLE="idle"
string ENGAGED="engaged"
string HOLD="hold"

std_msgs/Header header
string side
string state
bool engaged
geometry_msgs/Transform delta
uint32 sequence
