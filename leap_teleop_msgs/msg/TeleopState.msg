# Supervisor diagnosis, published alongside TeleopDelta.

# Supervisor states.
string IDLE="idle"
string ENGAGED="engaged"
string HOLD="hold"

# Time of the decision.
std_msgs/Header header

# IDLE, ENGAGED or HOLD.
string state

# Short reason token. Empty while ENGAGED; retrigger_required while HOLD with good data.
string reason

# Hand data is present and fresh.
bool tracking_ok

# A hand is in the field of view.
bool hand_present

# Mirrors TeleopDelta.engaged.
bool engaged
