# Supervisor diagnosis, published alongside TeleopTarget.

# Supervisor states.
uint8 IDLE=0
uint8 ENGAGED=1
uint8 HOLD=2

# Time of the decision.
std_msgs/Header header

# IDLE, ENGAGED or HOLD.
uint8 state

# Short reason token. Empty while ENGAGED.
string reason

# Hand data is present and fresh.
bool tracking_ok

# A hand is in the field of view.
bool hand_present

# Mirrors TeleopTarget.engaged.
bool engaged
