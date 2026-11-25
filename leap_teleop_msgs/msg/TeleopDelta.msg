# Robot-agnostic command contract. Translation is added and rotation is
# composed with the pose latched by the bridge; this is not a full transform.

# Hand identifiers used by side fields.
string LEFT="left"
string RIGHT="right"

# Supervisor states.
string IDLE="idle"
string ENGAGED="engaged"
string HOLD="hold"

# Stamp is propagated from HandPose; frame_id is base_frame.
std_msgs/Header header

# Copied from HandPose, informational at the bridge.
string side

# IDLE, ENGAGED or HOLD.
string state

# True only while motion is authorised.
bool engaged

# Base-frame displacement since latch. Identity at engage.
geometry_msgs/Transform delta

# Increasing counter, one per published target.
uint32 sequence
