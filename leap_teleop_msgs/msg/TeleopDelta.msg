# Robot-agnostic command contract. Translation is added and rotation is
# composed with the pose latched by the bridge; this is not a full transform.

# Hand identifiers used by side fields.
string LEFT="left"
string RIGHT="right"

# Supervisor states.
string IDLE="idle"
string ENGAGED="engaged"
string HOLD="hold"

# Rotation composition orders.
string WORLD="world"
string TOOL="tool"

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

# WORLD or TOOL, telling the bridge how to compose delta with the latched pose.
string rotation_frame

# Increasing counter, one per published target.
uint32 sequence
