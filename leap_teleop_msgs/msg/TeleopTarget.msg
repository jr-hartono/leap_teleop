# Robot-agnostic command contract. Translation is added and rotation is
# composed with the pose latched by the bridge; this is not a full transform.

# Hand identifiers used by side fields.
uint8 LEFT=1
uint8 RIGHT=2

# Supervisor states.
uint8 IDLE=0
uint8 ENGAGED=1
uint8 HOLD=2

# Rotation composition orders.
uint8 WORLD=0
uint8 TOOL=1

# Stamp is propagated from HandTarget; frame_id is base_frame.
std_msgs/Header header

# Copied from HandTarget, informational at the bridge.
uint8 side

# IDLE, ENGAGED or HOLD.
uint8 state

# True only while motion is authorised.
bool engaged

# Base-frame displacement since latch. Identity at engage.
geometry_msgs/Transform delta

# WORLD or TOOL, telling the bridge how to compose delta with the latched pose.
uint8 rotation_frame

# Increasing counter, one per published target.
uint32 sequence
