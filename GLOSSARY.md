# leap_teleop

Teleoperation of robots using hand tracking from the original Leap Motion Controller.
This glossary fixes the words used in code, topics, messages and documentation.

## Tracking

**Tracking frame**:
One frame of hand data from the Leap service, identified by a device frame id and a
device timestamp.
_Avoid_: sample, capture, tick

**Hand pose**:
The palm position and orientation for one hand, as reported by the tracker in the device
frame.
_Avoid_: hand state, skeleton

**Palm frame**:
The body-fixed frame carried by the palm. Its axes follow the LeapC basis
`{normal × direction, −normal, −direction}`; for a flat palm-down hand pointing away
from the operator it coincides with the device frame.
_Avoid_: hand axes, local frame

## Teleoperation

**Engagement**:
The operator's deliberate act of authorising motion. In v0.1 this is holding the
deadman key while a hand is tracked.
_Avoid_: clutch, activation, arming

**Deadman**:
An input that must be held continuously; releasing it stops motion. In v0.1 the deadman
is the spacebar read through `evdev`.
_Avoid_: enable switch, hold button

**Latch**:
To capture a reference pose at the rising edge of engagement. The supervisor latches the
hand pose; the bridge latches the tool pose.
_Avoid_: snapshot, anchor, zero

**Hand target**:
A scaled hand pose expressed in the operator frame, with a sequence number. Produced by
the hand mapper; still independent of any robot.
_Avoid_: retarget, mapped pose

**Teleop target**:
The robot-agnostic command contract: a pose delta relative to the latched tool pose,
plus engagement state and a sequence number. Produced by the supervisor and consumed by
bridges.
_Avoid_: command, setpoint, intent

**Hand mapper**:
The component that converts a hand pose from the device frame into a hand target in base
axes, applying rotation, gain, filtering and deadband.
_Avoid_: retargeter, mapper, transformer

**Teleop supervisor**:
The component that owns engagement, computes the teleop target, and decides when motion
is allowed. It holds no robot state.
_Avoid_: safety node, gate, controller

**Robot bridge**:
The only robot-specific component. It converts a teleop target into the control
interface of one robot.
_Avoid_: adapter, driver, backend

**Bridge dry run**:
A bridge mode that computes and logs commands without publishing them.
_Avoid_: simulation, mock

## States

**Idle**:
The supervisor state in which motion is not authorised and a new key press is required.
_Avoid_: disengaged, off

**Engaged**:
The supervisor state in which motion is authorised and deltas are produced.
_Avoid_: active, armed

**Hold**:
The supervisor state in which the key is held but tracking is unusable. No target deltas
are produced and the arm keeps its position; recovery re-latches the hand pose.
_Avoid_: paused, suspended

## Frames

**Device frame**:
The Leap camera frame: x right, y up, z toward the operator, origin at the device.
_Avoid_: camera frame, sensor frame

**Operator frame**:
The frame `leap_teleop/hand`, with the same axes as the base frame and an arbitrary
origin. Hand targets are expressed in it; only deltas leave it, so the origin never
matters.
_Avoid_: hand frame, world frame

**Base frame**:
The robot's fixed reference frame, following REP-103: x forward, y left, z up.
_Avoid_: world frame, robot frame

**Tool frame**:
The frame attached to the end effector whose origin is the rotation pivot. Its axis
convention is robot-specific and must not affect the mapping.
_Avoid_: end-effector frame, TCP (unless referring to the origin itself)

**Mount mode**:
How the device is mounted relative to the operator. `desktop` means lying flat in front
of the operator; `head_mounted` is reserved and not implemented in v0.1.
_Avoid_: camera mode, setup mode
