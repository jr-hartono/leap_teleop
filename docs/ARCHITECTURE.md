# Architecture

This document describes the shape of `leap_teleop` v0.1: the pipeline, the frames, the
contract between components, the parameter ownership, and the safety model.

## Scope

`leap_teleop` converts hand tracking into a robot-agnostic teleoperation stream and
adapts that stream to one robot at a time.

In scope for v0.1:

- Original Leap Motion Controller (v1), Gemini service, desktop mount, device lying flat.
- One 6-DOF arm controlled through MoveIt Servo.
- Right hand by default, configurable; palm pose only.
- Engage-relative delta control with fixed gains, filtering, deadband and speed caps.
- Full 3-DOF rotation, base-frame, about the tool origin.
- Keyboard deadman plus hand presence.

Out of scope for v0.1: absolute/calibrated mapping, velocity retargeting, palm-width
normalisation, dexterous hand retargeting, head-mounted mode, camera images, and bridges
other than MoveIt Servo.

## Pipeline

```
 Leap v1 ──▶ leap_tracker ──▶ /leap_teleop/leap/hands      HandArray   (diagnostics)
                │
                └──────────▶ /leap_teleop/hand_target      HandTarget  (scaled, operator frame)
                                        │
 /leap_teleop/engage ───────────────────┤
                                        ▼
                            leap_teleop_supervisor
                                        │
                                        ├──▶ /leap_teleop/target   TeleopTarget (delta, state)
                                        └──▶ /leap_teleop/state    TeleopState
                                                    │
                                                    ▼
                                        leap_moveit_servo_bridge
                                                    │
                                                    ▼
                                        TwistStamped → MoveIt Servo
```

| Role | Job | Knows about | Must not know about |
|---|---|---|---|
| Tracker | Talks to the camera; emits hands | LeapC, device state | robots, engagement |
| Hand mapper | Puts hand motion into base axes, scales and filters it | frames, gains, filters | robots, engagement |
| Supervisor | Decides whether motion is allowed; computes the delta | engagement, tracking health | robot kinematics |
| Robot bridge | Applies the delta to one specific arm | one robot's control interface | Leap, engagement policy |

The tracker and the hand mapper run in one node (`leap_tracker`). The mapper is a C++
library (`leap_hand_mapping`) so it remains independently testable.

## Packages and nodes

| Package | Nodes | Notes |
|---|---|---|
| `leap_teleop_msgs` | — | `ament_cmake`, `rosidl_default_generators` |
| `leap_teleop_core` | `leap_tracker`, `leap_teleop_engage`, `leap_teleop_supervisor` | C++17; mapper and FSM as libraries |
| `leap_teleop_bridge` | `leap_moveit_servo_bridge` | publishes a Servo topic; no build dependency on `moveit_servo`; further bridges later |
| `leap_teleop_bringup` | — | launch files and example config |

The supervisor is a separate process from the tracker so that a crash in device or
mapping code cannot take down the safety authority. The bridge is separate because it is
the only robot-specific code and may run in a different container.

## Topics and messages

All topics are relative to a `leap_teleop` namespace inside a launch namespace, so the
default is `/leap_teleop/...` and `namespace:=/ur5` gives `/ur5/leap_teleop/...`.

| Topic | Type | Producer | Consumer |
|---|---|---|---|
| `leap/hands` | `leap_teleop_msgs/HandArray` | `leap_tracker` | RViz, diagnostics |
| `hand_target` | `leap_teleop_msgs/HandTarget` | `leap_tracker` | supervisor |
| `engage` | `leap_teleop_msgs/EngageState` | `leap_teleop_engage` | supervisor |
| `target` | `leap_teleop_msgs/TeleopTarget` | supervisor | bridges |
| `state` | `leap_teleop_msgs/TeleopState` | supervisor | diagnostics |

Messages:

```text
# HandArray
std_msgs/Header header              # stamp = arrival time; frame_id = leap_device
uint64 tracking_frame_id
uint64 device_timestamp_us
float32 framerate                   # Hz; 0 when not yet estimated
bool is_tracking                    # false when the device or service is unavailable
Hand[] hands

# Hand
uint8 side                          # LEFT=1, RIGHT=2
uint64 visible_time_us
geometry_msgs/Pose palm             # device frame, metres

# HandTarget
std_msgs/Header header              # frame_id = leap_teleop/hand (constant)
uint8 side
geometry_msgs/Pose pose
uint32 sequence

# TeleopTarget
std_msgs/Header header              # frame_id = base_frame
uint8 side
uint8 state                         # IDLE=0, ENGAGED=1, HOLD=2
bool engaged
geometry_msgs/Transform delta       # pose change since engage; identity at engage
uint8 rotation_frame                # WORLD=0, TOOL=1; composition order for the delta
uint32 sequence

# TeleopState
std_msgs/Header header
uint8 state
string reason
bool tracking_ok
bool hand_present
bool engaged

# EngageState
builtin_interfaces/Time stamp
bool engaged
string source                       # evdev
```

`HandArray` keeps publishing at 10 Hz with `is_tracking: false` when the device or the
tracking service is gone, so the supervisor can distinguish "device died" from "hand left
the field of view" and from "tracker crashed", without a separate health topic.

A delta is a **displacement**, not a homogeneous transform. Position is added and
rotation is composed; see [Composition](#composition).

### QoS

| Topic | Reliability | Depth | Durability |
|---|---|---|---|
| `leap/hands` | best effort | 5 | volatile |
| `hand_target` | reliable | 1 | volatile |
| `engage` | reliable | 1 | volatile |
| `target` | reliable | 1 | volatile |
| `state` | reliable | 1 | volatile |

New commands are never replayed to a late subscriber: a latched command delivered on
connect is worse than no command.

## Coordinate frames

### Frames

| Frame | Source | Axes |
|---|---|---|
| Device `D` | Leap camera | x right, y up, z toward the operator |
| Palm `H` | `LEAP_PALM.orientation` | x = normal × direction, y = −normal, z = −direction |
| Operator `O` | `leap_teleop/hand` | same axes as the base frame; origin arbitrary |
| Base `B` | robot | REP-103: x forward, y left, z up |
| Tool `T` | robot TCP | robot-specific; only its origin matters |

`LEAP_PALM.position` is in millimetres. All published quantities are SI: metres and
radians.

The operator frame exists because the mapper produces coordinates in base *axes* whose
origin is the device, not the robot. Labelling that `base_link` would assert a hand
position that is not true. Since only deltas cross from the mapper to the supervisor, the
arbitrary origin never matters; the axes are what the supervisor needs.

For a flat right hand, palm down, fingers pointing away from the operator,
`normal = down`, `direction = forward`, so `H` coincides with `D`. The palm frame is
geometric, not anatomical: left and right hands produce the same frame geometry. Since
only orientation deltas are used, handedness never enters the maths.

### Device to base axes

```text
R_leap→ros = [ 0  0 -1 ]
             [-1  0  0 ]
             [ 0  1  0 ]

R_DB = Rz(yaw) · R_leap→ros        yaw in degrees at the config boundary, CCW about +z
p_B  = R_DB · p_D / 1000
```

With `yaw = 0`:

| Hand movement (device) | Robot movement (base) |
|---|---|
| away from operator (−z) | +x forward |
| up (+y) | +z up |
| right (+x) | −y right |

`yaw` describes how the robot faces relative to the operator. With `yaw = 0` the robot's
+x is treated as pointing away from the operator; when the arm faces the operator
across the desk, the value is 180. Yaw only; no mirror, no pitch or roll in v0.1.

`yaw` is a property of the desktop mount. `mount_mode: desktop` is the only supported
mode; `head_mounted` is reserved and rejected at startup, because a head-mounted device
needs a head pose to keep the mapping stable.

### Translation

```text
Δt_B = gain_t · R_DB · (p_h(t) − p_h(t₀)) / 1000
```

Translation is always expressed in base axes, so "hand down" means "end effector down"
regardless of how the hand is rotated. Expressing translation in the palm frame would make
"down" change as the wrist rotates.

### Rotation

```text
ΔR_B = R_DB · R_h(t) · R_h(t₀)ᵀ · R_DBᵀ
```

This is the world-frame rotation delta: the rotation that takes the hand's orientation at
engagement to its current orientation, expressed in base axes.

The alternative, tool-frame composition, is `ΔR_H = R_h(t₀)ᵀ · R_h(t)` applied as
`R_latched · ΔR_H`. It depends on the tool frame's axis convention, so the same hand
motion feels different on a robot whose gripper axes are defined differently. It remains
available as a parameter for A/B testing; world-frame is the default.

### Composition

```text
p_target = p_tool_latched + Δt_B
R_target = ΔR_B · R_tool_latched
```

Composing a full homogeneous transform is wrong. `T_delta · T_latched` rotates the
latched position about the base origin: an end effector at `(0.5, 0, 0.3)` with a 90° yaw
delta would swing to `(0, 0.5, 0.3)`. Adding translation and composing rotation about the
tool origin keeps pure rotations in place.

### Assumptions and caveats

- The device is flat and level. If the mount is tilted, "down" is off by that tilt;
  pitch and roll parameters are deliberately not part of v0.1.
- Palm orientation is least reliable when the palm is edge-on to the camera. Frame
  rejection and slerp filtering handle most of it; extreme angles remain the worst case.
- The tool frame's origin is the rotation pivot. If the URDF's end-effector link is
  offset from the real grasp point, rotations swing the grasp point. Configure
  `tool_frame` to the TCP.

## Engagement and safety

### Engagement

Engagement is a **deadman**: the operator holds a key, and motion is authorised only while
the key is held, the hand is present, and tracking is fresh.

- `leap_teleop_engage` reads the key through `evdev` and publishes `EngageState` at 20 Hz.
  The supervisor treats the last message as valid for `engage_timeout`.
- The keyboard is selected by `/dev/input/by-id/...` when configured, otherwise the first
  keyboard found is used and logged together with the other candidates.
- Hand presence is ANDed with the key. A held key with no hand produces no motion.
- Releasing the key stops new targets and the bridge stops publishing. The arm holds its
  position; nothing is commanded to zero.
- Re-engaging latches a new hand reference and a new tool reference, so the operator can
  re-centre by releasing and pressing again.

`evdev` is used because a terminal cannot report key release: holding a key produces one
character immediately and then repeats only after the keyboard's repeat delay, so a
terminal lease would keep the arm moving for hundreds of milliseconds after release.

### Supervisory state machine

| State | Condition | `engaged` | Exit |
|---|---|---|---|
| `IDLE` | key not held, or `hold_timeout` expired | false | key press with a usable hand → `ENGAGED` |
| `ENGAGED` | key held, tracking usable | true | key release → `IDLE`; unusable data → `HOLD` |
| `HOLD` | key held, tracking unusable | false | usable data → `ENGAGED`; key release or `hold_timeout` → `IDLE` |

`HOLD` is distinct from `IDLE` because the recovery rule differs: `HOLD` resumes
automatically when data returns, while `IDLE` requires a re-press. It covers a missing
hand, a stale frame, a hand that has not been tracked long enough, and rejected frames.

Recovery from `HOLD` **re-latches** the hand pose. Motion during the gap is discarded, so
the target cannot jump by however far the hand travelled while untracked. The bridge sees
`engaged` fall and rise and re-latches the tool pose for the same reason.

The `hold_timeout` timer starts when data became unusable and is only reset by a
transition into `ENGAGED`. A flapping tracker therefore escalates to `IDLE` on its own,
which requires an explicit re-press, without a separate fault state.

### Faults are not modelled

There is no `FAULT` state and no software e-stop topic. Every condition that would
populate them is either outside this component's control or fails safe without a state:

- A hardware e-stop removes power. Publishing into a dead controller changes nothing.
- A missing Servo node or unresolvable frame means there is no actuator to command. The
  bridge logs and stops publishing.
- Internal contract violations (NaN, sequence regression) are treated as `HOLD`: stop
  publishing and say why. If they persist, `hold_timeout` escalates to `IDLE`.

### Limits

- Speed caps are mandatory and live in the bridge: `max_linear_speed`, `max_angular_speed`
  clamp the twist produced by the P controller.
- The deadband and filter live in the mapper, before the delta.
- Servo's `incoming_command_timeout` (0.1 s by default) stops the arm when commands stop.
- There is no positional or workspace clamp. Hand displacement is already bounded by the
  device's interaction zone (roughly 60–70 cm above the sensor), a positional box would
  duplicate Servo's joint limits and collision checking, and the deadman allows an
  immediate stop. A per-robot box would also rot as an untested config file.

## Time and rates

| Stage | Rate | Timestamp |
|---|---|---|
| Tracker | device rate, ~90–120 Hz on v1 | `header.stamp` = arrival time on the ROS clock |
| Supervisor | per hand target, plus `heartbeat_rate` | propagates the hand target stamp |
| Bridge | 100 Hz timer (`publish_period` 0.01) | current ROS time |
| Engage | `publish_rate`, default 20 Hz | current ROS time |

The device timestamp counts microseconds on the tracking service's clock and cannot be
mapped to the ROS epoch, so it is published as `device_timestamp_us` for diagnostics and
never copied into `header.stamp`. Stamping at arrival keeps the stack correct under
`use_sim_time`; an offset estimate can be added later if latency must be measured.

## MoveIt Servo bridge

`leap_moveit_servo_bridge` is the only robot-specific component in v0.1.

- Subscribes to `target` and reads the frame from `header.frame_id`; no base-frame
  parameter exists, so the bridge cannot disagree with the supervisor about it.
- On the rising edge of `engaged`, latches the tool pose with TF from that frame to
  `tool_frame`.
- Each tick: `target = latched_pose + delta`, then `twist = Kp · pose_error`, clamped to
  the speed caps.
- Publishes `geometry_msgs/TwistStamped` on `cartesian_command_topic` (default
  `/servo_node/delta_twist_cmds`), transformed into `planning_frame`.
- Stops publishing when `engaged` is false, the target is older than `command_timeout`,
  or TF fails. Servo's `incoming_command_timeout` then halts the arm.
- `dry_run: true` computes and logs commands without publishing.

Servo 2.12.4 also accepts `geometry_msgs/PoseStamped` on `pose_command_in_topic`; the
twist path is the default because it is the more commonly exercised interface, and the P
controller keeps the reference positional either way.

The bridge needs a URDF/SRDF and a running Servo node. It does not use MoveIt planning,
only Servo's real-time IK and collision checks.

## Parameter ownership

Each knob has exactly one owner. Anything another component needs travels in a message
instead of being configured twice.

| Knob | Owner | Carried in the message |
|---|---|---|
| `side` (which hand) | tracker | `HandTarget.side`, copied to `TeleopTarget.side` |
| `mount_mode`, hand→robot yaw, gains, filters, deadband | tracker | — |
| `base_frame`, `rotation_frame` | supervisor | `TeleopTarget.header.frame_id` (base frame only) |
| Engage timeouts, `hold_timeout`, `heartbeat_rate` | supervisor | `TeleopState` |
| `tool_frame`, `planning_frame`, Servo topic, `kp_*`, speed caps, `command_timeout`, `dry_run` | bridge | — |
| Key, input device, publish rate | engage | `EngageState` |

The bridge has no `side` and no `base_frame`. A second arm is a second stack in its own
namespace, not a second filter on a shared stream.

Relations between nodes are documented, not enforced at runtime by a single node:

| Relation | Why |
|---|---|
| `supervisor.heartbeat_rate > 1 / bridge.command_timeout` | the stream must stay fresh between hand frames |
| `supervisor.engage_timeout >= 3 / engage.publish_rate` | jitter must not look like a key release |
| `bridge.command_timeout <= Servo's incoming_command_timeout` | the bridge must not outlive Servo's patience |
| `tracker.yaw` matches the orientation of `supervisor.base_frame` | the operator frame is defined relative to it |

## Deployment

The topic contract lets each stage run in a different container.

Recommended split when the robot stack is containerised:

- **Host**: `leapd`, `leap_tracker`, `leap_teleop_engage` (needs the USB device and the
  keyboard).
- **Robot container**: `leap_teleop_supervisor`, `leap_moveit_servo_bridge`, MoveIt Servo
  and the robot drivers.
- **DDS**: `--network host` on Linux, or a configured discovery server.

LeapC talks to `leapd` over a local IPC channel, not a documented network API. The
supported pattern is service and client on the same host. Running the tracker inside a
container therefore means installing the service in that container and passing the USB
device through; running the tracker on the host and publishing `leap/hands` over DDS is
the simpler deployment.

## Testing

- GTest for the mapping library: frame conversion, yaw, gain, One-Euro, deadband, delta
  composition, rotation composition.
- GTest for the supervisor: state transitions, latch and re-latch, timeouts, frame
  rejection.
- GTest for parameter validation: every declared range accepts its bounds and rejects
  values outside them.
- No synthetic hand source and no replay bags in v0.1. Correctness rests on pure-function
  tests plus a hardware smoke test.
- `dry_run` on the bridge allows exercising the pipeline with the device but no arm.
- CI builds and runs the pure tests on Jazzy. The tracker target is built only when
  `LeapSDK` is found (`find_package(LeapSDK QUIET)`), so the core libraries build on
  machines without the Ultraleap service.
