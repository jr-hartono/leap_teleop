# Plan

How `leap_teleop` gets built, in what order, and what "done" means at each step.
Architecture and frames live in [ARCHITECTURE.md](ARCHITECTURE.md); vocabulary in
[GLOSSARY.md](../GLOSSARY.md).

## Definition of done for v0.1

An operator, sitting at a desk with a Leap v1 in front of them, holds the spacebar, moves
a hand, and the arm's end effector follows that motion in the base frame: down means down,
forward means forward, turning the hand turns the tool. Releasing the spacebar stops the
arm within one Servo timeout without a jump. The arm never moves when the key is up, the
hand is absent, tracking is stale, or a speed cap would be exceeded.

Nothing else is required for v0.1. Gripper, second hand, head-mounted mode, camera
images, other robots and other bridges are deliberately absent.

## Guiding constraints

1. **The core builds without the robot.** `leap_teleop_msgs` and `leap_teleop_core` must
   compile and test on a machine with no MoveIt, no MoveIt Servo and no LeapSDK.
2. **The safety gate is a separate process.** A crash in device or mapping code must not
   remove the component that decides whether motion is allowed.
3. **Each knob has one owner.** Facts that several components need travel in messages,
   never in duplicated parameters.
4. **Every stage is observable.** Each arrow in the pipeline is a topic, so `ros2 topic
   echo` explains any unexpected motion without a debugger.
5. **Hardware is not a build dependency.** The tracker target is compiled only when
   `LeapSDK` is found; the rest of the workspace is unaffected.
6. **Tests are pure functions and state machines.** No simulation, no synthetic hands.
   The hardware smoke test is a documented manual step.

## Phases

Each phase ends in something runnable or testable. Tickets inside a phase are ordered but
mostly independent.

### Phase 0 — Scaffold

Goal: an empty but real workspace that builds and tests in CI on a machine with no
hardware.

| ID | Ticket | Acceptance |
|---|---|---|
| T01 | Create the four packages: `leap_teleop_msgs`, `leap_teleop_core`, `leap_teleop_bridge`, `leap_teleop_bringup`. `ament_cmake_auto`, C++17, `package.xml` with explicit deps and Apache-2.0 license. | `colcon build` succeeds on a bare Jazzy install. |
| T02 | Repo hygiene: `.gitignore` (`build/ install/ log/`), `.clang-format`, `.editorconfig`, a short `docs/DEVELOPMENT.md` with build and test commands. | Formatting is one command; docs match a fresh clone. |
| T03 | CI: one GitHub Actions workflow, `ros:jazzy` container, `colcon build` + `colcon test`, no hardware deps. | Green on a PR that only touches docs. |
| T04 | Message definitions: `HandArray`, `Hand`, `HandPose`, `TeleopDelta`, `TeleopState`, `DeadmanState`, with explicit enum values. | `ros2 interface show` works for all; fields match ARCHITECTURE. |

Phase 0 ends with a workspace that a stranger can clone, build and test.

### Phase 1 — Hand mapping library

Goal: the maths, fully tested, with no ROS node and no device.

| ID | Ticket | Acceptance |
|---|---|---|
| T05 | `frames` module: Leap→ROS rotation, yaw, mm→m, quaternion helpers (normalise, multiply, delta, slerp). | Unit tests assert `det(R)=1`, orthonormality, and the three axis mappings in the doc table. |
| T06 | Filters: One-Euro on position, slerp low-pass on orientation, deadband with hysteresis, time-based so behaviour does not change with frame rate. | Step-response and stationary-noise tests; no NaN on zero dt; deadband does not chatter. |
| T07 | `HandMapper`: hand pose (device frame) → `HandPose` (operator frame), fixed gain, yaw, side selection. | Golden-vector tests for yaw 0 and 180; hand down → −z; hand away → +x. |
| T08 | `DeltaBuilder`: latched pose + current pose → `Transform` delta; base-axis translation, world-frame rotation. | Pure rotation leaves translation unchanged (the bug called out in ARCHITECTURE); 90° yaw reported correctly; identity at engagement. |

Phase 1 ends with `colcon test` covering the whole mapping contract.

### Phase 2 — Tracker

Goal: hands on a topic, at device rate, with honest timestamps.

| ID | Ticket | Acceptance |
|---|---|---|
| T09 | `LeapSDK` discovery: `find_package(LeapSDK QUIET)`, optional target, compile-time guard so the package builds without it. | Build with and without the SDK; no `find_package` error either way. |
| T10 | Connection lifecycle: open, wait for device, handle disconnect and reconnect, set desktop tracking mode, clean shutdown. | Unplugging and replugging recovers without restarting the node. |
| T11 | Publish `HandArray` on `hands`: device frame, arrival timestamp, `tracking_frame_id`, `device_timestamp_us`, `framerate`, both hands, `visible_time`. Keep publishing at 10 Hz with `is_tracking: false` after device loss. | `ros2 topic hz` shows the device rate; timestamps are monotonic; unplugging the device produces `is_tracking: false`, not silence. |
| T12 | Wire `HandMapper` into the tracker; publish `hand_pose` for the configured hand in `leap_teleop_frame`. | Moving the tracked hand produces the expected operator-frame pose. |

Phase 2 ends with `hands` and `hand_pose` alive on real hardware. This is the first
phase that needs the device plugged in.

### Phase 3 — Engagement and supervisor

Goal: the gate. Nothing moves without it.

| ID | Ticket | Acceptance |
|---|---|---|
| T13 | `leap_teleop_deadman`: `evdev` reader, device selected by `/dev/input/by-id` or by scanning keyboards, keydown and keyup, `publish_rate` output, clean handling of device disappearance. | Key release is detected within one publish period; unplugging the keyboard publishes `engaged: false`, not silence; the chosen device and the rejected candidates are logged. |
| T14 | Supervisor FSM as a library: `IDLE`, `ENGAGED`, `HOLD`, with hand presence, hand age, freshness and reject inputs. | Transition table tests, including required key retrigger after `HOLD` and escalation to `IDLE` when `hold_timeout` expires. |
| T15 | Supervisor node: latch hand pose on rising edge, build delta, stamp `base_frame`, copy `side` from the hand pose, publish `pose_delta` and `state`. | Key up → `IDLE`, no deltas; stale tracking → `HOLD`, `engaged: false`; returning data stays `HOLD`; a new press re-latches with no jump. |
| T16 | Parameter declaration and validation for every node: declare ranges, reject out-of-range values, refuse to start with a message naming the parameter and its range. | Boundary values accepted; out-of-range rejected; one GTest per parameter group. |

Phase 3 ends with a target stream that can be inspected and trusted, with the arm absent.

### Phase 4 — MoveIt Servo bridge

Goal: motion on the real arm, bridge dry-run first.

| ID | Ticket | Acceptance |
|---|---|---|
| T17 | `leap_moveit_servo_bridge` skeleton: subscribe to `pose_delta`, read the frame from the message, latch the tool pose on the rising edge of `engaged`, `target = latched + delta`, `twist = Kp · error`, clamp to speed caps, publish `TwistStamped` in `planning_frame`. `dry_run` default true. | In dry run, commands are logged and nothing is published; the logged twist matches the delta for a known target. |
| T18 | Failure handling: stale target, `engaged: false`, TF failure, missing Servo node → stop publishing, log a reason once, never publish a stale command. | Killing the supervisor stops bridge output; a missing frame at startup produces a clear error and no publishing. |
| T19 | `leap_teleop_bringup`: launch file composing tracker, deadman, supervisor and bridge; `namespace`, `params_file` and `dry_run` arguments; the example config as default. | One launch command with a robot argument starts the pipeline; `dry_run` defaults to true; an external params file overrides single values. |
| T20 | Real-arm commissioning: Servo configured with conservative velocity and acceleration limits, the arm's working area documented, operator briefed on the deadman. | Documented checklist in `docs/COMMISSIONING.md`, signed off by the operator. |

Phase 4 ends with the definition of done above.

### Phase 5 — Hardening

| ID | Ticket | Acceptance |
|---|---|---|
| T21 | Tuning guide: which gain does what, symptom → parameter table. | A new operator can tune in under 30 minutes without reading code. |
| T22 | `ros2 bag` recipe for recording target and state streams during a session. | One command, documented; bags are for debugging, not CI. |
| T23 | Package READMEs and a five-line quick start. | A reader who has never seen the repo can build and dry-run it. |

Phase 5 is optional for the first demo and should not block it.

## Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Palm orientation is noisy when edge-on to the camera | Rotation jitter, worst case at extreme wrist angles | Frame rejection, slerp filtering, deadband; document the bad zone |
| `evdev` needs read access to keyboard event devices | Engage node cannot start for other users | Document the group requirement; log every candidate device at startup |
| LeapSDK packaging is Ubuntu-specific and Gemini is EOL-ish | Hard to build elsewhere; version drift breaks the tracker | Optional build target, version check at runtime, service version pinned in the README |
| Servo planning frame ≠ base frame | Silently wrong motion direction | Explicit TF transform in the bridge, asserted at startup |
| Latency stack: device → filters → supervisor → Servo → controller | Sluggish or laggy feel | Measure end-to-end in Phase 2 and Phase 4; keep filters minimal, prefer One-Euro over long averages |
| Supervisor crash while the arm moves | Arm keeps the last command until Servo timeout | Bridge stops on stale target; the operator releases the deadman |
| No positional clamp, so a hand glitch can ask for a long travel | Rapid motion towards a distant goal | Speed caps bound the violence; the interaction zone bounds the displacement; the deadman stops it; Servo checks collisions |
| Two latches (hand, tool) on the same edge | One-frame disagreement | Delta is zero at engagement, so disagreement cannot produce a jump; verified by test |

## Settled decisions

Recorded in [docs/adr/](adr/):

- Engage-relative delta pose only; no absolute mode, no velocity mode.
- The seam is a pose delta; the bridge latches the robot pose.
- C++ driver linking the `LeapSDK::LeapC` CMake target.
- One aggregate teleop topic with `side` in the message.
- Parameters have one owner; frames travel in messages.

Not ADR-worthy but fixed for v0.1: four packages, three runtime nodes plus the engage
node, `evdev` only, three states (`IDLE`, `ENGAGED`, `HOLD`), world-frame rotation,
`desktop` mount only, speed caps only.
