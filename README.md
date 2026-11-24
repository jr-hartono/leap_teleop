# leap_teleop

Teleoperation for robots with the original Leap Motion Controller (v1) on ROS 2.

> **Status: v0.1 implementation.** The pipeline builds and its pure logic is unit-tested; the hardware smoke test remains manual. The plan is in [docs/PLAN.md](docs/PLAN.md), the architecture and coordinate model in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md), and the vocabulary in [GLOSSARY.md](GLOSSARY.md).

## What this is

`leap_teleop` turns hand tracking from a Leap Motion Controller into a robot-agnostic
teleoperation stream, then adapts that stream to one specific robot. Adding a robot
means adding a bridge; it does not mean touching the tracking, mapping or safety code.

```
┌─────────────┐   ┌──────────────┐   ┌────────────────────┐   ┌──────────────────┐
│ Leap v1     │──▶│ leap_tracker │──▶│ leap_teleop_       │──▶│ leap_moveit_     │
│ (Gemini)    │   │ device +     │   │ supervisor         │   │ servo_bridge     │
│             │   │ hand mapping │   │ engage + delta     │   │ robot-specific   │
└─────────────┘   └──────────────┘   └────────────────────┘   └──────────────────┘
       raw hands        hand targets         teleop targets          robot commands
```

The seam is the **teleop target**: a pose delta relative to the tool pose latched at
engagement, plus engagement state and a sequence number. Every robot bridge consumes the
same type.

## Scope (v0.1)

- One Leap Motion Controller (v1), desktop mount, lying flat.
- One 6-DOF arm controlled through MoveIt Servo.
- Right hand by default, palm pose only. No finger or gripper control yet.
- Engage-relative delta control with fixed gains, filtering, deadband and speed caps.
- Full 3-DOF rotation, applied in the base frame about the tool origin.
- Keyboard deadman (`evdev`) plus hand presence.

Explicitly out of scope for v0.1: absolute/calibrated mapping, velocity retargeting,
palm-width normalisation, dexterous hand retargeting, head-mounted mode, camera image
streaming, and bridges other than MoveIt Servo.

## Packages

| Package | Contents |
|---|---|
| `leap_teleop_msgs` | Messages: hands, hand targets, teleop targets, state, engagement |
| `leap_teleop_core` | `leap_tracker`, `leap_teleop_supervisor`, `leap_teleop_engage`; `leap_hand_mapping` library |
| `leap_teleop_bridge` | `leap_moveit_servo_bridge` (further bridges later) |
| `leap_teleop_bringup` | Launch files and the example configuration |

## Requirements

- Ubuntu 24.04 and ROS 2 Jazzy. Humble and Lyrical are intended follow-ups; the code
  should avoid Jazzy-only APIs.
- Ultraleap Hand Tracking Service with Gemini. Verified here: service package
  `6.2.0.0`, `libLeapC` reporting Gemini `5.1.15`. Gemini is the last line that
  supports the original controller, so treat the service version as pinned.
- Leap Motion Controller (v1) on a USB 3 port, user in the `plugdev` group.
- `ros-jazzy-moveit-servo` for the first bridge.
- A C++17 compiler. No Python bindings are used.

## Build

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -y
colcon build --symlink-install
```

The Ultraleap service installs a CMake package. The tracker target is built only when it is
found, so the rest of the workspace builds without the SDK:

```cmake
find_package(LeapSDK QUIET)
if(LeapSDK_FOUND)
  target_link_libraries(leap_tracker LeapSDK::LeapC)
endif()
```

No vendored SDK, no `leapc_cffi` build, no Python version coupling.

Launch the stack with `ros2 launch leap_teleop_bringup teleop.launch.py`. The bridge starts in
`dry_run` mode; see [docs/COMMISSIONING.md](docs/COMMISSIONING.md) before enabling motion.

## Documentation

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — pipeline, frames, messages, safety, deployment
- [docs/PLAN.md](docs/PLAN.md) — phases, tickets, risks, settled decisions
- [leap_teleop_bringup/config/example_arm.yaml](leap_teleop_bringup/config/example_arm.yaml) — every parameter with units and ranges
- [docs/adr/](docs/adr/) — decisions that are hard to reverse
- [GLOSSARY.md](GLOSSARY.md) — the project's vocabulary

## License

Apache-2.0. See [LICENSE](LICENSE).
