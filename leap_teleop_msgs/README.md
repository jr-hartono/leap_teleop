# leap_teleop_msgs

## Purpose

Message definitions for the teleoperation pipeline. This package has no nodes.

## Messages

| Message | Role |
| --- | --- |
| `HandArray`, `Hand` | All tracked hands in one device frame; diagnostics and device state. |
| `HandTarget` | Scaled palm pose in the operator frame, with side and sequence. |
| `TeleopTarget` | The seam: base-frame pose delta, engagement state, sequence. |
| `TeleopState` | Supervisor diagnosis: state, reason, tracking and hand flags. |
| `EngageState` | Deadman key state, published from `evdev`. |

Inspect a definition with:

```sh
ros2 interface show leap_teleop_msgs/msg/TeleopTarget
```

## Details

Field definitions and topic contract: [ARCHITECTURE.md](../docs/ARCHITECTURE.md).
Build and test: [DEVELOPMENT.md](../docs/DEVELOPMENT.md).
Plan: [PLAN.md](../docs/PLAN.md). Vocabulary: [GLOSSARY.md](../GLOSSARY.md).
