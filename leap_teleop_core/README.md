# leap_teleop_core

## Purpose

Hand tracking, mapping, engagement and supervision, independent of any robot.

## Nodes and library

- `leap_tracker` — publishes `/leap_teleop/hands` (`HandArray`) and
  `/leap_teleop/hand_pose` (`HandPose`). Built only when LeapSDK is found.
- `leap_teleop_engage` — publishes `/leap_teleop/deadman` (`DeadmanState`) from the `evdev` key.
- `leap_teleop_supervisor` — publishes `/leap_teleop/pose_delta` (`TeleopDelta`) and
  `/leap_teleop/state` (`TeleopState`).
- `leap_hand_mapping` — library for frames, filters, deadband, delta composition and the
  supervisor FSM.

## Key parameters

`yaw`, `translation_gain`, `rotation_gain`, `filter_min_cutoff`, `filter_beta`,
`deadband_translation`, `deadband_rotation`, `deadband_hysteresis` (tracker); `key`,
`device`, `publish_rate` (engage); `base_frame`, `rotation_frame`, `tracking_timeout`,
`min_hand_age`, `hold_timeout`, `heartbeat_rate`, `engage_timeout` (supervisor). Values and
ranges: [example_arm.yaml](../leap_teleop_bringup/config/example_arm.yaml). Tuning:
[TUNING.md](../docs/TUNING.md).

## Details

[ARCHITECTURE.md](../docs/ARCHITECTURE.md) · [PLAN.md](../docs/PLAN.md) ·
[DEVELOPMENT.md](../docs/DEVELOPMENT.md) · [GLOSSARY.md](../GLOSSARY.md)
