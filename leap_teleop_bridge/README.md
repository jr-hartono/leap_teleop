# leap_teleop_bridge

## Purpose

Adapts the teleop delta to one robot. This is the only robot-specific package.

## Node

`leap_moveit_servo_bridge` subscribes to `pose_delta` and publishes `geometry_msgs/TwistStamped`
on `cartesian_command_topic` (default `/servo_node/delta_twist_cmds`) in `planning_frame`. It
latches the tool pose on engagement, applies `kp_linear`/`kp_angular`, clamps to the speed
caps, and stops publishing on stale target, disengagement or TF failure.

MoveIt Servo is a runtime peer reached over that topic. It is **not a build dependency**:
there is no `moveit_servo` link or package dependency. A running Servo node is needed only
at runtime.

## Key parameters

`tool_frame`, `planning_frame`, `cartesian_command_topic`, `kp_linear`, `kp_angular`,
`max_linear_speed`, `max_angular_speed`, `command_timeout`, `publish_period`, `dry_run`
(default `true`). Values and ranges:
[example_arm.yaml](../leap_teleop_bringup/config/example_arm.yaml).

## Details

[ARCHITECTURE.md](../docs/ARCHITECTURE.md) · [COMMISSIONING.md](../docs/COMMISSIONING.md) ·
[TUNING.md](../docs/TUNING.md) · [GLOSSARY.md](../GLOSSARY.md)
