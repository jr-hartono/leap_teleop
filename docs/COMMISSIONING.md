# Real-arm commissioning

Manual checklist for bringing `leap_teleop` onto a physical arm. Complete the steps in order.
The pipeline contract is in [ARCHITECTURE.md](ARCHITECTURE.md); every parameter and range is
in [example_arm.yaml](../leap_teleop_bringup/config/example_arm.yaml).

## Prerequisites

- MoveIt Servo running and healthy, configured with conservative velocity and acceleration limits.
- URDF and TF published for `base_frame` (`base_link`), `tool_frame` (`tool0`) and
  `planning_frame` (`base_link`). The bridge resolves these frames with TF and publishes
  nothing when a frame is missing.
- Servo subscribed to `cartesian_command_topic` (`/servo_node/delta_twist_cmds`) with the
  same topic name.
- Ultraleap tracking service and device available; `evdev` read access to the keyboard.
- Deadman understood: hold the configured key (`space`) to authorise motion, release to stop.
  Hand presence is also required.

## Step 1 — dry run

Start with the `dry_run:=true` launch argument (the default) and `max_linear_speed: 0.05`.
The bridge computes and logs twists (`dry_run twist: linear=... angular=...`) without
publishing, so the arm does not move.

```sh
ros2 launch leap_teleop_bringup teleop.launch.py params_file:=/path/to/commissioning.yaml
```

Use the params file to lower `max_linear_speed`; `dry_run` is not a config parameter. Confirm
the bridge log reports `dry_run is enabled; no commands will be published`.

Servo 2.12.4 also needs the command type selected after each start. The YAML setting
`command_in_type: speed_units` is not enough:

```sh
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
```

Use `1` for `TWIST`, the type published by this bridge. Expect `success: true`.

The shipped UR Servo config publishes on `/forward_position_controller/commands`, so that
controller must be active. The UR driver starts `scaled_joint_trajectory_controller` by
default; switch it out before enabling output:

```sh
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_position_controller --strict
ros2 control list_controllers
```

Other robots may use a different controller and topic; check `command_out_topic` in that
robot's Servo config.

## Step 2 — topic names

Confirm these topics exist under the `leap_teleop` namespace (default `/leap_teleop/...`;
`namespace:=NAME` gives `/NAME/leap_teleop/...`):

```sh
ros2 topic list | grep leap_teleop
```

Expected: `/leap_teleop/hands` (`HandArray`), `/leap_teleop/hand_pose` (`HandPose`),
`/leap_teleop/deadman` (`DeadmanState`), `/leap_teleop/pose_delta` (`TeleopDelta`),
`/leap_teleop/state` (`TeleopState`).

## Step 3 — tracking and state transitions

Inspect `is_tracking` and the state fields with no hand in view, then with a hand present:

```sh
ros2 topic echo /leap_teleop/hands
ros2 topic echo /leap_teleop/state
```

With no hand but a live device, `is_tracking` remains true and `hands` is empty; the state
reports `no_hand`, `hand_present=false`, `engaged=false`. After device loss, `is_tracking`
becomes false at 10 Hz. With no device at all, the tracker logs a warning after 2 s and an
error after 10 s. With a hand tracked longer than `min_hand_age` (0.2 s),
`hand_present=true` and `tracking_ok=true`. Holding the key moves the state to `ENGAGED`;
releasing it returns to `IDLE`. With the key held and unusable tracking, the state is `HOLD`
(`engaged=false`). When tracking returns, the state stays `HOLD` with reason
`retrigger_required`; release and press the key again to re-latch the hand pose. If
`hold_timeout` expires first, the state becomes `IDLE`, and a held key still cannot resume
motion: a fresh release and press is required. `ros2 topic echo /leap_teleop/state` shows the
lowercase wire values, for example `state: hold`.

## Step 4 — deadman stop

In dry run, hold the key, move the hand, then release the key. Confirm from the bridge log
that twists stop being logged. Only after that check passes, relaunch with `dry_run:=false`
and repeat: releasing the key must stop publication, the bridge logs
`not publishing: not_engaged`, and Servo halts the arm. Nothing is commanded to zero.

## Step 5 — timeout relation

Verify `command_timeout <= Servo incoming_command_timeout` (Servo default 0.1 s). The
shipped example sets `command_timeout: 0.09`. If a robot stack overrides it, keep the value
at or below Servo's timeout: the bridge must not outlive Servo's patience.

## Step 6 — raise speed caps

Raise speed caps gradually from the commissioning value:

```sh
ros2 param set /leap_teleop/leap_moveit_servo_bridge max_linear_speed 0.1
```

Increase in small steps, checking after each step that motion stays inside the interaction
area and stops on deadman release. Raise `max_angular_speed` (default 0.5) similarly. Leave
`kp_linear` and `kp_angular` (both 4.0) unchanged until the caps are trusted; see
[TUNING.md](TUNING.md).

## Interaction area and operator positioning

Record the arm-specific values before the first powered run:

- Interaction area: _<fill in per robot>_.
- Operator position: _<fill in per installation>_.

The hand's usable range is roughly 60–70 cm above the sensor. There is no positional or
workspace clamp: displacement is bounded by the device's interaction zone, the speed caps,
Servo joint limits and collision checks, and the deadman. Keep the operator outside the
arm's swept volume and in view of the device. Keep the device flat and level; `yaw` must
match the orientation of `base_frame` (0 when robot and operator face the same direction,
180 when the arm faces the operator across the desk).
