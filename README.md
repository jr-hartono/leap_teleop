# leap_teleop

Teleoperate a robot arm with your hand, using the original Leap Motion Controller (v1) and ROS 2.

Hold a key, move your hand, and the end effector follows. Release the key and the arm stops.

```
Leap v1 → leap_driver → leap_teleop_mapper → leap_moveit_servo_bridge → MoveIt Servo
          raw hands       filtering + pose delta    robot-specific twist
```

Only the bridge knows about the robot. It drives any arm that has a MoveIt Servo setup.

| Package | Contents |
|---|---|
| `leap_teleop_msgs` | Message definitions |
| `leap_teleop_core` | `leap_driver`, `leap_teleop_deadman`, `leap_teleop_mapper` nodes |
| `leap_teleop_bridge` | `leap_moveit_servo_bridge` node |
| `leap_teleop_viz` | `leap_teleop_viz` node, RViz markers |
| `leap_teleop_bringup` | Launch files and example configuration |

| Topic | Type | From | Content |
|---|---|---|---|
| `/leap_teleop/hands` | `HandArray` | driver | All tracked hands as the tracking service reports them, in the device frame, with pinch and grab values; the mapper's input |
| `/leap_teleop/teleop_hand/<side>` | `TeleopHand` | mapper | Filtered palm pose in the operator frame and filtered closure, while the hand is visible |
| `/leap_teleop/deadman` | `DeadmanState` | deadman | Whether the deadman key is held |
| `/leap_teleop/pose_delta/<side>` | `TeleopDelta` | mapper | Pose change since engagement; the bridge's input |
| `/leap_teleop/gripper/<side>` | `TeleopGripper` | mapper | Gripper closure from 0 (open) to 1 (closed), from the pinch; published only while engaged |
| `/leap_teleop/state/<side>` | `TeleopState` | mapper | `idle`, `engaged` or `hold`, with the reason |
| `/leap_teleop/command/<side>` | `geometry_msgs/TwistStamped` | bridge | Twist the bridge computed, also in dry run |
| `/leap_teleop/markers` | `visualization_msgs/MarkerArray` | viz | RViz markers of the command and the hands |

The bridge publishes `geometry_msgs/TwistStamped` on `cartesian_command_topic`
(default `/servo_node/delta_twist_cmds`).

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy
- Leap Motion Controller (v1) on a USB 3 port
- Ultraleap Hand Tracking Service, tested with 6.2.0

## Install

1. Download the Ultraleap Hand Tracking Service for Linux from
   <https://www.ultraleap.com/downloads/leap-controller/> and install it:

   ```bash
   sudo apt install ./tracking-software-linux-x64-6.2.0.deb
   ```

   Optional: the Ultraleap Control Panel (`ultraleap-hand-tracking-control-panel_1125862.deb`)
   shows what the controller sees. It is not required for use with ROS.
   Ultraleap's apt repository, which hosted it, is no longer available, but a copy is available on the
   [Internet Archive](https://web.archive.org/web/20260124060130if_/https://repo.ultraleap.com/apt/pool/main/u/ultraleap-hand-tracking-control-panel/ultraleap-hand-tracking-control-panel_1125862.deb).

2. Allow your user to read the keyboard, which is used as the deadman key. Log out and back in
   afterwards.

   ```bash
   sudo usermod -aG input $USER
   ```

3. Clone into the `src` directory of a colcon workspace, then build from the workspace root:

   ```bash
   source /opt/ros/jazzy/setup.bash
   git clone https://github.com/jr-hartono/leap_teleop.git src/leap_teleop
   rosdep install --from-paths src --ignore-src -y
   colcon build
   source install/setup.bash
   ```

4. Check the setup from the workspace root. Each failed check prints its fix.

   ```bash
   src/leap_teleop/tools/check_setup.sh
   ```

## Try the demo

The demo drives a simulated UR5e in RViz, so no robot is needed. It needs three more packages:

```bash
sudo apt install ros-jazzy-moveit-servo ros-jazzy-ur-robot-driver ros-jazzy-ur-moveit-config
export ROS_DOMAIN_ID=42   # keep the demo away from any real robot on your network
ros2 launch leap_teleop_bringup demo_ur_mock.launch.py
```

Hold **left Alt**, move your right hand above the controller, and the arm follows. Release it to
stop. To re-centre, release, move your hand back, and press again.

Servo logs `Waiting to receive robot state update.` for a few seconds at startup. That stops once
the arm has moved to its start pose.

The deadman listens on every keyboard it can read; the log lists them as `listening on
keyboard ...`. To use only one, copy
[example_arm.yaml](leap_teleop_bringup/config/example_arm.yaml), set `device` to its
`/dev/input/by-id/...-event-kbd` path, and add `params_file:=/path/to/copy.yaml` to the launch
command.

## Use it with your robot

1. Start your robot's MoveIt Servo, with conservative velocity and acceleration limits.
2. Copy [example_arm.yaml](leap_teleop_bringup/config/example_arm.yaml) and set:
   - `base_frame`, `tool_frame` and `planning_frame`. `tool_frame` must be the real TCP, because
     rotations pivot about its origin.
   - `cartesian_command_topic`, matching Servo's `cartesian_command_in_topic`.
   - `yaw`: 0 when the robot and the operator face the same way, 180 when the arm faces the
     operator across the desk.
3. Launch. `dry_run` defaults to true, which logs commands instead of sending them to Servo:

   ```bash
   ros2 launch leap_teleop_bringup teleop.launch.py params_file:=/path/to/my_arm.yaml
   ```

   Two launch arguments set values that several nodes must share:
   - `side`: `right` (default) or `left`, the hand that teleoperates the arm.
   - `mount_mode`: `desktop` (default) with the cameras facing up and LED facing the operator, or
     `head_mounted` with cameras facing forward and the LED facing down.

4. Select Servo's twist input. Servo ignores `command_in_type` in YAML, so do this after every
   Servo start. Also make sure the controller behind Servo's `command_out_topic` is active.

   ```bash
   ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
   ```

5. Check the states with `ros2 topic echo /leap_teleop/state/right`:

   | Do | Expect |
   |---|---|
   | Hand in view, key up | `state: idle`, `hand_present: true` |
   | Hold the key | `state: engaged` |
   | Pull the hand away, key held | `state: hold` |
   | Return the hand, key held | still `hold`, `reason: retrigger_required` |
   | Release and press again | `state: engaged` |

6. Hold the key, move, and release. The bridge log stops printing `dry_run twist: ...`.
7. Only then relaunch with `dry_run:=false` and repeat step 6. Releasing the key must stop the
   arm. Keep the operator outside the arm's reach.
8. Raise the speed caps in small steps while running, checking each time that release stops the
   arm:

   ```bash
   ros2 param set /leap_teleop/leap_moveit_servo_bridge max_linear_velocity 0.1
   ```

## Visualization

Add `rviz:=true` to `teleop.launch.py` or the demo to open a second RViz window with the
`leap_teleop_viz` markers. The fixed frame is `base_link`; change it in RViz if your base frame
differs.

- Grey axes: the tool pose latched on the key press.
- Solid axes: the target the bridge drives to.
- Faded axes: the target without `translation_scale` and `rotation_scale`.
- Yellow and magenta arrows at the tool: the linear and angular twist.
- Hands, drawn at `leap_operator_frame_offset`: faded axes raw from the controller, labelled L or
  R, and solid axes after filtering, with a ball from white (open) to black (closed) showing the
  gripper closure.

## Tuning

Tune in dry run first and change one parameter at a time. All of these can be changed with
`ros2 param set` while running; `ros2 param describe <node> <param>` shows the range.

| Symptom | Parameter | Change |
|---|---|---|
| Jitter | `filter_min_cutoff` | Decrease |
| Lag on all motion | `filter_min_cutoff` | Increase |
| Lag on fast motion only | `filter_beta` | Increase |
| Drift while the hand is still | `deadband_translation`, `deadband_rotation` | Increase |
| Chatter at the edge of the deadband | `deadband_hysteresis` | Increase |
| Gripper does not fully close or open | `pinch_closed_mm`, `pinch_open_mm` | Adjust to your hand |
| Gripper target jitters or lags | `gripper_filter_min_cutoff`, `gripper_filter_beta`, `gripper_deadband` | Adjust |
| Arm travels too little or too much | `translation_scale`, `rotation_scale` | Adjust |
| Sluggish or twitchy response | `kp_linear`, `kp_angular` | Increase or decrease |
| Motion too fast | `max_linear_velocity`, `max_angular_velocity` | Decrease |

A wrong `yaw` gives wrong directions, which no gain fixes. Palm orientation is noisiest when the
palm is edge-on to the camera.

## How it works

The driver only relays what the tracking service reports, in the device frame; `mount_mode` there
only selects the service's tracking mode. The mapper takes the hand of its `side`, maps the palm
into an operator frame following [REP 103](https://www.ros.org/reps/rep-0103.html) (x forward,
y left, z up), which depends on `mount_mode`, and filters the pose and the pinch closure. It
publishes the result as `teleop_hand/<side>`, rotates the pose into the robot's base axes by
`yaw`, and gates the gripper closure with `gripper_deadband` while engaged. When the key is
pressed, it latches the hand pose and publishes the change since then, with the translation
multiplied by `translation_scale` and the rotation angle multiplied by `rotation_scale`. The
bridge latches the tool pose from TF on the same press, adds the translation, applies the
rotation about the tool origin, and drives Servo with a P controller clamped to the speed caps.

| State | When | Motion |
|---|---|---|
| `idle` | key up, or `hold_timeout` expired | no |
| `engaged` | key held, hand tracked and fresh | yes |
| `hold` | key held, tracking lost or just recovered | no, until release and press |

Releasing the key stops the deltas. The bridge then stops publishing and Servo's
`incoming_command_timeout` halts the arm. A new press re-latches both poses, so the arm never
jumps to where the hand went while it was untracked.

The bridge stops publishing 0.1 s after the last fresh delta, so Servo's
`incoming_command_timeout` must be at least 0.1 s, which is its default.

LeapC talks to the tracking service on the same host. If the robot stack runs in a container,
run the driver and deadman on the host and the rest in the container.

## Planned

- Gripper control with a pinch gesture, with a demo.
- Dual-arm control: the left and right hands each drive one arm.
- Demo with the arms of a Unitree G1 humanoid.

## Develop

```bash
colcon build --symlink-install
colcon test && colcon test-result --verbose
```

The workspace builds without the Ultraleap service. Only `leap_driver` is skipped.

Each node's parameters are defined in its package's `config/*_parameters.yaml`, which sets types,
defaults, ranges and whether a parameter can change at runtime. CI checks `example_arm.yaml`
against these definitions with `generate_parameter_library_validate --strict`; see
[the workflow](.github/workflows/build-and-test.yml) for the command.

C++ is formatted with the repo's [.clang-format](.clang-format), and Python with `ruff format`.

## License

Apache-2.0
