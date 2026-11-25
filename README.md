# leap_teleop

Teleoperate a robot arm with your hand, using the original Leap Motion Controller (v1) and ROS 2.

Hold a key, move your hand, and the end effector follows. Release the key and the arm stops.

```
Leap v1 → leap_tracker → leap_teleop_supervisor → leap_moveit_servo_bridge → MoveIt Servo
          hand pose       deadman + pose delta      robot-specific twist
```

Only the bridge knows about the robot. It drives any arm that has a MoveIt Servo setup.

| Package | Contents |
|---|---|
| `leap_teleop_msgs` | Message definitions |
| `leap_teleop_core` | `leap_tracker`, `leap_teleop_deadman`, `leap_teleop_supervisor` nodes |
| `leap_teleop_bridge` | `leap_moveit_servo_bridge` node |
| `leap_teleop_bringup` | Launch files and example configuration |

| Topic | Type | From | Content |
|---|---|---|---|
| `/leap_teleop/hands` | `HandArray` | tracker | All tracked hands in the device frame, for diagnostics |
| `/leap_teleop/hand_pose` | `HandPose` | tracker | Filtered palm pose of the selected hand, in robot axes |
| `/leap_teleop/deadman` | `DeadmanState` | deadman | Whether the deadman key is held |
| `/leap_teleop/pose_delta` | `TeleopDelta` | supervisor | Pose change since engagement; the bridge's input |
| `/leap_teleop/state` | `TeleopState` | supervisor | `idle`, `engaged` or `hold`, with the reason |

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

Hold **space**, move your right hand above the controller, and the arm follows. Release space to
stop. To re-centre, release, move your hand back, and press again.

Servo logs `Waiting to receive robot state update.` for a few seconds at startup. That stops once
the arm has moved to its start pose.

If space does nothing, look for `using keyboard ...` in the log: the deadman listens to one
keyboard only. Copy [example_arm.yaml](leap_teleop_bringup/config/example_arm.yaml), set
`device` to your keyboard's `/dev/input/by-id/...-event-kbd` path, and add
`params_file:=/path/to/copy.yaml` to the launch command.

## Use it with your robot

1. Start your robot's MoveIt Servo.
2. Copy [example_arm.yaml](leap_teleop_bringup/config/example_arm.yaml) and set `base_frame`,
   `tool_frame`, `planning_frame`, `cartesian_command_topic` and `yaw` for your setup.
3. Launch in dry run, which logs commands without publishing them:

   ```bash
   ros2 launch leap_teleop_bringup teleop.launch.py params_file:=/path/to/my_arm.yaml
   ```

4. Follow [COMMISSIONING.md](docs/COMMISSIONING.md) before you add `dry_run:=false`.

Gains, filters and speed limits are explained in [TUNING.md](docs/TUNING.md). Run
`ros2 param describe <node> <param>` to see a parameter's range and whether it can be changed
while running.

## Develop

```bash
colcon build --symlink-install
colcon test && colcon test-result --verbose
```

The workspace builds without the Ultraleap service. Only `leap_tracker` is skipped.

## Documentation

- [ARCHITECTURE.md](docs/ARCHITECTURE.md): frames, messages, state machine and safety model
- [COMMISSIONING.md](docs/COMMISSIONING.md): bringing up a real arm
- [docs/adr/](docs/adr/): design decisions

## License

Apache-2.0
