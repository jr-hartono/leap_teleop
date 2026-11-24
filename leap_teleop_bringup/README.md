# leap_teleop_bringup

## Purpose

Launch file and example configuration for the whole pipeline. This package has no nodes.

## Launch

```sh
source /opt/ros/jazzy/setup.bash
ros2 launch leap_teleop_bringup teleop.launch.py
```

Arguments: `namespace` (default `/`; the stack adds `/leap_teleop`), `params_file` (default
`config/example_arm.yaml`), `use_sim_time` (default `false`). To use a robot-specific file:

```sh
ros2 launch leap_teleop_bringup teleop.launch.py params_file:=/path/to/robot.yaml
```

The launch starts `leap_tracker`, `leap_teleop_engage`, `leap_teleop_supervisor` and
`leap_moveit_servo_bridge`. `dry_run` defaults to `true`.

## Details

Configuration: [example_arm.yaml](config/example_arm.yaml) ·
[COMMISSIONING.md](../docs/COMMISSIONING.md) · [TUNING.md](../docs/TUNING.md) ·
[ARCHITECTURE.md](../docs/ARCHITECTURE.md) · [GLOSSARY.md](../GLOSSARY.md)
