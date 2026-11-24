# Development

Build and test guide for `leap_teleop`. Package roles and topic contracts live in
[ARCHITECTURE.md](ARCHITECTURE.md); build order and ticket status live in
[PLAN.md](PLAN.md).

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- `colcon` (`python3-colcon-common-extensions`)
- A C++17 compiler
- Optional: the Ultraleap tracking service, needed only by the tracker node
- `evdev` read access to the keyboard event device, needed only by the engage node. Add the
  operator to the `input` group, or configure a device under `/dev/input/by-id/`.

## Build

```sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Test

```sh
colcon test
colcon test-result --verbose
```

## Format

Apply clang-format to package sources and headers:

```sh
find . -path './.git' -prune -o \( -path '*/src/*' -o -path '*/include/*' \) \
  \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) -print0 \
  | xargs -0 -r clang-format -i
```

The command is restricted to files under package `src` and `include` directories. The style is defined
in [`.clang-format`](../.clang-format): Google-based, 100 columns, two-space indent.

## Build without hardware

The core packages build and test without LeapSDK and without MoveIt Servo. The tracker
target is conditional on `LeapSDK` discovery, so its absence does not affect the build.
The bridge only publishes a topic and needs no robot hardware to compile; a running Servo
node is required only at runtime.

## Hardware smoke test

The hardware smoke test is manual. Plug in the device and inspect the pipeline with
`ros2 topic` tools. No synthetic hand source or replay bag exists, so correctness of the
mapping and state machine rests on the unit tests plus this manual check.
