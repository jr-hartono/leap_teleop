#!/usr/bin/env bash
# Check that this machine can run leap_teleop. Run after sourcing your workspace.
# Usage: tools/check_setup.sh [--demo]

failures=0
pass() { printf '  \e[32mok\e[0m    %s\n' "$1"; }
fail() { printf '  \e[31mFAIL\e[0m  %s\n        fix: %s\n' "$1" "$2"; failures=$((failures + 1)); }
check() { local label=$1 fix=$2; shift 2; if "$@" >/dev/null 2>&1; then pass "$label"; else fail "$label" "$fix"; fi; }

has_package() { ros2 pkg prefix "$1"; }
service_version() { dpkg-query -W -f='${Version}' ultraleap-hand-tracking-service 2>/dev/null; }
leap_port_open() { ss -ltn | grep -q '127.0.0.1:12345 '; }
leap_on_usb() { lsusb -d f182: | grep -q .; }
tracker_built() { ros2 pkg executables leap_teleop_core | grep -q leap_tracker; }
keyboard_readable() {
  local dev
  for dev in /dev/input/by-id/*-event-kbd; do [ -r "$dev" ] && return 0; done
  return 1
}

echo "ROS"
check "ROS 2 Jazzy sourced" "source /opt/ros/jazzy/setup.bash" test "$ROS_DISTRO" = jazzy
check "leap_teleop workspace sourced" "source ./install/setup.bash" has_package leap_teleop_core

echo "Ultraleap"
version=$(service_version)
if [ -n "$version" ]; then
  pass "Hand Tracking Service installed (version: $version)"
else
  fail "Hand Tracking Service installed" \
    "install the .deb from https://www.ultraleap.com/downloads/leap-controller/"
fi
check "service running" "sudo systemctl restart ultraleap-hand-tracking-service" \
  systemctl is-active --quiet ultraleap-hand-tracking-service
check "service listening on 127.0.0.1:12345" \
  "sudo systemctl restart ultraleap-hand-tracking-service" leap_port_open
check "Leap Motion Controller on USB" "plug it into a USB 3 port" leap_on_usb
if has_package leap_teleop_core >/dev/null 2>&1; then
  check "leap_tracker built" \
    "rebuild after installing the service: colcon build --cmake-clean-cache" tracker_built
else
  echo "  skip  leap_tracker built (source the workspace first)"
fi

echo "Deadman key"
check "a keyboard under /dev/input is readable" \
  "sudo usermod -aG input \$USER, then log out and back in" keyboard_readable

if [ "$1" = "--demo" ]; then
  echo "Demo"
  for package in moveit_servo ur_robot_driver ur_moveit_config; do
    check "$package installed" "sudo apt install ros-jazzy-${package//_/-}" has_package "$package"
  done
fi

echo
if [ "$failures" -eq 0 ]; then echo "All checks passed."; else echo "$failures check(s) failed."; fi
exit $((failures > 0))
