#pragma once

#include <algorithm>

namespace leap_teleop {

// Maps a gripper closure, 0 open to 1 closed, linearly onto a joint position. Closures outside
// 0 to 1 are clamped, so the result stays between open_position and closed_position.
inline double gripper_position(double closure, double open_position, double closed_position) {
  const double clamped = std::clamp(closure, 0.0, 1.0);
  return open_position + clamped * (closed_position - open_position);
}

// Moves current toward target by at most max_step. A max_step of 0 or less jumps to target.
inline double step_toward(double current, double target, double max_step) {
  if (max_step <= 0.0) {
    return target;
  }
  return current + std::clamp(target - current, -max_step, max_step);
}

}  // namespace leap_teleop
