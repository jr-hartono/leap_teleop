#pragma once

#include <algorithm>

namespace leap_teleop {

// Maps a gripper closure, 0 open to 1 closed, linearly onto a joint position. Closures outside
// 0 to 1 are clamped, so the result stays between open_position and closed_position.
inline double gripper_position(double closure, double open_position, double closed_position) {
  const double clamped = std::clamp(closure, 0.0, 1.0);
  return open_position + clamped * (closed_position - open_position);
}

}  // namespace leap_teleop
