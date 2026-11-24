#pragma once

#include <cstdint>
#include <string>

namespace leap_teleop {

enum class State : std::uint8_t { kIdle = 0, kEngaged = 1, kHold = 2 };

struct FsmInputs {
  bool key_held = false;
  bool hand_usable = false;
  double time_sec = 0.0;
  std::string unusable_reason = "no_hand";
};

struct FsmResult {
  State state = State::kIdle;
  bool latch = false;
  std::string reason;
};

class SupervisorFsm {
 public:
  explicit SupervisorFsm(double hold_timeout);

  FsmResult update(const FsmInputs& inputs);
  void reset();

  [[nodiscard]] State state() const;

 private:
  double hold_timeout_;
  State state_ = State::kIdle;
  bool hold_timer_active_ = false;
  double hold_start_ = 0.0;
  std::string reason_ = "key_up";
};

}  // namespace leap_teleop
