#include "leap_teleop_core/supervisor_fsm.hpp"

namespace leap_teleop {

SupervisorFsm::SupervisorFsm(double hold_timeout) : hold_timeout_(hold_timeout) {}

FsmResult SupervisorFsm::update(const FsmInputs& inputs) {
  bool latch = false;

  switch (state_) {
    case State::kIdle:
      if (inputs.key_held && inputs.hand_usable) {
        state_ = State::kEngaged;
        reason_.clear();
        latch = true;
      } else {
        reason_ = inputs.key_held ? inputs.unusable_reason : "key_up";
      }
      break;

    case State::kEngaged:
      if (!inputs.key_held) {
        state_ = State::kIdle;
        reason_ = "key_up";
      } else if (!inputs.hand_usable) {
        state_ = State::kHold;
        reason_ = inputs.unusable_reason;
        hold_timer_active_ = true;
        hold_start_ = inputs.time_sec;
      }
      break;

    case State::kHold:
      if (!inputs.key_held) {
        state_ = State::kIdle;
        reason_ = "key_up";
        hold_timer_active_ = false;
      } else if (inputs.hand_usable) {
        state_ = State::kEngaged;
        reason_.clear();
        latch = true;
        hold_timer_active_ = false;
      } else if (hold_timeout_ > 0.0 && hold_timer_active_ &&
                 inputs.time_sec - hold_start_ >= hold_timeout_) {
        state_ = State::kIdle;
        reason_ = "hold_timeout";
        hold_timer_active_ = false;
      } else {
        reason_ = inputs.unusable_reason;
      }
      break;
  }

  return FsmResult{state_, latch, reason_};
}

void SupervisorFsm::reset() {
  state_ = State::kIdle;
  hold_timer_active_ = false;
  hold_start_ = 0.0;
  reason_ = "key_up";
}

State SupervisorFsm::state() const { return state_; }

}  // namespace leap_teleop
