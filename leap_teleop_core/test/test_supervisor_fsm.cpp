#include <gtest/gtest.h>

#include "leap_teleop_core/supervisor_fsm.hpp"

using leap_teleop::FsmInputs;
using leap_teleop::State;
using leap_teleop::SupervisorFsm;

namespace {

FsmInputs inputs(bool key_held, bool hand_usable, double time_sec = 0.0) {
  FsmInputs result;
  result.key_held = key_held;
  result.hand_usable = hand_usable;
  result.time_sec = time_sec;
  result.unusable_reason = "stale_tracking";
  return result;
}

}  // namespace

TEST(SupervisorFsm, KeyAndHandEngageWithLatch) {
  SupervisorFsm fsm(2.0);
  const auto idle = fsm.update(inputs(false, true));
  EXPECT_EQ(idle.state, State::kIdle);
  EXPECT_FALSE(idle.latch);

  const auto engaged = fsm.update(inputs(true, true, 1.0));
  EXPECT_EQ(engaged.state, State::kEngaged);
  EXPECT_TRUE(engaged.latch);
  EXPECT_TRUE(engaged.reason.empty());
}

TEST(SupervisorFsm, KeyReleaseReturnsToIdle) {
  SupervisorFsm fsm(2.0);
  fsm.update(inputs(true, true, 0.0));
  const auto idle = fsm.update(inputs(false, true, 0.1));
  EXPECT_EQ(idle.state, State::kIdle);
  EXPECT_EQ(idle.reason, "key_up");
}

TEST(SupervisorFsm, UnusableDataEntersHoldAndRecoveryRelatches) {
  SupervisorFsm fsm(2.0);
  fsm.update(inputs(true, true, 0.0));
  const auto hold = fsm.update(inputs(true, false, 0.1));
  EXPECT_EQ(hold.state, State::kHold);
  EXPECT_EQ(hold.reason, "stale_tracking");
  EXPECT_FALSE(hold.latch);

  const auto recovered = fsm.update(inputs(true, true, 0.2));
  EXPECT_EQ(recovered.state, State::kEngaged);
  EXPECT_TRUE(recovered.latch);
}

TEST(SupervisorFsm, HoldTimeoutEscalatesEvenWhenFramesKeepFailing) {
  SupervisorFsm fsm(1.0);
  fsm.update(inputs(true, true, 0.0));
  fsm.update(inputs(true, false, 0.5));
  fsm.update(inputs(true, false, 0.9));
  const auto escalated = fsm.update(inputs(true, false, 1.6));
  EXPECT_EQ(escalated.state, State::kIdle);
  EXPECT_EQ(escalated.reason, "hold_timeout");
}

TEST(SupervisorFsm, ZeroHoldTimeoutKeepsHoldIndefinitely) {
  SupervisorFsm fsm(0.0);
  fsm.update(inputs(true, true, 0.0));
  fsm.update(inputs(true, false, 1.0));
  const auto still_holding = fsm.update(inputs(true, false, 100.0));
  EXPECT_EQ(still_holding.state, State::kHold);
}

TEST(SupervisorFsm, IdleRequiresANewKeyPress) {
  SupervisorFsm fsm(1.0);
  fsm.update(inputs(true, true, 0.0));
  fsm.update(inputs(false, true, 0.1));
  const auto idle = fsm.update(inputs(false, true, 0.2));
  EXPECT_EQ(idle.state, State::kIdle);
  const auto engaged = fsm.update(inputs(true, true, 0.3));
  EXPECT_EQ(engaged.state, State::kEngaged);
  EXPECT_TRUE(engaged.latch);
}
