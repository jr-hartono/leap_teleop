#include <gtest/gtest.h>

#include "leap_teleop_core/mapper_fsm.hpp"

using leap_teleop::FsmInputs;
using leap_teleop::MapperFsm;
using leap_teleop::State;

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

TEST(MapperFsm, KeyAndHandEngageWithLatch) {
  MapperFsm fsm(2.0);
  const auto idle = fsm.update(inputs(false, true));
  EXPECT_EQ(idle.state, State::kIdle);
  EXPECT_FALSE(idle.latch);

  const auto engaged = fsm.update(inputs(true, true, 1.0));
  EXPECT_EQ(engaged.state, State::kEngaged);
  EXPECT_TRUE(engaged.latch);
  EXPECT_TRUE(engaged.reason.empty());
}

TEST(MapperFsm, KeyReleaseReturnsToIdle) {
  MapperFsm fsm(2.0);
  fsm.update(inputs(true, true, 0.0));
  const auto idle = fsm.update(inputs(false, true, 0.1));
  EXPECT_EQ(idle.state, State::kIdle);
  EXPECT_EQ(idle.reason, "key_up");
}

TEST(MapperFsm, UnusableDataEntersHoldAndRequiresRetrigger) {
  MapperFsm fsm(2.0);
  fsm.update(inputs(true, true, 0.0));
  const auto hold = fsm.update(inputs(true, false, 0.1));
  EXPECT_EQ(hold.state, State::kHold);
  EXPECT_EQ(hold.reason, "stale_tracking");
  EXPECT_FALSE(hold.latch);

  const auto still_held = fsm.update(inputs(true, true, 0.2));
  EXPECT_EQ(still_held.state, State::kHold);
  EXPECT_EQ(still_held.reason, "retrigger_required");
  EXPECT_FALSE(still_held.latch);

  const auto released = fsm.update(inputs(false, true, 0.3));
  EXPECT_EQ(released.state, State::kIdle);

  const auto retriggered = fsm.update(inputs(true, true, 0.4));
  EXPECT_EQ(retriggered.state, State::kEngaged);
  EXPECT_TRUE(retriggered.latch);
}

TEST(MapperFsm, HoldTimeoutEscalatesEvenWhenDataReturns) {
  MapperFsm fsm(1.0);
  fsm.update(inputs(true, true, 0.0));
  fsm.update(inputs(true, false, 0.2));
  const auto usable_again = fsm.update(inputs(true, true, 0.5));
  EXPECT_EQ(usable_again.state, State::kHold);

  const auto escalated = fsm.update(inputs(true, true, 1.3));
  EXPECT_EQ(escalated.state, State::kIdle);
  EXPECT_EQ(escalated.reason, "hold_timeout");
}

TEST(MapperFsm, HoldTimeoutEscalatesEvenWhenFramesKeepFailing) {
  MapperFsm fsm(1.0);
  fsm.update(inputs(true, true, 0.0));
  fsm.update(inputs(true, false, 0.5));
  fsm.update(inputs(true, false, 0.9));
  const auto escalated = fsm.update(inputs(true, false, 1.6));
  EXPECT_EQ(escalated.state, State::kIdle);
  EXPECT_EQ(escalated.reason, "hold_timeout");
}

TEST(MapperFsm, HoldTimeoutThenHeldKeyCannotReengage) {
  MapperFsm fsm(1.0);
  fsm.update(inputs(true, true, 0.0));
  fsm.update(inputs(true, false, 0.2));
  const auto idle = fsm.update(inputs(true, false, 1.3));
  EXPECT_EQ(idle.state, State::kIdle);

  const auto usable_again = fsm.update(inputs(true, true, 1.4));
  EXPECT_EQ(usable_again.state, State::kIdle);
  EXPECT_EQ(usable_again.reason, "retrigger_required");
  EXPECT_FALSE(usable_again.latch);

  const auto released = fsm.update(inputs(false, true, 1.5));
  EXPECT_EQ(released.state, State::kIdle);

  const auto pressed = fsm.update(inputs(true, true, 1.6));
  EXPECT_EQ(pressed.state, State::kEngaged);
  EXPECT_TRUE(pressed.latch);
}

TEST(MapperFsm, ZeroHoldTimeoutKeepsHoldIndefinitely) {
  MapperFsm fsm(0.0);
  fsm.update(inputs(true, true, 0.0));
  fsm.update(inputs(true, false, 1.0));
  const auto still_holding = fsm.update(inputs(true, false, 100.0));
  EXPECT_EQ(still_holding.state, State::kHold);
}

TEST(MapperFsm, IdleRequiresANewKeyPress) {
  MapperFsm fsm(1.0);
  fsm.update(inputs(true, true, 0.0));
  fsm.update(inputs(false, true, 0.1));
  const auto idle = fsm.update(inputs(false, true, 0.2));
  EXPECT_EQ(idle.state, State::kIdle);
  const auto engaged = fsm.update(inputs(true, true, 0.3));
  EXPECT_EQ(engaged.state, State::kEngaged);
  EXPECT_TRUE(engaged.latch);
}
