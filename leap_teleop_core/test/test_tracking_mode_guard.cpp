#include <gtest/gtest.h>

#include "leap_teleop_core/tracking_mode_guard.hpp"

using leap_teleop::TrackingModeGuard;
using Action = TrackingModeGuard::Action;

TEST(TrackingModeGuard, RequestsOnRestartAndStopsOnceConfirmed) {
  TrackingModeGuard guard(5, 1.0);
  EXPECT_EQ(guard.restart(0.0), Action::kRequest);
  EXPECT_EQ(guard.on_mode(true, 0.3), Action::kNone);
  EXPECT_TRUE(guard.confirmed());
  EXPECT_EQ(guard.on_tick(5.0), Action::kNone);
}

TEST(TrackingModeGuard, RetriesALostRequest) {
  TrackingModeGuard guard(5, 1.0);
  guard.restart(0.0);
  EXPECT_EQ(guard.on_tick(0.5), Action::kNone);
  EXPECT_EQ(guard.on_tick(1.0), Action::kRequest);
  EXPECT_EQ(guard.on_tick(1.5), Action::kNone);
  EXPECT_EQ(guard.on_tick(2.0), Action::kRequest);
}

TEST(TrackingModeGuard, RequestsAgainWhenTheModeIsChanged) {
  TrackingModeGuard guard(5, 1.0);
  guard.restart(0.0);
  guard.on_mode(true, 0.3);
  EXPECT_EQ(guard.on_mode(false, 10.0), Action::kRequest);
  EXPECT_FALSE(guard.confirmed());
}

TEST(TrackingModeGuard, GivesUpOnceWhenAnotherClientKeepsChangingTheMode) {
  TrackingModeGuard guard(3, 1.0);
  EXPECT_EQ(guard.restart(0.0), Action::kRequest);
  guard.on_mode(true, 0.1);
  EXPECT_EQ(guard.on_mode(false, 0.2), Action::kRequest);
  guard.on_mode(true, 0.3);
  EXPECT_EQ(guard.on_mode(false, 0.4), Action::kRequest);
  guard.on_mode(true, 0.5);
  EXPECT_EQ(guard.on_mode(false, 0.6), Action::kGiveUp);
  EXPECT_EQ(guard.on_mode(false, 0.7), Action::kNone);
  EXPECT_EQ(guard.on_tick(10.0), Action::kNone);
}

TEST(TrackingModeGuard, GivesUpWhenNeverConfirmed) {
  TrackingModeGuard guard(2, 1.0);
  guard.restart(0.0);
  EXPECT_EQ(guard.on_tick(1.0), Action::kRequest);
  EXPECT_EQ(guard.on_tick(2.0), Action::kGiveUp);
  EXPECT_EQ(guard.on_tick(3.0), Action::kNone);
}

TEST(TrackingModeGuard, RestartAllowsNewAttempts) {
  TrackingModeGuard guard(1, 1.0);
  guard.restart(0.0);
  EXPECT_EQ(guard.on_tick(1.0), Action::kGiveUp);
  EXPECT_EQ(guard.restart(5.0), Action::kRequest);
}
