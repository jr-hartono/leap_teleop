#include <gtest/gtest.h>

#include "leap_teleop_core/keyboard_deadman.hpp"

using leap_teleop::key_is_held;

TEST(KeyboardDeadman, PressAndRepeatCountAsHeld) {
  EXPECT_TRUE(key_is_held(1));
  EXPECT_TRUE(key_is_held(2));
}

TEST(KeyboardDeadman, ReleaseIsNotHeld) {
  EXPECT_FALSE(key_is_held(0));
}

TEST(KeyboardDeadman, UnexpectedValuesAreNotHeld) {
  EXPECT_FALSE(key_is_held(-1));
  EXPECT_FALSE(key_is_held(3));
}
