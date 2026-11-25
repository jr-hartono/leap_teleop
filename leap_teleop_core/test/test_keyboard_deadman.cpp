#include <gtest/gtest.h>

#include <cstddef>
#include <stdexcept>
#include <string>

#include "leap_teleop_core/keyboard_deadman.hpp"

using leap_teleop::key_is_held;

TEST(KeyboardDeadman, PressAndRepeatCountAsHeld) {
  EXPECT_TRUE(key_is_held(1));
  EXPECT_TRUE(key_is_held(2));
}

TEST(KeyboardDeadman, ReleaseIsNotHeld) { EXPECT_FALSE(key_is_held(0)); }

TEST(KeyboardDeadman, UnexpectedValuesAreNotHeld) {
  EXPECT_FALSE(key_is_held(-1));
  EXPECT_FALSE(key_is_held(3));
}

TEST(KeyboardDeadman, KnownKeyNamesResolve) {
  EXPECT_EQ(leap_teleop::key_code_from_name("space"), KEY_SPACE);
  EXPECT_EQ(leap_teleop::key_code_from_name("leftmeta"), KEY_LEFTMETA);
  EXPECT_EQ(leap_teleop::key_code_from_name("f12"), KEY_F12);
}

TEST(KeyboardDeadman, UnknownKeyNameListsAcceptedNames) {
  try {
    leap_teleop::key_code_from_name("hyper");
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("'hyper'"), std::string::npos);
    EXPECT_NE(message.find(" space"), std::string::npos);
    EXPECT_NE(message.find(" rightctrl"), std::string::npos);
  }
}

TEST(KeyboardDeadman, LettersDigitsAndFunctionKeysUseLayoutCodes) {
  const std::string letters = "abcdefghijklmnopqrstuvwxyz";
  const int letter_codes[] = {KEY_A, KEY_B, KEY_C, KEY_D, KEY_E, KEY_F, KEY_G, KEY_H, KEY_I,
                              KEY_J, KEY_K, KEY_L, KEY_M, KEY_N, KEY_O, KEY_P, KEY_Q, KEY_R,
                              KEY_S, KEY_T, KEY_U, KEY_V, KEY_W, KEY_X, KEY_Y, KEY_Z};
  for (std::size_t i = 0; i < letters.size(); ++i) {
    EXPECT_EQ(leap_teleop::key_code_from_name(std::string(1, letters[i])), letter_codes[i])
        << letters[i];
  }
  const int digit_codes[] = {KEY_0, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9};
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(leap_teleop::key_code_from_name(std::to_string(i)), digit_codes[i]) << i;
  }
  const int function_codes[] = {KEY_F1, KEY_F2, KEY_F3, KEY_F4,  KEY_F5,  KEY_F6,
                                KEY_F7, KEY_F8, KEY_F9, KEY_F10, KEY_F11, KEY_F12};
  for (int i = 0; i < 12; ++i) {
    EXPECT_EQ(leap_teleop::key_code_from_name("f" + std::to_string(i + 1)), function_codes[i])
        << "f" << i + 1;
  }
}
