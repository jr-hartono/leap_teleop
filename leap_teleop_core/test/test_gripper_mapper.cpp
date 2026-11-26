#include <gtest/gtest.h>

#include "leap_teleop_core/gripper_mapper.hpp"

using leap_teleop::ClosureFilter;
using leap_teleop::ClosureGate;
using leap_teleop::GripperMapperConfig;
using leap_teleop::pinch_to_closure;
using leap_teleop::valid_pinch_range;

namespace {}  // namespace

TEST(GripperMapper, MapsLinearlyAndClampsBeyondBothEnds) {
  EXPECT_DOUBLE_EQ(pinch_to_closure(70.0, 20.0, 70.0), 0.0);
  EXPECT_DOUBLE_EQ(pinch_to_closure(20.0, 20.0, 70.0), 1.0);
  EXPECT_DOUBLE_EQ(pinch_to_closure(45.0, 20.0, 70.0), 0.5);
  EXPECT_DOUBLE_EQ(pinch_to_closure(120.0, 20.0, 70.0), 0.0);
  EXPECT_DOUBLE_EQ(pinch_to_closure(0.0, 20.0, 70.0), 1.0);
  EXPECT_DOUBLE_EQ(pinch_to_closure(-5.0, 20.0, 70.0), 1.0);
}

TEST(GripperMapper, ValidatesPinchRange) {
  EXPECT_TRUE(valid_pinch_range(20.0, 70.0));
  EXPECT_FALSE(valid_pinch_range(70.0, 70.0));
  EXPECT_FALSE(valid_pinch_range(80.0, 70.0));
  EXPECT_FALSE(valid_pinch_range(0.0, 70.0));
  EXPECT_FALSE(valid_pinch_range(-1.0, 70.0));
  EXPECT_FALSE(valid_pinch_range(20.0, 0.0));
}

TEST(ClosureFilter, FirstSampleIsUnfiltered) {
  ClosureFilter filter(GripperMapperConfig{});
  const auto sample = filter.update(45.0, 0.0);
  EXPECT_DOUBLE_EQ(sample.raw, 0.5);
  EXPECT_DOUBLE_EQ(sample.filtered, 0.5);
}

TEST(ClosureFilter, SmoothsAStep) {
  GripperMapperConfig config;
  config.filter_min_cutoff_hz = 1.0;
  config.filter_beta = 0.0;
  ClosureFilter filter(config);
  (void)filter.update(70.0, 0.0);
  const auto step = filter.update(20.0, 0.02);
  EXPECT_DOUBLE_EQ(step.raw, 1.0);
  EXPECT_GT(step.filtered, 0.0);
  EXPECT_LT(step.filtered, 0.5);
}

TEST(ClosureFilter, ResetMakesTheNextSampleUnfiltered) {
  GripperMapperConfig config;
  config.filter_min_cutoff_hz = 1.0;
  config.filter_beta = 0.0;
  ClosureFilter filter(config);
  (void)filter.update(70.0, 0.0);
  EXPECT_LT(filter.update(20.0, 0.02).filtered, 0.5);
  filter.reset();
  EXPECT_DOUBLE_EQ(filter.update(20.0, 0.04).filtered, 1.0);
}

TEST(ClosureFilter, EngagementDoesNotResetIt) {
  GripperMapperConfig config;
  config.filter_min_cutoff_hz = 1.0;
  config.filter_beta = 0.0;
  ClosureFilter filter(config);
  ClosureGate gate(config);
  (void)filter.update(70.0, 0.0);
  gate.begin_engagement();
  const auto sample = filter.update(20.0, 0.02);
  // Still smoothed: the gate starting an engagement leaves the filter alone.
  EXPECT_LT(sample.filtered, 0.5);
  const auto first = gate.update(sample);
  ASSERT_TRUE(first.has_value());
  EXPECT_FLOAT_EQ(*first, static_cast<float>(sample.filtered));
}

TEST(ClosureGate, FirstSampleOfAnEngagementIsAlwaysEmitted) {
  ClosureGate gate(GripperMapperConfig{});
  gate.begin_engagement();
  const auto first = gate.update({0.5, 0.5});
  ASSERT_TRUE(first.has_value());
  EXPECT_FLOAT_EQ(*first, 0.5F);
  EXPECT_FALSE(gate.update({0.5, 0.5}).has_value());
  // Same value as the last engagement's output is still emitted first.
  gate.begin_engagement();
  EXPECT_TRUE(gate.update({0.5, 0.5}).has_value());
}

TEST(ClosureGate, DeadbandSuppressesSmallChanges) {
  ClosureGate gate(GripperMapperConfig{});
  gate.begin_engagement();
  ASSERT_TRUE(gate.update({0.5, 0.5}).has_value());
  EXPECT_FALSE(gate.update({0.51, 0.51}).has_value());
  EXPECT_FALSE(gate.update({0.49, 0.49}).has_value());
  const auto moved = gate.update({0.54, 0.54});
  ASSERT_TRUE(moved.has_value());
  EXPECT_NEAR(*moved, 0.54F, 1e-6F);
  // The deadband is relative to the last emitted value, not the last sample.
  EXPECT_FALSE(gate.update({0.55, 0.55}).has_value());
}

TEST(ClosureGate, EndsAreReachableInsideTheDeadband) {
  ClosureGate gate(GripperMapperConfig{});
  gate.begin_engagement();
  ASSERT_TRUE(gate.update({0.97, 0.97}).has_value());
  const auto closed = gate.update({1.0, 0.99});  // raw at the end, filtered within the deadband
  ASSERT_TRUE(closed.has_value());
  EXPECT_FLOAT_EQ(*closed, 1.0F);
  EXPECT_FALSE(gate.update({1.0, 0.995}).has_value());

  const auto open = gate.update({0.0, 0.0});
  ASSERT_TRUE(open.has_value());
  EXPECT_FLOAT_EQ(*open, 0.0F);
  EXPECT_FALSE(gate.update({0.0, 0.01}).has_value());
}

TEST(ClosureGate, DoesNotSnapWhenFilteredIsFarFromTheEnd) {
  ClosureGate gate(GripperMapperConfig{});
  gate.begin_engagement();
  ASSERT_TRUE(gate.update({0.5, 0.5}).has_value());
  const auto moved = gate.update({1.0, 0.8});
  ASSERT_TRUE(moved.has_value());
  EXPECT_FLOAT_EQ(*moved, 0.8F);
}
