#include <gtest/gtest.h>

#include "leap_teleop_core/gripper_mapper.hpp"

using leap_teleop::GripperMapper;
using leap_teleop::GripperMapperConfig;
using leap_teleop::pinch_to_closure;
using leap_teleop::valid_pinch_range;

namespace {

GripperMapperConfig fast_config() {
  GripperMapperConfig config;
  config.filter_min_cutoff_hz = 1000.0;
  config.filter_beta = 0.0;
  return config;
}

}  // namespace

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

TEST(GripperMapper, FirstSampleOfAnEngagementIsEmittedUnfiltered) {
  GripperMapper mapper(GripperMapperConfig{});
  mapper.begin_engagement();
  const auto first = mapper.update(45.0, 0.0);
  ASSERT_TRUE(first.has_value());
  EXPECT_FLOAT_EQ(*first, 0.5F);
}

TEST(GripperMapper, DeadbandSuppressesSmallChanges) {
  GripperMapper mapper(fast_config());
  mapper.begin_engagement();
  ASSERT_TRUE(mapper.update(45.0, 0.0).has_value());
  EXPECT_FALSE(mapper.update(44.5, 0.1).has_value());  // 0.01
  EXPECT_FALSE(mapper.update(45.5, 0.2).has_value());  // -0.01
  const auto moved = mapper.update(43.0, 0.3);         // 0.04
  ASSERT_TRUE(moved.has_value());
  EXPECT_NEAR(*moved, 0.54F, 1e-3F);
  // The deadband is relative to the last emitted value, not the last sample.
  EXPECT_FALSE(mapper.update(43.5, 0.4).has_value());
}

TEST(GripperMapper, EndsAreReachableInsideTheDeadband) {
  GripperMapper mapper(fast_config());
  mapper.begin_engagement();
  ASSERT_TRUE(mapper.update(21.5, 0.0).has_value());  // 0.97
  const auto closed = mapper.update(10.0, 0.1);       // raw 1
  ASSERT_TRUE(closed.has_value());
  EXPECT_FLOAT_EQ(*closed, 1.0F);
  EXPECT_FALSE(mapper.update(10.0, 0.2).has_value());

  const auto open = mapper.update(100.0, 0.3);
  ASSERT_TRUE(open.has_value());
  EXPECT_FLOAT_EQ(*open, 0.0F);
  EXPECT_FALSE(mapper.update(100.0, 0.4).has_value());
}

TEST(GripperMapper, NewEngagementResetsFilterAndDeadband) {
  GripperMapperConfig config;
  config.filter_min_cutoff_hz = 1.0;
  config.filter_beta = 0.0;
  GripperMapper mapper(config);
  mapper.begin_engagement();
  ASSERT_TRUE(mapper.update(70.0, 0.0).has_value());
  (void)mapper.update(20.0, 0.01);
  mapper.begin_engagement();
  const auto first = mapper.update(45.0, 5.0);
  ASSERT_TRUE(first.has_value());
  EXPECT_FLOAT_EQ(*first, 0.5F);
  // Same value as the last engagement's output is still emitted first.
  mapper.begin_engagement();
  const auto again = mapper.update(45.0, 6.0);
  ASSERT_TRUE(again.has_value());
  EXPECT_FLOAT_EQ(*again, 0.5F);
}

TEST(GripperMapper, FilterSmoothsAStep) {
  GripperMapperConfig config;
  config.filter_min_cutoff_hz = 1.0;
  config.filter_beta = 0.0;
  config.deadband = 0.0;
  GripperMapper mapper(config);
  mapper.begin_engagement();
  ASSERT_TRUE(mapper.update(70.0, 0.0).has_value());
  const auto step = mapper.update(20.0, 0.02);
  ASSERT_TRUE(step.has_value());
  EXPECT_GT(*step, 0.0F);
  EXPECT_LT(*step, 0.5F);
}
