#include <gtest/gtest.h>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "leap_teleop_core/parameters.hpp"

namespace {

struct RosContext {
  RosContext() { rclcpp::init(0, nullptr); }
  ~RosContext() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
} ros_context;

std::shared_ptr<rclcpp::Node> node_with(double value) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("gain", value)});
  return std::make_shared<rclcpp::Node>("parameter_test", options);
}

}  // namespace

TEST(Parameters, BoundsAreAccepted) {
  EXPECT_DOUBLE_EQ(leap_teleop::declare_bounded_double(node_with(0.001).get(), "gain", 1.0,
                                                       0.0, 10.0, false),
                   0.001);
  EXPECT_DOUBLE_EQ(
      leap_teleop::declare_bounded_double(node_with(10.0).get(), "gain", 1.0, 0.0, 10.0),
      10.0);
}

TEST(Parameters, OutOfRangeIsRejectedWithTheParameterName) {
  try {
    leap_teleop::declare_bounded_double(node_with(11.0).get(), "gain", 1.0, 0.0, 10.0);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("gain"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find("[0"), std::string::npos);
  }
}

TEST(Parameters, ExclusiveLowerBoundIsEnforced) {
  EXPECT_THROW(leap_teleop::declare_bounded_double(node_with(0.0).get(), "gain", 1.0,
                                                   0.0, 10.0, false),
               std::invalid_argument);
}

TEST(Parameters, ChoiceMustMatchAnAllowedValue) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("mode", std::string("head_mounted"))});
  auto node = std::make_shared<rclcpp::Node>("choice_test", options);
  EXPECT_THROW(leap_teleop::declare_choice(node.get(), "mode", "desktop", {"desktop"}),
               std::invalid_argument);
}
