#include <chrono>
#include <cmath>
#include <leap_teleop_bridge/leap_gripper_bridge_parameters.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <stdexcept>
#include <string>

#include "leap_teleop_bridge/gripper_command.hpp"
#include "leap_teleop_msgs/msg/teleop_gripper.hpp"

namespace leap_teleop {

// Drives one gripper joint toward the position of the latest closure, at most max_velocity fast.
// The mapper sends closures only while engaged. Once the command reaches the target it is no
// longer published, and the controller holds it.
class GripperBridge : public rclcpp::Node {
 public:
  GripperBridge() : rclcpp::Node("leap_gripper_bridge") {
    param_listener_ = std::make_shared<leap_gripper_bridge::ParamListener>(this);
    const auto params = param_listener_->get_params();
    open_position_ = params.open_position;
    closed_position_ = params.closed_position;
    max_velocity_ = params.max_velocity;
    publish_period_ = params.publish_period;
    dry_run_ = params.dry_run;

    // The single-threaded executor cannot run this callback, on_gripper() and on_tick() at the
    // same time.
    param_listener_->setUserCallback([this](const leap_gripper_bridge::Params& updated) {
      const auto log_change = [this](const char* name, double before, double after) {
        if (before != after) {
          RCLCPP_INFO(get_logger(), "parameter '%s' changed: %g -> %g", name, before, after);
        }
      };
      log_change("open_position", open_position_, updated.open_position);
      log_change("closed_position", closed_position_, updated.closed_position);
      log_change("max_velocity", max_velocity_, updated.max_velocity);
      open_position_ = updated.open_position;
      closed_position_ = updated.closed_position;
      max_velocity_ = updated.max_velocity;
    });

    command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        params.command_topic, rclcpp::QoS(rclcpp::KeepLast(1)));
    gripper_subscription_ = create_subscription<leap_teleop_msgs::msg::TeleopGripper>(
        "gripper/" + params.side, rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const leap_teleop_msgs::msg::TeleopGripper::SharedPtr message) {
          on_gripper(*message);
        });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_period_));
    timer_ = create_wall_timer(period, [this]() { on_tick(); });

    if (dry_run_) {
      RCLCPP_WARN(get_logger(), "dry_run is enabled; commands will not be sent to the gripper");
    }
  }

 private:
  void on_gripper(const leap_teleop_msgs::msg::TeleopGripper& message) {
    if (!std::isfinite(message.closure)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "ignoring non-finite closure");
      return;
    }
    target_ = gripper_position(message.closure, open_position_, closed_position_);
    if (!command_) {
      // Without joint feedback, the first move starts from open_position.
      command_ = open_position_;
    }
  }

  void on_tick() {
    // Publish at least once, then only while moving.
    if (!target_ || (published_ && *command_ == *target_)) {
      return;
    }
    command_ = step_toward(*command_, *target_, max_velocity_ * publish_period_);
    published_ = true;

    if (dry_run_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "dry_run gripper: target=%.4f command=%.4f", *target_, *command_);
      return;
    }
    if (command_publisher_->get_subscription_count() == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "no subscriber on %s; is the gripper controller active?",
                           command_publisher_->get_topic_name());
    }
    std_msgs::msg::Float64MultiArray command;
    command.data.push_back(*command_);
    command_publisher_->publish(command);
  }

  std::shared_ptr<leap_gripper_bridge::ParamListener> param_listener_;
  double open_position_ = 0.0;
  double closed_position_ = 1.0;
  double max_velocity_ = 0.5;
  double publish_period_ = 0.01;
  bool dry_run_ = true;

  std::optional<double> target_;
  std::optional<double> command_;
  bool published_ = false;

  rclcpp::Subscription<leap_teleop_msgs::msg::TeleopGripper>::SharedPtr gripper_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace leap_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<leap_teleop::GripperBridge>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("leap_gripper_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
