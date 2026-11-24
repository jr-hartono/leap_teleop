#include <unistd.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "leap_teleop_core/keyboard_deadman.hpp"
#include "leap_teleop_core/parameters.hpp"
#include "leap_teleop_msgs/msg/deadman_state.hpp"

namespace leap_teleop {

class EngageNode : public rclcpp::Node {
 public:
  EngageNode() : rclcpp::Node("leap_teleop_engage") {
    const std::string key_name = declare_parameter<std::string>("key", "space");
    const int key_code = key_code_from_name(key_name);
    const std::string configured_device = declare_parameter<std::string>("device", "");
    publish_rate_ = declare_bounded_double(this, "publish_rate", 20.0, 5.0, 100.0);

    const std::vector<std::string> candidates = discover_keyboards();
    std::string selected;
    if (!configured_device.empty()) {
      if (access(configured_device.c_str(), R_OK) != 0) {
        throw std::runtime_error("configured engage device '" + configured_device +
                                 "' is not readable");
      }
      selected = configured_device;
    } else {
      if (candidates.empty()) {
        throw std::runtime_error("no keyboard found under /dev/input; configure 'device'");
      }
      selected = candidates.front();
    }

    for (const auto& candidate : candidates) {
      if (candidate != selected) {
        RCLCPP_INFO(get_logger(), "rejected keyboard candidate: %s", candidate.c_str());
      }
    }
    RCLCPP_INFO(get_logger(), "using keyboard %s, key %s (code %d)", selected.c_str(),
                key_name.c_str(), key_code);

    deadman_ = std::make_unique<KeyboardDeadman>(selected, key_code);
    deadman_->open_device();

    publisher_ = create_publisher<leap_teleop_msgs::msg::DeadmanState>(
        "deadman", rclcpp::QoS(rclcpp::KeepLast(1)));
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_));
    timer_ = create_wall_timer(period, [this]() { on_tick(); });
  }

 private:
  void on_tick() {
    deadman_->poll();
    if (!deadman_->healthy() && !loss_reported_) {
      RCLCPP_WARN(get_logger(), "keyboard %s is gone; publishing engaged=false",
                  deadman_->device_path().c_str());
      loss_reported_ = true;
    }
    leap_teleop_msgs::msg::DeadmanState message;
    message.stamp = now();
    message.engaged = deadman_->healthy() && deadman_->engaged();
    message.source = "evdev";
    publisher_->publish(message);
  }

  double publish_rate_ = 20.0;
  bool loss_reported_ = false;
  std::unique_ptr<KeyboardDeadman> deadman_;
  rclcpp::Publisher<leap_teleop_msgs::msg::DeadmanState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace leap_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<leap_teleop::EngageNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("leap_teleop_engage"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
