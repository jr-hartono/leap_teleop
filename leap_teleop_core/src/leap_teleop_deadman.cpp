#include <chrono>
#include <cstddef>
#include <leap_teleop_core/leap_teleop_deadman_parameters.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "leap_teleop_core/keyboard_deadman.hpp"
#include "leap_teleop_core/timing.hpp"
#include "leap_teleop_msgs/msg/deadman_state.hpp"

namespace leap_teleop {

class DeadmanNode : public rclcpp::Node {
 public:
  DeadmanNode() : rclcpp::Node("leap_teleop_deadman") {
    param_listener_ = std::make_shared<leap_teleop_deadman::ParamListener>(this);
    const auto params = param_listener_->get_params();
    const int key_code = key_code_from_name(params.key);
    const std::string configured_device = params.device;

    // With no device configured, listen on every keyboard: the operator may press the key on
    // any of them, and the first one found is often not the one in use.
    const std::vector<std::string> paths = configured_device.empty()
                                               ? discover_keyboards()
                                               : std::vector<std::string>{configured_device};
    for (const auto& path : paths) {
      auto keyboard = std::make_unique<KeyboardDeadman>(path, key_code);
      try {
        keyboard->open_device();
      } catch (const std::runtime_error& error) {
        if (!configured_device.empty()) {
          throw;
        }
        RCLCPP_WARN(get_logger(), "skipping keyboard: %s", error.what());
        continue;
      }
      RCLCPP_INFO(get_logger(), "listening on keyboard %s", path.c_str());
      keyboards_.push_back(std::move(keyboard));
    }
    if (keyboards_.empty()) {
      throw std::runtime_error("no readable keyboard found under /dev/input; configure 'device'");
    }
    loss_reported_.assign(keyboards_.size(), false);
    RCLCPP_INFO(get_logger(), "deadman key %s (code %d)", params.key.c_str(), key_code);

    publisher_ = create_publisher<leap_teleop_msgs::msg::DeadmanState>(
        "deadman", rclcpp::QoS(rclcpp::KeepLast(1)));
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / timing::kDeadmanRate));
    timer_ = create_wall_timer(period, [this]() { on_tick(); });
  }

 private:
  void on_tick() {
    bool engaged = false;
    bool any_healthy = false;
    for (std::size_t i = 0; i < keyboards_.size(); ++i) {
      KeyboardDeadman& keyboard = *keyboards_[i];
      keyboard.poll();
      if (keyboard.healthy()) {
        any_healthy = true;
        engaged = engaged || keyboard.engaged();
      } else if (!loss_reported_[i]) {
        RCLCPP_WARN(get_logger(), "keyboard %s is gone", keyboard.device_path().c_str());
        loss_reported_[i] = true;
      }
    }
    if (!any_healthy && !all_lost_reported_) {
      RCLCPP_ERROR(get_logger(), "no keyboard left; publishing engaged=false");
      all_lost_reported_ = true;
    }
    leap_teleop_msgs::msg::DeadmanState message;
    message.stamp = now();
    message.engaged = engaged;
    message.source = "evdev";
    publisher_->publish(message);
  }

  std::shared_ptr<leap_teleop_deadman::ParamListener> param_listener_;
  std::vector<std::unique_ptr<KeyboardDeadman>> keyboards_;
  std::vector<bool> loss_reported_;
  bool all_lost_reported_ = false;
  rclcpp::Publisher<leap_teleop_msgs::msg::DeadmanState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace leap_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<leap_teleop::DeadmanNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("leap_teleop_deadman"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
