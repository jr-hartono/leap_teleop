#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <leap_teleop_core/leap_teleop_supervisor_parameters.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <utility>

#include "leap_teleop_core/delta_builder.hpp"
#include "leap_teleop_core/supervisor_fsm.hpp"
#include "leap_teleop_core/timing.hpp"
#include "leap_teleop_msgs/msg/deadman_state.hpp"
#include "leap_teleop_msgs/msg/hand_pose.hpp"
#include "leap_teleop_msgs/msg/teleop_delta.hpp"
#include "leap_teleop_msgs/msg/teleop_state.hpp"

namespace leap_teleop {

namespace {

double stamp_seconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool finite_pose(const geometry_msgs::msg::Pose& pose) {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

std::string state_token(State state) {
  switch (state) {
    case State::kIdle:
      return leap_teleop_msgs::msg::TeleopState::IDLE;
    case State::kEngaged:
      return leap_teleop_msgs::msg::TeleopState::ENGAGED;
    case State::kHold:
      return leap_teleop_msgs::msg::TeleopState::HOLD;
  }
  return "";
}

}  // namespace

class SupervisorNode : public rclcpp::Node {
 public:
  SupervisorNode() : rclcpp::Node("leap_teleop_supervisor") {
    param_listener_ = std::make_shared<leap_teleop_supervisor::ParamListener>(this);
    const auto params = param_listener_->get_params();
    base_frame_ = params.base_frame;
    tracking_timeout_ = params.tracking_timeout;
    min_hand_age_ = params.min_hand_age;
    yaw_deg_ = params.yaw;
    side_ = params.side;
    current_params_ = params;
    translation_scale_ = params.translation_scale;
    rotation_scale_ = params.rotation_scale;
    param_listener_->setUserCallback(
        [this](const leap_teleop_supervisor::Params& updated) { apply_tuning(updated); });

    delta_builder_ = std::make_unique<DeltaBuilder>();
    fsm_ = std::make_unique<SupervisorFsm>(params.hold_timeout);

    hand_subscription_ = create_subscription<leap_teleop_msgs::msg::HandPose>(
        "hand_pose/" + side_, rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const leap_teleop_msgs::msg::HandPose::SharedPtr message) {
          on_hand_pose(*message);
        });
    deadman_subscription_ = create_subscription<leap_teleop_msgs::msg::DeadmanState>(
        "deadman", rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const leap_teleop_msgs::msg::DeadmanState::SharedPtr message) {
          latest_deadman_ = *message;
        });

    target_publisher_ = create_publisher<leap_teleop_msgs::msg::TeleopDelta>(
        "pose_delta", rclcpp::QoS(rclcpp::KeepLast(1)));
    state_publisher_ = create_publisher<leap_teleop_msgs::msg::TeleopState>(
        "state", rclcpp::QoS(rclcpp::KeepLast(1)));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / timing::kHeartbeatRate));
    heartbeat_ = create_wall_timer(period, [this]() { publish_if_due(true); });
  }

 private:
  // Runs on the parameter callback thread; the scales are atomics read by publish_if_due.
  void apply_tuning(const leap_teleop_supervisor::Params& params) {
    const auto log_change = [this](const char* name, double before, double after) {
      if (before != after) {
        RCLCPP_INFO(get_logger(), "parameter '%s' changed: %g -> %g", name, before, after);
      }
    };
    log_change("translation_scale", current_params_.translation_scale, params.translation_scale);
    log_change("rotation_scale", current_params_.rotation_scale, params.rotation_scale);
    current_params_ = params;
    translation_scale_ = params.translation_scale;
    rotation_scale_ = params.rotation_scale;
  }

  void on_hand_pose(const leap_teleop_msgs::msg::HandPose& message) {
    const double time = now().seconds();
    if (!have_hand_ || time - last_hand_time_ > tracking_timeout_) {
      hand_first_seen_ = time;
    }
    have_hand_ = true;
    last_hand_time_ = time;
    latest_hand_ = message;
    latest_hand_->pose = operator_pose_to_base(message.pose, yaw_deg_);
    publish_if_due(false);
  }

  void publish_if_due(bool heartbeat) {
    const double time = now().seconds();
    // Half a period: skip only a real duplicate, not a tick that fired early.
    if (heartbeat && last_publish_ > 0.0 && time - last_publish_ < 0.5 / timing::kHeartbeatRate) {
      return;
    }
    last_publish_ = time;

    bool present = false;
    bool usable = false;
    std::string reason = "no_hand";
    if (have_hand_ && latest_hand_) {
      const double age = time - stamp_seconds(latest_hand_->header.stamp);
      const double tracked_for = time - hand_first_seen_;
      present = age <= tracking_timeout_;
      if (!present) {
        reason = "stale_tracking";
      } else if (tracked_for < min_hand_age_) {
        reason = "hand_too_young";
      } else if (have_sequence_ && latest_hand_->sequence < last_sequence_) {
        reason = "invalid_sequence";
      } else if (!finite_pose(latest_hand_->pose)) {
        reason = "invalid_pose";
      } else {
        usable = true;
      }
      if (latest_hand_->sequence > last_sequence_ || !have_sequence_) {
        last_sequence_ = latest_hand_->sequence;
        have_sequence_ = true;
      }
    }

    bool key_held = false;
    if (latest_deadman_ && latest_deadman_->engaged) {
      key_held = (time - stamp_seconds(latest_deadman_->stamp)) <= timing::kEngageTimeout;
    }

    FsmInputs inputs;
    inputs.key_held = key_held;
    inputs.hand_usable = usable;
    inputs.time_sec = time;
    inputs.unusable_reason = reason;
    const FsmResult result = fsm_->update(inputs);
    const bool engaged = result.state == State::kEngaged;

    if (result.latch && latest_hand_) {
      delta_builder_->latch(latest_hand_->pose);
    }

    leap_teleop_msgs::msg::TeleopDelta target;
    if (latest_hand_) {
      target.header.stamp = latest_hand_->header.stamp;
    } else {
      target.header.stamp = now();
    }
    target.header.frame_id = base_frame_;
    target.side = side_;
    const std::string state_name = state_token(result.state);
    target.state = state_name;
    target.engaged = engaged;
    if (engaged && latest_hand_) {
      target.delta = to_transform_msg(
          delta_builder_->compute(latest_hand_->pose, translation_scale_, rotation_scale_));
    }
    target.sequence = ++sequence_;

    leap_teleop_msgs::msg::TeleopState state;
    state.header.stamp = now();
    state.state = state_name;
    state.reason = result.reason;
    state.tracking_ok = usable;
    state.hand_present = present;
    state.engaged = engaged;

    target_publisher_->publish(target);
    state_publisher_->publish(state);
  }

  std::shared_ptr<leap_teleop_supervisor::ParamListener> param_listener_;
  std::string base_frame_;
  leap_teleop_supervisor::Params current_params_;
  std::atomic<double> translation_scale_{1.0};
  std::atomic<double> rotation_scale_{1.0};
  std::string side_;
  double yaw_deg_ = 0.0;
  double tracking_timeout_ = 0.15;
  double min_hand_age_ = 0.2;
  double last_publish_ = 0.0;
  double last_hand_time_ = 0.0;
  double hand_first_seen_ = 0.0;
  bool have_hand_ = false;
  bool have_sequence_ = false;
  std::uint32_t last_sequence_ = 0;
  std::uint32_t sequence_ = 0;

  std::optional<leap_teleop_msgs::msg::HandPose> latest_hand_;
  std::optional<leap_teleop_msgs::msg::DeadmanState> latest_deadman_;
  std::unique_ptr<DeltaBuilder> delta_builder_;
  std::unique_ptr<SupervisorFsm> fsm_;

  rclcpp::Subscription<leap_teleop_msgs::msg::HandPose>::SharedPtr hand_subscription_;
  rclcpp::Subscription<leap_teleop_msgs::msg::DeadmanState>::SharedPtr deadman_subscription_;
  rclcpp::Publisher<leap_teleop_msgs::msg::TeleopDelta>::SharedPtr target_publisher_;
  rclcpp::Publisher<leap_teleop_msgs::msg::TeleopState>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_;
};

}  // namespace leap_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<leap_teleop::SupervisorNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("leap_teleop_supervisor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
