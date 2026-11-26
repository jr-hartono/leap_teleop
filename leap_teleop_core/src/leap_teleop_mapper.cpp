#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <leap_teleop_core/leap_teleop_mapper_parameters.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <utility>

#include "leap_teleop_core/delta_builder.hpp"
#include "leap_teleop_core/frames.hpp"
#include "leap_teleop_core/gripper_mapper.hpp"
#include "leap_teleop_core/hand_mapper.hpp"
#include "leap_teleop_core/mapper_fsm.hpp"
#include "leap_teleop_core/timing.hpp"
#include "leap_teleop_msgs/msg/deadman_state.hpp"
#include "leap_teleop_msgs/msg/hand.hpp"
#include "leap_teleop_msgs/msg/hand_array.hpp"
#include "leap_teleop_msgs/msg/teleop_delta.hpp"
#include "leap_teleop_msgs/msg/teleop_gripper.hpp"
#include "leap_teleop_msgs/msg/teleop_hand.hpp"
#include "leap_teleop_msgs/msg/teleop_state.hpp"

namespace leap_teleop {

namespace {

constexpr char kOperatorFrame[] = "leap_operator_frame";
constexpr char kDeviceFrame[] = "leap_device_frame";

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

GripperMapperConfig gripper_config(const leap_teleop_mapper::Params& params) {
  GripperMapperConfig config;
  config.pinch_closed_mm = params.pinch_closed_mm;
  config.pinch_open_mm = params.pinch_open_mm;
  config.filter_min_cutoff_hz = params.gripper_filter_min_cutoff;
  config.filter_beta = params.gripper_filter_beta;
  config.deadband = params.gripper_deadband;
  return config;
}

HandMapperConfig hand_config(const leap_teleop_mapper::Params& params, MountMode mount_mode) {
  HandMapperConfig config;
  config.mount_mode = mount_mode;
  config.deadband_translation_m = params.deadband_translation;
  config.deadband_rotation_rad = degrees_to_radians(params.deadband_rotation);
  config.deadband_hysteresis = params.deadband_hysteresis;
  config.filter_min_cutoff_hz = params.filter_min_cutoff;
  config.filter_beta = params.filter_beta;
  return config;
}

}  // namespace

class MapperNode : public rclcpp::Node {
 public:
  MapperNode() : rclcpp::Node("leap_teleop_mapper") {
    param_listener_ = std::make_shared<leap_teleop_mapper::ParamListener>(this);
    const auto params = param_listener_->get_params();
    base_frame_ = params.base_frame;
    tracking_timeout_ = params.tracking_timeout;
    min_hand_age_ = params.min_hand_age;
    yaw_deg_ = params.yaw;
    side_ = params.side;
    current_params_ = params;
    translation_scale_ = params.translation_scale;
    rotation_scale_ = params.rotation_scale;
    if (!valid_pinch_range(params.pinch_closed_mm, params.pinch_open_mm)) {
      throw std::runtime_error("pinch_closed_mm must be below pinch_open_mm");
    }
    mount_mode_ = mount_mode_from_string(params.mount_mode);
    hand_mapper_ = std::make_unique<HandMapper>(hand_config(params, mount_mode_));
    closure_filter_ = std::make_unique<ClosureFilter>(gripper_config(params));
    closure_gate_ = std::make_unique<ClosureGate>(gripper_config(params));

    // The device's mounting, as TF, so the raw hands can be shown next to the processed one.
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped operator_to_device;
    operator_to_device.header.stamp = now();
    operator_to_device.header.frame_id = kOperatorFrame;
    operator_to_device.child_frame_id = kDeviceFrame;
    operator_to_device.transform = operator_to_device_transform(mount_mode_);
    static_broadcaster_->sendTransform(operator_to_device);
    param_listener_->setUserCallback(
        [this](const leap_teleop_mapper::Params& updated) { apply_tuning(updated); });

    delta_builder_ = std::make_unique<DeltaBuilder>();
    fsm_ = std::make_unique<MapperFsm>(params.hold_timeout);

    hands_subscription_ = create_subscription<leap_teleop_msgs::msg::HandArray>(
        "hands", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile(),
        [this](const leap_teleop_msgs::msg::HandArray::SharedPtr message) { on_hands(*message); });
    deadman_subscription_ = create_subscription<leap_teleop_msgs::msg::DeadmanState>(
        "deadman", rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const leap_teleop_msgs::msg::DeadmanState::SharedPtr message) {
          latest_deadman_ = *message;
        });

    target_publisher_ = create_publisher<leap_teleop_msgs::msg::TeleopDelta>(
        "pose_delta/" + side_, rclcpp::QoS(rclcpp::KeepLast(1)));
    hand_publisher_ = create_publisher<leap_teleop_msgs::msg::TeleopHand>(
        "teleop_hand/" + side_, rclcpp::QoS(rclcpp::KeepLast(1)));
    gripper_publisher_ = create_publisher<leap_teleop_msgs::msg::TeleopGripper>(
        "gripper/" + side_, rclcpp::QoS(rclcpp::KeepLast(1)));
    state_publisher_ = create_publisher<leap_teleop_msgs::msg::TeleopState>(
        "state/" + side_, rclcpp::QoS(rclcpp::KeepLast(1)));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / timing::kHeartbeatRate));
    heartbeat_ = create_wall_timer(period, [this]() { publish_if_due(true); });
  }

 private:
  // Runs on the parameter callback thread; the scales are atomics read by publish_if_due.
  void apply_tuning(const leap_teleop_mapper::Params& params) {
    const auto log_change = [this](const char* name, double before, double after) {
      if (before != after) {
        RCLCPP_INFO(get_logger(), "parameter '%s' changed: %g -> %g", name, before, after);
      }
    };
    log_change("translation_scale", current_params_.translation_scale, params.translation_scale);
    log_change("rotation_scale", current_params_.rotation_scale, params.rotation_scale);
    log_change("deadband_translation", current_params_.deadband_translation,
               params.deadband_translation);
    log_change("deadband_rotation", current_params_.deadband_rotation, params.deadband_rotation);
    log_change("deadband_hysteresis", current_params_.deadband_hysteresis,
               params.deadband_hysteresis);
    log_change("filter_min_cutoff", current_params_.filter_min_cutoff, params.filter_min_cutoff);
    log_change("filter_beta", current_params_.filter_beta, params.filter_beta);
    {
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      hand_mapper_->set_tuning(hand_config(params, mount_mode_));
    }
    if (valid_pinch_range(params.pinch_closed_mm, params.pinch_open_mm)) {
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      closure_filter_->set_config(gripper_config(params));
      closure_gate_->set_config(gripper_config(params));
    } else {
      RCLCPP_WARN(get_logger(),
                  "ignoring pinch_closed_mm %g and pinch_open_mm %g: closed must be below open",
                  params.pinch_closed_mm, params.pinch_open_mm);
    }
    current_params_ = params;
    translation_scale_ = params.translation_scale;
    rotation_scale_ = params.rotation_scale;
  }

  // Maps the configured side's hand to the operator frame, filters it and publishes it. A hand
  // that is absent or not tracked is lost: its filters restart with the next sighting.
  void on_hands(const leap_teleop_msgs::msg::HandArray& array) {
    const leap_teleop_msgs::msg::Hand* hand = nullptr;
    if (array.is_tracking) {
      for (const auto& candidate : array.hands) {
        if (candidate.side == side_) {
          hand = &candidate;
          break;
        }
      }
    }
    if (hand == nullptr) {
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      hand_mapper_->reset();
      closure_filter_->reset();
      return;
    }

    const double time = stamp_seconds(array.header.stamp);
    HandPoseInput input;
    input.position_m = point_from_msg(hand->pose.position);
    input.orientation = normalized(quaternion_from_msg(hand->pose.orientation));
    input.time_sec = time;

    leap_teleop_msgs::msg::TeleopHand processed;
    processed.header.stamp = array.header.stamp;
    processed.header.frame_id = kOperatorFrame;
    ClosureSample closure;
    {
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      processed.pose = hand_mapper_->update(input);
      closure = closure_filter_->update(static_cast<double>(hand->pinch_distance), time);
    }
    processed.closure = static_cast<float>(closure.filtered);
    hand_publisher_->publish(processed);
    on_teleop_hand(processed, closure);
  }

  void on_teleop_hand(const leap_teleop_msgs::msg::TeleopHand& message,
                      const ClosureSample& closure) {
    const double time = now().seconds();
    if (!have_hand_ || time - last_hand_time_ > tracking_timeout_) {
      hand_first_seen_ = time;
    }
    have_hand_ = true;
    last_hand_time_ = time;
    latest_hand_ = message;
    latest_hand_->pose = operator_pose_to_base(message.pose, yaw_deg_);
    latest_closure_ = closure;
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
      } else if (!finite_pose(latest_hand_->pose)) {
        reason = "invalid_pose";
      } else {
        usable = true;
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
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      closure_gate_->begin_engagement();
      last_gripper_stamp_ = -1.0;
    }

    leap_teleop_msgs::msg::TeleopDelta target;
    if (latest_hand_) {
      target.header.stamp = latest_hand_->header.stamp;
    } else {
      target.header.stamp = now();
    }
    target.header.frame_id = base_frame_;
    target.engaged = engaged;
    if (engaged && latest_hand_) {
      target.delta = to_transform_msg(
          delta_builder_->compute(latest_hand_->pose, translation_scale_, rotation_scale_));
    }

    leap_teleop_msgs::msg::TeleopState state;
    state.header.stamp = now();
    state.state = state_token(result.state);
    state.reason = result.reason;
    state.tracking_ok = usable;
    state.hand_present = present;
    state.engaged = engaged;

    target_publisher_->publish(target);
    if (engaged && usable && latest_hand_) {
      publish_gripper(*latest_hand_);
    }
    state_publisher_->publish(state);
  }

  // Feeds each hand sample once, so heartbeat ticks do not repeat it.
  void publish_gripper(const leap_teleop_msgs::msg::TeleopHand& hand) {
    const double stamp = stamp_seconds(hand.header.stamp);
    if (stamp == last_gripper_stamp_) {
      return;
    }
    last_gripper_stamp_ = stamp;
    std::optional<float> closure;
    {
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      closure = closure_gate_->update(latest_closure_);
    }
    if (closure) {
      leap_teleop_msgs::msg::TeleopGripper message;
      message.header.stamp = hand.header.stamp;
      message.closure = *closure;
      gripper_publisher_->publish(message);
    }
  }

  std::shared_ptr<leap_teleop_mapper::ParamListener> param_listener_;
  std::string base_frame_;
  leap_teleop_mapper::Params current_params_;
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

  std::optional<leap_teleop_msgs::msg::TeleopHand> latest_hand_;
  std::optional<leap_teleop_msgs::msg::DeadmanState> latest_deadman_;
  std::unique_ptr<DeltaBuilder> delta_builder_;
  std::unique_ptr<MapperFsm> fsm_;
  MountMode mount_mode_ = MountMode::kDesktop;
  std::mutex processing_mutex_;
  std::unique_ptr<HandMapper> hand_mapper_;
  std::unique_ptr<ClosureFilter> closure_filter_;
  std::unique_ptr<ClosureGate> closure_gate_;
  ClosureSample latest_closure_;
  double last_gripper_stamp_ = -1.0;

  rclcpp::Subscription<leap_teleop_msgs::msg::HandArray>::SharedPtr hands_subscription_;
  rclcpp::Subscription<leap_teleop_msgs::msg::DeadmanState>::SharedPtr deadman_subscription_;
  rclcpp::Publisher<leap_teleop_msgs::msg::TeleopDelta>::SharedPtr target_publisher_;
  rclcpp::Publisher<leap_teleop_msgs::msg::TeleopHand>::SharedPtr hand_publisher_;
  rclcpp::Publisher<leap_teleop_msgs::msg::TeleopGripper>::SharedPtr gripper_publisher_;
  rclcpp::Publisher<leap_teleop_msgs::msg::TeleopState>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

}  // namespace leap_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<leap_teleop::MapperNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("leap_teleop_mapper"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
