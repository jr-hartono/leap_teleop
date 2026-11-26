#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
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
#include "leap_teleop_msgs/msg/teleop_delta.hpp"
#include "leap_teleop_msgs/msg/teleop_gripper.hpp"
#include "leap_teleop_msgs/msg/teleop_hand.hpp"
#include "leap_teleop_msgs/msg/teleop_state.hpp"
#include "leap_teleop_msgs/msg/tracking_frame.hpp"

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

// The tracking mode is not a parameter; the hand mapper gets it from set_tracking_mode.
HandMapperConfig hand_config(const leap_teleop_mapper::Params& params) {
  HandMapperConfig config;
  config.deadband_translation_m = params.deadband_translation;
  config.deadband_rotation_rad = degrees_to_radians(params.deadband_rotation);
  config.deadband_hysteresis = params.deadband_hysteresis;
  config.filter_min_cutoff_hz = params.filter_min_cutoff;
  config.filter_beta = params.filter_beta;
  return config;
}

const leap_teleop_msgs::msg::Hand* find_hand(const leap_teleop_msgs::msg::TrackingFrame& frame,
                                             const std::string& side) {
  for (const auto& candidate : frame.hands) {
    if (candidate.side == side) {
      return &candidate;
    }
  }
  return nullptr;
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
    current_params_ = params;
    translation_scale_ = params.translation_scale;
    rotation_scale_ = params.rotation_scale;
    if (!valid_pinch_range(params.pinch_closed_mm, params.pinch_open_mm)) {
      throw std::runtime_error("pinch_closed_mm must be below pinch_open_mm");
    }

    // Broadcasts the device's mounting once the first tracking frame names the tracking mode.
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    param_listener_->setUserCallback(
        [this](const leap_teleop_mapper::Params& updated) { apply_tuning(updated); });

    for (std::size_t index = 0; index < sides_.size(); ++index) {
      init_side(sides_[index], kSideNames[index], params);
    }

    hands_subscription_ = create_subscription<leap_teleop_msgs::msg::TrackingFrame>(
        "hands", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile(),
        [this](const leap_teleop_msgs::msg::TrackingFrame::SharedPtr message) {
          on_hands(*message);
        });
    deadman_subscription_ = create_subscription<leap_teleop_msgs::msg::DeadmanState>(
        "deadman", rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const leap_teleop_msgs::msg::DeadmanState::SharedPtr message) {
          latest_deadman_ = *message;
        });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / timing::kHeartbeatRate));
    heartbeat_ = create_wall_timer(period, [this]() {
      for (auto& side : sides_) {
        publish_if_due(side, true);
      }
    });
  }

 private:
  // Everything one hand needs: its filters, state machine, bookkeeping and publishers. The two
  // sides share the parameters, the deadman key and the tracking mode, nothing else.
  struct SideChannel {
    std::string name;
    std::unique_ptr<HandMapper> hand_mapper;
    std::unique_ptr<ClosureFilter> closure_filter;
    std::unique_ptr<ClosureGate> closure_gate;
    std::unique_ptr<DeltaBuilder> delta_builder;
    std::unique_ptr<MapperFsm> fsm;
    ClosureSample latest_closure;
    double last_gripper_stamp = -1.0;
    double last_publish = 0.0;
    double last_hand_time = 0.0;
    double hand_first_seen = 0.0;
    bool have_hand = false;
    std::optional<leap_teleop_msgs::msg::TeleopHand> latest_hand;
    rclcpp::Publisher<leap_teleop_msgs::msg::TeleopDelta>::SharedPtr target_publisher;
    rclcpp::Publisher<leap_teleop_msgs::msg::TeleopHand>::SharedPtr hand_publisher;
    rclcpp::Publisher<leap_teleop_msgs::msg::TeleopGripper>::SharedPtr gripper_publisher;
    rclcpp::Publisher<leap_teleop_msgs::msg::TeleopState>::SharedPtr state_publisher;
  };

  void init_side(SideChannel& side, const std::string& name,
                 const leap_teleop_mapper::Params& params) {
    side.name = name;
    side.hand_mapper = std::make_unique<HandMapper>(hand_config(params));
    side.closure_filter = std::make_unique<ClosureFilter>(gripper_config(params));
    side.closure_gate = std::make_unique<ClosureGate>(gripper_config(params));
    side.delta_builder = std::make_unique<DeltaBuilder>();
    side.fsm = std::make_unique<MapperFsm>(params.hold_timeout);
    const std::string suffix = "/" + name;
    side.target_publisher = create_publisher<leap_teleop_msgs::msg::TeleopDelta>(
        "pose_delta" + suffix, rclcpp::QoS(rclcpp::KeepLast(1)));
    side.hand_publisher = create_publisher<leap_teleop_msgs::msg::TeleopHand>(
        "teleop_hand" + suffix, rclcpp::QoS(rclcpp::KeepLast(1)));
    side.gripper_publisher = create_publisher<leap_teleop_msgs::msg::TeleopGripper>(
        "gripper" + suffix, rclcpp::QoS(rclcpp::KeepLast(1)));
    side.state_publisher = create_publisher<leap_teleop_msgs::msg::TeleopState>(
        "state" + suffix, rclcpp::QoS(rclcpp::KeepLast(1)));
  }

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
      for (auto& side : sides_) {
        side.hand_mapper->set_tuning(hand_config(params));
      }
    }
    if (valid_pinch_range(params.pinch_closed_mm, params.pinch_open_mm)) {
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      for (auto& side : sides_) {
        side.closure_filter->set_config(gripper_config(params));
        side.closure_gate->set_config(gripper_config(params));
      }
    } else {
      RCLCPP_WARN(get_logger(),
                  "ignoring pinch_closed_mm %g and pinch_open_mm %g: closed must be below open",
                  params.pinch_closed_mm, params.pinch_open_mm);
    }
    current_params_ = params;
    translation_scale_ = params.translation_scale;
    rotation_scale_ = params.rotation_scale;
  }

  // Takes the tracking mode from the tracking frame. Returns false when the hands must be dropped:
  // the mode is unknown or has just changed, so poses from before and after do not connect. The
  // drop goes through the same path as a lost hand, so the FSM holds and needs a new key press.
  bool update_tracking_mode(const std::string& name) {
    std::optional<TrackingMode> mode;
    if (name != leap_teleop_msgs::msg::TrackingFrame::UNKNOWN) {
      try {
        mode = tracking_mode_from_string(name);
      } catch (const std::invalid_argument&) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "unknown tracking_mode '%s'",
                             name.c_str());
      }
    }
    if (!mode) {
      drop_hands();
      return false;
    }
    if (tracking_mode_ == mode) {
      return true;
    }
    const bool changed = tracking_mode_.has_value();
    if (changed) {
      RCLCPP_INFO(get_logger(), "tracking_mode changed: %s -> %s",
                  tracking_mode_to_string(*tracking_mode_), tracking_mode_to_string(*mode));
    } else {
      RCLCPP_INFO(get_logger(), "tracking_mode is %s", tracking_mode_to_string(*mode));
    }
    tracking_mode_ = mode;
    {
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      for (auto& side : sides_) {
        side.hand_mapper->set_tracking_mode(*mode);
        side.closure_filter->reset();
      }
    }
    geometry_msgs::msg::TransformStamped operator_to_device;
    operator_to_device.header.stamp = now();
    operator_to_device.header.frame_id = kOperatorFrame;
    operator_to_device.child_frame_id = kDeviceFrame;
    operator_to_device.transform = operator_to_device_transform(*mode);
    static_broadcaster_->sendTransform(operator_to_device);
    if (changed) {
      drop_hands();
      return false;
    }
    return true;
  }

  void drop_hands() {
    for (auto& side : sides_) {
      drop_hand(side);
    }
  }

  // Forgets the hand now rather than after tracking_timeout, so the FSM sees no hand.
  void drop_hand(SideChannel& side) {
    {
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      side.hand_mapper->reset();
      side.closure_filter->reset();
    }
    side.have_hand = false;
    side.latest_hand.reset();
  }

  // Maps each side's hand to the operator frame, filters it and publishes it. A hand that is
  // absent or not tracked is lost: its filters restart with the next sighting.
  void on_hands(const leap_teleop_msgs::msg::TrackingFrame& tracking_frame) {
    if (!update_tracking_mode(tracking_frame.tracking_mode)) {
      return;
    }
    for (auto& side : sides_) {
      const leap_teleop_msgs::msg::Hand* hand = nullptr;
      if (tracking_frame.is_tracking) {
        hand = find_hand(tracking_frame, side.name);
      }
      if (hand == nullptr) {
        const std::lock_guard<std::mutex> lock(processing_mutex_);
        side.hand_mapper->reset();
        side.closure_filter->reset();
        continue;
      }
      process_hand(side, *hand, tracking_frame.header.stamp);
    }
  }

  void process_hand(SideChannel& side, const leap_teleop_msgs::msg::Hand& hand,
                    const builtin_interfaces::msg::Time& stamp) {
    const double time = stamp_seconds(stamp);
    HandPoseInput input;
    input.position_m = point_from_msg(hand.pose.position);
    input.orientation = normalized(quaternion_from_msg(hand.pose.orientation));
    input.time_sec = time;

    leap_teleop_msgs::msg::TeleopHand processed;
    processed.header.stamp = stamp;
    processed.header.frame_id = kOperatorFrame;
    ClosureSample closure;
    {
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      processed.pose = side.hand_mapper->update(input);
      closure = side.closure_filter->update(static_cast<double>(hand.pinch_distance), time);
    }
    processed.closure = static_cast<float>(closure.filtered);
    side.hand_publisher->publish(processed);
    on_teleop_hand(side, processed, closure);
  }

  void on_teleop_hand(SideChannel& side, const leap_teleop_msgs::msg::TeleopHand& message,
                      const ClosureSample& closure) {
    const double time = now().seconds();
    if (!side.have_hand || time - side.last_hand_time > tracking_timeout_) {
      side.hand_first_seen = time;
    }
    side.have_hand = true;
    side.last_hand_time = time;
    side.latest_hand = message;
    side.latest_hand->pose = operator_pose_to_base(message.pose, yaw_deg_);
    side.latest_closure = closure;
    publish_if_due(side, false);
  }

  void publish_if_due(SideChannel& side, bool heartbeat) {
    const double time = now().seconds();
    // Half a period: skip only a real duplicate, not a tick that fired early.
    if (heartbeat && side.last_publish > 0.0 &&
        time - side.last_publish < 0.5 / timing::kHeartbeatRate) {
      return;
    }
    side.last_publish = time;

    bool present = false;
    bool usable = false;
    std::string reason = "no_hand";
    if (side.have_hand && side.latest_hand) {
      const double age = time - stamp_seconds(side.latest_hand->header.stamp);
      const double tracked_for = time - side.hand_first_seen;
      present = age <= tracking_timeout_;
      if (!present) {
        reason = "stale_tracking";
      } else if (tracked_for < min_hand_age_) {
        reason = "hand_too_young";
      } else if (!finite_pose(side.latest_hand->pose)) {
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
    const FsmResult result = side.fsm->update(inputs);
    const bool engaged = result.state == State::kEngaged;

    if (result.latch && side.latest_hand) {
      side.delta_builder->latch(side.latest_hand->pose);
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      side.closure_gate->begin_engagement();
      side.last_gripper_stamp = -1.0;
    }

    leap_teleop_msgs::msg::TeleopDelta target;
    if (side.latest_hand) {
      target.header.stamp = side.latest_hand->header.stamp;
    } else {
      target.header.stamp = now();
    }
    target.header.frame_id = base_frame_;
    target.engaged = engaged;
    if (engaged && side.latest_hand) {
      target.delta = to_transform_msg(
          side.delta_builder->compute(side.latest_hand->pose, translation_scale_, rotation_scale_));
    }

    leap_teleop_msgs::msg::TeleopState state;
    state.header.stamp = now();
    state.state = state_token(result.state);
    state.reason = result.reason;
    state.tracking_ok = usable;
    state.hand_present = present;
    state.engaged = engaged;

    side.target_publisher->publish(target);
    if (engaged && usable && side.latest_hand) {
      publish_gripper(side, *side.latest_hand);
    }
    side.state_publisher->publish(state);
  }

  // Feeds each hand sample once, so heartbeat ticks do not repeat it.
  void publish_gripper(SideChannel& side, const leap_teleop_msgs::msg::TeleopHand& hand) {
    const double stamp = stamp_seconds(hand.header.stamp);
    if (stamp == side.last_gripper_stamp) {
      return;
    }
    side.last_gripper_stamp = stamp;
    std::optional<float> closure;
    {
      const std::lock_guard<std::mutex> lock(processing_mutex_);
      closure = side.closure_gate->update(side.latest_closure);
    }
    if (closure) {
      leap_teleop_msgs::msg::TeleopGripper message;
      message.header.stamp = hand.header.stamp;
      message.closure = *closure;
      side.gripper_publisher->publish(message);
    }
  }

  std::shared_ptr<leap_teleop_mapper::ParamListener> param_listener_;
  std::string base_frame_;
  leap_teleop_mapper::Params current_params_;
  std::atomic<double> translation_scale_{1.0};
  std::atomic<double> rotation_scale_{1.0};
  double yaw_deg_ = 0.0;
  double tracking_timeout_ = 0.15;
  double min_hand_age_ = 0.2;
  std::optional<leap_teleop_msgs::msg::DeadmanState> latest_deadman_;
  const std::array<std::string, 2> kSideNames{leap_teleop_msgs::msg::Hand::LEFT,
                                              leap_teleop_msgs::msg::Hand::RIGHT};
  std::array<SideChannel, 2> sides_;
  // The mode the last tracking frame reported; unset until one does.
  std::optional<TrackingMode> tracking_mode_;
  std::mutex processing_mutex_;

  rclcpp::Subscription<leap_teleop_msgs::msg::TrackingFrame>::SharedPtr hands_subscription_;
  rclcpp::Subscription<leap_teleop_msgs::msg::DeadmanState>::SharedPtr deadman_subscription_;
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
