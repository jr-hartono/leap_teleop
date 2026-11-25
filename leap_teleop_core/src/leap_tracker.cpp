#include "leap_teleop_core/leap_tracker.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <leap_teleop_core/leap_tracker_parameters.hpp>
#include <leap_teleop_msgs/msg/hand.hpp>
#include <leap_teleop_msgs/msg/hand_array.hpp>
#include <leap_teleop_msgs/msg/hand_pose.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <thread>

#include "leap_teleop_core/frames.hpp"
#include "leap_teleop_core/hand_mapper.hpp"

namespace leap_teleop {

LeapConnection::LeapConnection() {
  LEAP_CONNECTION_CONFIG config{};
  config.size = sizeof(config);
  config.flags = 0;

  eLeapRS result = LeapCreateConnection(&config, &handle_);
  if (LEAP_FAILED(result) || handle_ == nullptr) {
    handle_ = nullptr;
    throw std::runtime_error("LeapCreateConnection failed with code " +
                             std::to_string(static_cast<int>(result)));
  }

  result = LeapOpenConnection(handle_);
  if (LEAP_FAILED(result)) {
    LeapDestroyConnection(handle_);
    handle_ = nullptr;
    throw std::runtime_error("LeapOpenConnection failed with code " +
                             std::to_string(static_cast<int>(result)));
  }
}

LeapConnection::~LeapConnection() {
  if (handle_ != nullptr) {
    LeapCloseConnection(handle_);
    LeapDestroyConnection(handle_);
    handle_ = nullptr;
  }
}

namespace {

using leap_teleop_msgs::msg::Hand;
using leap_teleop_msgs::msg::HandArray;
using leap_teleop_msgs::msg::HandPose;

constexpr std::uint32_t kPollTimeoutMs = 10;
constexpr auto kIdlePeriod = std::chrono::milliseconds(100);
constexpr auto kFrameTimeout = std::chrono::milliseconds(250);
constexpr auto kDeviceWarnDelay = std::chrono::seconds(2);
constexpr auto kDeviceErrorDelay = std::chrono::seconds(10);
constexpr char kDeviceFrame[] = "leap_device_frame";
constexpr char kHandFrame[] = "leap_teleop_frame";

class LeapTrackerNode : public rclcpp::Node {
 public:
  explicit LeapTrackerNode(const rclcpp::NodeOptions& options);
  ~LeapTrackerNode() override;

  void start();
  void stop();

 private:
  void apply_tuning(const leap_tracker::Params& params);
  void reset_mapper();
  void poll_loop();
  void handle_event(const LEAP_CONNECTION_MESSAGE& message);
  void publish_tracking(const LEAP_TRACKING_EVENT* frame);
  void publish_idle();
  void set_desktop_mode();

  rclcpp::Publisher<HandArray>::SharedPtr hands_pub_;
  rclcpp::Publisher<HandPose>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr idle_timer_;
  std::shared_ptr<leap_tracker::ParamListener> param_listener_;
  leap_tracker::Params current_params_;
  std::unique_ptr<HandMapper> mapper_;
  std::mutex mapper_mutex_;
  std::unique_ptr<LeapConnection> connection_;
  std::unique_ptr<std::thread> poll_thread_;

  std::atomic<bool> running_{false};
  std::atomic<bool> device_available_{false};
  std::string target_side_ = Hand::RIGHT;
  std::uint32_t sequence_ = 0;

  std::mutex state_mutex_;
  const std::chrono::steady_clock::time_point start_time_ = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_frame_time_{};
  bool missing_device_warned_ = false;
  bool missing_device_reported_ = false;
};

LeapTrackerNode::LeapTrackerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("leap_tracker", options) {
  param_listener_ = std::make_shared<leap_tracker::ParamListener>(this);
  const auto params = param_listener_->get_params();
  current_params_ = params;
  if (params.mount_mode != "desktop") {
    throw std::invalid_argument(
        "parameter 'mount_mode': only 'desktop' is implemented; "
        "'head_mounted' requires head pose composition");
  }

  HandMapperConfig config;
  config.yaw_deg = params.yaw;
  config.translation_scale = params.translation_scale;
  config.rotation_scale = params.rotation_scale;
  config.deadband_translation_m = params.deadband_translation;
  config.deadband_rotation_rad = degrees_to_radians(params.deadband_rotation);
  config.deadband_hysteresis = params.deadband_hysteresis;
  config.filter_min_cutoff_hz = params.filter_min_cutoff;
  config.filter_beta = params.filter_beta;
  mapper_ = std::make_unique<HandMapper>(config);
  param_listener_->setUserCallback(
      [this](const leap_tracker::Params& updated) { apply_tuning(updated); });

  target_side_ = (params.side == "left") ? Hand::LEFT : Hand::RIGHT;

  hands_pub_ = create_publisher<HandArray>(
      "hands", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile());
  target_pub_ = create_publisher<HandPose>("hand_pose",
                                           rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile());

  idle_timer_ = create_wall_timer(kIdlePeriod, [this]() { publish_idle(); });

  connection_ = std::make_unique<LeapConnection>();
}

void LeapTrackerNode::apply_tuning(const leap_tracker::Params& params) {
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
  current_params_ = params;

  HandMapperConfig config;
  config.translation_scale = params.translation_scale;
  config.rotation_scale = params.rotation_scale;
  config.deadband_translation_m = params.deadband_translation;
  config.deadband_rotation_rad = degrees_to_radians(params.deadband_rotation);
  config.deadband_hysteresis = params.deadband_hysteresis;
  config.filter_min_cutoff_hz = params.filter_min_cutoff;
  config.filter_beta = params.filter_beta;

  std::lock_guard<std::mutex> lock(mapper_mutex_);
  mapper_->set_tuning(config);
}

void LeapTrackerNode::reset_mapper() {
  std::lock_guard<std::mutex> lock(mapper_mutex_);
  mapper_->reset();
}

LeapTrackerNode::~LeapTrackerNode() { stop(); }

void LeapTrackerNode::start() {
  if (poll_thread_) {
    return;
  }
  running_.store(true, std::memory_order_relaxed);
  poll_thread_ = std::make_unique<std::thread>(&LeapTrackerNode::poll_loop, this);
}

void LeapTrackerNode::stop() {
  running_.store(false, std::memory_order_relaxed);
  if (poll_thread_ && poll_thread_->joinable()) {
    poll_thread_->join();
  }
}

void LeapTrackerNode::set_desktop_mode() {
  const eLeapRS result = LeapSetTrackingMode(connection_->handle(), eLeapTrackingMode_Desktop);
  if (LEAP_FAILED(result)) {
    RCLCPP_WARN(this->get_logger(), "LeapSetTrackingMode(desktop) failed with code %d",
                static_cast<int>(result));
  }
}

void LeapTrackerNode::poll_loop() {
  LEAP_CONNECTION_MESSAGE message{};

  while (running_.load(std::memory_order_relaxed)) {
    message.size = sizeof(message);
    const eLeapRS result = LeapPollConnection(connection_->handle(), kPollTimeoutMs, &message);
    if (LEAP_FAILED(result)) {
      if (result != eLeapRS_Timeout) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000,
                             "LeapPollConnection failed with code %d", static_cast<int>(result));
      }
      continue;
    }
    if (message.type == eLeapEventType_None) {
      continue;
    }
    handle_event(message);
  }
}

void LeapTrackerNode::handle_event(const LEAP_CONNECTION_MESSAGE& message) {
  switch (message.type) {
    case eLeapEventType_Connection:
      RCLCPP_INFO(this->get_logger(), "tracking service connected");
      set_desktop_mode();
      break;

    case eLeapEventType_ConnectionLost:
      RCLCPP_WARN(this->get_logger(), "tracking service connection lost");
      device_available_.store(false);
      reset_mapper();
      break;

    case eLeapEventType_Device:
      RCLCPP_INFO(this->get_logger(), "device %u attached",
                  static_cast<unsigned int>(message.device_event->device.id));
      device_available_.store(true);
      set_desktop_mode();
      break;

    case eLeapEventType_DeviceLost:
      RCLCPP_WARN(this->get_logger(), "device %u detached",
                  static_cast<unsigned int>(message.device_event->device.id));
      device_available_.store(false);
      reset_mapper();
      break;

    case eLeapEventType_DeviceFailure:
      RCLCPP_WARN(this->get_logger(), "device failure reported");
      device_available_.store(false);
      reset_mapper();
      break;

    case eLeapEventType_TrackingMode:
      if (message.tracking_mode_event->current_tracking_mode != eLeapTrackingMode_Desktop) {
        RCLCPP_WARN(this->get_logger(), "tracking mode %d is not desktop; requesting desktop mode",
                    static_cast<int>(message.tracking_mode_event->current_tracking_mode));
        set_desktop_mode();
      }
      break;

    case eLeapEventType_Tracking:
      publish_tracking(message.tracking_event);
      break;

    default:
      break;
  }
}

void LeapTrackerNode::publish_tracking(const LEAP_TRACKING_EVENT* frame) {
  if (frame == nullptr) {
    return;
  }

  device_available_.store(true);
  const auto stamp = now();
  HandArray array;
  array.header.stamp = stamp;
  array.header.frame_id = kDeviceFrame;
  array.tracking_frame_id = static_cast<std::uint64_t>(frame->tracking_frame_id);
  array.device_timestamp_us = static_cast<std::uint64_t>(frame->info.timestamp);
  array.framerate = frame->framerate;
  array.is_tracking = true;

  Eigen::Vector3d selected_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond selected_orientation = Eigen::Quaterniond::Identity();
  bool selected_found = false;

  if (frame->pHands != nullptr) {
    array.hands.reserve(frame->nHands);
    for (std::uint32_t index = 0; index < frame->nHands; ++index) {
      const LEAP_HAND& hand = frame->pHands[index];
      Hand reported;
      reported.side = (hand.type == eLeapHandType_Left) ? Hand::LEFT : Hand::RIGHT;
      reported.visible_time_us = hand.visible_time;

      reported.palm.position.x = hand.palm.position.x * 0.001F;
      reported.palm.position.y = hand.palm.position.y * 0.001F;
      reported.palm.position.z = hand.palm.position.z * 0.001F;
      reported.palm.orientation.x = hand.palm.orientation.x;
      reported.palm.orientation.y = hand.palm.orientation.y;
      reported.palm.orientation.z = hand.palm.orientation.z;
      reported.palm.orientation.w = hand.palm.orientation.w;

      if (reported.side == target_side_) {
        selected_position =
            Eigen::Vector3d(hand.palm.position.x, hand.palm.position.y, hand.palm.position.z) *
            0.001;
        selected_orientation =
            normalized(Eigen::Quaterniond(hand.palm.orientation.w, hand.palm.orientation.x,
                                          hand.palm.orientation.y, hand.palm.orientation.z));
        selected_found = true;
      }
      array.hands.push_back(reported);
    }
  }

  HandPoseInput input;
  HandPose target;
  if (selected_found) {
    input.position_m = selected_position;
    input.orientation = selected_orientation;
    input.time_sec = stamp.seconds();
    input.side = target_side_;

    target.header.stamp = stamp;
    target.header.frame_id = kHandFrame;
    target.side = target_side_;
    {
      std::lock_guard<std::mutex> lock(mapper_mutex_);
      target.pose = mapper_->update(input);
    }
    target.sequence = ++sequence_;
  } else {
    reset_mapper();
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hands_pub_->publish(array);
    if (selected_found) {
      target_pub_->publish(target);
    }
    last_frame_time_ = std::chrono::steady_clock::now();
    missing_device_warned_ = false;
    missing_device_reported_ = false;
  }
}

void LeapTrackerNode::publish_idle() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto now_steady = std::chrono::steady_clock::now();
  const auto without_device = now_steady - start_time_;
  if (!device_available_.load() && without_device >= kDeviceWarnDelay && !missing_device_warned_) {
    RCLCPP_WARN(get_logger(),
                "no Leap device detected for %.1f s; check the USB connection and leapd service",
                std::chrono::duration<double>(without_device).count());
    missing_device_warned_ = true;
  }
  if (!device_available_.load() && without_device >= kDeviceErrorDelay &&
      !missing_device_reported_) {
    RCLCPP_ERROR(get_logger(), "no Leap device detected for %.1f s; tracking stays unavailable",
                 std::chrono::duration<double>(without_device).count());
    missing_device_reported_ = true;
  }
  if (device_available_.load() && now_steady - last_frame_time_ < kFrameTimeout) {
    return;
  }

  HandArray array;
  array.header.stamp = this->now();
  array.header.frame_id = kDeviceFrame;
  array.framerate = 0.0F;
  array.is_tracking = false;
  hands_pub_->publish(array);
}

}  // namespace
}  // namespace leap_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<leap_teleop::LeapTrackerNode>(options);
    node->start();
    try {
      rclcpp::spin(node);
    } catch (...) {
      node->stop();
      throw;
    }
    node->stop();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("leap_tracker"), "leap_tracker could not start: %s",
                 error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
