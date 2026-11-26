#include "leap_teleop_core/leap_driver.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <leap_teleop_core/leap_driver_parameters.hpp>
#include <leap_teleop_msgs/msg/hand.hpp>
#include <leap_teleop_msgs/msg/tracking_frame.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>

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
using leap_teleop_msgs::msg::TrackingFrame;

constexpr std::uint32_t kPollTimeoutMs = 10;
constexpr auto kIdlePeriod = std::chrono::milliseconds(100);
constexpr auto kFrameTimeout = std::chrono::milliseconds(250);
constexpr auto kDeviceWarnDelay = std::chrono::seconds(2);
constexpr char kDeviceFrame[] = "leap_device_frame";

// Product names for the devices whose LeapC names are codenames, such as "Peripheral" for the
// first Leap Motion Controller.
const char* device_type_name(eLeapDevicePID pid) {
  switch (pid) {
    case eLeapDevicePID_Peripheral:
      return "Leap Motion Controller";
    case eLeapDevicePID_LMC2:
      return "Leap Motion Controller 2";
    case eLeapDevicePID_SIR170:
      return "Stereo IR 170";
    case eLeapDevicePID_3Di:
      return "3Di";
    default:
      return LeapDevicePIDToString(pid);
  }
}

const char* tracking_mode_name(eLeapTrackingMode mode) {
  switch (mode) {
    case eLeapTrackingMode_Desktop:
      return "desktop";
    case eLeapTrackingMode_HMD:
      return "head_mounted";
    case eLeapTrackingMode_ScreenTop:
      return "screen_top";
    default:
      return "unknown";
  }
}

// The mode value for TrackingFrame, unknown for a mode other than desktop or head-mounted.
std::string reported_mode_name(eLeapTrackingMode mode) {
  switch (mode) {
    case eLeapTrackingMode_Desktop:
      return TrackingFrame::DESKTOP;
    case eLeapTrackingMode_HMD:
      return TrackingFrame::HEAD_MOUNTED;
    default:
      return TrackingFrame::UNKNOWN;
  }
}

class LeapDriverNode : public rclcpp::Node {
 public:
  explicit LeapDriverNode(const rclcpp::NodeOptions& options);
  ~LeapDriverNode() override;

  void start();
  void stop();

 private:
  void mark_device_lost();
  void poll_loop();
  void handle_event(const LEAP_CONNECTION_MESSAGE& message);
  void publish_tracking(const LEAP_TRACKING_EVENT* frame);
  void publish_idle();
  void on_device_attached(const LEAP_DEVICE_REF& device);
  void on_tracking_mode(eLeapTrackingMode current);
  std::string reported_mode_name_now() const;
  std::string describe_device(const LEAP_DEVICE_REF& device) const;

  rclcpp::Publisher<TrackingFrame>::SharedPtr hands_pub_;
  rclcpp::TimerBase::SharedPtr idle_timer_;
  std::shared_ptr<leap_driver::ParamListener> param_listener_;
  std::unique_ptr<LeapConnection> connection_;
  std::unique_ptr<std::thread> poll_thread_;

  std::atomic<bool> running_{false};
  std::atomic<bool> device_available_{false};
  // The mode requested from the service.
  eLeapTrackingMode requested_mode_ = eLeapTrackingMode_Desktop;
  // The mode the service reports, as an eLeapTrackingMode, or -1 while unknown. Written on the
  // poll thread, read by the idle timer.
  std::atomic<int> reported_mode_{-1};
  // Used only on the poll thread.
  bool unsupported_mode_warned_ = false;
  // Used only on the poll thread.
  bool mode_requested_ = false;
  std::unordered_map<uint32_t, std::string> device_names_;

  std::mutex state_mutex_;
  const std::chrono::steady_clock::time_point start_time_ = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_frame_time_{};
  // When the device was last lost; node start until a device first shows up.
  std::chrono::steady_clock::time_point device_lost_time_ = start_time_;
  bool missing_device_warned_ = false;
};

LeapDriverNode::LeapDriverNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("leap_driver", options) {
  param_listener_ = std::make_shared<leap_driver::ParamListener>(this);
  const auto params = param_listener_->get_params();
  const TrackingMode tracking_mode = tracking_mode_from_string(params.tracking_mode);
  requested_mode_ = (tracking_mode == TrackingMode::kHeadMounted) ? eLeapTrackingMode_HMD
                                                                  : eLeapTrackingMode_Desktop;

  hands_pub_ = create_publisher<TrackingFrame>(
      "hands", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile());
  idle_timer_ = create_wall_timer(kIdlePeriod, [this]() { publish_idle(); });

  connection_ = std::make_unique<LeapConnection>();
}

void LeapDriverNode::mark_device_lost() {
  device_available_.store(false);
  reported_mode_.store(-1);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    device_lost_time_ = std::chrono::steady_clock::now();
    missing_device_warned_ = false;
  }
}

LeapDriverNode::~LeapDriverNode() { stop(); }

void LeapDriverNode::start() {
  if (poll_thread_) {
    return;
  }
  running_.store(true, std::memory_order_relaxed);
  poll_thread_ = std::make_unique<std::thread>(&LeapDriverNode::poll_loop, this);
}

void LeapDriverNode::stop() {
  running_.store(false, std::memory_order_relaxed);
  if (poll_thread_ && poll_thread_->joinable()) {
    poll_thread_->join();
  }
}

std::string LeapDriverNode::describe_device(const LEAP_DEVICE_REF& device) const {
  LEAP_DEVICE handle = nullptr;
  if (LEAP_FAILED(LeapOpenDevice(device, &handle))) {
    return "unknown device";
  }
  LEAP_DEVICE_INFO info{};
  info.size = sizeof(info);
  info.serial = nullptr;
  // The first call reports the serial length, the second fills in the serial.
  eLeapRS result = LeapGetDeviceInfo(handle, &info);
  std::string serial;
  if (result == eLeapRS_InsufficientBuffer ||
      (result == eLeapRS_Success && info.serial_length > 0)) {
    serial.resize(info.serial_length);
    info.serial = serial.data();
    result = LeapGetDeviceInfo(handle, &info);
    serial.resize(std::strlen(serial.c_str()));
  }
  LeapCloseDevice(handle);
  if (LEAP_FAILED(result)) {
    return "unknown device";
  }
  return std::string(device_type_name(info.pid)) + ", serial " + serial;
}

// The service keeps one tracking mode for all applications and keeps it after they exit. On each
// attach the driver asks for the current mode and requests its own once if it differs; it does
// not fight an application that changes the mode afterwards, it only warns.
void LeapDriverNode::on_device_attached(const LEAP_DEVICE_REF& device) {
  const std::string name = describe_device(device);
  device_names_[device.id] = name;
  RCLCPP_INFO(this->get_logger(), "device attached: %s", name.c_str());
  device_available_.store(true);
  mode_requested_ = false;
  const eLeapRS result = LeapGetTrackingMode(connection_->handle());
  if (LEAP_FAILED(result)) {
    RCLCPP_WARN(this->get_logger(), "LeapGetTrackingMode failed with code %d",
                static_cast<int>(result));
  }
}

void LeapDriverNode::on_tracking_mode(eLeapTrackingMode current) {
  if (reported_mode_name(current) == TrackingFrame::UNKNOWN) {
    reported_mode_.store(-1);
    if (!unsupported_mode_warned_) {
      unsupported_mode_warned_ = true;
      RCLCPP_WARN(
          this->get_logger(),
          "ultraleap-hand-tracking-service is in tracking_mode %s, which is neither desktop nor "
          "head_mounted; tracking frames carry tracking_mode unknown",
          tracking_mode_name(current));
    }
  } else {
    reported_mode_.store(static_cast<int>(current));
  }
  if (current == requested_mode_) {
    RCLCPP_INFO(this->get_logger(), "ultraleap-hand-tracking-service is in tracking_mode %s",
                tracking_mode_name(current));
    return;
  }
  if (!mode_requested_) {
    mode_requested_ = true;
    RCLCPP_INFO(this->get_logger(),
                "ultraleap-hand-tracking-service is in tracking_mode %s, requesting %s",
                tracking_mode_name(current), tracking_mode_name(requested_mode_));
    const eLeapRS result = LeapSetTrackingMode(connection_->handle(), requested_mode_);
    if (LEAP_FAILED(result)) {
      RCLCPP_WARN(this->get_logger(), "LeapSetTrackingMode(%s) failed with code %d",
                  tracking_mode_name(requested_mode_), static_cast<int>(result));
    }
    return;
  }
  RCLCPP_WARN(this->get_logger(),
              "ultraleap-hand-tracking-service is in tracking_mode %s, expected %s; another "
              "application may have "
              "changed it",
              tracking_mode_name(current), tracking_mode_name(requested_mode_));
}

std::string LeapDriverNode::reported_mode_name_now() const {
  const int mode = reported_mode_.load();
  return mode < 0 ? std::string(TrackingFrame::UNKNOWN)
                  : reported_mode_name(static_cast<eLeapTrackingMode>(mode));
}

void LeapDriverNode::poll_loop() {
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

void LeapDriverNode::handle_event(const LEAP_CONNECTION_MESSAGE& message) {
  switch (message.type) {
    case eLeapEventType_Connection:
      // The service sends a Device event for each attached device after connecting.
      RCLCPP_INFO(this->get_logger(), "ultraleap-hand-tracking-service connected");
      break;

    case eLeapEventType_ConnectionLost:
      RCLCPP_WARN(this->get_logger(), "ultraleap-hand-tracking-service connection lost");
      mark_device_lost();
      break;

    case eLeapEventType_Device:
      on_device_attached(message.device_event->device);
      break;

    case eLeapEventType_DeviceLost: {
      const auto it = device_names_.find(message.device_event->device.id);
      RCLCPP_WARN(this->get_logger(), "device detached: %s",
                  it != device_names_.end() ? it->second.c_str() : "unknown device");
      if (it != device_names_.end()) {
        device_names_.erase(it);
      }
      mark_device_lost();
    } break;

    case eLeapEventType_DeviceFailure:
      RCLCPP_WARN(this->get_logger(), "device failure reported");
      mark_device_lost();
      break;

    case eLeapEventType_TrackingMode:
      on_tracking_mode(message.tracking_mode_event->current_tracking_mode);
      break;

    case eLeapEventType_Tracking:
      publish_tracking(message.tracking_event);
      break;

    default:
      break;
  }
}

void LeapDriverNode::publish_tracking(const LEAP_TRACKING_EVENT* frame) {
  if (frame == nullptr) {
    return;
  }

  device_available_.store(true);
  const auto stamp = now();
  TrackingFrame tracking_frame;
  tracking_frame.header.stamp = stamp;
  tracking_frame.header.frame_id = kDeviceFrame;
  tracking_frame.tracking_frame_id = static_cast<std::uint64_t>(frame->tracking_frame_id);
  tracking_frame.device_timestamp_us = static_cast<std::uint64_t>(frame->info.timestamp);
  tracking_frame.framerate = frame->framerate;
  tracking_frame.is_tracking = true;
  tracking_frame.tracking_mode = reported_mode_name_now();

  if (frame->pHands != nullptr) {
    tracking_frame.hands.reserve(frame->nHands);
    for (std::uint32_t index = 0; index < frame->nHands; ++index) {
      const LEAP_HAND& hand = frame->pHands[index];
      Hand reported;
      reported.side = (hand.type == eLeapHandType_Left) ? Hand::LEFT : Hand::RIGHT;
      reported.visible_time_us = hand.visible_time;

      reported.pose.position.x = hand.palm.position.x * 0.001F;
      reported.pose.position.y = hand.palm.position.y * 0.001F;
      reported.pose.position.z = hand.palm.position.z * 0.001F;
      reported.pose.orientation.x = hand.palm.orientation.x;
      reported.pose.orientation.y = hand.palm.orientation.y;
      reported.pose.orientation.z = hand.palm.orientation.z;
      reported.pose.orientation.w = hand.palm.orientation.w;

      reported.pinch_strength = clamp_unit_strength(hand.pinch_strength);
      reported.pinch_distance = non_negative_distance(hand.pinch_distance);
      reported.grab_strength = clamp_unit_strength(hand.grab_strength);

      tracking_frame.hands.push_back(reported);
    }
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hands_pub_->publish(tracking_frame);
    last_frame_time_ = std::chrono::steady_clock::now();
    missing_device_warned_ = false;
  }
}

void LeapDriverNode::publish_idle() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto now_steady = std::chrono::steady_clock::now();
  const auto without_device = now_steady - device_lost_time_;
  if (!device_available_.load() && without_device >= kDeviceWarnDelay && !missing_device_warned_) {
    RCLCPP_WARN(get_logger(), "no Leap device detected; check the USB connection");
    missing_device_warned_ = true;
  }
  if (device_available_.load() && now_steady - last_frame_time_ < kFrameTimeout) {
    return;
  }

  TrackingFrame tracking_frame;
  tracking_frame.header.stamp = this->now();
  tracking_frame.header.frame_id = kDeviceFrame;
  tracking_frame.framerate = 0.0F;
  tracking_frame.is_tracking = false;
  tracking_frame.tracking_mode = reported_mode_name_now();
  hands_pub_->publish(tracking_frame);
}

}  // namespace
}  // namespace leap_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<leap_teleop::LeapDriverNode>(options);
    node->start();
    try {
      rclcpp::spin(node);
    } catch (...) {
      node->stop();
      throw;
    }
    node->stop();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("leap_driver"), "leap_driver could not start: %s",
                 error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
