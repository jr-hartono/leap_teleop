#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <chrono>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <leap_teleop_viz/leap_teleop_viz_parameters.hpp>
#include <memory>
#include <optional>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "leap_teleop_core/delta_builder.hpp"
#include "leap_teleop_core/frames.hpp"
#include "leap_teleop_msgs/msg/hand.hpp"
#include "leap_teleop_msgs/msg/hand_array.hpp"
#include "leap_teleop_msgs/msg/teleop_delta.hpp"
#include "leap_teleop_msgs/msg/teleop_hand.hpp"
#include "leap_teleop_viz/markers.hpp"

namespace leap_teleop {

namespace {

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double kRenderRate = 30.0;
// Age after which a pose_delta or command no longer counts, s.
constexpr double kInputTimeout = 0.2;
// Lifetime of markers that are only refreshed while their input is fresh, s.
constexpr double kMarkerLifetime = 0.2;
// Arrows shorter than this are not drawn, m.
constexpr double kMinArrowLength = 1e-4;

constexpr char kLatchedToolNs[] = "latched_tool";
constexpr char kTargetNs[] = "target";
constexpr char kUnscaledTargetNs[] = "target_before_scaling";
constexpr char kTwistNs[] = "twist";
constexpr char kRawPalmNs[] = "raw_palm";
constexpr char kFilteredPalmNs[] = "filtered_palm";

// Frames of the driver's messages.
constexpr char kOperatorFrame[] = "leap_operator_frame";
constexpr char kDeviceFrame[] = "leap_device_frame";

using leap_teleop_msgs::msg::Hand;

std_msgs::msg::ColorRGBA hand_color(const std::string& side, float alpha) {
  return side == Hand::LEFT ? viz::color(1.0F, 0.5F, 0.0F, alpha)
                            : viz::color(0.0F, 0.7F, 1.0F, alpha);
}

Marker sphere(const geometry_msgs::msg::Point& position, double diameter,
              const std_msgs::msg::ColorRGBA& rgba) {
  Marker marker;
  marker.type = Marker::SPHERE;
  marker.pose.position = position;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = diameter;
  marker.color = rgba;
  return marker;
}

geometry_msgs::msg::Pose pose_from_transform(const geometry_msgs::msg::Transform& transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;
  return pose;
}

}  // namespace

class VizNode : public rclcpp::Node {
 public:
  VizNode() : rclcpp::Node("leap_teleop_viz") {
    param_listener_ = std::make_shared<leap_teleop_viz::ParamListener>(this);
    params_ = param_listener_->get_params();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    delta_subscription_ = create_subscription<leap_teleop_msgs::msg::TeleopDelta>(
        "pose_delta/" + params_.side, rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const leap_teleop_msgs::msg::TeleopDelta::SharedPtr message) {
          on_delta(*message);
        });
    command_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "command/" + params_.side, rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
          command_ = *message;
          command_time_ = now().seconds();
        });
    marker_publisher_ = create_publisher<MarkerArray>("markers", rclcpp::QoS(rclcpp::KeepLast(10)));

    hands_subscription_ = create_subscription<leap_teleop_msgs::msg::HandArray>(
        // Best effort, matching the driver; a reliable subscription would receive nothing.
        "hands", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](const leap_teleop_msgs::msg::HandArray::SharedPtr message) {
          hands_ = *message;
          hands_time_ = now().seconds();
        });
    for (std::size_t index = 0; index < kSides.size(); ++index) {
      hand_pose_subscriptions_[index] = create_subscription<leap_teleop_msgs::msg::TeleopHand>(
          "teleop_hand/" + kSides[index], rclcpp::QoS(rclcpp::KeepLast(1)),
          [this, index](const leap_teleop_msgs::msg::TeleopHand::SharedPtr message) {
            hand_poses_[index] = *message;
            hand_pose_times_[index] = now().seconds();
          });
    }
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    driver_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, params_.driver_node);
    driver_name_ = get_node_topics_interface()->resolve_topic_name(params_.driver_node, true);

    // The scales follow the mapper's parameters: read once, then from parameter events.
    mapper_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, params_.mapper_node);
    mapper_name_ = get_node_topics_interface()->resolve_topic_name(params_.mapper_node, true);
    mapper_events_ = mapper_client_->on_parameter_event(
        [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
          if (event->node != mapper_name_) {
            return;
          }
          for (const auto& parameter : event->changed_parameters) {
            set_mapper_parameter(parameter.name, rclcpp::Parameter::from_parameter_msg(parameter));
          }
        });
    parameter_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      fetch_mapper();
      fetch_driver();
    });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / kRenderRate));
    render_timer_ = create_wall_timer(period, [this]() { render(); });
  }

 private:
  void set_mapper_parameter(const std::string& name, const rclcpp::Parameter& parameter) {
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return;
    }
    if (name == "yaw") {
      if (!yaw_) {
        yaw_ = parameter.as_double();
        publish_static_transforms();
      }
      return;
    }
    std::optional<double>* scale = nullptr;
    if (name == "translation_scale") {
      scale = &translation_scale_;
    } else if (name == "rotation_scale") {
      scale = &rotation_scale_;
    } else {
      return;
    }
    if (*scale != parameter.as_double()) {
      RCLCPP_INFO(get_logger(), "mapper %s: %g", name.c_str(), parameter.as_double());
    }
    *scale = parameter.as_double();
  }

  // Reads the mapper's parameters until all are known. yaw is read-only; the scales
  // then follow parameter events.
  void fetch_mapper() {
    if ((translation_scale_ && rotation_scale_ && yaw_) || fetching_mapper_) {
      return;
    }
    if (!mapper_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                           "waiting for %s; no target before scaling or hand view until then",
                           mapper_name_.c_str());
      return;
    }
    fetching_mapper_ = true;
    mapper_client_->get_parameters(
        {"translation_scale", "rotation_scale", "yaw"},
        [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
          fetching_mapper_ = false;
          try {
            for (const auto& parameter : future.get()) {
              set_mapper_parameter(parameter.get_name(), parameter);
            }
          } catch (const std::exception& error) {
            RCLCPP_WARN(get_logger(), "cannot read the mapper's parameters: %s", error.what());
          }
        });
  }

  // Reads the driver's read-only mount_mode once.
  void fetch_driver() {
    if (mount_mode_ || fetching_driver_) {
      return;
    }
    if (!driver_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                           "waiting for %s; no hand view until then", driver_name_.c_str());
      return;
    }
    fetching_driver_ = true;
    driver_client_->get_parameters(
        {"mount_mode"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
          fetching_driver_ = false;
          try {
            const auto parameters = future.get();
            if (parameters.empty() ||
                parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
              return;
            }
            mount_mode_ = mount_mode_from_string(parameters[0].as_string());
            publish_static_transforms();
          } catch (const std::exception& error) {
            RCLCPP_WARN(get_logger(), "cannot read the driver's mount_mode: %s", error.what());
          }
        });
  }

  void publish_static_transforms() {
    if (!yaw_ || !mount_mode_) {
      return;
    }
    const auto& offset = params_.leap_operator_frame_offset;
    geometry_msgs::msg::TransformStamped base_to_operator;
    base_to_operator.header.stamp = now();
    base_to_operator.header.frame_id = params_.base_frame;
    base_to_operator.child_frame_id = kOperatorFrame;
    base_to_operator.transform =
        viz::base_to_operator(*yaw_, Eigen::Vector3d(offset[0], offset[1], offset[2]));
    geometry_msgs::msg::TransformStamped operator_to_device = base_to_operator;
    operator_to_device.header.frame_id = kOperatorFrame;
    operator_to_device.child_frame_id = kDeviceFrame;
    operator_to_device.transform = viz::operator_to_device(*mount_mode_);
    static_broadcaster_->sendTransform({base_to_operator, operator_to_device});
    RCLCPP_INFO(get_logger(), "hand view: yaw %g, mount_mode %s, at (%g, %g, %g) in %s", *yaw_,
                *mount_mode_ == MountMode::kDesktop ? "desktop" : "head_mounted", offset[0],
                offset[1], offset[2], params_.base_frame.c_str());
  }

  void on_delta(const leap_teleop_msgs::msg::TeleopDelta& message) {
    const bool rising = message.engaged && !(delta_ && delta_->engaged);
    delta_ = message;
    delta_time_ = now().seconds();
    if (message.header.frame_id != params_.base_frame) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "pose_delta is in '%s' but base_frame is '%s'",
                           message.header.frame_id.c_str(), params_.base_frame.c_str());
    }
    if (!rising) {
      return;
    }
    // Latch like the bridge does, on the same press.
    latched_ = false;
    try {
      const auto transform =
          tf_buffer_->lookupTransform(params_.base_frame, params_.tool_frame, tf2::TimePointZero);
      latched_tool_ = pose_from_transform(transform.transform);
      latched_ = true;
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN(get_logger(), "cannot latch tool pose: %s", error.what());
    }
  }

  Marker stamped(Marker marker, const std::string& frame, const char* ns, int id,
                 double lifetime) const {
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = id;
    marker.lifetime = rclcpp::Duration::from_seconds(lifetime);
    return marker;
  }

  template <typename Markers>
  void add(MarkerArray& array, const Markers& markers, const std::string& frame, const char* ns,
           double lifetime = 0.0) const {
    int id = 0;
    for (const auto& marker : markers) {
      array.markers.push_back(stamped(marker, frame, ns, id++, lifetime));
    }
  }

  void delete_axes(MarkerArray& array, const char* ns) const {
    for (int id = 0; id < 3; ++id) {
      Marker marker;
      marker.header.frame_id = params_.base_frame;
      marker.ns = ns;
      marker.id = id;
      marker.action = Marker::DELETE;
      array.markers.push_back(marker);
    }
  }

  void render_command_poses(MarkerArray& array, double time) {
    const bool engaged =
        delta_ && delta_->engaged && time - delta_time_ <= kInputTimeout && latched_;
    if (!engaged) {
      if (poses_shown_) {
        delete_axes(array, kLatchedToolNs);
        delete_axes(array, kTargetNs);
        delete_axes(array, kUnscaledTargetNs);
        poses_shown_ = false;
      }
      return;
    }
    poses_shown_ = true;
    const std::string& frame = params_.base_frame;
    const TransformDelta delta = delta_from_transform_msg(delta_->delta);
    // The lifetime also clears them if the delete below is missed.
    add(array, viz::axes(latched_tool_, 0.1, 0.006, viz::color(0.6F, 0.6F, 0.6F)), frame,
        kLatchedToolNs, kMarkerLifetime);
    // The bridge's target: the same composition, from the same inputs.
    add(array, viz::axes(apply_delta(latched_tool_, delta), 0.1, 0.008, 1.0F), frame, kTargetNs,
        kMarkerLifetime);
    if (translation_scale_ && rotation_scale_) {
      const TransformDelta unscaled =
          viz::unscale_delta(delta, *translation_scale_, *rotation_scale_);
      add(array, viz::axes(apply_delta(latched_tool_, unscaled), 0.08, 0.005, 0.35F), frame,
          kUnscaledTargetNs, kMarkerLifetime);
    } else {
      delete_axes(array, kUnscaledTargetNs);
    }
  }

  void render_twist(MarkerArray& array, double time) {
    if (!command_ || time - command_time_ > kInputTimeout) {
      return;
    }
    const std::string& frame = params_.base_frame;
    Eigen::Vector3d tool_position;
    Eigen::Quaterniond to_base;
    try {
      const auto tool = tf_buffer_->lookupTransform(frame, params_.tool_frame, tf2::TimePointZero);
      tool_position = vector_from_msg(tool.transform.translation);
      const auto rotation =
          tf_buffer_->lookupTransform(frame, command_->header.frame_id, tf2::TimePointZero);
      to_base = quaternion_from_msg(rotation.transform.rotation);
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cannot draw the twist: %s",
                           error.what());
      return;
    }
    const Eigen::Vector3d linear =
        params_.linear_arrow_scale * (to_base * vector_from_msg(command_->twist.linear));
    const Eigen::Vector3d angular =
        params_.angular_arrow_scale * (to_base * vector_from_msg(command_->twist.angular));
    const std::array<std::pair<Eigen::Vector3d, std_msgs::msg::ColorRGBA>, 2> arrows = {{
        {linear, viz::color(1.0F, 0.85F, 0.0F)},
        {angular, viz::color(0.9F, 0.0F, 0.9F)},
    }};
    for (int id = 0; id < 2; ++id) {
      if (arrows[id].first.norm() < kMinArrowLength) {
        continue;
      }
      array.markers.push_back(
          stamped(viz::arrow(tool_position, arrows[id].first, 0.01, arrows[id].second), frame,
                  kTwistNs, id, kMarkerLifetime));
    }
  }

  void render_hands(MarkerArray& array, double time) {
    if (hands_ && time - hands_time_ <= kInputTimeout) {
      std::vector<Marker> raw;
      for (const auto& hand : hands_->hands) {
        const auto palm_axes = viz::axes(hand.pose, 0.05, 0.004, 0.35F);
        raw.insert(raw.end(), palm_axes.begin(), palm_axes.end());
        raw.push_back(sphere(hand.pose.position, 0.03, hand_color(hand.side, 0.35F)));
      }
      add(array, raw, kDeviceFrame, kRawPalmNs, kMarkerLifetime);
    }
    if (!mount_mode_) {
      return;
    }
    for (std::size_t index = 0; index < kSides.size(); ++index) {
      if (!hand_poses_[index] || time - hand_pose_times_[index] > kInputTimeout) {
        continue;
      }
      const geometry_msgs::msg::Pose& palm = hand_poses_[index]->pose;
      std::vector<Marker> filtered;
      const auto palm_axes = viz::axes(palm, 0.07, 0.006, 1.0F);
      filtered.assign(palm_axes.begin(), palm_axes.end());
      filtered.push_back(sphere(palm.position, 0.03, hand_color(kSides[index], 1.0F)));
      Marker label;
      label.type = Marker::TEXT_VIEW_FACING;
      label.pose.position = palm.position;
      label.pose.position.z += 0.06;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.03;
      label.color = viz::color(1.0F, 1.0F, 1.0F);
      label.text = kSides[index];
      filtered.push_back(label);
      // Each side keeps its own ids, so one hand leaving does not touch the other's markers.
      const int first_id = static_cast<int>(index) * 10;
      for (std::size_t offset = 0; offset < filtered.size(); ++offset) {
        array.markers.push_back(stamped(filtered[offset], kOperatorFrame, kFilteredPalmNs,
                                        first_id + static_cast<int>(offset), kMarkerLifetime));
      }
    }
  }

  void render() {
    params_ = param_listener_->get_params();
    const double time = now().seconds();
    MarkerArray array;
    render_command_poses(array, time);
    render_twist(array, time);
    render_hands(array, time);
    if (!array.markers.empty()) {
      marker_publisher_->publish(array);
    }
  }

  std::shared_ptr<leap_teleop_viz::ParamListener> param_listener_;
  leap_teleop_viz::Params params_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::optional<leap_teleop_msgs::msg::TeleopDelta> delta_;
  double delta_time_ = 0.0;
  geometry_msgs::msg::Pose latched_tool_;
  bool latched_ = false;
  bool poses_shown_ = false;
  std::optional<geometry_msgs::msg::TwistStamped> command_;
  double command_time_ = 0.0;

  inline static const std::array<std::string, 2> kSides = {Hand::LEFT, Hand::RIGHT};
  std::optional<leap_teleop_msgs::msg::HandArray> hands_;
  double hands_time_ = 0.0;
  std::array<std::optional<leap_teleop_msgs::msg::TeleopHand>, 2> hand_poses_;
  std::array<double, 2> hand_pose_times_{};

  std::shared_ptr<rclcpp::AsyncParametersClient> mapper_client_;
  std::string mapper_name_;
  std::optional<double> translation_scale_;
  std::optional<double> rotation_scale_;
  std::optional<double> yaw_;
  bool fetching_mapper_ = false;
  std::shared_ptr<rclcpp::AsyncParametersClient> driver_client_;
  std::string driver_name_;
  std::optional<MountMode> mount_mode_;
  bool fetching_driver_ = false;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  rclcpp::Subscription<leap_teleop_msgs::msg::TeleopDelta>::SharedPtr delta_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr mapper_events_;
  rclcpp::Subscription<leap_teleop_msgs::msg::HandArray>::SharedPtr hands_subscription_;
  std::array<rclcpp::Subscription<leap_teleop_msgs::msg::TeleopHand>::SharedPtr, 2>
      hand_pose_subscriptions_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr parameter_timer_;
  rclcpp::TimerBase::SharedPtr render_timer_;
};

}  // namespace leap_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<leap_teleop::VizNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("leap_teleop_viz"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
