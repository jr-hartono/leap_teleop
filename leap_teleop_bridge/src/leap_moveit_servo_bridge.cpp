#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <leap_teleop_bridge/leap_moveit_servo_bridge_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

#include "leap_teleop_core/delta_builder.hpp"
#include "leap_teleop_core/frames.hpp"
#include "leap_teleop_core/timing.hpp"
#include "leap_teleop_msgs/msg/teleop_delta.hpp"

namespace leap_teleop {

namespace {

double stamp_seconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

geometry_msgs::msg::Pose pose_from_transform(const geometry_msgs::msg::Transform& transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;
  return pose;
}

Eigen::Vector3d clamp_norm(const Eigen::Vector3d& value, double maximum) {
  const double norm = value.norm();
  if (norm > maximum && norm > 0.0) {
    return value * (maximum / norm);
  }
  return value;
}

}  // namespace

class MoveItServoBridge : public rclcpp::Node {
 public:
  MoveItServoBridge() : rclcpp::Node("leap_moveit_servo_bridge") {
    param_listener_ = std::make_shared<leap_moveit_servo_bridge::ParamListener>(this);
    const auto params = param_listener_->get_params();
    tool_frame_ = params.tool_frame;
    planning_frame_ = params.planning_frame;
    const std::string topic = params.cartesian_command_topic;
    kp_linear_ = params.kp_linear;
    kp_angular_ = params.kp_angular;
    max_linear_velocity_ = params.max_linear_velocity;
    max_angular_velocity_ = params.max_angular_velocity;
    publish_period_ = params.publish_period;
    dry_run_ = params.dry_run;

    // The single-threaded executor cannot run this callback and on_tick() at the same time.
    param_listener_->setUserCallback([this](const leap_moveit_servo_bridge::Params& updated) {
      const auto log_change = [this](const char* name, double before, double after) {
        if (before != after) {
          RCLCPP_INFO(get_logger(), "parameter '%s' changed: %g -> %g", name, before, after);
        }
      };
      log_change("kp_linear", kp_linear_, updated.kp_linear);
      log_change("kp_angular", kp_angular_, updated.kp_angular);
      log_change("max_linear_velocity", max_linear_velocity_, updated.max_linear_velocity);
      log_change("max_angular_velocity", max_angular_velocity_, updated.max_angular_velocity);

      kp_linear_ = updated.kp_linear;
      kp_angular_ = updated.kp_angular;
      max_linear_velocity_ = updated.max_linear_velocity;
      max_angular_velocity_ = updated.max_angular_velocity;
    });

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    target_subscription_ = create_subscription<leap_teleop_msgs::msg::TeleopDelta>(
        "pose_delta", rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const leap_teleop_msgs::msg::TeleopDelta::SharedPtr message) {
          on_target(*message);
        });
    command_publisher_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(topic, rclcpp::QoS(rclcpp::KeepLast(1)));
    // Every computed command, also in dry run, for visualization.
    computed_command_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "command", rclcpp::QoS(rclcpp::KeepLast(1)));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_period_));
    timer_ = create_wall_timer(period, [this]() { on_tick(); });

    if (dry_run_) {
      RCLCPP_WARN(get_logger(), "dry_run is enabled; commands will not be sent to Servo");
    }
  }

 private:
  void on_target(const leap_teleop_msgs::msg::TeleopDelta& message) {
    const bool rising = message.engaged && !last_engaged_;
    last_engaged_ = message.engaged;
    target_ = message;
    have_target_ = true;

    if (rising) {
      latched_ = false;
      try {
        const auto transform =
            tf_buffer_->lookupTransform(message.header.frame_id, tool_frame_, tf2::TimePointZero);
        latched_tool_ = pose_from_transform(transform.transform);
        latched_frame_ = message.header.frame_id;
        latched_ = true;
      } catch (const tf2::TransformException& error) {
        RCLCPP_ERROR(get_logger(), "cannot latch tool pose: %s", error.what());
      }
    }
  }

  void report(const std::string& reason) {
    if (reason == last_reason_) {
      return;
    }
    if (reason.empty()) {
      RCLCPP_INFO(get_logger(), "commands resumed");
    } else {
      RCLCPP_WARN(get_logger(), "not publishing: %s", reason.c_str());
    }
    last_reason_ = reason;
  }

  void on_tick() {
    std::string reason;
    geometry_msgs::msg::TwistStamped command;

    if (!have_target_) {
      reason = "no_target";
    } else if (!target_.engaged) {
      reason = "not_engaged";
    } else if (now().seconds() - stamp_seconds(target_.header.stamp) > timing::kCommandTimeout) {
      reason = "stale_target";
    } else if (!latched_) {
      reason = "no_latch";
    } else {
      try {
        const auto current_transform =
            tf_buffer_->lookupTransform(latched_frame_, tool_frame_, tf2::TimePointZero);
        const geometry_msgs::msg::Pose current = pose_from_transform(current_transform.transform);

        const TransformDelta delta = delta_from_transform_msg(target_.delta);
        const geometry_msgs::msg::Pose desired = apply_delta(latched_tool_, delta);

        const Eigen::Vector3d position_error =
            point_from_msg(desired.position) - point_from_msg(current.position);
        const Eigen::Quaterniond orientation_error = world_delta(
            quaternion_from_msg(desired.orientation), quaternion_from_msg(current.orientation));
        const Eigen::AngleAxisd angle_axis(orientation_error);

        Eigen::Vector3d linear = clamp_norm(kp_linear_ * position_error, max_linear_velocity_);
        Eigen::Vector3d angular =
            clamp_norm(kp_angular_ * angle_axis.angle() * angle_axis.axis(), max_angular_velocity_);

        if (planning_frame_ != latched_frame_) {
          const auto rotation =
              tf_buffer_->lookupTransform(planning_frame_, latched_frame_, tf2::TimePointZero);
          const Eigen::Quaterniond to_planning = quaternion_from_msg(rotation.transform.rotation);
          linear = to_planning * linear;
          angular = to_planning * angular;
        }

        command.header.stamp = now();
        command.header.frame_id = planning_frame_;
        const Eigen::Vector3d linear_msg = linear;
        command.twist.linear.x = linear_msg.x();
        command.twist.linear.y = linear_msg.y();
        command.twist.linear.z = linear_msg.z();
        command.twist.angular.x = angular.x();
        command.twist.angular.y = angular.y();
        command.twist.angular.z = angular.z();
      } catch (const tf2::TransformException& error) {
        reason = std::string("tf_failure: ") + error.what();
      }
    }

    if (reason.empty()) {
      computed_command_publisher_->publish(command);
    }

    if (!dry_run_ && reason.empty() && command_publisher_->get_subscription_count() == 0) {
      reason = "no_servo_subscriber";
    }
    report(reason);
    if (!reason.empty()) {
      return;
    }

    if (dry_run_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "dry_run twist: linear=(%.3f, %.3f, %.3f) angular=(%.3f, %.3f, %.3f)",
                           command.twist.linear.x, command.twist.linear.y, command.twist.linear.z,
                           command.twist.angular.x, command.twist.angular.y,
                           command.twist.angular.z);
      return;
    }
    command_publisher_->publish(command);
  }

  std::shared_ptr<leap_moveit_servo_bridge::ParamListener> param_listener_;
  std::string tool_frame_;
  std::string planning_frame_;
  std::string latched_frame_;
  double kp_linear_ = 4.0;
  double kp_angular_ = 4.0;
  double max_linear_velocity_ = 0.2;
  double max_angular_velocity_ = 0.5;
  double publish_period_ = 0.01;
  bool dry_run_ = true;
  bool have_target_ = false;
  bool last_engaged_ = false;
  bool latched_ = false;
  std::string last_reason_ = "no_target";

  leap_teleop_msgs::msg::TeleopDelta target_;
  geometry_msgs::msg::Pose latched_tool_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<leap_teleop_msgs::msg::TeleopDelta>::SharedPtr target_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr computed_command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace leap_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<leap_teleop::MoveItServoBridge>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("leap_moveit_servo_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
