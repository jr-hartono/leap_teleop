#pragma once

#include <Eigen/Core>
#include <array>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "leap_teleop_core/delta_builder.hpp"

namespace leap_teleop::viz {

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0F);

// Undoes the supervisor's scaling: the translation divided by translation_scale, the rotation
// angle divided by rotation_scale about the same axis. Scaled angles beyond 180 degrees wrap, so
// they come back as the shorter rotation.
TransformDelta unscale_delta(const TransformDelta& delta, double translation_scale,
                             double rotation_scale);

// An arrow from start to start + vector. Header, ns and id are left to the caller.
visualization_msgs::msg::Marker arrow(const Eigen::Vector3d& start, const Eigen::Vector3d& vector,
                                      double diameter, const std_msgs::msg::ColorRGBA& rgba);

// Three arrows along the pose's x, y and z axes, colored red, green and blue with the given alpha.
std::array<visualization_msgs::msg::Marker, 3> axes(const geometry_msgs::msg::Pose& pose,
                                                    double length, double diameter, float alpha);

// The same arrows, all in one color.
std::array<visualization_msgs::msg::Marker, 3> axes(const geometry_msgs::msg::Pose& pose,
                                                    double length, double diameter,
                                                    const std_msgs::msg::ColorRGBA& rgba);

}  // namespace leap_teleop::viz
