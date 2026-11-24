#pragma once

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace leap_teleop {

inline std::string range_text(double min, double max, bool min_inclusive, bool max_inclusive) {
  return std::string(min_inclusive ? "[" : "(") + std::to_string(min) + ", " + std::to_string(max) +
         (max_inclusive ? "]" : ")");
}

inline double declare_bounded_double(rclcpp::Node* node, const std::string& name,
                                     double default_value, double min, double max,
                                     bool min_inclusive = true, bool max_inclusive = true) {
  const double value = node->declare_parameter<double>(name, default_value);
  const bool below = min_inclusive ? value < min : value <= min;
  const bool above = max_inclusive ? value > max : value >= max;
  if (!std::isfinite(value) || below || above) {
    throw std::invalid_argument("parameter '" + name + "' must be in " +
                                range_text(min, max, min_inclusive, max_inclusive) + "; got " +
                                std::to_string(value));
  }
  return value;
}

inline int declare_bounded_int(rclcpp::Node* node, const std::string& name, int default_value,
                               int min, int max) {
  const int value = node->declare_parameter<int>(name, default_value);
  if (value < min || value > max) {
    throw std::invalid_argument("parameter '" + name + "' must be in [" + std::to_string(min) +
                                ", " + std::to_string(max) + "]; got " + std::to_string(value));
  }
  return value;
}

inline std::string declare_choice(rclcpp::Node* node, const std::string& name,
                                  const std::string& default_value,
                                  std::initializer_list<const char*> choices) {
  const std::string value = node->declare_parameter<std::string>(name, default_value);
  if (std::find(choices.begin(), choices.end(), value) == choices.end()) {
    std::string allowed;
    for (const char* choice : choices) {
      if (!allowed.empty()) {
        allowed += ", ";
      }
      allowed += choice;
    }
    throw std::invalid_argument("parameter '" + name + "' must be one of {" + allowed + "}; got '" +
                                value + "'");
  }
  return value;
}

inline bool declare_bool(rclcpp::Node* node, const std::string& name, bool default_value) {
  return node->declare_parameter<bool>(name, default_value);
}

}  // namespace leap_teleop
