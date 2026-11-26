#pragma once

#include <optional>

#include "leap_teleop_core/filters.hpp"

namespace leap_teleop {

struct GripperMapperConfig {
  double pinch_closed_mm = 20.0;
  double pinch_open_mm = 70.0;
  double filter_min_cutoff_hz = 1.0;
  double filter_beta = 1.0;
  double deadband = 0.02;
};

// True when both distances are positive and closed is below open.
[[nodiscard]] bool valid_pinch_range(double pinch_closed_mm, double pinch_open_mm);

// Absolute closure of a pinch distance: 0 at or beyond pinch_open_mm, 1 at or below
// pinch_closed_mm, linear in between.
[[nodiscard]] double pinch_to_closure(double pinch_mm, double pinch_closed_mm,
                                      double pinch_open_mm);

class GripperMapper {
 public:
  explicit GripperMapper(const GripperMapperConfig& config);

  // Starts a new engagement: clears the filter, and the next sample is always emitted.
  void begin_engagement();
  // Returns the closure to publish, or nothing when it changed less than the deadband. A raw
  // closure of exactly 0 or 1 snaps the output to that end once the filter is within the
  // deadband of it, so the ends are reachable.
  [[nodiscard]] std::optional<float> update(double pinch_mm, double time_sec);
  void set_config(const GripperMapperConfig& config);

 private:
  GripperMapperConfig config_;
  OneEuroFilter filter_;
  std::optional<float> last_emitted_;
};

}  // namespace leap_teleop
