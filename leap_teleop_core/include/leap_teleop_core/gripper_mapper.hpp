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

// One closure reading: the raw value from the pinch distance and its filtered value.
struct ClosureSample {
  double raw = 0.0;
  double filtered = 0.0;
};

// Pinch distance -> filtered closure. Runs continuously while the hand is visible.
class ClosureFilter {
 public:
  explicit ClosureFilter(const GripperMapperConfig& config);

  [[nodiscard]] ClosureSample update(double pinch_mm, double time_sec);
  // Clears the filter, for a lost hand. The next sample passes unfiltered.
  void reset();
  void set_config(const GripperMapperConfig& config);

 private:
  GripperMapperConfig config_;
  OneEuroFilter filter_;
};

// Decides which closures to publish during one engagement.
class ClosureGate {
 public:
  explicit ClosureGate(const GripperMapperConfig& config);

  // Starts a new engagement: the next closure is always emitted.
  void begin_engagement();
  // Returns the closure to publish, or nothing when it changed less than the deadband from the
  // last one emitted. A raw closure of exactly 0 or 1 snaps the output to that end once the
  // filtered closure is within the deadband of it, so the ends are reachable.
  [[nodiscard]] std::optional<float> update(const ClosureSample& sample);
  void set_config(const GripperMapperConfig& config);

 private:
  GripperMapperConfig config_;
  std::optional<float> last_emitted_;
};

}  // namespace leap_teleop
