#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace hardware
{
// Board-time version of tools/opencr_air.py PositionRate. Feed once per received
// STATE, not once per controller update: several STATEs can arrive in one read.
// No allocations. At 1 ms minimum spacing a 60 ms window needs at most 61 slots.
class PositionRate
{
public:
  double feed(uint32_t ms, double position)
  {
    if (fault_) {return unknown();}
    if (have_time_) {
      const uint32_t delta = ms - last_ms_;  // normal millis wrap is allowed
      if (delta == 0 || delta > 50) {
        fault_ = true;  // new configuration required after a clock fault
        return unknown();
      }
      elapsed_ += delta;
    }
    have_time_ = true;
    last_ms_ = ms;
    if (!std::isfinite(position)) {
      size_ = 0;  // missing measurement: require a new full warm-up window
      return unknown();
    }
    std::size_t keep = 0;
    for (std::size_t i = 0; i < size_; ++i) {
      if (elapsed_ - samples_[i].ms <= 60) {samples_[keep++] = samples_[i];}
    }
    size_ = keep;
    samples_[size_++] = {elapsed_, position};
    if (size_ < 4 || elapsed_ - samples_[0].ms < 30) {return unknown();}
    double mean_t = 0.0, mean_q = 0.0;
    for (std::size_t i = 0; i < size_; ++i) {
      mean_t += -static_cast<double>(elapsed_ - samples_[i].ms) / 1000.0;
      mean_q += samples_[i].q;
    }
    mean_t /= size_;
    mean_q /= size_;
    double numerator = 0.0, denominator = 0.0;
    for (std::size_t i = 0; i < size_; ++i) {
      const double w = -static_cast<double>(elapsed_ - samples_[i].ms) / 1000.0 - mean_t;
      numerator += w * (samples_[i].q - mean_q);
      denominator += w * w;
    }
    return numerator / denominator;
  }

private:
  static double unknown() {return std::numeric_limits<double>::quiet_NaN();}
  struct Sample {uint64_t ms; double q;};
  std::array<Sample, 61> samples_{};
  std::size_t size_ = 0;
  uint64_t elapsed_ = 0;
  uint32_t last_ms_ = 0;
  bool have_time_ = false, fault_ = false;
};
}  // namespace hardware
