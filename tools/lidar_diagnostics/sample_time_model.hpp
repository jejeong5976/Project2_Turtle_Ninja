#ifndef PROJECT2_SAMPLE_TIME_MODEL_HPP
#define PROJECT2_SAMPLE_TIME_MODEL_HPP
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace lidar_timing {
struct RelativeTime {
  int64_t source_index;
  // Both are hypotheses relative to the first decoded sample, NOT ROS timestamps.
  double nominal_ns;
  double period_scaled_ns;
};
// A ring interval spans N sample intervals; its final sample is at (N-1)*dt.
// Empty bins retain source=-1 and NaN offsets. Do not add a period to conceal a
// backward or wrap step: that would move a measurement into a different revolution.
inline std::vector<RelativeTime> model_times(std::size_t raw_count, double period_ns,
    const std::vector<int64_t>& sources, double nominal_hz=4000.0) {
  if (raw_count==0 || raw_count>10000 || !std::isfinite(period_ns) || period_ns<=0 ||
      !std::isfinite(nominal_hz) || nominal_hz<=0)
    throw std::invalid_argument("Invalid raw count, ring period, or nominal rate");
  std::vector<RelativeTime> result;
  for (const auto index:sources) {
    if (index < -1 || index >= static_cast<int64_t>(raw_count))
      throw std::invalid_argument("Source index outside this scan");
    if (index==-1) result.push_back({-1,NAN,NAN});
    else result.push_back({index,index*(1e9/nominal_hz),index*(period_ns/raw_count)});
  }
  return result;
}
} // namespace lidar_timing
#endif
