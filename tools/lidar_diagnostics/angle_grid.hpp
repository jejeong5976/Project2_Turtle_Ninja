// Offline candidate only: no ROS publisher or acquisition-time model.
#ifndef PROJECT2_ANGLE_GRID_HPP
#define PROJECT2_ANGLE_GRID_HPP
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace lidar_diagnostic {
constexpr double pi = 3.14159265358979323846;
struct Sample {
  double angle_deg;
  double distance_mm;
  unsigned intensity;
  bool exposure;
};
struct Bin {
  double range = std::numeric_limits<double>::quiet_NaN();
  unsigned intensity = 0;
  std::size_t source = std::numeric_limits<std::size_t>::max();
};
struct Grid {
  std::vector<Bin> bins;
  double angle_increment;
  double angle_max;
  std::size_t rejected = 0;
  std::size_t collisions = 0;
};
inline double wrap_degrees(double angle) {
  const double result = std::fmod(angle, 360.0);
  return result < 0 ? result + 360.0 : result;
}
// reverse=false maps decoded clockwise angles to counterclockwise bearings.
// At collisions retain the nearest valid return, with first input winning ties.
inline Grid convert(const std::vector<Sample>& samples, std::size_t count = 400,
                    bool reverse = false, double range_min = 0.10,
                    double range_max = 100.0) {
  if (count < 2 || count > 10000 || !std::isfinite(range_min) ||
      !std::isfinite(range_max) || range_min <= 0 || range_max <= range_min) {
    throw std::invalid_argument("Invalid grid or range limits");
  }
  Grid out{std::vector<Bin>(count), 2 * pi / count, 2 * pi * (count - 1) / count};
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto& s = samples[i];
    const double range = s.distance_mm / 1000.0;
    const unsigned intensity = s.exposure ? 255 : (s.intensity >= 255 ? 254 : s.intensity);
    if (!std::isfinite(s.angle_deg) || s.angle_deg < 0 || s.angle_deg > 360 ||
        !std::isfinite(range) || range < range_min || range > range_max ||
        (range <= 0.15 && intensity <= 65)) {
      ++out.rejected;
      continue;
    }
    const double angle = wrap_degrees(reverse ? s.angle_deg : -s.angle_deg);
    const auto index = static_cast<std::size_t>(std::floor(angle * count / 360.0 + 0.5)) % count;
    auto& bin = out.bins[index];
    if (std::isfinite(bin.range)) ++out.collisions;
    if (!std::isfinite(bin.range) || range < bin.range) bin = {range, intensity, i};
  }
  return out;
}
}  // namespace lidar_diagnostic
#endif
