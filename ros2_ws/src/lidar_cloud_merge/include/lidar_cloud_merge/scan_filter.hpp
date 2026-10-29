#pragma once
#include "lidar_cloud_merge/core.hpp"
#include <limits>

namespace lidar_cloud_merge {
// Single physical origin: preserve the grid and timing, never turn an excluded
// return into +inf (which a downstream consumer could interpret as free space).
inline sensor_msgs::msg::LaserScan filter_front_scan(
    const sensor_msgs::msg::LaserScan &input) {
  if (input.header.frame_id != "lidar_front_link" || input.ranges.empty() ||
      !std::isfinite(input.angle_min) || !std::isfinite(input.angle_max) ||
      !std::isfinite(input.angle_increment) || input.angle_increment <= 0 ||
      !std::isfinite(input.range_min) || !std::isfinite(input.range_max) ||
      input.range_min < 0 || input.range_max <= input.range_min ||
      !std::isfinite(input.scan_time) || input.scan_time <= 0 ||
      !std::isfinite(input.time_increment) || input.time_increment < 0 ||
      (!input.intensities.empty() && input.intensities.size() != input.ranges.size())) {
    throw std::runtime_error("Invalid front scan metadata/frame");
  }
  const double last = input.angle_min + (input.ranges.size()-1)*double(input.angle_increment);
  if (std::abs(last-input.angle_max) > 1e-4) {
    throw std::runtime_error("Inconsistent scan angle grid");
  }
  const BodyMask body{.110, .005, .2552, .3050, .010};
  auto output = input;
  for (size_t i=0; i<input.ranges.size(); ++i) {
    const double r=input.ranges[i], angle=input.angle_min+i*double(input.angle_increment);
    if (!std::isfinite(r) || r<input.range_min || r>input.range_max ||
        body.contains({float(r*std::cos(angle)),float(r*std::sin(angle)),0,0,0})) {
      output.ranges[i]=std::numeric_limits<float>::quiet_NaN();
      if (!output.intensities.empty()) output.intensities[i]=0;
    }
  }
  return output;
}
}  // namespace lidar_cloud_merge
