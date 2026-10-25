// Stationary LDS-03 experiment: SDK conversion adapter, not a timing correction.
#ifndef PROJECT2_EXPERIMENTAL_SCAN_HPP
#define PROJECT2_EXPERIMENTAL_SCAN_HPP
#include "angle_grid.hpp"
#include "coin_d4_driver/lidar_sdk/lidar_information.hpp"
namespace lidar_diagnostic {
// ROBOTIS LDS-03 specifications, checked 2026-10-25:
// https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/
constexpr double lds03_min_range = 0.05;
constexpr double lds03_max_range = 12.0;
inline void experimental_scan(LaserScan& scan, const node_info* raw, std::size_t count,
                                  bool reverse, double scan_seconds) {
  std::vector<Sample> samples;
  samples.reserve(count);
  for (std::size_t i=0;i<count;++i) {
    samples.push_back({(raw[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0,
      static_cast<double>(raw[i].distance_q2), raw[i].sync_quality, raw[i].exp_m == 1});
  }
  const auto grid = convert(samples,400,reverse,lds03_min_range,lds03_max_range);
  scan.stamp = 0;
  scan.points.clear();
  scan.config.min_angle = 0;
  scan.config.angle_increment = static_cast<float>(grid.angle_increment);
  scan.config.max_angle = (grid.bins.size()-1)*scan.config.angle_increment;
  scan.config.min_range = lds03_min_range;
  scan.config.max_range = lds03_max_range;
  scan.config.scan_time = static_cast<float>(scan_seconds);
  // Unknown per-ray times: stationary visualization only; no fabricated deskew times.
  scan.config.time_increment = 0;
  for (std::size_t i=0;i<grid.bins.size();++i) {
    LaserPoint point{};
    point.angle = static_cast<float>(i*360.0/grid.bins.size());
    point.range = static_cast<float>(grid.bins[i].range);
    point.intensity = grid.bins[i].intensity;
    scan.points.push_back(point);
  }
}
}  // namespace lidar_diagnostic
#endif
