#pragma once
#include "options.hpp"
#include "coin_d4_driver/mapping_frame.hpp"
#include <array>
#include <map>
#include <tuple>

namespace workbench {
using Point = lidar_motion::Point;
using Pose = lidar_motion::Pose;
struct Ray {
  Point origin, endpoint;
  int source = 0, input = 0;
  double acquisition_offset = 0;
};
inline Pose mount(int side) {
  require(side == 0 || side == 1, "Invalid sensor index");
  return {side == 0 ? .110 : -.110, .005, side == 0 ? std::acos(-1.) : 0.};
}
inline bool body(Point p) { return std::abs(p.x) <= .1376 && std::abs(p.y) <= .1625; }
inline void validate_motion(const std::vector<lidar_motion::Odom> &samples, const Options &o) {
  require(samples.size() >= 2, "Insufficient wheel odometry");
  for (size_t i = 1; i < samples.size(); ++i) {
    const auto &a = samples[i-1], &b = samples[i];
    const double dt = (b.stamp_ns - a.stamp_ns) * 1e-9;
    require(dt > 0 && dt <= o.max_gap + 1e-9, "Odometry gap/regression");
    require(std::hypot(b.pose.x-a.pose.x, b.pose.y-a.pose.y) <= o.max_linear*dt + 1e-9 &&
      std::abs(lidar_motion::wrap(b.pose.yaw-a.pose.yaw)) <= o.max_angular*dt + 1e-9,
      "Motion exceeds configured scan admission envelope");
  }
}
struct Prepared { std::vector<Ray> raw, corrected; lidar_mapping::Scan scan; };
inline Prepared prepare(const lidar_mapping::TimedFrame &frame,
    const std::vector<lidar_motion::Odom> &samples, int side, const Options &o) {
  validate_motion(samples, o);
  const lidar_motion::Trajectory trajectory(samples, static_cast<int64_t>(o.max_gap*1e9));
  const auto sensor = mount(side);
  Prepared result;
  std::vector<lidar_mapping::Ray> projection;
  for (const auto &b : frame.beams) {
    const Point endpoint{b.beam.range*std::cos(b.beam.angle), b.beam.range*std::sin(b.beam.angle)};
    const double offset = (b.stamp-frame.reference)*1e-9;
    result.raw.push_back({{0, 0}, endpoint, side, b.beam.bin, offset});
    if (b.beam.range < o.min_range || b.beam.range > o.max_range || body(lidar_motion::apply(sensor, endpoint))) continue;
    const auto ray = lidar_motion::correct(trajectory, b.stamp, frame.reference, sensor, b.beam.range, b.beam.angle);
    result.corrected.push_back({ray.origin, ray.endpoint, side, b.beam.bin, offset});
    projection.push_back({b.beam.bin, ray.origin, ray.endpoint});
  }
  lidar_mapping::Policy policy;
  policy.bins = static_cast<size_t>(o.bins); policy.min_range = o.min_range; policy.max_range = o.max_range;
  policy.max_endpoint_error = o.endpoint_error; policy.max_origin = o.origin_shift;
  result.scan = lidar_mapping::resample(projection, policy);
  return result;
}
// Re-express a full ray; retaining its physical origin prevents synthetic free-space claims.
inline Ray transform(const Ray &r, Pose pose, double source_to_output_time) {
  return {lidar_motion::apply(pose, r.origin), lidar_motion::apply(pose, r.endpoint),
    r.source, r.input, r.acquisition_offset + source_to_output_time};
}
struct Filtered { std::vector<Ray> kept, rejected; };
inline Filtered spatial_filter(const std::vector<Ray> &rays, const Options &o) {
  Filtered result;
  if (o.neighbors == 0) {result.kept = rays; return result;}
  using Cell = std::pair<int, int>;
  std::map<Cell, std::vector<size_t>> grid;
  auto cell = [&](Point p) { return Cell{static_cast<int>(std::floor(p.x/o.filter_radius)),
    static_cast<int>(std::floor(p.y/o.filter_radius))}; };
  for (size_t i = 0; i < rays.size(); ++i) grid[cell(rays[i].endpoint)].push_back(i);
  for (size_t i = 0; i < rays.size(); ++i) {
    const auto &r = rays[i];
    // Near returns survive even when isolated. This is a diagnostic filter, not a noise classifier.
    bool keep = std::hypot(r.endpoint.x-r.origin.x, r.endpoint.y-r.origin.y) <= o.protect_radius;
    int count = 0; const auto c = cell(r.endpoint);
    for (int x = -1; x <= 1 && !keep; ++x) for (int y = -1; y <= 1 && !keep; ++y) {
      const auto found = grid.find({c.first+x, c.second+y});
      if (found == grid.end()) continue;
      for (const auto j : found->second) if (j != i && std::hypot(r.endpoint.x-rays[j].endpoint.x,
          r.endpoint.y-rays[j].endpoint.y) <= o.filter_radius && ++count >= o.neighbors) {keep = true; break;}
    }
    (keep ? result.kept : result.rejected).push_back(r);
  }
  return result;
}
// Analytic static room used only by mock sensors. Never a hardware fallback.
inline double scene_range(Point origin, double angle) {
  const double dx = std::cos(angle), dy = std::sin(angle);
  double range = 12.;
  const std::array<std::array<double, 4>, 6> walls{{{-2,-2,4,-2}, {4,-2,4,3},
    {4,3,-2,3}, {-2,3,-2,-2}, {1.5,.8,2.3,.8}, {2.3,.8,2.3,1.6}}};
  for (const auto &w : walls) {
    const double sx=w[2]-w[0], sy=w[3]-w[1], det=dx*sy-dy*sx;
    if (std::abs(det)<1e-9) continue;
    const double px=w[0]-origin.x, py=w[1]-origin.y;
    const double t=(px*sy-py*sx)/det, u=(px*dy-py*dx)/det;
    if (t>.05 && u>=0 && u<=1) range=std::min(range,t);
  }
  return range;
}
}  // namespace workbench
