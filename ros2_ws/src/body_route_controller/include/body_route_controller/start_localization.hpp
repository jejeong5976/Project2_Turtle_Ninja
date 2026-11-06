#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace body_route_controller::start_localization {
using Pose = std::array<double, 3>;
using Point = std::array<double, 2>;
struct Wall {
  std::string name;
  // Map-frame occupied-cell ROI: xmin, ymin, xmax, ymax.
  std::array<double, 4> roi;
  // Body-frame ray bearing, counterclockwise from +X, normalized to [0,360).
  std::array<double, 2> sector_deg;
};
struct Config {
  Pose reference;
  Pose rear_mount;
  std::array<Wall, 2> walls;
  double range_min, range_max, association_margin;
  std::size_t min_points;
  double min_span, max_p90, max_snapshot_distance, max_snapshot_yaw;
};
struct Sample {Point body; std::size_t wall, index;};
struct Metrics {std::size_t count; double span, median, p90;};
struct Result {
  Pose seed, pose;
  std::array<Metrics, 2> train, heldout;
  bool accepted;
};
Config readConfig(const std::string & path);
Point transform(const Point & p, const Pose & pose);
Point inverse(const Point & p, const Pose & pose);
// Selection depends on the reviewed reference, not the changing AMCL estimate.
// Returns -1 for all excluded rays; caller keeps the full scan metadata/array.
int select(const Point & body, const Point & sensor_origin, const Config & cfg);
std::array<std::vector<Point>, 2> readMap(const std::string & yaml, const Config & cfg);
Result fit(const std::array<std::vector<Point>, 2> & map,
  const std::vector<Sample> & samples, const Config & cfg);
}  // namespace body_route_controller::start_localization
