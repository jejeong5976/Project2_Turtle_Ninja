#pragma once
#include "body_route_controller/core.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace body_route_controller::layout
{
// Offline stop-and-scan data. All angles are radians; timestamps are seconds.
struct Scan {
  std::string sensor_id;
  double stamp = 0, angle_min = 0, angle_increment = 0, range_min = .1, range_max = 3;
  Pose sensor;  // base_footprint -> laser, never a merged fictitious ray origin
  std::vector<double> ranges;
};
struct Frame {double stamp = 0, settled_seconds = 0; Pose odom; Velocity measured; std::vector<Scan> scans;};
struct Wall {std::string id; Point a, b; std::string assembly = "";};
struct Segment {Point a, b; bool edge_a = false, edge_b = false; std::size_t sensor = 0;};
struct Settings {
  double max_wall_shift = .05, max_wall_yaw = .10;
  double split_error = .012, cluster_jump = .08, min_segment = .12;
  double association_distance = .12, association_angle = .15, association_margin = .012;
  double line_sigma = .01, edge_sigma = .015, odom_sigma = .025, odom_yaw_sigma = .025;
  double start_sigma = .003, start_yaw_sigma = .0087;
  double max_position_sigma = .025, max_yaw_sigma = .035, max_residual = .025;
  double min_view_baseline = .04, min_view_angle = .04, max_scan_skew = .05;
  int min_points = 6, max_frames = 32, max_walls = 64;
  void validate() const;
};
struct PoseEstimate {Pose pose; bool verified = false; double position_sigma = 0, yaw_sigma = 0;};
struct WallEstimate {
  Wall wall; bool verified = false, line_verified = false; bool edge_a = false, edge_b = false;
  double max_displacement = 0, position_sigma = 0, line_sigma = 0; int views = 0;
};
struct Estimate {
  bool accepted = false; std::string reason;
  std::vector<PoseEstimate> poses; std::vector<WallEstimate> walls;
  std::size_t matched = 0, ambiguous = 0, unmatched = 0;
  double residual_rms = 0, max_residual = 0;
  int geometric_rank = 0, variables = 0;
};
struct Grid {
  int width = 0, height = 0; double resolution = .01; Point origin;
  std::vector<std::int8_t> cells;  // -1 unknown, 0 free, 100 occupied
  void validate() const;
  int index(Point p) const;
  Point center(int index) const;
};
struct MapUpdate {Grid map; std::size_t cleared = 0, hits = 0, uncertain_cells = 0;};
struct Portal {std::string first, second; bool first_b = true, second_b = false; double heading = 0, approach = .35, exit = .35;};
struct Crossing {bool accepted = false; std::string reason; double width = 0; std::vector<Pose> path;};

Point transform(Pose pose, Point point);
std::vector<Segment> extract(const Scan &, const Settings & = {});
Estimate estimate(const std::vector<Wall> &, const std::vector<Frame> &, Pose fixed_start,
  const Settings & = {});
MapUpdate updateMap(const Grid &, const std::vector<Wall> &, const std::vector<Frame> &,
  const Estimate &, const Settings & = {});
// Geometry proposal only. Caller must validate the swept footprint against the map.
Crossing crossing(const Portal &, const Estimate &, double required_width);
}
