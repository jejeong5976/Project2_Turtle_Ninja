#pragma once

#include <string>
#include <vector>

namespace body_route_controller
{
struct Point {double x = 0.0, y = 0.0;};
struct Pose {double x = 0.0, y = 0.0, yaw = 0.0;};
struct Velocity {double x = 0.0, y = 0.0, yaw = 0.0;};

struct Settings
{
  std::string drive_style = "axis-stop";
  std::string heading_policy = "reactive";
  double heading_preview_time = 5.0; // seconds of final-approach heading preview
  bool round_corners = true; // Internal clearance fallback follows exact route corners.
  double max_travel_angle = 0.7853981633974483; // radians, body +X versus travel

  double max_speed = 0.025;
  double correction_speed = 0.015;
  double max_yaw_rate = 0.1;
  double heading_hold_rate = 0.02;
  double heading_rate_per_speed = 0.4;
  double heading_kp = 0.6;
  double linear_kp = 0.5;
  double linear_accel = 0.05;
  double angular_accel = 0.2;
  double align_enter = 0.12;
  double align_exit = 0.035;
  double lookahead = 0.12;
  double arrival_lookahead = 0.4;
  double corner_tolerance = 0.02;
  double path_simplify_tolerance = 0.01;
  double cross_track_enter = 0.025;
  double cross_track_exit = 0.01;
  double max_path_distance = 0.25;
  double max_correction_distance = 0.15;
  double max_correction_time = 20.0;
  double stopped_linear = 0.003;
  double stopped_angular = 0.01;
  double goal_xy = 0.03;
  double goal_yaw = 0.05;
  double max_dt = 0.25;
  void validate() const;
};

struct RouteTarget
{
  Point point;
  double heading = 0.0;
  double distance_to_path = 0.0;
  double remaining = 0.0;
  bool final_segment = false;
};
struct RouteProgress
{
  Point corner;
  double heading = 0.0;
  bool committed = false;
};
struct Result
{
  Velocity command;
  std::string mode;
  double heading = 0.0;
  double heading_error = 0.0;
};

std::vector<Point> routeCorners(const std::vector<Point> &, double tolerance);
double wrap(double angle);
bool fresh(double now, double received, double stamped, double max_age);
RouteTarget routeTarget(const std::vector<Point> & path, const Pose & pose, const Settings & cfg,
  RouteProgress * progress = nullptr);

class Core
{
public:
  explicit Core(Settings cfg = {});
  Result step(const Pose &, const Velocity &, const std::vector<Point> &, const Pose & goal,
    double dt, double speed_limit);
  void reset();
  const Settings & settings() const {return cfg_;}

private:
  Result continuous(const Pose &, const Velocity &, const RouteTarget &, const Pose & goal, double dt, double speed_limit);
  bool heading_latched_ = false;
  double held_heading_ = 0.0, previous_yaw_ = 0.0;
  Settings cfg_;
  RouteProgress progress_;
  bool aligning_ = true;
  bool lateral_ = false;
  int axis_ = 0;  // +/-1 longitudinal, +/-2 lateral; zero stopped.
  Velocity previous_;
  Point correction_start_;
  double correction_time_ = 0.0;
  int correction_axis_ = 0;
};
}  // namespace body_route_controller
