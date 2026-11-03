#include "body_route_controller/core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace body_route_controller
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double distance(Point a, Point b) {return std::hypot(a.x - b.x, a.y - b.y);}
double approach(double current, double target, double step)
{return current + std::clamp(target - current, -step, step);}
bool finite(const Pose & p) {return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);}
}

double wrap(double a) {return std::remainder(a, 2.0 * kPi);}

bool fresh(double now, double received, double stamped, double age)
{
  return std::isfinite(now) && std::isfinite(received) && std::isfinite(stamped) &&
         now >= received && now - received <= age && now - stamped >= -0.05 &&
         now - stamped <= age;
}

void Settings::validate() const
{
  for (double v : {max_speed, correction_speed, max_yaw_rate, heading_hold_rate, heading_rate_per_speed,
      heading_kp, linear_kp, linear_accel, angular_accel, align_enter, align_exit,
      lookahead, corner_tolerance, cross_track_enter, cross_track_exit, max_path_distance,
      max_correction_distance, max_correction_time, stopped_linear, stopped_angular,
      goal_xy, goal_yaw, max_dt})
  {
    if (!std::isfinite(v) || v <= 0.0) {throw std::invalid_argument("All body-route limits must be finite and positive");}
  }
  if (align_exit >= align_enter || align_enter >= kPi / 2 ||
    cross_track_exit >= cross_track_enter || correction_speed > max_speed ||
    heading_hold_rate > max_yaw_rate || corner_tolerance >= lookahead)
  {throw std::invalid_argument("Inconsistent body-route hysteresis or limits");}
}

RouteTarget routeTarget(const std::vector<Point> & input, const Pose & pose, const Settings & cfg)
{
  if (!finite(pose) || input.empty()) {throw std::invalid_argument("Invalid pose or empty path");}
  std::vector<Point> path;
  for (auto p : input) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {throw std::invalid_argument("Non-finite path");}
    if (path.empty() || distance(path.back(), p) > 1e-6) {path.push_back(p);}
  }
  const Point here{pose.x, pose.y};
  if (path.size() == 1) {
    const double d = distance(here, path[0]);
    // A single point has no route tangent. It can support final goal alignment,
    // but must not turn a backward correction into an invented 180-degree turn.
    if (d > cfg.goal_xy) {throw std::runtime_error("Single-point route has no translation heading; replan required");}
    return {path[0], pose.yaw, d};
  }
  std::size_t nearest = 0;
  Point projected = path[0];
  double best = std::numeric_limits<double>::infinity();
  double searched = 0.0;
  // The Nav2 path handler supplies a pruned path. Restrict the nearest search to
  // its beginning so a crossing path cannot jump to a much later segment.
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x, dy = path[i + 1].y - path[i].y;
    const double length = std::hypot(dx, dy);
    const double t = std::clamp(((pose.x - path[i].x) * dx + (pose.y - path[i].y) * dy) /
        (length * length), 0.0, 1.0);
    Point q{path[i].x + t * dx, path[i].y + t * dy};
    const double d = distance(here, q);
    if (d <= best + 1e-9) {best = d; projected = q; nearest = i;}
    searched += length;
    if (searched > 1.0) {break;}
  }
  if (best > cfg.max_path_distance) {throw std::runtime_error("Robot too far from route; replan required");}
  // Advance a segment only after reaching its corner. Do not turn early just
  // because a lookahead point lies beyond a sharp bend.
  if (nearest + 2 < path.size() && distance(here, path[nearest + 1]) < cfg.corner_tolerance) {
    ++nearest;
    projected = path[nearest];
  }
  Point target = projected;
  double remaining = cfg.lookahead;
  const double initial_heading = std::atan2(path[nearest + 1].y - path[nearest].y,
      path[nearest + 1].x - path[nearest].x);
  for (std::size_t i = nearest; i + 1 < path.size(); ++i) {
    const Point start = i == nearest ? projected : path[i];
    const double length = distance(start, path[i + 1]);
    if (length < 1e-9) {continue;}
    const double heading = std::atan2(path[i + 1].y - start.y, path[i + 1].x - start.x);
    if (i > nearest && std::abs(wrap(heading - initial_heading)) > cfg.align_enter) {break;}
    const double fraction = std::min(1.0, remaining / length);
    target = {start.x + fraction * (path[i + 1].x - start.x),
      start.y + fraction * (path[i + 1].y - start.y)};
    remaining -= fraction * length;
    if (remaining <= 1e-9) {break;}
  }
  return {target, initial_heading, best};
}

Core::Core(Settings cfg) : cfg_(cfg) {cfg_.validate();}

void Core::reset()
{
  aligning_ = true;
  lateral_ = false;
  axis_ = 0;
  previous_ = {};
  correction_axis_ = 0;
  correction_time_ = 0.0;
}

Result Core::step(const Pose & pose, const Velocity & measured, const std::vector<Point> & path,
  const Pose & goal, double dt, double speed_limit)
{
  if (!finite(pose) || !finite(goal) || !finite({measured.x, measured.y, measured.yaw}) ||
    !std::isfinite(dt) || dt <= 0.0 || !std::isfinite(speed_limit) || speed_limit < 0.0)
  {reset(); throw std::invalid_argument("Invalid body-route state/time/speed limit");}
  if (dt > cfg_.max_dt) {reset(); return {{}, "clock_gap", pose.yaw, 0.0};}
  const auto target = routeTarget(path, pose, cfg_);
  const double goal_distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  const bool final = goal_distance <= cfg_.goal_xy;
  const double heading = final ? goal.yaw : target.heading;
  const double error = wrap(heading - pose.yaw);
  const bool stopped = std::hypot(measured.x, measured.y) <= cfg_.stopped_linear;
  auto stop = [&](const std::string & reason) -> Result {
      previous_ = {}; return {{}, reason, heading, error};
    };
  if (speed_limit == 0.0) {reset(); return stop("speed_limit_stop");}
  if (final && std::abs(error) <= cfg_.goal_yaw) {return stop("goal_stop");}
  if (final || std::abs(error) > cfg_.align_enter) {aligning_ = true;}
  if (aligning_) {
    axis_ = 0;
    lateral_ = false;
    correction_axis_ = 0;
    correction_time_ = 0.0;
    if (!stopped) {return stop("stop_before_turn");}
    const double tolerance = final ? cfg_.goal_yaw : cfg_.align_exit;
    if (std::abs(error) > tolerance) {
      const double desired = std::copysign(std::min({cfg_.max_yaw_rate,
          cfg_.heading_kp * std::abs(error), std::sqrt(2 * cfg_.angular_accel * std::abs(error))}), error);
      previous_ = {0.0, 0.0, approach(measured.yaw, desired, cfg_.angular_accel * dt)};
      previous_.yaw = std::clamp(previous_.yaw, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
      return {previous_, final ? "final_turn" : "route_turn", heading, error};
    }
    if (std::abs(measured.yaw) > cfg_.stopped_angular) {return stop("settle_turn");}
    aligning_ = false;
    return stop("aligned");
  }
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  const double dx = target.point.x - pose.x, dy = target.point.y - pose.y;
  const double bx = c * dx + s * dy, by = -s * dx + c * dy;
  lateral_ = lateral_ ? std::abs(by) > cfg_.cross_track_exit : std::abs(by) > cfg_.cross_track_enter;
  const int axis = lateral_ ? (by >= 0.0 ? 2 : -2) : (bx >= 0.0 ? 1 : -1);
  if (axis != axis_) {
    if (!stopped) {return stop("stop_before_axis_change");}
    axis_ = axis;
    previous_ = {};
  }
  if (axis != 1) {
    if (correction_axis_ != axis) {
      correction_start_ = {pose.x, pose.y}; correction_time_ = 0.0; correction_axis_ = axis;
    }
    correction_time_ += dt;
    if (distance(correction_start_, {pose.x, pose.y}) > cfg_.max_correction_distance ||
      correction_time_ > cfg_.max_correction_time)
    {throw std::runtime_error("Correction bound exceeded; replan required");}
  } else {correction_axis_ = 0; correction_time_ = 0.0;}
  const double limit = std::min(speed_limit, axis == 1 ? cfg_.max_speed : cfg_.correction_speed);
  const double residual = std::abs(axis) == 1 ? bx : by;
  const double desired = std::clamp(cfg_.linear_kp * residual, -limit, limit);
  const double old = std::abs(axis) == 1 ? previous_.x : previous_.y;
  const double speed = std::clamp(approach(old, desired, cfg_.linear_accel * dt), -limit, limit);
  // Yaw correction is bounded and falls to zero as translation stops. Larger
  // errors are handled by the rotate-in-place phase, never a translating turn.
  const double yaw_limit = std::min(cfg_.heading_hold_rate, cfg_.heading_rate_per_speed * std::abs(speed));
  const double yaw = std::clamp(cfg_.heading_kp * error, -yaw_limit, yaw_limit);
  previous_ = std::abs(axis) == 1 ? Velocity{speed, 0.0, yaw} : Velocity{0.0, speed, yaw};
  return {previous_, axis == 1 ? "forward" : axis == -1 ? "backward" : axis == 2 ? "left" : "right",
    heading, error};
}
}  // namespace body_route_controller
