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
// Iterative Ramer-Douglas-Peucker removes centimetre-grid tangent noise without
// moving an endpoint or deviating more than tolerance from the supplied route.
// Live footprint/stopping checks still decide whether a shortcut is traversable.
std::vector<Point> simplify(const std::vector<Point> & input, double tolerance)
{
  if (input.size() < 3) {return input;}
  std::vector<bool> keep(input.size(), false);
  keep.front() = keep.back() = true;
  std::vector<std::pair<std::size_t, std::size_t>> pending{{0, input.size() - 1}};
  while (!pending.empty()) {
    const auto [first, last] = pending.back(); pending.pop_back();
    const auto a = input[first], b = input[last];
    const double dx = b.x - a.x, dy = b.y - a.y, norm = dx * dx + dy * dy;
    double worst = tolerance; std::size_t split = first;
    for (std::size_t i = first + 1; i < last; ++i) {
      const double t = norm > 0 ? std::clamp(
        ((input[i].x - a.x) * dx + (input[i].y - a.y) * dy) / norm, 0.0, 1.0) : 0.0;
      const double error = distance(input[i], {a.x + t * dx, a.y + t * dy});
      if (error > worst) {worst = error; split = i;}
    }
    if (split != first) {
      keep[split] = true; pending.emplace_back(first, split); pending.emplace_back(split, last);
    }
  }
  std::vector<Point> out;
  for (std::size_t i = 0; i < input.size(); ++i) {if (keep[i]) {out.push_back(input[i]);}}
  return out;
}

}

std::vector<Point> routeCorners(const std::vector<Point> & input, double tolerance)
{
  if (!std::isfinite(tolerance) || tolerance <= 0) {throw std::invalid_argument("Invalid route tolerance");}
  std::vector<Point> path;
  for (auto p : input) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {throw std::invalid_argument("Non-finite path");}
    if (path.empty() || distance(path.back(), p) > 1e-6) {path.push_back(p);}
  }
  return simplify(path, tolerance);
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
  if (drive_style != "axis-stop" && drive_style != "continuous") {
    throw std::invalid_argument("drive_style must be axis-stop or continuous");
  }
  if (heading_policy != "reactive" && heading_policy != "preview") {
    throw std::invalid_argument("heading_policy must be reactive or preview");
  }
  if (heading_policy == "preview" && drive_style != "continuous") {
    throw std::invalid_argument("preview heading requires continuous driving");
  }
  if (!std::isfinite(heading_preview_time) || heading_preview_time < .5 || heading_preview_time > 10) {
    throw std::invalid_argument("heading_preview_time must be .5..10 seconds");
  }
  if (!std::isfinite(max_travel_angle) || max_travel_angle < kPi / 18 || max_travel_angle > kPi / 3) {
    throw std::invalid_argument("max_travel_angle must be 10..60 degrees in radians");
  }
  for (double v : {max_speed, correction_speed, max_yaw_rate, heading_hold_rate, heading_rate_per_speed,
      heading_kp, linear_kp, linear_accel, angular_accel, align_enter, align_exit,
      lookahead, arrival_lookahead, corner_tolerance, path_simplify_tolerance, cross_track_enter, cross_track_exit, max_path_distance,
      max_correction_distance, max_correction_time, stopped_linear, stopped_angular,
      goal_xy, goal_yaw, max_dt})
  {
    if (!std::isfinite(v) || v <= 0.0) {throw std::invalid_argument("All body-route limits must be finite and positive");}
  }
  if (align_exit >= align_enter || align_enter >= kPi / 2 ||
    cross_track_exit >= cross_track_enter || correction_speed > max_speed ||
    heading_hold_rate > max_yaw_rate || corner_tolerance >= lookahead || path_simplify_tolerance >= corner_tolerance)
  {throw std::invalid_argument("Inconsistent body-route hysteresis or limits");}
}

RouteTarget routeTarget(const std::vector<Point> & input, const Pose & pose, const Settings & cfg,
  RouteProgress * progress)
{
  if (!finite(pose) || input.empty()) {throw std::invalid_argument("Invalid pose or empty path");}
  const auto path = routeCorners(input, cfg.path_simplify_tolerance);
  const Point here{pose.x, pose.y};
  if (path.size() == 1) {
    if (progress) {*progress = {};}
    const double d = distance(here, path[0]);
    // A single point has no route tangent. It can support final goal alignment,
    // but must not turn a backward correction into an invented 180-degree turn.
    if (d > cfg.goal_xy) {throw std::runtime_error("Single-point route has no translation heading; replan required");}
    return {path[0], pose.yaw, d, 0.0};
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
    // Preserve the earlier leg on overlapping out-and-back routes. Corner
    // advancement below, rather than a distance tie, selects the outgoing leg.
    if (d < best - 1e-9) {best = d; projected = q; nearest = i;}
    searched += length;
    if (searched > 1.0) {break;}
  }
  if (best > cfg.max_path_distance) {throw std::runtime_error("Robot too far from route; replan required");}
  // Enter a corner only inside the normal tolerance, then retain the selected
  // outgoing leg through small localization shifts. Without this hysteresis a
  // 2 mm pose update can reverse the target halfway through a stationary turn.
  // Match the CURRENT path's corner and tangent, so replans cannot reuse a
  // removed turn. Larger displacement releases the commitment.
  auto tangent = [&](std::size_t i) {
      return std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
    };
  const double release = 3 * cfg.corner_tolerance;
  bool advance = false;
  if (nearest + 2 < path.size()) {
    const auto corner = path[nearest + 1];
    advance = distance(here, corner) < cfg.corner_tolerance;
    if (!advance && progress && progress->committed) {
      advance = distance(here, corner) < release &&
        distance(corner, progress->corner) < 2 * cfg.corner_tolerance &&
        std::abs(wrap(tangent(nearest + 1) - progress->heading)) < cfg.align_enter;
    }
  }
  if (advance) {
    ++nearest;
    projected = path[nearest];
    if (progress) {*progress = {path[nearest], tangent(nearest), true};}
  } else if (progress && progress->committed) {
    if (distance(here, progress->corner) >= release ||
      distance(path[nearest], progress->corner) >= 2 * cfg.corner_tolerance ||
      std::abs(wrap(tangent(nearest) - progress->heading)) >= cfg.align_enter)
    {*progress = {};}
  }
  Point target = projected;
  // A short final lateral stub must be approached diagonally early enough to
  // stay in the forward cone; waiting until its corner would require strafing.
  const bool short_arrival = cfg.drive_style == "continuous" && cfg.round_corners && nearest + 3 == path.size() &&
    distance(path[nearest + 1], path.back()) <= .15;
  double remaining = short_arrival ? std::max(cfg.lookahead,cfg.arrival_lookahead) : cfg.lookahead;
  const double initial_heading = std::atan2(path[nearest + 1].y - path[nearest].y,
      path[nearest + 1].x - path[nearest].x);
  for (std::size_t i = nearest; i + 1 < path.size(); ++i) {
    const Point start = i == nearest ? projected : path[i];
    const double length = distance(start, path[i + 1]);
    if (length < 1e-9) {continue;}
    const double heading = std::atan2(path[i + 1].y - start.y, path[i + 1].x - start.x);
    if (i > nearest && std::abs(wrap(heading - initial_heading)) >
      (cfg.drive_style == "continuous" && cfg.round_corners ? kPi / 2 + 1e-6 : cfg.align_enter)) {break;}
    if (cfg.drive_style == "continuous" && cfg.round_corners && i == nearest && nearest + 2 < path.size() &&
      std::abs(wrap(tangent(nearest + 1) - initial_heading)) > cfg.align_enter) {
      remaining = std::min(remaining, std::max(length, short_arrival ? cfg.arrival_lookahead : .24));
    }
    const double fraction = std::min(1.0, remaining / length);
    target = {start.x + fraction * (path[i + 1].x - start.x),
      start.y + fraction * (path[i + 1].y - start.y)};
    remaining -= fraction * length;
    if (remaining <= 1e-9) {break;}
  }
  double remaining_path = distance(projected, path[nearest + 1]);
  for (std::size_t i = nearest + 1; i + 1 < path.size(); ++i) {
    remaining_path += distance(path[i], path[i + 1]);
  }
  return {target, initial_heading, best, remaining_path, nearest + 2 == path.size()};
}

Core::Core(Settings cfg) : cfg_(cfg) {cfg_.validate();}

void Core::reset()
{
  aligning_ = true;
  heading_latched_ = false;
  held_heading_ = previous_yaw_ = 0.0;
  progress_ = {};
  lateral_ = false;
  axis_ = 0;
  previous_ = {};
  correction_axis_ = 0;
  correction_time_ = 0.0;
}

Result Core::continuous(const Pose & pose, const Velocity & measured, const RouteTarget & target,
  const Pose & goal, double dt, double speed_limit)
{
  const double dx = target.point.x - pose.x, dy = target.point.y - pose.y;
  const double distance_to_target = std::hypot(dx, dy);
  const double travel = distance_to_target > 1e-6 ? std::atan2(dy, dx) : target.heading;
  const double error = wrap(travel - pose.yaw);
  const double measured_speed = std::hypot(measured.x, measured.y);
  auto stop = [&](const std::string & mode) -> Result {
      previous_ = {}; previous_yaw_ = pose.yaw;
      return {{}, mode, travel, error};
    };
  // A changed path can demand a reversal. Brake before a stationary recovery;
  // never relax the forward cone, the final-turn guard, or the route corridor.
  if (measured_speed > cfg_.stopped_linear &&
    std::abs(std::atan2(measured.y, measured.x)) > cfg_.max_travel_angle)
  {
    heading_latched_ = false;
    return stop("travel_limit_stop");
  }
  if (!heading_latched_) {
    if (std::abs(error) > cfg_.max_travel_angle || std::abs(measured.yaw) > cfg_.stopped_angular) {
      if (measured_speed > cfg_.stopped_linear) {return stop("stop_before_turn");}
      if (std::abs(error) > cfg_.align_exit) {
        const double wanted = std::clamp(cfg_.heading_kp * error, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
        previous_ = {0, 0, approach(measured.yaw, wanted, cfg_.angular_accel * dt)};
        previous_.yaw = std::clamp(previous_.yaw, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
        previous_yaw_ = pose.yaw;
        return {previous_, "route_turn", travel, error};
      }
      return stop("settle_turn");
    }
    held_heading_ = pose.yaw; previous_ = {}; previous_yaw_ = pose.yaw;
    heading_latched_ = true;
  }
  if (std::abs(error) > kPi / 2) {
    heading_latched_ = false;
    return stop("stop_before_turn");
  }
  // Project the stored heading into an inner cone. Within that cone the body
  // reference is unchanged, even as travel changes from forward to diagonal.
  // Reserve tracking/actuation margin; yaw starts before the hard CLI limit.
  const double margin = std::min(15 * kPi / 180, cfg_.max_travel_angle / 3);
  const double soft = cfg_.max_travel_angle - margin;
  if (cfg_.heading_policy == "preview" && cfg_.round_corners && target.final_segment &&
    target.remaining <= cfg_.max_speed * cfg_.heading_preview_time &&
    std::hypot(goal.x - pose.x, goal.y - pose.y) <= cfg_.max_speed * cfg_.heading_preview_time &&
    std::abs(wrap(goal.yaw - travel)) <= soft) {
    // Finish heading alignment during the approach only when the final heading
    // itself admits the current translation inside the same conservative cone.
    // Final measured-stop/arrival checks remain unchanged.
    held_heading_ = goal.yaw;
  }
  const double relative = wrap(travel - held_heading_);
  if (std::abs(relative) > soft) {held_heading_ = wrap(travel - std::copysign(soft, relative));}
  const double heading_error = wrap(held_heading_ - pose.yaw);
  const double wanted_yaw = std::clamp(cfg_.heading_kp * heading_error, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
  double yaw = std::clamp(approach(previous_.yaw, wanted_yaw, cfg_.angular_accel * dt),
      -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
  // Also keep commanded travel within the cone after angular latency/braking.
  // The live footprint guard separately encloses linear/rotational stopping.
  const double w0 = std::min({0.0, measured.yaw, yaw}), w1 = std::max({0.0, measured.yaw, yaw});
  const double low = -cfg_.max_travel_angle + margin / 2 + w1 * .5 + w1 * w1 / (2 * cfg_.angular_accel);
  const double high = cfg_.max_travel_angle - margin / 2 + w0 * .5 - w0 * w0 / (2 * cfg_.angular_accel);
  if (low > high) {heading_latched_ = false; return stop("travel_limit_stop");}
  const double direction = std::clamp(error, low, high);
  const double limit = std::min(speed_limit, cfg_.max_speed);
  // Slow as the pursuit target moves outside the permitted direction. A
  // right-angle route corner can be rounded progressively within live clearance.
  const double speed = std::min(limit, cfg_.linear_kp * distance_to_target) *
    std::max(0.0, std::cos(error - direction));
  Velocity desired{speed * std::cos(direction), speed * std::sin(direction), yaw};
  const double rotation = wrap(previous_yaw_ - pose.yaw);
  Velocity old{std::cos(rotation) * previous_.x - std::sin(rotation) * previous_.y,
    std::sin(rotation) * previous_.x + std::cos(rotation) * previous_.y, previous_.yaw};
  // This cone is convex (<90 degrees). Blending two vectors inside it preserves
  // the direction bound and limits total acceleration, not each axis separately.
  if (std::hypot(old.x, old.y) > cfg_.stopped_linear &&
    (std::atan2(old.y, old.x) < low || std::atan2(old.y, old.x) > high)) {
    heading_latched_ = false; return stop("travel_limit_stop");
  }
  const double delta = std::hypot(desired.x - old.x, desired.y - old.y);
  const double fraction = delta > 0 ? std::min(1.0, cfg_.linear_accel * dt / delta) : 1.0;
  previous_ = {old.x + fraction * (desired.x - old.x), old.y + fraction * (desired.y - old.y), yaw};
  const double norm = std::hypot(previous_.x, previous_.y);
  if (norm > limit) {previous_.x *= limit / norm; previous_.y *= limit / norm;}
  // Avoid rotation-dominated module commands as translation ramps from rest.
  // With module radius about .13 m, |w| <= 3|v| leaves translation dominant;
  // final/stationary turns retain their separate measured-stop phases.
  const double mixed_yaw_limit = std::min(cfg_.max_yaw_rate, 3 * std::hypot(previous_.x, previous_.y));
  previous_.yaw = std::clamp(previous_.yaw, -mixed_yaw_limit, mixed_yaw_limit);
  previous_yaw_ = pose.yaw;
  return {previous_, "continuous", held_heading_, heading_error};
}

Result Core::step(const Pose & pose, const Velocity & measured, const std::vector<Point> & path,
  const Pose & goal, double dt, double speed_limit)
{
  if (!finite(pose) || !finite(goal) || !finite({measured.x, measured.y, measured.yaw}) ||
    !std::isfinite(dt) || dt <= 0.0 || !std::isfinite(speed_limit) || speed_limit < 0.0)
  {reset(); throw std::invalid_argument("Invalid body-route state/time/speed limit");}
  if (dt > cfg_.max_dt) {reset(); return {{}, "clock_gap", pose.yaw, 0.0};}
  const auto target = routeTarget(path, pose, cfg_, &progress_);
  const double goal_distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  const bool final = goal_distance <= cfg_.goal_xy && target.remaining <= cfg_.goal_xy;
  // Exact-corner continuous fallback must align to the same pursuit bearing
  // that continuous() will use. Near the endpoint, cross-track error can put
  // that bearing outside the tangent's cone; alternating the two stalls forever.
  const bool pursuit_alignment = cfg_.drive_style == "continuous" && !cfg_.round_corners &&
    std::hypot(target.point.x - pose.x, target.point.y - pose.y) > 1e-6;
  const double heading = final ? goal.yaw : pursuit_alignment ?
    std::atan2(target.point.y - pose.y, target.point.x - pose.x) : target.heading;
  const double error = wrap(heading - pose.yaw);
  const bool stopped = std::hypot(measured.x, measured.y) <= cfg_.stopped_linear;
  auto stop = [&](const std::string & reason) -> Result {
      previous_ = {}; return {{}, reason, heading, error};
    };
  if (speed_limit == 0.0) {reset(); return stop("speed_limit_stop");}
  if (cfg_.drive_style == "continuous" && cfg_.round_corners && !final) {
    return continuous(pose, measured, target, goal, dt, speed_limit);
  }
  if (final && std::abs(error) <= cfg_.goal_yaw) {return stop("goal_stop");}
  if (final || std::abs(error) > cfg_.align_enter) {aligning_ = true;}
  if (aligning_) {
    heading_latched_ = false;
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
  if (cfg_.drive_style == "continuous") {return continuous(pose, measured, target, goal, dt, speed_limit);}
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
