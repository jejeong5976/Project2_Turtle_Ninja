#include "body_route_controller/observation.hpp"
#include <nav2_costmap_2d/cost_values.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace body_route_controller
{
namespace
{
double radius(const Footprint & f)
{
  double r = 0.0;
  for (const auto & p : f) {r = std::max(r, std::hypot(p.x, p.y));}
  return r;
}
double edgeDistance(Point p, Point a, Point b)
{
  const double dx = b.x - a.x, dy = b.y - a.y, n = dx * dx + dy * dy;
  const double t = n > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / n, 0.0, 1.0) : 0.0;
  return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}
bool touchesCell(const std::vector<Point> & poly, Point p, double half_diagonal)
{
  bool inside = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const auto a = poly[i], b = poly[j];
    if (edgeDistance(p, a, b) <= half_diagonal) {return true;}
    if ((a.y > p.y) != (b.y > p.y) &&
      p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) {inside = !inside;}
  }
  return inside;
}
double slow(double v, double step) {return std::copysign(std::max(0.0, std::abs(v) - step), v);}
}

bool visitFootprint(nav2_costmap_2d::Costmap2D & map, const Footprint & f, Pose pose,
  const std::function<bool(unsigned,unsigned)> & accept, bool only_blocking)
{
  if (f.size() < 3 || !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {return false;}
  std::vector<Point> poly;
  double xmin = std::numeric_limits<double>::infinity(), ymin = xmin;
  double xmax = -xmin, ymax = -xmin;
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  for (auto p : f) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return false;}
    Point q{pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y};
    poly.push_back(q);
    xmin = std::min(xmin, q.x); xmax = std::max(xmax, q.x);
    ymin = std::min(ymin, q.y); ymax = std::max(ymax, q.y);
  }
  unsigned int x0, y0, x1, y1;
  if (!map.worldToMap(xmin, ymin, x0, y0) || !map.worldToMap(xmax, ymax, x1, y1)) {return false;}
  const double half_diagonal = map.getResolution() / std::sqrt(2.0);
  for (unsigned int y = y0; y <= y1; ++y) {
    for (unsigned int x = x0; x <= x1; ++x) {
      if(only_blocking&&map.getCost(x,y)<nav2_costmap_2d::LETHAL_OBSTACLE){continue;}
      Point p;
      map.mapToWorld(x, y, p.x, p.y);
      if (touchesCell(poly, p, half_diagonal) && !accept(x,y)) {return false;}
    }
  }
  return true;
}

bool footprintSafe(nav2_costmap_2d::Costmap2D & map,const Footprint & f,const Pose & pose,CollisionWitness * witness)
{
  if(witness)*witness={-1,pose.x,pose.y,0};
  return visitFootprint(map,f,pose,[&](unsigned x,unsigned y){
    if(map.getCost(x,y)<nav2_costmap_2d::LETHAL_OBSTACLE)return true;
    if(witness){witness->cost=map.getCost(x,y);map.mapToWorld(x,y,witness->x,witness->y);}
    return false;
  },true);
}

bool motionSafe(nav2_costmap_2d::Costmap2D & map, const Footprint & f, Pose pose,
  Velocity v, double seconds, double linear_decel, double angular_decel, CollisionWitness * witness)
{
  if (!std::isfinite(seconds) || seconds < 0.0 || !std::isfinite(v.x) ||
    !std::isfinite(v.y) || !std::isfinite(v.yaw)) {return false;}
  const double extent_speed = std::hypot(v.x, v.y) + radius(f) * std::abs(v.yaw);
  const double step = std::min(0.05, map.getResolution() * 0.5 / std::max(extent_speed, 1e-6));
  const double count = std::ceil(seconds / step);
  if (!std::isfinite(count) || count > 10000) {return false;}
  const int n = std::max(1, static_cast<int>(count));
  const double dt = seconds / n;
  for (int i = 0; i <= n; ++i) {
    if (!footprintSafe(map, f, pose,witness)) {if(witness)witness->time=i*dt;return false;}
    const double c = std::cos(pose.yaw + v.yaw * dt / 2), s = std::sin(pose.yaw + v.yaw * dt / 2);
    pose.x += (c * v.x - s * v.y) * dt;
    pose.y += (s * v.x + c * v.y) * dt;
    pose.yaw = wrap(pose.yaw + v.yaw * dt);
    if (linear_decel > 0.0) {
      const double speed = std::hypot(v.x, v.y);
      const double ratio = speed > 0.0 ? std::max(0.0, speed - linear_decel * dt) / speed : 0.0;
      v.x *= ratio; v.y *= ratio;
    }
    if (angular_decel > 0.0) {v.yaw = slow(v.yaw, angular_decel * dt);}
  }
  return true;
}

bool turnSafe(nav2_costmap_2d::Costmap2D & map, const Footprint & f, Pose pose, double target_yaw)
{
  if (!std::isfinite(target_yaw)) {return false;}
  const double delta = wrap(target_yaw - pose.yaw);
  const double count = std::ceil(std::abs(delta) * radius(f) / (map.getResolution() * 0.5));
  if (!std::isfinite(count) || count > 10000) {return false;}
  const int n = std::max(1, static_cast<int>(count));
  const double start = pose.yaw;
  for (int i = 0; i <= n; ++i) {
    pose.yaw = start + delta * i / n;
    if (!footprintSafe(map, f, pose)) {return false;}
  }
  return true;
}
}
