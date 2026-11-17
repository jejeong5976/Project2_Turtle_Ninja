// placed_memory.cpp — 놓은 퍽 기억 (설명은 include/manipulation/placed_memory.hpp)
#include "manipulation/placed_memory.hpp"

#include <algorithm>
#include <cmath>

namespace manipulation
{

int expire_placed(std::vector<PlacedPuck> & v, const Eigen::Vector2d & robot_now, double now_s, double max_move, double max_age_s)
{
  const auto n0 = v.size();
  v.erase(std::remove_if(v.begin(), v.end(), [&](const PlacedPuck & e) {
      if (robot_now.allFinite() && std::isfinite(max_move) && (robot_now - e.robot).norm() > max_move) {return true;}
      return std::isfinite(max_age_s) && max_age_s > 0.0 && now_s - e.t > max_age_s;
    }), v.end());
  return static_cast<int>(n0 - v.size());
}

int forget_placed_near(std::vector<PlacedPuck> & v, const Eigen::Vector2d & p, double r)
{
  const auto n0 = v.size();
  v.erase(std::remove_if(v.begin(), v.end(), [&](const PlacedPuck & e) {return (e.p - p).norm() < r;}), v.end());
  return static_cast<int>(n0 - v.size());
}

std::vector<NeighborPuck> placed_neighbors(const std::vector<PlacedPuck> & v, const Eigen::Matrix2d & R, const Eigen::Vector2d & T,
  double dyaw, const Eigen::Vector2d & j1, double reach)
{
  std::vector<NeighborPuck> out;
  for (const auto & e : v) {
    NeighborPuck n;
    n.c = R * e.p + T;
    if (!n.c.allFinite() || (n.c - j1).norm() > reach) {continue;}
    n.square = e.square;
    n.yaw_known = e.square && e.yaw_known;
    n.yaw = e.yaw + dyaw;
    n.radius = e.radius;
    out.push_back(n);
  }
  return out;
}

std::vector<NeighborPuck> inflate_neighbors(std::vector<NeighborPuck> nb, double margin, double square_half)
{
  const double m = std::isfinite(margin) ? std::max(0.0, margin) : 0.0;
  for (auto & n : nb) {n.radius = std::max(n.radius, square_half) + m;}
  return nb;
}

std::vector<Eigen::Vector2d> far_putdown_points(const Eigen::Vector2d & j1, const Eigen::Vector2d & p0, double min_x, double max_abs_y)
{
  std::vector<Eigen::Vector2d> out;
  const double r0 = (p0 - j1).norm();
  if (r0 < 1e-6) {return out;}
  const Eigen::Vector2d dir = (p0 - j1) / r0, side(-dir.y(), dir.x());
  for (double r = r0 - 0.03; r > 0.0 && j1.x() + dir.x() * r >= min_x - 0.05; r -= 0.01) {
    for (double lat : {0.0, 0.04, -0.04, 0.08, -0.08}) {
      const Eigen::Vector2d q = j1 + dir * r + side * lat;
      if (q.x() < min_x || std::fabs(q.y()) > max_abs_y || (q - p0).norm() < 0.06) {continue;}   // 박스 밖 (앞 · 옆) · 목표와 가까움
      out.push_back(q);
    }
  }
  return out;
}

bool putdown_clear(const Eigen::Vector2d & s, double half_t, const std::vector<NeighborPuck> & placed_inflated)
{
  return placed_inflated.empty() || spot_gap(s, half_t, placed_inflated) >= 0.0;
}

bool far_put_left_after(const std::string & prev_pose, bool left, bool regrip_empty)
{
  if (prev_pose == "far_put") {return true;}
  if (prev_pose == "far_re_grasp") {return regrip_empty;}
  return left;
}

}  // namespace manipulation
