// lying_backoff.cpp — 누운 퍽 뒤로 빼기 계산 (lying_backoff.hpp)

#include "mission/lying_backoff.hpp"

#include <algorithm>
#include <cmath>

namespace mission
{

std::optional<LyingBackoffChoice> lying_backoff_choice(const std::vector<LyingCandidate> & picks, const LyingBackoffConfig & cfg)
{
  const LyingCandidate * best = nullptr;
  for (const auto & p : picks) {
    const double lim = (p.lying || p.bad) ? cfg.near_x_m : cfg.up_near_x_m;
    if (lim <= 0.0 || !std::isfinite(p.x) || p.x >= lim) {continue;}
    if (!best || p.x < best->x) {best = &p;}
  }
  if (!best) {return std::nullopt;}
  const double d = std::min(cfg.max_m, cfg.target_x_m - best->x);
  if (!(d >= cfg.min_m)) {return std::nullopt;}
  return LyingBackoffChoice{best->id, best->x, best->lying, d, best->bad && !best->lying};
}

StraightBackCmd straight_back_cmd(double x0, double y0, double yaw0, double goal_m, double x, double y, double yaw,
  const StraightBackConfig & cfg)
{
  StraightBackCmd c;
  const double dx = x - x0, dy = y - y0;
  const double cs = std::cos(yaw0), sn = std::sin(yaw0);
  c.traveled = -(dx * cs + dy * sn);
  c.lateral = -dx * sn + dy * cs;
  c.dyaw = std::remainder(yaw - yaw0, 2.0 * M_PI);
  c.remaining = goal_m - c.traveled;
  if (c.remaining <= cfg.tol_m) {
    c.done = true;
    return c;
  }
  const double v = std::clamp(cfg.k_lin * c.remaining, cfg.v_min, cfg.v_max);
  // 몸 좌표: 지금 방향이 시작 방향에서 dyaw 돎 → 시작 방향 기준 (−v, −k·lateral) 을 몸으로 돌림
  const double ux = -v, uy = std::clamp(-cfg.k_lat * c.lateral, -cfg.vy_max, cfg.vy_max);
  const double cd = std::cos(c.dyaw), sd = std::sin(c.dyaw);
  c.vx = cd * ux + sd * uy;
  c.vy = -sd * ux + cd * uy;
  c.wz = std::clamp(-cfg.k_yaw * c.dyaw, -cfg.wz_max, cfg.wz_max);
  return c;
}

}  // namespace mission
