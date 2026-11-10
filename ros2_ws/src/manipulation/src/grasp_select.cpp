// grasp_select.cpp — 이웃 퍽을 보고 닫힘 방향 · 벌림 폭 고르기 (설명은 include/manipulation/grasp_select.hpp)

#include "manipulation/grasp_select.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace manipulation
{

namespace
{
using V2 = Eigen::Vector2d;

// 점 p 에서 축 정렬 사각 (중심 0, 반 크기 h) 까지 거리 (안이면 0)
double point_rect(const V2 & p, const V2 & h)
{
  const double dx = std::max(std::fabs(p.x()) - h.x(), 0.0), dy = std::max(std::fabs(p.y()) - h.y(), 0.0);
  return std::hypot(dx, dy);
}

V2 rot(const V2 & v, double a) {return {std::cos(a) * v.x() - std::sin(a) * v.y(), std::sin(a) * v.x() + std::cos(a) * v.y()};}

std::array<V2, 4> corners(const V2 & c, const V2 & h, double a)
{
  const std::array<V2, 4> s{V2(1, 1), V2(1, -1), V2(-1, -1), V2(-1, 1)};
  std::array<V2, 4> out;
  for (int k = 0; k < 4; ++k) {out[k] = c + rot(s[k].cwiseProduct(h), a);}
  return out;
}

// 분리축 (두 사각의 변 법선 4 개) 로 겹침 판정
bool rects_overlap(const std::array<V2, 4> & A, const std::array<V2, 4> & B, double aa, double ab)
{
  for (double ang : {aa, aa + M_PI / 2.0, ab, ab + M_PI / 2.0}) {
    const V2 n(std::cos(ang), std::sin(ang));
    double a0 = 1e9, a1 = -1e9, b0 = 1e9, b1 = -1e9;
    for (const auto & p : A) {a0 = std::min(a0, n.dot(p)); a1 = std::max(a1, n.dot(p));}
    for (const auto & p : B) {b0 = std::min(b0, n.dot(p)); b1 = std::max(b1, n.dot(p));}
    if (a1 < b0 || b1 < a0) {return false;}
  }
  return true;
}

// 사각 A (중심 ca, 반 크기 ha, 방향 aa) ↔ 사각 B 최소 거리 (겹치면 0): 겹침은 분리축, 거리는 꼭짓점 ↔ 상대 사각의 최소
double rect_rect(const V2 & ca, const V2 & ha, double aa, const V2 & cb, const V2 & hb, double ab)
{
  const auto A = corners(ca, ha, aa), B = corners(cb, hb, ab);
  if (rects_overlap(A, B, aa, ab)) {return 0.0;}
  double d = std::numeric_limits<double>::infinity();
  for (int k = 0; k < 4; ++k) {
    d = std::min(d, point_rect(rot(A[k] - cb, -ab), hb));
    d = std::min(d, point_rect(rot(B[k] - ca, -aa), ha));
  }
  return d;
}

// 닫힘축은 180° 주기 → 기본 방향과 차이
double axis_off(double a, double ref) {return std::fabs(std::remainder(a - ref, M_PI));}

// 사각 (중심 c, 반 크기 h, 방향 a) ↔ 이웃 퍽 하나 거리 (겹치면 0 이하). 원기둥 = 반지름 puck_half, 네모 = 반폭 square_half 정사각 (면 방향 모르면 외접원)
double rect_puck(const V2 & c, const V2 & h, double a, const NeighborPuck & p, double puck_half, double slop, double square_half)
{
  if (p.radius > 0.0) {return point_rect(rot(p.c - c, -a), h) - (p.radius + slop);}
  if (p.square && p.yaw_known) {return rect_rect(c, h, a, p.c, V2(square_half + slop, square_half + slop), p.yaw);}
  const double r = (p.square ? square_half * std::sqrt(2.0) : puck_half) + slop;
  return point_rect(rot(p.c - c, -a), h) - r;
}
}  // namespace

double finger_clearance(
  const Eigen::Vector2d & center, double close_rad, double grip, const std::vector<NeighborPuck> & nb,
  const FingerTip & f, double puck_half, double slop, double square_half)
{
  double best = std::numeric_limits<double>::infinity();
  const V2 hf(0.5 * (f.outer0 - f.inner0), 0.5 * (f.w_hi - f.w_lo));   // 손가락 반 크기 (닫힘축, 폭)
  for (int side : {1, -1}) {
    // 손가락 중심 (손 좌표): 닫힘축 side · ((안쪽 + 바깥)/2 + grip), 폭 (w_lo + w_hi)/2
    const V2 local(side * (0.5 * (f.inner0 + f.outer0) + grip), 0.5 * (f.w_lo + f.w_hi));
    const V2 cf = center + rot(local, close_rad);
    for (const auto & p : nb) {
      if (!std::isfinite(p.c.x()) || !std::isfinite(p.c.y())) {continue;}
      best = std::min(best, rect_puck(cf, hf, close_rad, p, puck_half, slop, square_half));
    }
  }
  return best;
}

double footprint_clearance(
  const Eigen::Vector2d & center, double axis_rad, const Eigen::Vector2d & half, const std::vector<NeighborPuck> & nb,
  double puck_half, double slop, double square_half)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & p : nb) {
    if (!std::isfinite(p.c.x()) || !std::isfinite(p.c.y())) {continue;}
    best = std::min(best, rect_puck(center, half, axis_rad, p, puck_half, slop, square_half));
  }
  return best;
}

double target_half_along(const TargetPuck & t, double close_rad, double puck_half, double square_half)
{
  if (!t.square) {return puck_half;}
  if (!t.yaw_known) {return square_half * std::sqrt(2.0);}
  const double d = close_rad - t.yaw;
  return square_half * (std::fabs(std::cos(d)) + std::fabs(std::sin(d)));   // 닫힘축 방향으로 본 사각의 반 길이 (17.4·(|cos δ|+|sin δ|))
}

std::vector<double> close_candidates(bool square_yaw_known, double yaw, double default_close)
{
  std::vector<double> out;
  if (square_yaw_known) {
    out = {yaw, yaw + M_PI / 2.0};
  } else {
    for (int k = 0; k < 12; ++k) {out.push_back(default_close + k * M_PI / 12.0);}
  }
  return out;
}

std::vector<GraspChoice> rank_grasps(
  const Eigen::Vector2d & center, const TargetPuck & target, const std::vector<double> & closes, const std::vector<double> & grips,
  const std::vector<NeighborPuck> & nb, double default_close, std::string & why, const GraspLimits & lim, const FingerTip & f)
{
  std::vector<GraspChoice> out;
  double best_nb = -std::numeric_limits<double>::infinity();
  bool any_target_ok = false;
  for (double g : grips) {   // 넓은 벌림부터 (그 폭 안에서 순서를 정해 붙임)
    std::vector<GraspChoice> same;
    for (double c : closes) {
      if (target.gate && f.inner0 + g - target_half_along(target, c) - target.offset < lim.target_clear) {continue;}   // 목표 퍽 모서리 위로 내려감
      any_target_ok = true;
      const double clr = nb.empty() ? 1.0 : finger_clearance(center, c, g, nb, f);
      best_nb = std::max(best_nb, clr);
      if (clr < lim.min_clear) {continue;}
      same.push_back({true, c, g, clr, ""});
    }
    // 기본 방향 (가장 가까운 후보) 여유가 충분하면 맨 앞, 나머지는 여유 큰 순 · 같으면 기본 방향에 가까운 순
    std::stable_sort(same.begin(), same.end(), [&](const GraspChoice & a, const GraspChoice & b) {
        const bool da = axis_off(a.close_rad, default_close) < 1e-6 && a.clearance >= lim.prefer_default;
        const bool db = axis_off(b.close_rad, default_close) < 1e-6 && b.clearance >= lim.prefer_default;
        if (da != db) {return da;}
        if (std::fabs(a.clearance - b.clearance) > 1e-4) {return a.clearance > b.clearance;}
        return axis_off(a.close_rad, default_close) < axis_off(b.close_rad, default_close);
      });
    out.insert(out.end(), same.begin(), same.end());
  }
  if (out.empty()) {
    char buf[200];
    if (!any_target_ok) {
      std::snprintf(buf, sizeof(buf), "목표 퍽보다 손가락을 넓게 벌릴 수 있는 후보 없음 (모양 · 면 방향 확인)");
    } else {
      std::snprintf(buf, sizeof(buf), "이웃 퍽이 가까워 손가락이 들어갈 자리 없음 (가장 나은 여유 %.1f mm < %.1f mm)",
        best_nb * 1000.0, lim.min_clear * 1000.0);
    }
    why = buf;
  }
  return out;
}

GraspChoice choose_grasp(
  const Eigen::Vector2d & center, const TargetPuck & target, const std::vector<double> & closes, const std::vector<double> & grips,
  const std::vector<NeighborPuck> & nb, double default_close, const GraspLimits & lim, const FingerTip & f)
{
  std::string why;
  const auto r = rank_grasps(center, target, closes, grips, nb, default_close, why, lim, f);
  if (r.empty()) {GraspChoice g; g.why = why; return g;}
  return r.front();
}

}  // namespace manipulation
