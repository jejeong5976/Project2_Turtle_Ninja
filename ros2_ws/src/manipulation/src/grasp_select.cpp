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

std::vector<NeighborPuck> unbias_neighbors(const Eigen::Vector2d & p0, double half_t, std::vector<NeighborPuck> nb,
  double max_push, double puck_half, double square_half)
{
  for (auto & n : nb) {
    const V2 v = n.c - p0;
    if (!v.allFinite() || v.norm() < 1e-6) {continue;}
    const double half_n = n.radius > 0.0 ? n.radius : (n.square ? square_half : puck_half);
    const double g = v.norm() - half_t - half_n;
    if (g < 0.0) {n.c += v.normalized() * std::min(-g, max_push);}
  }
  return nb;
}

double spot_gap(const Eigen::Vector2d & s, double half_t, const std::vector<NeighborPuck> & nb, double puck_half, double square_half)
{
  double best = 1e9;
  for (const auto & p : nb) {
    if (!p.c.allFinite()) {continue;}
    double d;
    if (p.radius > 0.0) {
      d = (s - p.c).norm() - p.radius;
    } else if (p.square && p.yaw_known) {
      const V2 l = rot(s - p.c, -p.yaw);
      const double ex = std::fabs(l.x()) - square_half, ey = std::fabs(l.y()) - square_half;
      d = ex <= 0.0 && ey <= 0.0 ? std::max(ex, ey) : point_rect(l, V2(square_half, square_half));   // 안이면 가까운 면까지 (음수)
    } else {
      d = (s - p.c).norm() - (p.square ? square_half * std::sqrt(2.0) : puck_half);
    }
    best = std::min(best, d - half_t);
  }
  return best;
}

double spot_edge_short(const Eigen::Vector2d & s, double min_x, double max_abs_y, double margin,
  const std::vector<Eigen::Vector2d> & known, double band)
{
  if (!std::isfinite(margin) || margin <= 0.0) {return 0.0;}
  double front = std::max(0.0, min_x + margin - s.x()), side = std::max(0.0, std::fabs(s.y()) - (max_abs_y - margin));
  for (const auto & k : known) {
    if (!k.allFinite()) {continue;}
    if (s.x() >= k.x() - 1e-3 && std::fabs(s.y() - k.y()) <= band) {front = 0.0;}
    if (s.y() * k.y() >= 0.0 && std::fabs(s.y()) <= std::fabs(k.y()) + 1e-3 && std::fabs(s.x() - k.x()) <= band) {side = 0.0;}
  }
  return front + side;
}

int spot_tier(const RegraspSpot & s, double need)
{
  if (s.gap < need) {return 3;}
  return s.edge_short > 1e-9 ? 2 : 1;
}

std::vector<RegraspSpot> order_regrasp_spots(std::vector<RegraspSpot> spots, double need, const Eigen::Vector2d * keep_first)
{
  // 비교는 칸으로 (가장자리 2 mm, 사이 1 mm): 후보 격자가 J1 방향으로 돌아 생기는 0.1 mm 차이로 순서가 바뀌지 않게 — 같은 칸이면 pref
  auto bin = [](double v, double w) {return static_cast<long>(std::floor(std::clamp(v, -1.0, 1.0) / w));};
  std::stable_sort(spots.begin(), spots.end(), [&](const RegraspSpot & a, const RegraspSpot & b) {
      const int ta = spot_tier(a, need), tb = spot_tier(b, need);
      if (ta != tb) {return ta < tb;}
      if (ta == 3) {
        const long ka = bin(a.gap - a.edge_short, 0.001), kb = bin(b.gap - b.edge_short, 0.001);
        if (ka != kb) {return ka > kb;}
      }
      if (ta == 2) {   // 가장자리 여유가 덜 모자란 칸 (5 mm) 부터 — 박스가 돌아 놓여도 벽 위에 안 놓게
        const long ea = bin(a.edge_short, 0.005), eb = bin(b.edge_short, 0.005);
        if (ea != eb) {return ea < eb;}
      }
      const long ra = bin(a.reach, 0.005), rb = bin(b.reach, 0.005);
      if (ra != rb) {return ra < rb;}
      return a.pref < b.pref;
    });
  if (keep_first && keep_first->allFinite()) {   // 원래 자리 (층 1 · 2) 는 맨 앞 그대로
    const auto it = std::find_if(spots.begin(), spots.end(), [&](const RegraspSpot & s) {return (s.p - *keep_first).norm() < 1e-6;});
    if (it != spots.end() && spot_tier(*it, need) <= 2) {std::rotate(spots.begin(), it, it + 1);}
  }
  return spots;
}

std::vector<Eigen::Vector2d> regrasp_spot_points(const Eigen::Vector2d & j1, const Eigen::Vector2d & p0, double min_x, double max_abs_y,
  bool origin_first, double out_m)
{
  std::vector<V2> out;
  // tol: 앞 후보와 이만큼 안이면 뺌 (기본 후보끼리는 같은 점만, 넓힌 후보는 5 mm — 기본 후보를 밀어내지 않게 먼저 넣음)
  auto add = [&](const V2 & q, double tol) {
      if (!q.allFinite() || q.x() < min_x || std::fabs(q.y()) > max_abs_y) {return;}
      for (const auto & o : out) {if ((o - q).norm() < tol) {return;}}
      out.push_back(q);
    };
  const double r0 = (p0 - j1).norm();
  if (r0 < 1e-6) {return out;}
  const V2 dir = (p0 - j1) / r0, side(-dir.y(), dir.x());
  if (origin_first) {add(p0, 1e-6);}
  const double r_near = dir.x() > 0.1 ? (min_x + 0.002 - j1.x()) / dir.x() : r0 - 0.01;   // x = min_x + 2 mm 가 되는 반지름
  const double r_lo = std::min(r_near, r0 - 0.01), r_hi = std::max(r0 - 0.01, r0 + out_m);
  // 기본 후보: 반지름 r_lo … 퍽 1 cm 앞 × 옆 0 · ±4 · ±8 cm → 박스 앞쪽 끝을 따라 가운데 (y = 0) 쪽으로 (구석 퍽)
  for (double r = r_lo; r <= r0 - 0.01 + 1e-9; r += 0.01) {
    for (double lat : {0.0, 0.04, -0.04, 0.08, -0.08}) {add(j1 + dir * r + side * lat, 1e-6);}
  }
  for (double dx = 0.002; dx <= 0.0221; dx += 0.01) {
    const double x = min_x + dx, y_line = std::fabs(dir.x()) > 0.1 ? j1.y() + dir.y() * (x - j1.x()) / dir.x() : p0.y();
    for (double y = y_line; std::fabs(y) > 1e-9 && y * y_line > 0.0; y -= std::copysign(0.01, y_line)) {add(V2(x, y), 1e-6);}
  }
  // 넓힌 후보: 반지름 (기본 줄 + 박스 앞쪽 끝 줄 r_near 부터 1 cm 씩, 퍽 너머 out_m 까지) × 옆 0 · ±2 … ±16 cm, 옆 끝 (|y| = max_abs_y) 점.
  //   (퍽이 앞쪽 끝에 있으면 기본 줄은 박스 앞 (x < min_x) 이라 옆 자리가 없음)
  std::vector<double> rs;
  for (double r = r_lo; r <= r_hi + 1e-9; r += 0.01) {rs.push_back(r);}
  for (double r = r_near; r <= r_hi + 1e-9; r += 0.01) {rs.push_back(r);}
  std::stable_sort(rs.begin(), rs.end());
  for (double r : rs) {
    const V2 base = j1 + dir * r;
    add(base, 0.005);
    bool edge_done[2] = {false, false};
    for (int k = 1; k <= 8; ++k) {
      for (int i = 0; i < 2; ++i) {
        const double sg = i == 0 ? 1.0 : -1.0;
        const V2 q = base + side * (sg * 0.02 * k);
        if (std::fabs(q.y()) <= max_abs_y) {add(q, 0.005); continue;}
        // 옆 끝을 넘음 → 그 줄의 |y| = max_abs_y 점 (한 번)
        if (!edge_done[i] && std::fabs(side.y()) > 0.3) {
          edge_done[i] = true;
          const double lat = (std::copysign(max_abs_y - 1e-6, q.y()) - base.y()) / side.y();
          if (lat * sg > 0.0) {add(base + side * lat, 0.005);}
        }
      }
    }
  }
  if (!origin_first) {add(p0, 0.005);}
  return out;
}

}  // namespace manipulation
