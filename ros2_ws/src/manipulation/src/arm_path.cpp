// arm_path.cpp — 실물 경로 → 시간표 · 동작 목록 (설명은 include/manipulation/arm_path.hpp)

#include "manipulation/arm_path.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace manipulation
{

namespace
{
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kGridDeg = 0.1;   // 속도 계획 간격 [°]

double span_deg(const std::vector<double> & a, const std::vector<double> & b)
{
  double m = 0.0;
  for (std::size_t j = 0; j < a.size() && j < b.size(); ++j) {m = std::max(m, std::fabs(b[j] - a[j]) * kRad2Deg);}
  return m;
}

// 꺾이는 곳 통과 속도 비율 cos²(θ/2): 일직선 1 · 90° 0.5 · 거꾸로 0
double corner_factor(const std::vector<double> & a, const std::vector<double> & b, const std::vector<double> & c)
{
  double d1d2 = 0.0, n1 = 0.0, n2 = 0.0;
  for (std::size_t j = 0; j < a.size(); ++j) {
    const double d1 = b[j] - a[j], d2 = c[j] - b[j];
    d1d2 += d1 * d2;
    n1 += d1 * d1;
    n2 += d2 * d2;
  }
  if (n1 < 1e-18 || n2 < 1e-18) {return 0.0;}
  const double cosb = std::clamp(d1d2 / std::sqrt(n1 * n2), -1.0, 1.0);
  const double h = std::cos(std::acos(cosb) / 2.0);
  return h * h;
}

struct Cell
{
  std::size_t seg;   // 구간 번호
  double frac;       // 구간 안 비율 (칸 시작)
  double vmax;       // 속도 상한 [°/s]
  double len;        // 칸 길이 [°]
};
}  // namespace

std::vector<TrajPoint> time_path(
  const std::vector<double> & start, const std::vector<PathNode> & nodes, const PathTiming & timing)
{
  if (nodes.empty()) {return {};}
  std::vector<const std::vector<double> *> q{&start};
  for (const auto & n : nodes) {q.push_back(&n.q);}
  const std::size_t segs = nodes.size();
  std::vector<double> L(segs);
  double total = 0.0;
  for (std::size_t k = 0; k < segs; ++k) {
    L[k] = span_deg(*q[k], *q[k + 1]);
    total += L[k];
  }
  if (total < 1e-6) {return {{nodes.back().q, 0.3}};}

  std::vector<Cell> grid;
  std::vector<std::size_t> starts;
  for (std::size_t k = 0; k < segs; ++k) {
    double vmax = nodes[k].stand ? timing.stand_speed_dps : timing.speed_dps;
    if (nodes[k].speed_dps > 0.0) {vmax = std::min(vmax, nodes[k].speed_dps);}
    const auto n = static_cast<std::size_t>(std::max(1.0, std::ceil(L[k] / kGridDeg)));
    starts.push_back(grid.size());
    for (std::size_t j = 0; j < n; ++j) {grid.push_back({k, static_cast<double>(j) / n, vmax, L[k] / n});}
  }
  grid.push_back({segs - 1, 1.0, 0.0, 0.0});

  std::vector<double> v(grid.size());
  for (std::size_t i = 0; i < grid.size(); ++i) {v[i] = grid[i].vmax;}
  v[0] = 0.0;
  for (std::size_t k = 0; k + 1 < segs; ++k) {
    const std::size_t p = starts[k + 1];
    v[p] = std::min(grid[p - 1].vmax, grid[p].vmax) * corner_factor(*q[k], *q[k + 1], *q[k + 2]);
  }
  for (std::size_t i = 1; i < v.size(); ++i) {
    v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2.0 * timing.accel_dps2 * grid[i - 1].len));
  }
  for (std::size_t i = v.size() - 1; i-- > 0;) {
    v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2.0 * timing.accel_dps2 * grid[i].len));
  }
  std::vector<double> T{0.0};
  for (std::size_t i = 1; i < v.size(); ++i) {
    T.push_back(T.back() + grid[i - 1].len / std::max((v[i - 1] + v[i]) / 2.0, 1e-3));
  }

  std::vector<TrajPoint> out;
  const auto n = static_cast<std::size_t>(std::max(1.0, std::ceil(T.back() / timing.dt_s)));
  std::size_t i = 0;
  for (std::size_t m = 1; m <= n; ++m) {
    const double t = T.back() * static_cast<double>(m) / static_cast<double>(n);
    while (i + 2 < T.size() && T[i + 1] < t) {++i;}
    const double w = T[i + 1] <= T[i] ? 0.0 : std::min(1.0, (t - T[i]) / (T[i + 1] - T[i]));
    const std::size_t k = grid[i].seg;
    const double f0 = grid[i].frac;
    const double f1 = grid[i + 1].seg == k ? grid[i + 1].frac : 1.0;
    const double f = f0 + (f1 - f0) * w;
    std::vector<double> p(q[k]->size());
    for (std::size_t j = 0; j < p.size(); ++j) {p[j] = (*q[k])[j] + ((*q[k + 1])[j] - (*q[k])[j]) * f;}
    out.push_back({p, t});
  }
  return out;
}

std::vector<ArmAction> path_actions(const ArmPath & path, const PathTiming & timing, const GripperValues & grip)
{
  std::vector<ArmAction> out;
  if (path.nodes.size() < 2) {return out;}
  std::vector<double> from = path.nodes.front().q;
  std::vector<PathNode> seg;
  auto flush = [&](const PathNode & last, bool is_last) {
      ArmAction a{"", false, "", 0.0};
      a.phase = last.grip == "release" ? "release" : (last.grip == "close" ? "grasp" : (is_last && path.ends_home ? "retreat" : "approach"));
      a.pose = is_last && path.ends_home ? "home" : last.name;
      a.points = time_path(from, seg, timing);
      a.duration_s = a.points.empty() ? 0.0 : a.points.back().t;
      out.push_back(a);
      from = last.q;
      seg.clear();
    };
  for (std::size_t i = 1; i < path.nodes.size(); ++i) {
    const PathNode & n = path.nodes[i];
    seg.push_back(n);
    const bool is_last = i + 1 == path.nodes.size();
    if (n.stop || !n.grip.empty() || is_last) {
      flush(n, is_last);
      if (!n.grip.empty()) {
        const double g = n.grip == "open" ? grip.open : (n.grip == "release" ? grip.release : grip.close);
        out.push_back({n.grip == "close" ? "grasp" : "release", true, "", g});
      }
    }
  }
  // 그리퍼로 끝나면 ArmRunner 의 "마지막 동작이 home" 판정이 안 됨 → 받침 경로는 마지막이 팔 동작이어야 함
  return out;
}

bool path_start_ok(const ArmPath & path, const std::vector<double> & q_now, double tol, std::string & why)
{
  if (path.nodes.empty()) {why = "경로 " + path.name + " 비어 있음"; return false;}
  const auto & q0 = path.nodes.front().q;
  if (q_now.size() != q0.size()) {why = "관절 측정 수가 맞지 않음"; return false;}
  for (std::size_t j = 0; j < q0.size(); ++j) {
    if (!std::isfinite(q_now[j]) || std::fabs(q_now[j] - q0[j]) > tol) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "경로 %s 출발 자세(%s)와 관절 %zu 가 %.3f rad 다름 (허용 %.3f)",
        path.name.c_str(), path.nodes.front().name.c_str(), j + 1, q_now[j] - q0[j], tol);
      why = buf;
      return false;
    }
  }
  why.clear();
  return true;
}

}  // namespace manipulation
