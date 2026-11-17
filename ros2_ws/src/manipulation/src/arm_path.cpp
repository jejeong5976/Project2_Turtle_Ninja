// arm_path.cpp — 실물 경로 → 시간표 · 동작 목록 (설명은 include/manipulation/arm_path.hpp)

#include "manipulation/arm_path.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>

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
  bool timed = true;
  for (const auto & n : nodes) {timed = timed && n.dt_s > 0.0;}
  if (timed) {   // 녹화 간격 그대로
    std::vector<TrajPoint> out;
    double t = 0.0;
    for (const auto & n : nodes) {
      t += n.dt_s;
      out.push_back({n.q, t});
    }
    return out;
  }
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
    // 0 이 아니면 2 칸 이상: 1 칸이면 양 끝 속도 0 이라 평균 속도가 0 (0.1° 이하 미세 이동이 수십 초가 됨)
    const auto n = L[k] > 0.0 ? static_cast<std::size_t>(std::max(2.0, std::ceil(L[k] / kGridDeg))) : std::size_t{1};
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

  // 출력 시각 = dt 간격 + 경유점 도착 시각 (경유점에서 dt/4 안의 등간격 점은 뺌)
  //   이웃 시각은 kMinGap 넘게 띄움: 끝 바로 앞 (1 ns 안) 경유점 도착 시각이 JTC 의 나노초 변환에서 끝 시각과 같아져
  //   "Time between points … is not strictly increasing" 으로 그 동작 전체가 거절됨
  constexpr double kMinGap = 1e-3;
  const auto n = static_cast<std::size_t>(std::max(1.0, std::ceil(T.back() / timing.dt_s)));
  std::vector<double> corner_t;
  for (std::size_t k = 1; k < segs; ++k) {corner_t.push_back(T[starts[k]]);}
  std::vector<double> times;
  std::size_t c = 0;
  for (std::size_t m = 1; m <= n; ++m) {
    const double t = T.back() * static_cast<double>(m) / static_cast<double>(n);
    while (c < corner_t.size() && corner_t[c] <= t) {
      const double last = times.empty() ? 0.0 : times.back();
      if (corner_t[c] > kMinGap && corner_t[c] > last + kMinGap && corner_t[c] < T.back() - kMinGap) {times.push_back(corner_t[c]);}
      ++c;
    }
    const double last = times.empty() ? 0.0 : times.back();
    if (m == n) {
      if (t - last <= kMinGap && !times.empty()) {times.back() = t;} else {times.push_back(t);}   // 끝 시각 = 마지막 점
    } else if (t - last > 0.25 * timing.dt_s) {
      times.push_back(t);
    }
  }

  std::vector<TrajPoint> out;
  std::size_t i = 0;
  for (const double t : times) {
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

std::vector<ArmAction> path_actions(
  const ArmPath & path, const PathTiming & timing, const GripperValues & grip, const std::vector<double> & q_now)
{
  std::vector<ArmAction> out;
  if (path.nodes.size() < 2) {return out;}
  bool measured = q_now.size() == path.nodes.front().q.size();
  for (double v : q_now) {measured = measured && std::isfinite(v);}
  std::vector<double> from = measured ? q_now : path.nodes.front().q;
  std::vector<PathNode> seg;
  double side_t = -1.0, side_g = 0.0, seg_t = 0.0;   // 팔을 안 멈추고 여는 그리퍼 (grip_on_way)
  int side_idx = -1;                                  // 그 경유점 (seg 안 번호) — 시간 안 맞춘 구간은 시간표에서 도착 시각을 찾음
  auto flush = [&](const PathNode & last, bool is_last) {
      ArmAction a{"", false, "", 0.0};
      a.phase = last.grip == "release" ? "release" : (last.grip == "close" ? "grasp" : (is_last && path.ends_home ? "retreat" : "approach"));
      a.pose = is_last && path.ends_home ? "home" : last.name;
      a.points = time_path(from, seg, timing);
      if (side_idx == -2) {side_t = 0.0;}   // 구간 처음 (출발할 때)
      if (side_idx >= 0 && side_t < 0.0 && !a.points.empty()) {   // 속도 시간표: 그 경유점에 가장 가까운 점의 시각 (경유점 도착 점이 들어 있음)
        double best = 1e18;
        for (const auto & p : a.points) {
          double d = 0.0;
          for (std::size_t j = 0; j < p.q.size(); ++j) {d += (p.q[j] - seg[side_idx].q[j]) * (p.q[j] - seg[side_idx].q[j]);}
          if (d < best) {best = d; side_t = p.t;}
        }
      }
      const double hold = std::max(last.hold_s, last.grip.empty() ? 0.0 : timing.grip_settle_s);   // 그리퍼 앞이면 팔이 다 따라오게
      if (hold > 0.0 && !a.points.empty()) {a.points.push_back({last.q, a.points.back().t + hold});}   // 같은 자세로 기다림
      a.duration_s = a.points.empty() ? 0.0 : a.points.back().t;
      a.side_grip_t = side_t;
      a.side_grip = side_g;
      out.push_back(a);
      from = last.q;
      seg.clear();
      side_t = -1.0;
      side_idx = -1;
      seg_t = 0.0;
    };
  for (std::size_t i = 1; i < path.nodes.size(); ++i) {
    const PathNode & n = path.nodes[i];
    seg.push_back(n);
    seg_t += n.dt_s;
    const bool is_last = i + 1 == path.nodes.size();
    // 끊지 않고 그리퍼만. stop 점은 settle (hold_s > 0: pick · place · store_settle_s) 로 멈춘 점만 그 시점에 열고 그 점에서 멈춤 —
    //   hold 없는 stop + 가며 그리퍼 점 (park · standup 이 만들 수 있음) 은 멈춤 → grip_settle_s → 차단 그리퍼
    if (!n.grip.empty() && (n.grip_on_way || n.grip_early) && (!n.stop || n.hold_s > 0.0) && !is_last && side_idx == -1) {
      bool timed = true;
      for (const auto & sn : seg) {timed = timed && sn.dt_s > 0.0;}
      if (n.grip_early) {   // 이 점으로 출발할 때 = 앞 점 도착 (구간 첫 점이면 구간 처음)
        side_idx = seg.size() >= 2 ? static_cast<int>(seg.size()) - 2 : -2;
        side_t = side_idx == -2 ? 0.0 : (timed ? seg_t - n.dt_s : -1.0);
      } else {
        side_t = timed ? seg_t : -1.0;   // 속도 시간표면 flush 에서 찾음
        side_idx = static_cast<int>(seg.size()) - 1;
      }
      side_g = n.grip == "open" ? grip.open : (n.grip == "release" ? grip.release : (n.grip == "loose" ? grip.loose : grip.close));
      seg.back().grip.clear();
      if (!n.stop) {continue;}
    }
    if (n.stop || !n.grip.empty() || is_last) {
      const PathNode last = seg.back();   // 가며 하는 그리퍼는 지워져 있음 → 멈춤 · hold 만, 그리퍼 동작은 안 만듦
      flush(last, is_last);
      if (!last.grip.empty()) {
        const double g = last.grip == "open" ? grip.open : (last.grip == "release" ? grip.release : (last.grip == "loose" ? grip.loose : grip.close));
        out.push_back({last.grip == "close" ? "grasp" : "release", true, "", g});
      }
    }
  }
  // 그리퍼로 끝나면 ArmRunner 의 "마지막 동작이 home" 판정이 안 됨 → 받침 경로는 마지막이 팔 동작이어야 함
  return out;
}

bool transit_node(const std::string & name)
{
  static const char * const names[] = {"re_place_above", "re_place_clear", "re_above", "re_lift", "reloc_put_above", "reloc_put_clear",
    "reloc_lift", "far_put_above", "far_put_clear", "far_re_above", "far_re_lift"};
  for (const char * n : names) {
    if (name == n) {return true;}
  }
  return false;
}

int set_transit_speed(std::vector<PathNode> & nodes, double dps)
{
  if (!(dps > 0.0)) {return 0;}
  int k = 0;
  for (auto & n : nodes) {
    if (!transit_node(n.name) || n.dt_s > 0.0 || n.stand || n.speed_dps == dps) {continue;}
    n.speed_dps = dps;
    ++k;
  }
  return k;
}

double arm_motion_time(const std::vector<ArmAction> & plan)
{
  double t = 0.0;
  for (const auto & a : plan) {
    if (!a.is_gripper && !a.points.empty()) {t += a.duration_s;}
  }
  return t;
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

namespace
{
bool same_nodes(const std::vector<PathNode> & a, std::size_t from, const std::vector<PathNode> & ref)
{
  if (ref.empty() || from + ref.size() > a.size()) {return false;}
  for (std::size_t i = 0; i < ref.size(); ++i) {
    const auto & p = a[from + i].q;
    const auto & r = ref[i].q;
    if (p.size() != r.size()) {return false;}
    for (std::size_t j = 0; j < p.size(); ++j) {
      if (std::fabs(p[j] - r[j]) > 1e-9) {return false;}
    }
  }
  return true;
}

// 잘라내도 되는 점: 그리퍼 동작 · 멈춤이 없음 (잘라내면 그 동작이 사라지므로)
bool plain(const std::vector<PathNode> & a, std::size_t from, std::size_t to)
{
  for (std::size_t i = from; i < to && i < a.size(); ++i) {
    if (!a[i].grip.empty() || a[i].stop) {return false;}
  }
  return true;
}
}  // namespace

bool cut_fold_tail(ArmPath & path, const ArmPath & fold)
{
  if (path.nodes.size() <= fold.nodes.size()) {return false;}
  const std::size_t k = path.nodes.size() - fold.nodes.size();
  if (!same_nodes(path.nodes, k, fold.nodes) || !plain(path.nodes, k + 1, path.nodes.size())) {return false;}
  path.nodes.resize(k + 1);
  path.ends_home = false;
  return true;
}

bool skip_unfold(ArmPath & path, const ArmPath & unfold)
{
  if (path.nodes.size() <= unfold.nodes.size() || !same_nodes(path.nodes, 0, unfold.nodes) ||
      !plain(path.nodes, 0, unfold.nodes.size())) {return false;}   // P0 (새 출발점) 까지 그리퍼 동작 없어야
  path.nodes.erase(path.nodes.begin(), path.nodes.begin() + static_cast<std::ptrdiff_t>(unfold.nodes.size() - 1));
  return true;
}

bool p0_distinct(const ArmPath & fold, const ArmPath & unfold, double tol)
{
  if (fold.nodes.empty() || unfold.nodes.empty()) {return false;}
  const auto & p0 = fold.nodes.front().q;
  const auto & rest = unfold.nodes.front().q;
  if (p0.size() != rest.size()) {return false;}
  for (std::size_t j = 0; j < p0.size(); ++j) {
    if (std::fabs(p0[j] - rest[j]) > tol) {return true;}
  }
  return false;
}

std::vector<PathNode> retrace_nodes(const std::vector<PathNode> & nodes, std::size_t from,
  const std::vector<std::string> & free_names, const std::function<bool(const PathNode &)> & is_exit, double same_tol)
{
  if (from >= nodes.size()) {return {};}
  auto same = [same_tol](const PathNode & a, const PathNode & b) {
      if (a.q.empty() || a.q.size() != b.q.size()) {return false;}
      for (std::size_t j = 0; j < a.q.size(); ++j) {
        if (!std::isfinite(a.q[j]) || !std::isfinite(b.q[j]) || std::fabs(a.q[j] - b.q[j]) > same_tol) {return false;}
      }
      return true;
    };
  auto plain_node = [](PathNode n) {   // 손은 빈 채 닫혀 있음 → 그리퍼 · 멈춤 · 시간 맞춤 없이 지나감
      n.grip.clear();
      n.stop = false;
      n.hold_s = 0.0;
      n.dt_s = 0.0;
      n.grip_on_way = false;
      n.grip_early = false;
      return n;
    };
  // ① 거꾸로 + 고리 빼기: 이미 지난 (남긴) 자세와 같은 점이 또 나오면 그 사이 (고리) 를 버리고 그 자세에서 이어 감
  std::vector<PathNode> kept{plain_node(nodes[from])};
  for (std::size_t i = from; i-- > 0;) {
    std::size_t loop = kept.size();
    for (std::size_t k = 0; k < kept.size(); ++k) {
      if (same(kept[k], nodes[i])) {loop = k; break;}
    }
    if (loop < kept.size()) {kept.resize(loop + 1); continue;}
    PathNode n = plain_node(nodes[i]);
    n.speed_dps = nodes[i + 1].speed_dps;   // 거꾸로 가는 구간 (→ nodes[i]) = 지나온 구간 nodes[i] → nodes[i + 1] 의 속도 · 받침 표시
    if (!(n.speed_dps > 0.0) && nodes[i + 1].dt_s > 0.0) {   // 시간 맞춘 구간 (표 재생 0.1 s 간격): 지나올 때 평균 속도 그대로 (기본 경로 속도로 빨라지지 않게)
      const double v = span_deg(nodes[i].q, nodes[i + 1].q) / nodes[i + 1].dt_s;
      if (std::isfinite(v) && v > 0.0) {n.speed_dps = v;}
    }
    n.stand = nodes[i + 1].stand;
    kept.push_back(n);
  }
  // ② 끝점 (검사된 복귀 경로가 있는 자세) 까지. 그 전에는 아는 빈 공간 점 (퍽 위 · 들어 올린 점) 만
  for (std::size_t k = 1; k < kept.size(); ++k) {
    if (is_exit(kept[k])) {
      kept.resize(k + 1);
      return kept;
    }
    if (std::find(free_names.begin(), free_names.end(), kept[k].name) == free_names.end()) {return {};}
  }
  return {};
}

namespace
{
// [위 점, 퍽 위, exit]: nodes[from] = 위 점 (up), 그 앞이 닫기 · 내림 (= 위 점 자세) · 퍽 위 (이름 차례 names = above, down, grasp, up)
std::vector<PathNode> up_above_exit(const std::vector<PathNode> & nodes, std::size_t from, const PathNode & exit,
  const char * const (&names)[4])
{
  if (from < 3 || from >= nodes.size() || exit.q.size() != 5) {return {};}
  const auto & up = nodes[from];
  const auto & grasp = nodes[from - 1];
  const auto & down = nodes[from - 2];
  const auto & above = nodes[from - 3];
  if (up.name != names[3] || grasp.name != names[2] || down.name != names[1] || above.name != names[0] || up.q.size() != 5 ||
    up.q != down.q || above.q.size() != 5)
  {
    return {};
  }
  auto plain = [](PathNode n, double speed) {
      n.grip.clear();
      n.stop = false;
      n.hold_s = 0.0;
      n.dt_s = 0.0;
      n.grip_on_way = false;
      n.grip_early = false;
      n.speed_dps = speed;
      return n;
    };
  return {plain(up, 0.0), plain(above, down.speed_dps), plain(exit, 0.0)};
}
}  // namespace

std::vector<PathNode> regrasp_retreat_nodes(const std::vector<PathNode> & nodes, std::size_t from, const PathNode & exit)
{
  static const char * const kRe[4] = {"re_above", "re_down", "re_grasp", "re_up"};
  return up_above_exit(nodes, from, exit, kRe);
}

std::vector<PathNode> park_retreat_nodes(const std::vector<PathNode> & nodes, std::size_t from, const PathNode & exit)
{
  static const char * const kPick[4] = {"above", "down", "grasp", "up"};
  static const char * const kReloc[4] = {"reloc_above", "reloc_down", "reloc_grasp", "reloc_up"};
  const bool park = std::any_of(nodes.begin(), nodes.end(), [](const PathNode & n) {return n.name.rfind("park_", 0) == 0;});
  if (!park) {return {};}   // 보통 PICK 은 retrace 로 됨 (여기 안 옴)
  auto r = up_above_exit(nodes, from, exit, kPick);
  return !r.empty() ? r : up_above_exit(nodes, from, exit, kReloc);
}

void add_joint_offsets(std::vector<double> & q, const std::array<double, 5> & d)
{
  if (q.size() < 5) {return;}
  for (std::size_t j = 0; j < 5; ++j) {
    if (d[j] != 0.0) {q[j] += d[j];}
  }
}

std::vector<PathNode> look_rack_back_join(const ArmPath & store, const ArmPath & look, const PathNode & look_start,
  const std::vector<double> & q_now, double tol, std::string & why, double same_tol, std::size_t * store_from_out)
{
  if (store.nodes.size() < 2 || look.nodes.size() < 2) {why = "칸 경로 · look_rack 경로 점 2 개 미만"; return {};}
  const std::size_t nq = store.nodes.front().q.size();
  auto ok_q = [nq](const std::vector<double> & q) {
      if (q.size() != nq || nq == 0) {return false;}
      for (double v : q) {if (!std::isfinite(v)) {return false;}}
      return true;
    };
  if (!ok_q(q_now) || !ok_q(look.nodes.back().q) || !ok_q(store.nodes[1].q) || !ok_q(look.nodes[1].q)) {why = "관절 수 다름 · NaN"; return {};}
  if (!path_start_ok(ArmPath{look.name, {look.nodes.back()}, false}, q_now, tol, why)) {why = "관찰 자세가 아님 (" + why + ")"; return {};}
  auto same = [same_tol](const std::vector<double> & a, const std::vector<double> & b) {
      if (a.size() != b.size()) {return false;}
      for (std::size_t j = 0; j < a.size(); ++j) {if (!(std::fabs(a[j] - b[j]) <= same_tol)) {return false;}}
      return true;
    };
  std::vector<std::string> names;   // 방금 실행한 경로 점 = 지나온 빈 공간
  for (const auto & n : look.nodes) {names.push_back(n.name);}
  const std::size_t from = look.nodes.size() - 1;
  std::vector<PathNode> out;
  std::size_t store_from = 0;
  // ① 칸 경로 점 (lift_real 부터, 같은 자세가 여럿이면 앞 점) 이 나올 때까지 거꾸로 — S3 처럼 look_rack 이 칸 경로를 따라가다 갈라지면 갈라진 점까지만
  auto on_store = [&](const PathNode & n) {
      for (std::size_t j = 1; j < store.nodes.size(); ++j) {if (same(n.q, store.nodes[j].q)) {return j;}}
      return std::size_t{0};
    };
  auto back = retrace_nodes(look.nodes, from, names, [&](const PathNode & n) {return on_store(n) > 0;});
  if (!back.empty()) {
    const std::size_t j = on_store(back.back());
    out.assign(back.begin(), back.end() - 1);
    PathNode s = store.nodes[j];
    s.speed_dps = back.back().speed_dps;   // 거꾸로 가는 구간 속도 (지나올 때 그 구간)
    s.stand = back.back().stand;
    out.push_back(s);
    store_from = j + 1;
  } else {
    // ② look_rack 둘째 점까지 거꾸로 → look_rack 첫 점 (= 칸 경로 첫 점일 때만)
    if (!ok_q(look_start.q) || !same(look_start.q, store.nodes.front().q)) {why = "look_rack 첫 점이 칸 경로 첫 점과 다름"; return {};}
    back = retrace_nodes(look.nodes, from, names, [&](const PathNode & n) {return same(n.q, look.nodes[1].q);});
    if (back.empty()) {why = "look_rack 을 거꾸로 못 감"; return {};}
    out = back;
    PathNode s = store.nodes.front();
    s.speed_dps = look.nodes[1].speed_dps;   // look_rack 첫 구간 (첫 점 → 둘째 점) 거꾸로
    s.stand = look.nodes[1].stand;
    out.push_back(s);
    store_from = 1;
  }
  out.insert(out.end(), store.nodes.begin() + static_cast<std::ptrdiff_t>(store_from), store.nodes.end());
  if (store_from_out) {*store_from_out = store_from;}
  out.front().q = q_now;
  out.front().name = "look_rack_end";
  why.clear();
  return out;
}

bool join_via_return(ArmPath & path, const ArmPath & returning, const ArmPath & fold, const ArmPath & unfold, bool p0_ok)
{
  ArmPath to_p0 = returning;
  ArmPath rest = path;
  if (p0_ok && cut_fold_tail(to_p0, fold) && skip_unfold(rest, unfold)) {
    to_p0.nodes.insert(to_p0.nodes.end(), rest.nodes.begin() + 1, rest.nodes.end());
    path.nodes = to_p0.nodes;
    return true;
  }
  std::vector<PathNode> nodes = returning.nodes;
  if (!path.nodes.empty()) {nodes.insert(nodes.end(), path.nodes.begin() + 1, path.nodes.end());}
  path.nodes = nodes;
  return false;
}

StorePick store_path_pick(bool held, bool over_look, bool look_ok, bool held_joinable)
{
  if (look_ok) {return held && over_look && held_joinable ? StorePick::kHeld : StorePick::kLook;}
  return held ? StorePick::kHeld : StorePick::kPlain;
}

bool look_rack_skip_for_store(const std::string & held_name, bool over_look, bool joinable_now)
{
  static const std::string d50 = "_d50";
  if (held_name.empty() || !over_look || !joinable_now) {return false;}
  const bool box50 = held_name.size() >= d50.size() && held_name.compare(held_name.size() - d50.size(), d50.size(), d50) == 0;
  return !box50;   // 박스 퍽 50 mm 는 기본 흐름 (관찰 → store_look_<칸> 이음)
}

bool park_store_join(ArmPath & path, const std::string & name, const std::vector<std::string> & table, std::string & to, std::size_t & skipped)
{
  to.clear();
  skipped = 0;
  if (path.nodes.size() < 3) {return false;}
  for (const auto & e : table) {
    const auto eq = e.find('=');
    if (eq == std::string::npos || e.compare(0, eq, name) != 0 || eq != name.size()) {continue;}
    const std::string node = e.substr(eq + 1);
    for (std::size_t k = 2; k < path.nodes.size(); ++k) {
      if (path.nodes[k].name != node) {continue;}
      if (!path.nodes[k].grip.empty()) {return false;}   // 그리퍼 점 (놓기 등) 으로는 안 이음
      for (std::size_t j = 1; j < k; ++j) {if (!path.nodes[j].grip.empty()) {return false;}}   // 건너뛸 점에 그리퍼 동작이 있으면 안 함
      path.nodes.erase(path.nodes.begin() + 1, path.nodes.begin() + static_cast<std::ptrdiff_t>(k));
      path.nodes[1].stand = false;
      to = node;
      skipped = k - 1;
      return true;
    }
    return false;
  }
  return false;
}

// ============================================================================ 도중 정지 뒤 HOME (home_retreat)
namespace
{
bool finite_q(const std::vector<double> & q)
{
  if (q.empty()) {return false;}
  for (const double v : q) {if (!std::isfinite(v)) {return false;}}
  return true;
}

double max_diff(const std::vector<double> & a, const std::vector<double> & b)
{
  double m = 0.0;
  for (std::size_t j = 0; j < a.size() && j < b.size(); ++j) {m = std::max(m, std::fabs(a[j] - b[j]));}
  return m;
}
}  // namespace

double segment_distance(const std::vector<double> & q, const std::vector<double> & a, const std::vector<double> & b)
{
  if (q.size() != a.size() || a.size() != b.size() || q.empty()) {return std::numeric_limits<double>::infinity();}
  double ab2 = 0.0, aq_ab = 0.0;
  for (std::size_t j = 0; j < q.size(); ++j) {
    ab2 += (b[j] - a[j]) * (b[j] - a[j]);
    aq_ab += (q[j] - a[j]) * (b[j] - a[j]);
  }
  const double t = ab2 > 1e-12 ? std::clamp(aq_ab / ab2, 0.0, 1.0) : 0.0;
  double m = 0.0;
  for (std::size_t j = 0; j < q.size(); ++j) {m = std::max(m, std::fabs(q[j] - (a[j] + t * (b[j] - a[j]))));}
  return std::isfinite(m) ? m : std::numeric_limits<double>::infinity();
}

void trace_append(std::vector<std::vector<double>> & trace, const std::vector<double> & q, double line_tol, std::size_t max_points,
  std::size_t keep_before)
{
  if (!finite_q(q) || (!trace.empty() && trace.back().size() != q.size())) {return;}
  if (!trace.empty() && max_diff(trace.back(), q) < 1e-4) {return;}
  if (trace.size() >= 2 && trace.size() - 1 >= keep_before && segment_distance(trace.back(), trace[trace.size() - 2], q) <= line_tol) {
    // 가운데 점이 양 끝 사이 일직선 위 (되돌아 나가는 꺾임은 아님: 가운데가 구간 안에 있을 때만 거리가 작음). keep_before 앞 점 (그리퍼 점) 은 안 지움
    trace.back() = q;
  } else {
    trace.push_back(q);
  }
  if (max_points >= 2 && trace.size() > max_points) {trace.erase(trace.begin(), trace.begin() + static_cast<std::ptrdiff_t>(trace.size() - max_points));}
}

std::vector<PathNode> retreat_from_trace(const std::vector<std::vector<double>> & trace, const std::vector<double> & q_now,
  double snap_tol, double loop_tol, const std::function<bool(const std::vector<double> &)> & is_exit, double speed_dps,
  std::string & why, std::size_t * exit_index, const std::vector<std::vector<double>> * blocked)
{
  const std::size_t n = trace.size();
  if (n == 0) {why = "지나온 길 기록 없음"; return {};}
  if (!finite_q(q_now) || q_now.size() != trace.front().size()) {why = "관절 실측 없음"; return {};}
  // 막힌 점: 퍽에 닿는 그리퍼 점 (닫기 · 놓기) — 그 점을 지나는 길은 퍽 · 칸 안으로 다시 들어감 (빼기 도중 멈췄을 때 넣은 깊이로 되돌아가지 않게)
  std::vector<char> bad(n, 0), exit(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    if (blocked) {
      for (const auto & b : *blocked) {if (b.size() == trace[i].size() && max_diff(b, trace[i]) <= loop_tol) {bad[i] = 1; break;}}
    }
    exit[i] = !bad[i] && is_exit(trace[i]);
  }
  // ① 출발: 지금 자세가 있는 구간 (snap_tol 안) 의 양 끝 (지나온 · 보낸 구간 위로만 움직임). 같은 구간이 여러 번 (내려갔다 올라옴) 이면 모두
  constexpr double kInf = std::numeric_limits<double>::infinity();
  std::vector<double> dist(n, kInf);
  std::vector<std::size_t> from(n, n);   // n = 출발 (지금 자세에서 옴)
  std::vector<char> jump(n, 0), done(n, 0);
  double best = kInf;
  auto seed = [&](std::size_t i) {
      if (bad[i]) {return;}
      const double c = max_diff(q_now, trace[i]);
      if (c < dist[i]) {dist[i] = c; from[i] = n;}
    };
  if (n == 1) {
    best = max_diff(q_now, trace[0]);
    if (best <= snap_tol) {seed(0);}
  } else {
    // 가장 가까운 구간만 (같은 명령 점을 다시 지난 구간 = 같은 거리, 1 mrad 안). 더 먼 구간의 끝점으로 곧장 가는 새 직선은 안 만듦
    std::vector<double> ds(n - 1);
    for (std::size_t i = 0; i + 1 < n; ++i) {
      ds[i] = segment_distance(q_now, trace[i], trace[i + 1]);
      best = std::min(best, ds[i]);
    }
    for (std::size_t i = 0; i + 1 < n; ++i) {
      if (ds[i] <= snap_tol && ds[i] <= best + 1e-3) {seed(i); seed(i + 1);}
    }
  }
  if (!(best <= snap_tol)) {
    char b[120];
    std::snprintf(b, sizeof(b), "지금 자세가 지나온 길에서 %.3f rad 벗어남 (허용 %.3f)", best, snap_tol);
    why = b;
    return {};
  }
  // ② 가장 짧은 길 (관절 이동, 관절마다 차이 중 큰 값의 합) 로 검사된 복귀 자세까지: 이웃 점 (지나온 구간, 어느 쪽으로든) + 같은 자세 (loop_tol 안, 0 —
  //   내려갔다 같은 점으로 올라온 고리를 건너뜀). 막힌 점은 안 지남. 바깥 (출구) 쪽이 짧으니 빼는 중이면 바깥으로 감
  std::size_t goal = n;
  for (;;) {
    std::size_t u = n;
    for (std::size_t i = 0; i < n; ++i) {if (!done[i] && dist[i] < kInf && (u == n || dist[i] < dist[u])) {u = i;}}
    if (u == n) {break;}
    done[u] = 1;
    if (exit[u]) {goal = u; break;}
    auto relax = [&](std::size_t v, double c, bool j) {
        if (bad[v] || done[v] || !(dist[u] + c < dist[v])) {return;}
        dist[v] = dist[u] + c;
        from[v] = u;
        jump[v] = j;
      };
    if (u > 0) {relax(u - 1, max_diff(trace[u], trace[u - 1]), false);}
    if (u + 1 < n) {relax(u + 1, max_diff(trace[u], trace[u + 1]), false);}
    for (std::size_t v = 0; v < n; ++v) {
      if (v != u && max_diff(trace[u], trace[v]) <= loop_tol) {relax(v, 0.0, true);}
    }
  }
  if (goal == n) {
    why = "지나온 길로 검사된 복귀 경로가 있는 자세 (P0 · LOOK · 받침 근처) 에 닿는 길이 없음 (퍽에 닿는 점을 지나야 하거나 기록 끝)";
    return {};
  }
  std::vector<std::size_t> chain;
  for (std::size_t v = goal; v != n; v = from[v]) {chain.push_back(v);}
  std::reverse(chain.begin(), chain.end());
  auto make = [speed_dps](const std::string & name, const std::vector<double> & q) {
      PathNode nd;
      nd.name = name;
      nd.q = q;
      nd.speed_dps = speed_dps > 0.0 ? speed_dps : 0.0;
      return nd;
    };
  std::vector<PathNode> kept{make("retreat_start", q_now)};
  for (const std::size_t v : chain) {
    if (jump[v] && kept.size() > 1) {kept.back().name = "retreat_" + std::to_string(v); continue;}   // 같은 자세 (움직임 없음)
    kept.push_back(make("retreat_" + std::to_string(v), trace[v]));
  }
  if (kept.size() == 1) {kept.push_back(make("retreat_exit", trace[goal]));}
  kept.back().name = "retreat_exit";
  kept.back().stop = true;
  if (exit_index) {*exit_index = goal;}
  return kept;
}

std::string home_retreat_block(bool home_request, bool enabled, bool park_left)
{
  if (!home_request) {return "HOME 요청에서만 (자동 접기 아님)";}
  if (!enabled) {return "home_retreat 꺼짐";}
  if (park_left) {return "받침 안 다시 쥐기 빈손 — 퍽이 받침에 남았을 수 있음 (사람 확인)";}
  return "";
}

bool home_retreat_landed(const std::vector<double> & q_meas, const std::function<bool(const std::vector<double> &)> & is_exit)
{
  return finite_q(q_meas) && is_exit(q_meas);
}

}  // namespace manipulation
