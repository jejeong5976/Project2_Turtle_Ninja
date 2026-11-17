// dice_verify.cpp — 재검증 판정 구현, 설명은 dice_verify.hpp

#include "mission/dice_verify.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <sstream>

namespace mission
{

namespace
{
std::string place_name(int p) {return "L" + std::to_string(p + 1);}

std::string fmt(double v, int digits = 3)
{
  char b[32];
  std::snprintf(b, sizeof(b), "%.*f", digits, v);
  return b;
}

// 문구용 색 · 모양 (모르면 "")
std::string ko_color(const std::string & c)
{
  if (c == "red") {return "빨강";}
  if (c == "green") {return "초록";}
  return c;
}
std::string ko_shape(const std::string & s)
{
  if (s == "cylinder") {return "원기둥";}
  if (s == "box") {return "사각기둥";}
  return s;
}
// "빨강 원기둥" · "빨강 퍽" · "퍽"
std::string ko_puck(const VerifyObs & o)
{
  std::string c = ko_color(o.color), s = ko_shape(o.shape);
  if (c.empty() && s.empty()) {return "퍽";}
  if (s.empty()) {return c + " 퍽";}
  return c.empty() ? s : c + " " + s;
}
// 칸 k 자리 이름 (없으면 "칸 k")
std::string spot_of(const PlaceVerify & pv, size_t k)
{
  return k < pv.cell_spots.size() ? pv.cell_spots[k] : "칸 " + std::to_string(k);
}
// 가장 가까운 칸 번호
size_t nearest_cell(const std::vector<std::array<double, 2>> & cells, double x, double y)
{
  size_t best = 0;
  for (size_t k = 1; k < cells.size(); ++k) {
    if (std::hypot(x - cells[k][0], y - cells[k][1]) < std::hypot(x - cells[best][0], y - cells[best][1])) {best = k;}
  }
  return best;
}
// 같은 장 두 검출이 이만큼 넘게 떨어지면 다른 퍽 (퍽 지름 30 mm, 기대도 중심 25 mm 넘게 떨어짐 · 한 퍽 마스크 중복은 1 cm 안)
constexpr double kSameFrameSep = 0.015;
// 튄 장 묶음 = 붙일 퍽 장 수의 이 비율 이하
constexpr double kStrayRatio = 0.25;

// 퍽 하나의 장별 읽기 모음
struct ObsVotes
{
  VerifyObs last;                       // 가장 최근 장 (ID · 촬영 시각 · 상자, 위치는 good 이 있으면 good)
  std::map<std::string, int> color, shape;
  std::array<double, 2> q{{0.0, 0.0}};  // 마지막 장 묶기 점 (깊이 틀린 장은 광선 · 윗면 교점, 아니면 그 장 위치)
  bool has_good = false;                // 윗면 높이에 맞는 장 있음
  VerifyObs good;                       // 마지막 맞는 장
  int off_n = 0;                        // 깊이 틀린 장 수
  std::vector<std::pair<double, std::array<double, 2>>> frames;   // (촬영 시각, 묶기 점) 장마다
  std::array<double, 2> sum{{0.0, 0.0}};

  // 묶기 점 평균 (한 장 튐에 덜 흔들림)
  std::array<double, 2> mean() const
  {
    const double n = frames.empty() ? 1.0 : static_cast<double>(frames.size());
    return {sum[0] / n, sum[1] / n};
  }

  // 같은 장에서 qo 와 sep 넘게 떨어진 검출이 이미 있음 = 다른 퍽 (시각 모름 < 0 은 안 봄)
  bool apart_in_frame(double stamp, const std::array<double, 2> & qo, double sep) const
  {
    if (stamp < 0.0) {return false;}
    for (const auto & [t, p] : frames) {
      if (std::fabs(t - stamp) < 1e-6 && std::hypot(p[0] - qo[0], p[1] - qo[1]) >= sep) {return true;}
    }
    return false;
  }

  // 같은 촬영 시각 장이 하나라도 있음
  bool shares_frame(const ObsVotes & other) const
  {
    for (const auto & [t, p] : other.frames) {
      if (t < 0.0) {continue;}
      for (const auto & [u, r] : frames) {
        if (std::fabs(t - u) < 1e-6) {return true;}
      }
    }
    return false;
  }

  // 다른 퍽: 같은 장에 같이 보인 장 중 절반 넘게 (3 장 이상) sep 넘게 떨어짐. 한두 장 섞인 것에는 안 흔들림
  bool apart(const ObsVotes & other, double sep) const
  {
    int shared = 0, far = 0;
    for (const auto & [t, p] : other.frames) {
      if (t < 0.0) {continue;}
      bool same = false, away = false;
      for (const auto & [u, r] : frames) {
        if (std::fabs(t - u) >= 1e-6) {continue;}
        same = true;
        if (std::hypot(p[0] - r[0], p[1] - r[1]) >= sep) {away = true;}
      }
      shared += same;
      far += away;
    }
    return far >= 3 && 2 * far > shared;
  }

  void add(const VerifyObs & o, bool off, const std::array<double, 2> & qo)
  {
    const std::string id = last.id.empty() ? o.id : last.id;
    last = o;
    last.id = id;
    q = qo;
    frames.emplace_back(o.stamp, qo);
    sum[0] += qo[0];
    sum[1] += qo[1];
    if (off) {
      ++off_n;
    } else {
      has_good = true;
      good = o;
    }
    if (!o.color.empty()) {++color[o.color];}
    if (!o.shape.empty()) {++shape[o.shape];}
  }

  void absorb(const ObsVotes & other)
  {
    frames.insert(frames.end(), other.frames.begin(), other.frames.end());
    sum[0] += other.sum[0];
    sum[1] += other.sum[1];
    for (const auto & [k, n] : other.color) {color[k] += n;}
    for (const auto & [k, n] : other.shape) {shape[k] += n;}
    off_n += other.off_n;
    if (!has_good && other.has_good) {   // 이 퍽의 ID 장이 다 틀렸고 ID 못 읽은 장이 맞으면 그 위치
      has_good = true;
      good = other.good;
    }
  }
};

// 다수결: 1 등 비율 >= ratio 면 그 값, 아니면 ""
std::string vote(const std::map<std::string, int> & counts, double ratio)
{
  int total = 0, best_n = 0;
  std::string best;
  for (const auto & [k, n] : counts) {
    total += n;
    if (n > best_n) {best_n = n; best = k;}
  }
  if (total == 0) {return "";}
  return static_cast<double>(best_n) / total >= ratio ? best : "";
}

// 같은 장 (촬영 시각 같음). 시각 모름 (< 0) 이면 같은 장으로 안 봄 (같은 ID 는 한 퍽)
bool same_frame(const VerifyObs & a, const VerifyObs & b)
{
  return a.stamp >= 0.0 && b.stamp >= 0.0 && std::fabs(a.stamp - b.stamp) < 1e-6;
}
}  // namespace

std::vector<VerifyObs> group_place_obs(const std::vector<VerifyObs> & raw, double merge_m, double vote_ratio, double plane_tol_m)
{
  // 윗면 높이: 이 장소 장들 z 중앙값. 장 대부분이 박스 위 퍽이라 깊이 틀린 몇 장에 안 끌림
  bool plane = false;
  double plane_z = 0.0;
  if (plane_tol_m > 0.0) {
    std::vector<double> zs;
    for (const auto & o : raw) {
      if (std::isfinite(o.z)) {zs.push_back(o.z);}
    }
    if (zs.size() >= 3) {
      std::nth_element(zs.begin(), zs.begin() + static_cast<long>(zs.size() / 2), zs.end());
      plane_z = zs[zs.size() / 2];
      plane = true;
    }
  }
  // 장마다: 깊이 틀림 여부 · 묶기 점
  auto judge = [&](const VerifyObs & o, std::array<double, 2> & qo) {
      qo = {o.x, o.y};
      if (!plane || !std::isfinite(o.z) || std::fabs(o.z - plane_z) <= plane_tol_m) {return false;}
      const auto & c = o.cam;
      if (std::isfinite(c[0]) && std::isfinite(c[1]) && std::isfinite(c[2]) && std::fabs(o.z - c[2]) > 1e-6) {
        const double t = (plane_z - c[2]) / (o.z - c[2]);   // 카메라 원점 → 그 점 광선이 윗면 높이를 지나는 곳
        if (t > 0.0) {qo = {c[0] + t * (o.x - c[0]), c[1] + t * (o.y - c[1])};}
      }
      return true;
    };
  auto dist = [](const std::array<double, 2> & a, const std::array<double, 2> & b) {return std::hypot(a[0] - b[0], a[1] - b[1]);};
  // ID 마다 퍽 목록: 보통 하나. 같은 장에서 떨어진 두 자리에 같은 ID 가 보이면 둘 (중복으로 남김)
  std::map<std::string, std::vector<ObsVotes>> by_id;
  std::vector<ObsVotes> anon;
  for (const auto & o : raw) {
    std::array<double, 2> qo;
    const bool off = judge(o, qo);
    if (!o.id.empty()) {
      auto & tracks = by_id[o.id];
      ObsVotes * best = nullptr;
      double best_d = 0.0;
      for (auto & t : tracks) {
        const double d = dist(t.q, qo);
        if (same_frame(t.last, o) && d >= merge_m) {continue;}   // 같은 장 다른 자리 = 다른 퍽
        if (!best || d < best_d) {best = &t; best_d = d;}        // 다음 장: 가장 가까운 퍽 (하나뿐이면 그 퍽)
      }
      if (!best) {
        tracks.emplace_back();
        best = &tracks.back();
      }
      best->add(o, off, qo);
      continue;
    }
    // ID 없는 장: 묶기 점 평균이 merge_m 안인 가장 가까운 퍽. 같은 장에 kSameFrameSep 넘게 떨어져
    //   이미 보인 퍽은 뺌 (같은 장 두 자리 = 다른 퍽, 그보다 가까우면 한 퍽의 마스크 중복)
    ObsVotes * best = nullptr;
    double best_d = merge_m;
    for (auto & a : anon) {
      if (a.apart_in_frame(o.stamp, qo, kSameFrameSep)) {continue;}
      const double d = dist(a.mean(), qo);
      if (d < best_d) {best = &a; best_d = d;}
    }
    if (!best) {
      anon.emplace_back();
      best = &anon.back();
    }
    best->add(o, off, qo);
  }
  // ID 없는 묶음끼리: 평균 merge_m 안 · 같은 장에 떨어져 보인 적 없으면 한 퍽 (튄 장이 따로 시작한 묶음)
  for (bool again = true; again;) {
    again = false;
    size_t bi = 0, bj = 0;
    double bd = merge_m;
    for (size_t i = 0; i < anon.size(); ++i) {
      for (size_t j = i + 1; j < anon.size(); ++j) {
        const double d = dist(anon[i].mean(), anon[j].mean());
        if (d < bd && !anon[i].apart(anon[j], kSameFrameSep)) {bd = d; bi = i; bj = j;}
      }
    }
    if (bd < merge_m) {
      const bool j_later = anon[bj].last.stamp >= anon[bi].last.stamp;
      ObsVotes merged = j_later ? anon[bj] : anon[bi];
      merged.absorb(j_later ? anon[bi] : anon[bj]);
      if (!merged.has_good) {merged.good = (j_later ? anon[bi] : anon[bj]).good;}
      anon[bi] = std::move(merged);
      anon.erase(anon.begin() + static_cast<long>(bj));
      again = true;
    }
  }
  // 튄 장 묶음: 작은 (장 수 kStrayRatio 이하) ID 없는 묶음이 2 × merge_m 안 퍽과 같은 장에 한 번도 같이
  //   안 보였으면 그 퍽의 튄 장 (그 퍽이 그 장에서 이 자리로 튐). 같이 보인 적 있으면 진짜 다른 퍽이라 둠
  for (bool again = true; again;) {
    again = false;
    for (size_t i = 0; i < anon.size() && !again; ++i) {
      ObsVotes * to = nullptr;
      double td = 2.0 * merge_m;
      auto consider = [&](ObsVotes & t) {
          if (&t == &anon[i] || anon[i].frames.size() > kStrayRatio * t.frames.size()) {return;}
          const double d = dist(t.mean(), anon[i].mean());
          if (d < td && !t.shares_frame(anon[i])) {to = &t; td = d;}
        };
      for (auto & a : anon) {consider(a);}
      for (auto & kv : by_id) {
        for (auto & t : kv.second) {consider(t);}
      }
      if (to) {
        to->absorb(anon[i]);
        anon.erase(anon.begin() + static_cast<long>(i));
        again = true;
      }
    }
  }
  // ID 없는 퍽이 ID 있는 퍽 가까이 (평균 merge_m 안) 이고 같은 장에 떨어져 보인 적 없으면 같은 퍽 → 읽기만 합침
  //   같은 장에 둘 다 보였으면 붙어 있어도 다른 퍽 (기운 퍽이 옆 퍽에 기댐)
  std::vector<ObsVotes> pucks;
  for (const auto & a : anon) {
    ObsVotes * near = nullptr;
    double near_d = merge_m;
    for (auto & kv : by_id) {
      for (auto & t : kv.second) {
        const double d = dist(t.mean(), a.mean());
        if (d < near_d && !t.apart(a, kSameFrameSep)) {near = &t; near_d = d;}
      }
    }
    if (near) {near->absorb(a);} else {pucks.push_back(a);}
  }
  std::vector<VerifyObs> out;
  auto emit = [&](const ObsVotes & v) {
      VerifyObs o = v.last;
      if (v.has_good && v.off_n > 0) {   // 맞는 장 위치 (상자 · 시각은 마지막 장 = 그림과 같은 장)
        o.pos_from_earlier = v.good.stamp != v.last.stamp || v.good.x != v.last.x || v.good.y != v.last.y;
        o.x = v.good.x;
        o.y = v.good.y;
        o.z = v.good.z;
      }
      o.off_plane_n = v.off_n;
      o.all_off_plane = v.off_n > 0 && !v.has_good;
      o.color = vote(v.color, vote_ratio);
      o.shape = vote(v.shape, vote_ratio);
      out.push_back(o);
    };
  for (const auto & kv : by_id) {
    for (const auto & t : kv.second) {emit(t);}
  }
  for (const auto & p : pucks) {emit(p);}
  return out;
}

namespace
{
// 칸 점 = center + R(dyaw) · offset + (dx, dy)
std::array<double, 2> fit_point(const std::array<double, 2> & c, const std::array<double, 2> & o, double dx, double dy, double dyaw)
{
  const double cs = std::cos(dyaw), sn = std::sin(dyaw);
  return {c[0] + cs * o[0] - sn * o[1] + dx, c[1] + sn * o[0] + cs * o[1] + dy};
}

// 가까운 쌍부터 한 점 · 한 칸에 하나 (tol 안). 반환 = (점, 칸) 짝
std::vector<std::pair<size_t, size_t>> greedy_pairs(
  const std::vector<std::array<double, 2>> & pts, const std::vector<std::array<double, 2>> & cells, double tol)
{
  struct P {double d; size_t i, k;};
  std::vector<P> all;
  for (size_t i = 0; i < pts.size(); ++i) {
    for (size_t k = 0; k < cells.size(); ++k) {
      const double d = std::hypot(pts[i][0] - cells[k][0], pts[i][1] - cells[k][1]);
      if (d <= tol) {all.push_back({d, i, k});}
    }
  }
  std::sort(all.begin(), all.end(), [](const P & a, const P & b) {return a.d < b.d;});
  std::vector<bool> pi(pts.size(), false), ck(cells.size(), false);
  std::vector<std::pair<size_t, size_t>> out;
  for (const auto & p : all) {
    if (pi[p.i] || ck[p.k]) {continue;}
    pi[p.i] = ck[p.k] = true;
    out.emplace_back(p.i, p.k);
  }
  return out;
}
}  // namespace

double box_inner_dist(const VerifyBox & b, double x, double y)
{
  const double c = std::cos(b.yaw), s = std::sin(b.yaw);
  const double dx = x - b.x, dy = y - b.y;
  const double u = c * dx + s * dy, v = -s * dx + c * dy;
  return b.half - std::max(std::abs(u), std::abs(v));
}

std::optional<PatternFit> fit_pattern(
  const std::array<double, 2> & center, const std::vector<std::array<double, 2>> & offsets,
  const std::vector<std::array<double, 2>> & points, double cell_tol_m, double pair_tol_m, const VerifyFitParams & fp,
  const VerifyBox * box)
{
  if (points.empty() || offsets.empty()) {return std::nullopt;}
  // 시작 가설: 회전 (범위 안 15° 간격) × ("점 i 가 칸 k 위" 인 평행이동 + 기준 그대로), 이동 범위 안만
  //   회전을 풀어 두면 (max_rot 180°) 방향 무관 = 주사위 모양만 봄 (3 × 3 격자 눈 모양은 좌우 반전 = 회전이라 반전도 됨)
  struct Start {double dx, dy, dyaw;};
  std::vector<Start> starts;
  const double step = 15.0 * M_PI / 180.0;
  const int n_rot = static_cast<int>(std::floor(fp.max_rot_rad / step + 1e-9));
  for (int r = -n_rot; r <= n_rot; ++r) {
    const double th = r * step;
    if (r > -n_rot && std::abs(th) >= M_PI - 1e-9) {continue;}   // ±180° 하나만
    const double cs = std::cos(th), sn = std::sin(th);
    if (r == 0) {starts.push_back({0.0, 0.0, 0.0});}
    for (const auto & q : points) {
      for (const auto & o : offsets) {
        const double dx = q[0] - center[0] - (cs * o[0] - sn * o[1]);
        const double dy = q[1] - center[1] - (sn * o[0] + cs * o[1]);
        if (std::hypot(dx, dy) <= fp.max_shift_m) {starts.push_back({dx, dy, th});}
      }
    }
  }
  std::optional<PatternFit> best;
  double best_err = 0.0;
  for (const auto & s0 : starts) {
    PatternFit f;
    f.dx = s0.dx;
    f.dy = s0.dy;
    f.dyaw = s0.dyaw;
    bool in_range = true;
    std::vector<std::pair<size_t, size_t>> pairs;
    for (int it = 0; it < 4; ++it) {   // 짝 맞추기 ↔ 강체 맞춤 번갈아
      std::vector<std::array<double, 2>> cells;
      for (const auto & o : offsets) {cells.push_back(fit_point(center, o, f.dx, f.dy, f.dyaw));}
      // 첫 판만 넓게 (pair_tol), 그 뒤는 칸 허용 (cell_tol) 안 짝만 → 칸 밖 퍽이 모양을 끌고 가지 않게
      pairs = greedy_pairs(points, cells, it == 0 ? pair_tol_m : cell_tol_m);
      if (pairs.empty() && it > 0) {pairs = greedy_pairs(points, cells, pair_tol_m);}
      if (pairs.empty()) {break;}
      // 중심 둘레 회전 + 이동: 짝 무게중심끼리 맞추고, 2 짝 이상이면 회전도 (범위로 자름)
      double mq[2] = {0, 0}, mo[2] = {0, 0};
      for (const auto & [i, k] : pairs) {
        mq[0] += points[i][0]; mq[1] += points[i][1];
        mo[0] += offsets[k][0]; mo[1] += offsets[k][1];
      }
      const double n = static_cast<double>(pairs.size());
      for (double & v : mq) {v /= n;}
      for (double & v : mo) {v /= n;}
      if (pairs.size() >= 2) {
        double sd = 0.0, sc = 0.0;
        for (const auto & [i, k] : pairs) {
          const double ax = offsets[k][0] - mo[0], ay = offsets[k][1] - mo[1];
          const double bx = points[i][0] - mq[0], by = points[i][1] - mq[1];
          sd += ax * bx + ay * by;
          sc += ax * by - ay * bx;
        }
        f.dyaw = std::clamp(std::atan2(sc, sd), -fp.max_rot_rad, fp.max_rot_rad);
      }
      const double cs = std::cos(f.dyaw), sn = std::sin(f.dyaw);
      f.dx = mq[0] - center[0] - (cs * mo[0] - sn * mo[1]);
      f.dy = mq[1] - center[1] - (sn * mo[0] + cs * mo[1]);
      if (std::hypot(f.dx, f.dy) > fp.max_shift_m) {in_range = false; break;}
    }
    if (!in_range) {continue;}
    std::vector<std::array<double, 2>> cells;
    for (const auto & o : offsets) {cells.push_back(fit_point(center, o, f.dx, f.dy, f.dyaw));}
    if (box && std::any_of(cells.begin(), cells.end(), [box](const std::array<double, 2> & q) {
        return box_inner_dist(*box, q[0], q[1]) < box->margin;})) {continue;}   // 칸이 윗면 밖 = 놓으면 떨어지는 모양
    pairs = greedy_pairs(points, cells, cell_tol_m);
    f.matched = static_cast<int>(pairs.size());
    double err = 0.0;
    for (const auto & [i, k] : pairs) {
      err += std::pow(points[i][0] - cells[k][0], 2) + std::pow(points[i][1] - cells[k][1], 2);
    }
    // 같은 짝 수면 오차 작은 쪽, 오차도 같으면 (대칭 모양: 180° 돌려도 같음) 덜 돈 쪽
    const bool tie = best && f.matched == best->matched && std::abs(err - best_err) < 1e-8;
    if (!best || f.matched > best->matched || (f.matched == best->matched && err < best_err - 1e-8) ||
      (tie && std::abs(f.dyaw) < std::abs(best->dyaw) - 1e-6))
    {
      best = f;
      best_err = err;
    }
  }
  if (best && best->matched == 0) {return std::nullopt;}   // 칸 허용 안에 든 퍽이 하나도 없음 → 기준 칸 그대로
  return best;
}

namespace
{
// 칸 ID 판정: 칸 k 에 놓은 퍽 cell_ids[k] 가 칸 중심 cell_tol_m 안 · 박스 위에 보이고 다시 본 색 · 모양이 그룹 조건에 맞아야 함
//   같은 ID 가 둘이면 칸에 가까운 쪽. 칸에 안 쓰인 검출 = ID 못 읽음 · 계획에 없는 ID → 불합격
//   Y 기준 모양 = 칸에 든 퍽 중 모양 아는 것의 최다 (동률이면 기준 없음 = 모양 섞임)
void judge_cell_ids(
  PlaceVerify & pv, const std::string & name, const std::vector<std::array<double, 2>> & cells,
  const std::vector<std::string> & cell_ids, const std::vector<std::array<std::string, 2>> & cell_kinds,
  std::vector<VerifyObs> & obs, double cell_tol_m)
{
  std::vector<bool> used(obs.size(), false);
  for (auto & o : obs) {
    o.cell = -1;
    o.ok = false;
    o.cell_error_m = 1e9;
    for (const auto & q : cells) {o.cell_error_m = std::min(o.cell_error_m, std::hypot(o.x - q[0], o.y - q[1]));}
  }
  int planned = 0;
  for (size_t k = 0; k < cells.size(); ++k) {
    const std::string id = k < cell_ids.size() ? cell_ids[k] : "";
    if (id.empty()) {
      pv.issues.push_back(name + " " + spot_of(pv, k) + " 자리: 놓을 퍽이 모자라 비워 둔 자리");
      continue;
    }
    ++planned;
    int best = -1;
    double best_d = 0.0;
    for (size_t i = 0; i < obs.size(); ++i) {
      if (used[i] || obs[i].id != id) {continue;}
      const double d = std::hypot(obs[i].x - cells[k][0], obs[i].y - cells[k][1]);
      if (best < 0 || d < best_d) {best = static_cast<int>(i); best_d = d;}
    }
    if (best < 0) {
      // 박스 밖이라 뺀 검출에 그 ID 가 있으면 같이 적음 (위치를 잘못 잰 경우 · 정말 떨어진 경우를 구분해 보게)
      std::string off;
      for (const auto & g : pv.ignored) {
        if (g.id == id) {off = " (박스 가장자리 밖 " + fmt(std::max(-g.edge_m, 0.0) * 1000, 0) + " mm 에 퍽 " + id + " 검출 있음, 박스 밖이라 판정에서 뺌)";}
      }
      pv.issues.push_back(name + " " + spot_of(pv, k) + " (퍽 " + id + " 놓은 자리): 박스 위에서 퍽 " + id + " 이 안 보임" + off);
      continue;
    }
    auto & o = obs[static_cast<size_t>(best)];
    used[static_cast<size_t>(best)] = true;
    o.cell_error_m = best_d;
    if (best_d <= cell_tol_m) {o.cell = static_cast<int>(k);}
    o.ok = o.cell >= 0 && o.on_box != 0;   // 그룹 조건은 아래에서 (Y 기준 모양은 칸 퍽 다 본 뒤)
    if (o.on_box == 0) {
      pv.issues.push_back(name + " " + spot_of(pv, k) + ": 퍽 " + id + " 이 박스 윗면 밖 (가장자리에서 " + fmt(-o.edge_m * 1000, 0) +
        " mm 밖, 떨어짐)");
    }
    if (o.cell < 0) {
      pv.issues.push_back(name + " " + spot_of(pv, k) + ": 퍽 " + id + " 이 자리에서 " + fmt(best_d * 1000, 0) + " mm 떨어짐 (허용 " +
        fmt(cell_tol_m * 1000, 0) + " mm)");
    }
  }
  // 그룹 조건 (다시 본 색 · 모양): 칸에 든 퍽만
  if (pv.group == 'Y') {
    std::map<std::string, int> n;
    for (const auto & o : obs) {
      if (o.cell >= 0 && !o.shape.empty()) {++n[o.shape];}
    }
    int top = 0;
    for (const auto & [sh, c] : n) {
      if (c > top) {top = c; pv.y_shape = sh;} else if (c == top) {pv.y_shape.clear();}
    }
  }
  for (auto & o : obs) {
    if (o.cell < 0) {continue;}
    const auto k = static_cast<size_t>(o.cell);
    std::string rec;   // 기록 색 · 모양 (다르면 이유에 같이)
    if (k < cell_kinds.size() && (cell_kinds[k][0] != o.color || cell_kinds[k][1] != o.shape) &&
      !(cell_kinds[k][0].empty() && cell_kinds[k][1].empty()))
    {
      rec = ", 기록 " + cell_kinds[k][0] + "/" + cell_kinds[k][1];
    }
    const std::string seen = o.color.empty() ? std::string("?") : o.color;
    const std::string shp = o.shape.empty() ? std::string("?") : o.shape;
    std::string why;
    if (pv.group == 'R' || pv.group == 'G') {
      const std::string want = pv.group == 'R' ? "red" : "green";
      if (o.color.empty()) {
        why = "색 모름 (그룹 " + std::string(1, pv.group) + " 확인 못 함" + rec + ")";
      } else if (o.color != want) {
        why = "색 " + seen + " ≠ " + want + " (그룹 " + std::string(1, pv.group) + rec + ")";
      }
    } else if (pv.group == 'Y') {
      if (o.shape.empty()) {
        why = "모양 모름 (그룹 Y 확인 못 함" + rec + ")";
      } else if (pv.y_shape.empty()) {
        why = "모양 " + shp + " · Y 모양 섞임 (같은 수" + rec + ")";
      } else if (o.shape != pv.y_shape) {
        why = "모양 " + shp + " ≠ Y 모양 " + pv.y_shape + rec;
      }
    }
    if (!why.empty()) {
      o.ok = false;
      pv.issues.push_back(name + " " + spot_of(pv, k) + " 퍽 " + o.id + ": " + why);
    }
  }
  for (size_t i = 0; i < obs.size(); ++i) {
    if (used[i]) {continue;}
    const std::string near = cells.empty() ? std::string("박스 위") :
      spot_of(pv, nearest_cell(cells, obs[i].x, obs[i].y)) + " 자리 근처 (" + fmt(std::min(obs[i].cell_error_m, 9.999) * 1000, 0) + " mm)";
    if (obs[i].id.empty()) {
      pv.issues.push_back(name + " " + near + " 에 " + ko_puck(obs[i]) + " 이 보이는데 ID 를 못 읽음");
    } else {
      pv.issues.push_back(name + " " + near + " 에 퍽 " + obs[i].id + " 이 보이는데 이 장소에 놓을 퍽이 아님");
    }
  }
  if (static_cast<int>(obs.size()) != pv.expected) {
    pv.issues.push_back(name + " 퍽 " + std::to_string(obs.size()) + " 개 보임, 눈 " + std::to_string(pv.expected) + " 개 필요" +
      (planned != pv.expected ? " (놓은 퍽 " + std::to_string(planned) + " 개)" : ""));
  }
}
}  // namespace

std::string cell_spot_name(
  const std::array<double, 2> & center, const std::array<double, 2> & view_dir, const std::array<double, 2> & cell,
  bool en, double half_m)
{
  const double n = std::hypot(view_dir[0], view_dir[1]);
  const double vx = n > 1e-9 ? view_dir[0] / n : 1.0, vy = n > 1e-9 ? view_dir[1] / n : 0.0;
  const double dx = cell[0] - center[0], dy = cell[1] - center[1];
  const double ahead = dx * vx + dy * vy;    // + = 로봇에서 먼 쪽
  const double left = -dx * vy + dy * vx;    // + = 로봇이 볼 때 왼쪽
  const int row = ahead > half_m ? 1 : (ahead < -half_m ? -1 : 0);
  const int col = left > half_m ? 1 : (left < -half_m ? -1 : 0);
  if (en) {
    if (row == 0 && col == 0) {return "center";}
    const std::string r = row > 0 ? "far" : (row < 0 ? "near" : "mid");
    const std::string c = col > 0 ? "left" : (col < 0 ? "right" : "center");
    return r + "-" + c;
  }
  if (row == 0 && col == 0) {return "가운데";}
  const std::string r = row > 0 ? "먼 줄" : (row < 0 ? "가까운 줄" : "가운데 줄");
  const std::string c = col > 0 ? "왼쪽" : (col < 0 ? "오른쪽" : "가운데");
  return r + " " + c;
}

PlaceVerify judge_place(
  int place, const DicePlan & plan, const DiceMap & arena, std::vector<VerifyObs> obs,
  double cell_tol_m, double stamp_s, const VerifyFitParams & fit, const std::optional<VerifyBox> & box,
  const std::vector<std::string> & cell_ids, double ignore_inset_m, const std::vector<std::array<std::string, 2>> & cell_kinds)
{
  PlaceVerify pv;
  pv.place = place;
  pv.stamp_s = stamp_s;
  pv.cell_tol_m = cell_tol_m;
  if (place < 0 || place > 2) {
    pv.issues.push_back("장소 번호 잘못됨");
    return pv;
  }
  const auto & dp = plan.places[place];
  pv.group = dp.group;
  pv.expected = dp.pips;
  const auto & c = arena.places[place].center;
  const std::string name = place_name(place);
  pv.ref_center = c;

  // 박스 위 확인 · 박스 밖 거르기: 윗면 가장자리에서 ignore_inset_m 안쪽이 아닌 검출은 판정 · 개수에서 뺌
  //   (박스 모서리 · 로봇 몸 오검출. 맞춤 전에 빼서 모양을 끌고 가지 않게)
  //   이 장소에 놓은 퍽 ID 와 같은 ID 검출은 오검출이 아님 → 중심이 윗면 밖 (edge < 0) 일 때만 뺌
  pv.box = box;
  if (box) {
    std::vector<VerifyObs> kept;
    for (auto & o : obs) {
      o.edge_m = box_inner_dist(*box, o.x, o.y);
      o.on_box = o.edge_m >= box->margin ? 1 : 0;
      const bool placed_id = !o.id.empty() && std::find(cell_ids.begin(), cell_ids.end(), o.id) != cell_ids.end();
      const double inset = placed_id ? 0.0 : ignore_inset_m;
      if (ignore_inset_m > 0.0 && o.edge_m < inset) {
        o.on_box = 0;
        pv.ignored.push_back(o);
      } else {
        kept.push_back(o);
      }
    }
    obs = std::move(kept);
  }

  // 모양 맞춤: 관측 퍽에 눈 모양을 옮기고 돌려 맞춤 (짝 찾기는 칸 간격 반 정도까지 봐 줌)
  std::vector<std::array<double, 2>> cells;
  for (const auto & o : dp.pip_offsets) {cells.push_back({c[0] + o[0], c[1] + o[1]});}
  if (fit.enable && !obs.empty()) {
    std::vector<std::array<double, 2>> pts;
    for (const auto & o : obs) {pts.push_back({o.x, o.y});}
    if (const auto f = fit_pattern(c, dp.pip_offsets, pts, cell_tol_m, std::max(2.0 * cell_tol_m, 0.04), fit, box ? &*box : nullptr)) {
      pv.fitted = true;
      pv.fit_dx = f->dx;
      pv.fit_dy = f->dy;
      pv.fit_dyaw = f->dyaw;
      for (size_t k = 0; k < cells.size(); ++k) {cells[k] = fit_point(c, dp.pip_offsets[k], f->dx, f->dy, f->dyaw);}
    }
  }
  // 이 판정에 쓴 칸 기준 (B15, 기록 파일용): 기준 중심 + 모양 맞춤 이동
  pv.has_frame = true;
  pv.center = {c[0] + pv.fit_dx, c[1] + pv.fit_dy};
  for (const auto & q : cells) {
    pv.cell_spots.push_back(cell_spot_name(pv.center, arena.places[place].view_dir, q));
    pv.cell_spots_en.push_back(cell_spot_name(pv.center, arena.places[place].view_dir, q, true));
  }

  // 칸 맞추기: 가까운 쌍부터 (한 칸에 하나)
  struct Pair {double d; size_t o; int k;};
  std::vector<Pair> pairs;
  for (size_t o = 0; o < obs.size(); ++o) {
    for (size_t k = 0; k < cells.size(); ++k) {
      const double d = std::hypot(obs[o].x - cells[k][0], obs[o].y - cells[k][1]);
      pairs.push_back({d, o, static_cast<int>(k)});
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) {return a.d < b.d;});
  std::vector<bool> obs_done(obs.size(), false), cell_done(cells.size(), false);
  for (auto & o : obs) {o.cell_error_m = 1e9;}
  for (const auto & p : pairs) {
    obs[p.o].cell_error_m = std::min(obs[p.o].cell_error_m, p.d);
    if (obs_done[p.o] || cell_done[p.k] || p.d > cell_tol_m) {continue;}
    obs_done[p.o] = true;
    cell_done[p.k] = true;
    obs[p.o].cell = p.k;
    obs[p.o].cell_error_m = p.d;
  }

  if (std::any_of(cell_ids.begin(), cell_ids.end(), [](const std::string & id) {return !id.empty();})) {
    judge_cell_ids(pv, name, cells, cell_ids, cell_kinds, obs, cell_tol_m);
    pv.cell_ids = cell_ids;
    pv.seen = std::move(obs);
    pv.cells = std::move(cells);
    pv.ok = pv.issues.empty();
    return pv;
  }
  // 그룹 조건 (발제 p.2: Y 는 색 섞어도 됨, 원기둥 · 직육면체 중 한 모양으로 통일)
  //   Y 기준 모양 = 칸에 든 · 박스 위 퍽 중 모양 아는 것의 최다 모양. 동률이면 기준 없음 (모양 섞임 → 모양 아는 퍽 다 불합격)
  //   칸 밖 · 떨어진 · 모양 모르는 퍽은 기준에 안 넣음 → 그 퍽만 자기 이유로 불합격, 제자리 퍽까지 떨어뜨리지 않음
  std::string y_shape;
  if (pv.group == 'Y') {
    std::map<std::string, int> n;
    for (const auto & o : obs) {
      if (o.cell >= 0 && o.on_box != 0 && !o.shape.empty()) {++n[o.shape];}
    }
    int best = 0;
    for (const auto & [s, k] : n) {
      if (k > best) {best = k; y_shape = s;} else if (k == best) {y_shape.clear();}
    }
  }
  for (auto & o : obs) {
    bool g = false;
    if (pv.group == 'R') {g = o.color == "red";}
    if (pv.group == 'G') {g = o.color == "green";}
    if (pv.group == 'Y') {g = !o.shape.empty() && o.shape == y_shape;}
    o.ok = o.cell >= 0 && g && o.on_box != 0;
    const std::string who = o.id.empty() ? ko_puck(o) + " (ID 모름)" : "퍽 " + o.id;
    if (o.on_box == 0) {
      pv.issues.push_back(name + " " + who + " 이 박스 윗면 밖 (가장자리에서 " + fmt(-o.edge_m * 1000, 0) + " mm 밖, 떨어짐)");
    }
    if (o.cell < 0) {
      pv.issues.push_back(name + " " + who + " 이 눈 자리에서 벗어남 (가장 가까운 " +
        (cells.empty() ? std::string("자리") : spot_of(pv, nearest_cell(cells, o.x, o.y)) + " 자리") + " 에서 " +
        fmt(o.cell_error_m * 1000, 0) + " mm)");
    } else if (pv.group == 'Y' && o.shape.empty()) {
      pv.issues.push_back(name + " 퍽 " + (o.id.empty() ? "(ID 모름)" : o.id) + " 모양 모름 (그룹 Y 확인 못 함)");
    } else if (!g) {
      pv.issues.push_back(name + " 퍽 " + (o.id.empty() ? "(ID 모름)" : o.id) + " 그룹 " + pv.group + " 조건 안 맞음 (" +
        o.color + "/" + o.shape + (pv.group != 'Y' ? "" : y_shape.empty() ? " · 모양 섞임" : " · Y 모양 " + y_shape) + ")");
    }
    if (o.id.empty()) {pv.issues.push_back(name + " ID 못 읽은 퍽 있음");}
  }
  if (static_cast<int>(obs.size()) != pv.expected) {
    pv.issues.push_back(name + " 퍽 " + std::to_string(obs.size()) + " 개 보임, 눈 " + std::to_string(pv.expected) + " 개 필요");
  }
  pv.seen = std::move(obs);
  pv.cells = std::move(cells);
  pv.ok = pv.issues.empty();
  return pv;
}

VerifyResult summarize(const std::vector<PlaceVerify> & places, int total_ids)
{
  VerifyResult r;
  r.places = places;
  std::set<int> done;
  std::map<std::string, int> count;
  for (const auto & p : places) {
    done.insert(p.place);
    for (const auto & o : p.seen) {
      if (!o.id.empty()) {++count[o.id];}
    }
    r.issues.insert(r.issues.end(), p.issues.begin(), p.issues.end());
  }
  r.complete = done.size() == 3;
  for (const auto & [id, n] : count) {
    if (n > 1) {r.duplicate_ids.push_back(id);}
  }
  if (r.complete) {
    for (int i = 0; i < total_ids; ++i) {
      if (!count.count(std::to_string(i))) {r.missing_ids.push_back(std::to_string(i));}
    }
  }
  for (const auto & id : r.duplicate_ids) {r.issues.push_back("ID " + id + " 두 번 이상 봄");}
  for (const auto & id : r.missing_ids) {r.issues.push_back("퍽 " + id + ": 세 장소 어디에서도 안 보임");}
  r.pass = r.complete && r.issues.empty();
  return r;
}

std::string verify_to_yaml(const VerifyResult & r, const DicePlan & plan, const DiceMap & arena,
  const std::string & barcode, const std::string & signal_sequence)
{
  std::ostringstream y;
  y << "# 재검증 기록 (planning dice_verify). 관측 = VERIFY 단계 LOOK 뒤 새 검출, map 프레임 [m]\n";
  y << "barcode: \"" << barcode << "\"\n";
  y << "signal: \"" << signal_sequence << "\"\n";
  y << "status: " << (r.pass ? "PASS" : (r.complete ? "FAIL" : "PENDING")) << "\n";
  y << "places:\n";
  for (const auto & p : r.places) {
    const auto c = p.has_frame ? p.center : arena.places[p.place].center;
    y << "  - place: " << place_name(p.place) << "\n";
    y << "    group: " << p.group << "\n";
    y << "    pips: " << p.expected << "\n";
    y << "    seen: " << p.seen.size() << "\n";
    y << "    ok: " << (p.ok ? "true" : "false") << "\n";
    y << "    stamp_s: " << fmt(p.stamp_s, 2) << "\n";
    if (p.has_frame) {
      const auto & m = arena.places[p.place].center;
      y << "    center: [" << fmt(c[0]) << ", " << fmt(c[1]) << "]     # 판정에 쓴 중심 (박스 측정 · 모양 맞춤 반영)\n";
      y << "    map_center: [" << fmt(m[0]) << ", " << fmt(m[1]) << "]     # 지도 원래 중심 (참고)\n";
    } else {
      y << "    center: [" << fmt(c[0]) << ", " << fmt(c[1]) << "]\n";
    }
    if (p.group == 'Y' && !p.cell_ids.empty()) {
      y << "    y_shape: " << (p.y_shape.empty() ? "\"\"" : p.y_shape) << "   # Y 기준 모양 (다시 본 모양 최다, \"\" = 섞임 · 모름)\n";
    }
    y << "    frame_source: " << p.frame_source << "   # 칸 기준: remeasured (재검증 측정) · placed (놓을 때 박스 측정) · map\n";
    y << "    ref_center: [" << fmt(p.ref_center[0]) << ", " << fmt(p.ref_center[1]) << "]\n";
    y << "    fit: {used: " << (p.fitted ? "true" : "false") << ", dx_mm: " << fmt(p.fit_dx * 1000, 1) << ", dy_mm: " <<
      fmt(p.fit_dy * 1000, 1) << ", dyaw_deg: " << fmt(p.fit_dyaw * 180.0 / M_PI, 2) << "}   # 기준 칸 → 관측에 맞춘 눈 모양\n";
    if (p.box) {
      y << "    box: {source: " << p.box->source << ", center: [" << fmt(p.box->x) << ", " << fmt(p.box->y) << "], yaw_deg: " <<
        fmt(p.box->yaw * 180.0 / M_PI, 1) << ", half: " << fmt(p.box->half, 4) << "}   # 박스 위 확인에 쓴 윗면\n";
    } else {
      y << "    box: null   # 박스 측정 없음 → 박스 위 확인 안 함\n";
    }
    if (p.has_frame && !p.cells.empty()) {
      y << "    cells:     # 판정에 쓴 칸 (맞춤 뒤, 먼 칸부터, cell_error_mm 의 기준), 뒤 = 놓은 퍽 ID · 로봇이 볼 때 자리\n";
      for (size_t k = 0; k < p.cells.size(); ++k) {
        y << "      - [" << fmt(p.cells[k][0]) << ", " << fmt(p.cells[k][1]) << "]";
        y << "   # ";
        if (k < p.cell_ids.size()) {y << (p.cell_ids[k].empty() ? "(안 놓음)" : "ID " + p.cell_ids[k]) << " · ";}
        y << (k < p.cell_spots.size() ? p.cell_spots[k] : "칸 " + std::to_string(k));
        y << "\n";
      }
    } else {
      y << "    cells:     # 계획 칸 (먼 칸부터)\n";
      for (const auto & o : plan.places[p.place].pip_offsets) {
        y << "      - [" << fmt(c[0] + o[0]) << ", " << fmt(c[1] + o[1]) << "]\n";
      }
    }
    y << "    observed:\n";
    for (const auto & o : p.seen) {
      y << "      - {id: \"" << o.id << "\", color: " << (o.color.empty() ? "\"\"" : o.color) << ", shape: " <<
        (o.shape.empty() ? "\"\"" : o.shape) << ", x: " << fmt(o.x) << ", y: " << fmt(o.y) << ", cell: " << o.cell <<
        ", spot: " << (o.cell >= 0 && static_cast<size_t>(o.cell) < p.cell_spots_en.size() ? p.cell_spots_en[static_cast<size_t>(o.cell)] : "none") <<
        ", cell_error_mm: " << fmt(std::min(o.cell_error_m, 9.999) * 1000, 0) << ", on_box: " <<
        (o.on_box < 0 ? "unknown" : (o.on_box ? "true" : "false")) << ", ok: " << (o.ok ? "true" : "false") <<
        (o.off_plane_n > 0 ? ", depth_bad_frames: " + std::to_string(o.off_plane_n) : std::string()) << "}\n";
    }
    if (!p.ignored.empty()) {
      y << "    ignored:     # 박스 윗면 가장자리 가까이 · 밖이라 판정 · 개수에서 뺀 검출\n";
      for (const auto & o : p.ignored) {
        y << "      - {id: \"" << o.id << "\", color: " << (o.color.empty() ? "\"\"" : o.color) << ", shape: " <<
          (o.shape.empty() ? "\"\"" : o.shape) << ", x: " << fmt(o.x) << ", y: " << fmt(o.y) << ", edge_mm: " <<
          fmt(o.edge_m * 1000, 0) << (o.off_plane_n > 0 ? ", depth_bad_frames: " + std::to_string(o.off_plane_n) : std::string()) <<
          "}\n";
      }
    }
  }
  auto list = [&y](const char * key, const std::vector<std::string> & v) {
      y << key << ": [";
      for (size_t i = 0; i < v.size(); ++i) {y << (i ? ", " : "") << "\"" << v[i] << "\"";}
      y << "]\n";
    };
  list("duplicate_ids", r.duplicate_ids);
  list("missing_ids", r.missing_ids);
  y << "issues:\n";
  for (const auto & s : r.issues) {y << "  - \"" << s << "\"\n";}
  return y.str();
}

}  // namespace mission
