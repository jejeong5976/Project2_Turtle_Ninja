// place_box.cpp — 장소 박스 측정 기준 구현, 설명은 place_box.hpp

#include "mission/place_box.hpp"

#include <algorithm>
#include <cmath>

namespace mission
{

namespace
{
double median(std::vector<double> v)
{
  const std::size_t m = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<long>(m), v.end());
  if (v.size() % 2) {return v[m];}
  const double hi = v[m];
  std::nth_element(v.begin(), v.begin() + static_cast<long>(m) - 1, v.end());
  return (v[m - 1] + hi) / 2.0;
}
}  // namespace

double fold_quarter_diff(double a, double ref)
{
  const double q = M_PI / 2;
  double d = std::remainder(a - ref, q);
  if (d <= -q / 2) {d += q;}
  return d;
}

std::optional<PlaceFrame> place_frame_from(
  const std::vector<BoxSample> & samples, double since, const std::array<double, 2> & nominal_center,
  double nominal_yaw, double max_shift_m, int min_n, std::string * why)
{
  std::vector<double> xs, ys, ds;
  double s4 = 0.0, c4 = 0.0;   // 4θ 원형 합 (90° 주기 평균 방향)
  int far = 0;
  for (const auto & s : samples) {
    if (s.stamp < since) {continue;}
    if (std::hypot(s.x - nominal_center[0], s.y - nominal_center[1]) > max_shift_m) {++far; continue;}
    xs.push_back(s.x);
    ys.push_back(s.y);
    ds.push_back(fold_quarter_diff(s.yaw, nominal_yaw));
    s4 += std::sin(4.0 * s.yaw);
    c4 += std::cos(4.0 * s.yaw);
  }
  if (static_cast<int>(xs.size()) < std::max(1, min_n)) {
    if (why) {
      *why = "측정 " + std::to_string(xs.size()) + " 개 (필요 " + std::to_string(min_n) + ")" +
        (far ? ", 지도 중심에서 먼 측정 " + std::to_string(far) + " 개 버림" : "");
    }
    return std::nullopt;
  }
  PlaceFrame f;
  f.x = median(xs);
  f.y = median(ys);
  // ±45° 경계 (예: 44° · 46° → 접으면 44° · −44°, 짝수 중앙값 0°) 에서 안 갈리게
  //   4θ 평균 방향 r (nominal 대비, ±45°) 에서 45° 넘게 떨어진 값만 90° 옮겨 r 둘레로 펼친 뒤 중앙값 → 다시 ±45° 로 접음
  //   r 에서 45° 안 값은 그대로 (0° 근처 측정은 산술 안 거침)
  const double r = (s4 != 0.0 || c4 != 0.0) ? fold_quarter_diff(std::atan2(s4, c4) / 4.0, nominal_yaw) : 0.0;
  for (auto & d : ds) {
    if (d - r > M_PI / 4) {d -= M_PI / 2;} else if (d - r < -M_PI / 4) {d += M_PI / 2;}
  }
  f.dyaw = fold_quarter_diff(median(ds), 0.0);
  f.n = static_cast<int>(xs.size());
  return f;
}

std::array<double, 2> frame_point(const PlaceFrame & f, const std::array<double, 2> & offset)
{
  const double c = std::cos(f.dyaw), s = std::sin(f.dyaw);
  return {f.x + c * offset[0] - s * offset[1], f.y + s * offset[0] + c * offset[1]};
}

bool clamp_offset_in_box(std::array<double, 2> & offset, double box_yaw, const PlaceLimits & lim)
{
  const double r = std::max(0.0, lim.box_half - lim.puck_half - lim.margin);
  const double c = std::cos(box_yaw), s = std::sin(box_yaw);
  // 지도 방향 → 박스 좌표
  const double u = c * offset[0] + s * offset[1], v = -s * offset[0] + c * offset[1];
  const double uc = std::clamp(u, -r, r), vc = std::clamp(v, -r, r);
  if (uc == u && vc == v) {return false;}
  offset = {c * uc - s * vc, s * uc + c * vc};
  return true;
}

void apply_place_frame(DiceMap & arena, DicePlan & plan, int place, const PlaceFrame & f)
{
  if (place < 0 || place >= static_cast<int>(arena.places.size())) {return;}
  arena.places[place].center = {f.x, f.y};
  const double c = std::cos(f.dyaw), s = std::sin(f.dyaw);
  for (auto & o : plan.places[place].pip_offsets) {
    o = {c * o[0] - s * o[1], s * o[0] + c * o[1]};
  }
}

std::optional<PlaceFrame> verify_place_frame(
  const std::optional<PlaceFrame> & placed, const std::optional<PlaceFrame> & remeasured)
{
  return remeasured ? remeasured : placed;
}

PlaceFrame expected_box_at(const Pose2D & used_stop, const Pose2D & map_stop, const std::array<double, 2> & map_center)
{
  const double d = std::remainder(used_stop.yaw - map_stop.yaw, 2.0 * M_PI);
  const double c = std::cos(d), s = std::sin(d);
  const double vx = map_center[0] - map_stop.x, vy = map_center[1] - map_stop.y;
  PlaceFrame e;
  e.x = used_stop.x + c * vx - s * vy;
  e.y = used_stop.y + s * vx + c * vy;
  e.dyaw = d;
  return e;
}

Pose2D realigned_stop(const PlaceFrame & f, const Pose2D & map_stop, const std::array<double, 2> & map_center)
{
  const double c = std::cos(f.dyaw), s = std::sin(f.dyaw);
  const double vx = map_stop.x - map_center[0], vy = map_stop.y - map_center[1];
  Pose2D p;
  p.x = f.x + c * vx - s * vy;
  p.y = f.y + s * vx + c * vy;
  p.yaw = std::remainder(map_stop.yaw + f.dyaw, 2.0 * M_PI);
  return p;
}

RealignCheck realign_check(const PlaceFrame & f, const PlaceFrame & e, double max_shift_m, double max_yaw, int done, int max_n)
{
  RealignCheck r;
  r.shift_m = std::hypot(f.x - e.x, f.y - e.y);
  r.dyaw = fold_quarter_diff(f.dyaw, e.dyaw);
  r.over = r.shift_m > max_shift_m || std::abs(r.dyaw) > max_yaw;
  r.realign = r.over && done < max_n;
  return r;
}

// ---------------------------------------------------------------- 박스 다시 찾기

int OccGrid::at(double x, double y) const
{
  if (empty() || res <= 0.0) {return -1;}
  const int i = static_cast<int>(std::floor((x - ox) / res));
  const int j = static_cast<int>(std::floor((y - oy) / res));
  if (i < 0 || j < 0 || i >= w || j >= h) {return -1;}
  return data[static_cast<std::size_t>(j) * static_cast<std::size_t>(w) + static_cast<std::size_t>(i)];
}

int footprint_blocked(const OccGrid & g, const Pose2D & stop, double half_x, double half_y, double margin, int occ,
  const std::vector<std::array<double, 2>> & extra, const std::vector<std::array<double, 3>> & ignore)
{
  const auto ignored = [&ignore](double x, double y) {
      for (const auto & b : ignore) {
        const double dx = x - b[0], dy = y - b[1], co = std::cos(b[2]), si = std::sin(b[2]);
        if (std::fabs(co * dx + si * dy) < 0.1325 && std::fabs(-si * dx + co * dy) < 0.1325) {return true;}
      }
      return false;
    };
  const double hx = half_x + margin, hy = half_y + margin;
  const double co = std::cos(stop.yaw), si = std::sin(stop.yaw);
  int n = 0;
  if (!g.empty() && g.res > 0.0) {
    const double st = 0.5 * g.res;
    for (double a = -hx; a <= hx + 1e-9; a += st) {
      for (double b = -hy; b <= hy + 1e-9; b += st) {
        const double x = stop.x + co * a - si * b, y = stop.y + si * a + co * b;
        const int v = g.at(x, y);
        if ((v < 0 || v >= occ) && !ignored(x, y)) {++n;}
      }
    }
  }
  for (const auto & p : extra) {
    const double dx = p[0] - stop.x, dy = p[1] - stop.y;
    if (std::fabs(co * dx + si * dy) <= hx && std::fabs(-si * dx + co * dy) <= hy) {++n;}
  }
  return n;
}

std::vector<std::array<double, 2>> box_outline(const std::array<double, 2> & c, double yaw, double size, double step)
{
  std::vector<std::array<double, 2>> out;
  const double h = size / 2, co = std::cos(yaw), si = std::sin(yaw);
  const int n = std::max(1, static_cast<int>(std::ceil(size / std::max(1e-3, step))));
  for (int k = 0; k < 4; ++k) {
    for (int i = 0; i <= n; ++i) {
      const double t = -h + size * i / n;
      double a = 0.0, b = 0.0;
      if (k == 0) {a = h; b = t;} else if (k == 1) {a = -h; b = t;} else if (k == 2) {a = t; b = h;} else {a = t; b = -h;}
      out.push_back({c[0] + co * a - si * b, c[1] + si * a + co * b});
    }
  }
  return out;
}

std::vector<FaceStop> box_face_stops(const std::array<double, 2> & c, double box_yaw, double dist, double ref_yaw)
{
  std::vector<FaceStop> out;
  for (int k = 0; k < 4; ++k) {
    FaceStop f;
    f.face = k;
    f.stop.yaw = std::remainder(box_yaw + k * M_PI / 2, 2 * M_PI);
    f.stop.x = c[0] - dist * std::cos(f.stop.yaw);
    f.stop.y = c[1] - dist * std::sin(f.stop.yaw);
    f.turn = std::fabs(std::remainder(f.stop.yaw - ref_yaw, 2 * M_PI));
    out.push_back(f);
  }
  std::stable_sort(out.begin(), out.end(), [](const FaceStop & a, const FaceStop & b) {return a.turn < b.turn;});
  return out;
}

std::optional<FaceStop> choose_face(std::vector<FaceStop> cands, const std::function<int(const Pose2D &)> & blocked)
{
  std::optional<FaceStop> best;
  for (auto & f : cands) {
    f.blocked = blocked ? blocked(f.stop) : 0;
    if (f.blocked == 0) {return f;}
    if (!best || f.blocked < best->blocked) {best = f;}
  }
  return best;
}

std::optional<BoxFind> box_find_from(
  const std::vector<BoxSample> & samples, double since, const std::array<double, 2> & map_center, double ref_yaw,
  const std::vector<std::array<double, 2>> & pucks, bool use_detector, double box_size, double max_shift_m, int min_n,
  std::string * why)
{
  std::string note;
  if (use_detector) {
    std::string w;
    // 90° 주기 평균 방향 (4θ 원형 평균) 기준으로 접음 → 45° 돈 박스도 ±45° 경계에서 갈리지 않음
    double s4 = 0.0, c4 = 0.0;
    for (const auto & sm : samples) {
      if (sm.stamp < since || std::hypot(sm.x - map_center[0], sm.y - map_center[1]) > max_shift_m) {continue;}
      s4 += std::sin(4.0 * sm.yaw);
      c4 += std::cos(4.0 * sm.yaw);
    }
    const double ref = (s4 != 0.0 || c4 != 0.0) ? ref_yaw + fold_quarter_diff(std::atan2(s4, c4) / 4.0, ref_yaw) : ref_yaw;
    if (auto f = place_frame_from(samples, since, map_center, ref, max_shift_m, min_n, &w)) {
      const double yaw = ref + f->dyaw, co = std::cos(yaw), si = std::sin(yaw), lim = box_size / 2 + 0.02;
      int out = 0;
      for (const auto & p : pucks) {
        const double dx = p[0] - f->x, dy = p[1] - f->y;
        if (std::fabs(co * dx + si * dy) > lim || std::fabs(-si * dx + co * dy) > lim) {++out;}
      }
      if (out == 0) {
        BoxFind b;
        b.center = {f->x, f->y};
        b.yaw = yaw;
        b.from_detector = true;
        b.n = f->n;
        return b;
      }
      note = "박스 측정 밖에 퍽 " + std::to_string(out) + " 개 → 측정 안 씀";
    } else {
      note = "박스 측정 못 씀 (" + w + ")";
    }
  }
  if (pucks.empty()) {
    if (why) {*why = note.empty() ? "퍽 못 봄" : note + ", 퍽 못 봄";}
    return std::nullopt;
  }
  double cx = 0.0, cy = 0.0;
  for (const auto & p : pucks) {cx += p[0]; cy += p[1];}
  cx /= static_cast<double>(pucks.size());
  cy /= static_cast<double>(pucks.size());
  const double shift = std::hypot(cx - map_center[0], cy - map_center[1]);
  if (shift > max_shift_m) {
    if (why) {*why = "퍽 평균이 지도에서 " + std::to_string(static_cast<int>(shift * 100)) + " cm (너무 멂)";}
    return std::nullopt;
  }
  BoxFind b;
  b.center = {cx, cy};
  b.yaw = ref_yaw;
  b.n = static_cast<int>(pucks.size());
  b.note = note;
  return b;
}

}  // namespace mission
