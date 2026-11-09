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
  int far = 0;
  for (const auto & s : samples) {
    if (s.stamp < since) {continue;}
    if (std::hypot(s.x - nominal_center[0], s.y - nominal_center[1]) > max_shift_m) {++far; continue;}
    xs.push_back(s.x);
    ys.push_back(s.y);
    ds.push_back(fold_quarter_diff(s.yaw, nominal_yaw));
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
  f.dyaw = median(ds);
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

}  // namespace mission
