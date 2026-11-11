#include "manipulation/look_variant.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace manipulation
{
namespace
{
bool in_range(const ArmModel & m, const std::vector<double> & q)
{
  Ticks t;
  for (int j = 0; j < 5; ++j) {t[j] = static_cast<int>(std::lround(m.zero_tick[j] + q[j] * 4096.0 / (2.0 * M_PI)));}
  return ticks_in_range(m, t);
}
double distance(const std::vector<double> & a, const std::vector<double> & b)
{
  if (a.size() != 5 || b.size() != 5) {return std::numeric_limits<double>::infinity();}
  double d = 0.0;
  for (int j = 0; j < 5; ++j) {
    if (!std::isfinite(a[j]) || !std::isfinite(b[j])) {return std::numeric_limits<double>::infinity();}
    d = std::max(d, std::fabs(a[j] - b[j]));
  }
  return d;
}
}  // namespace

bool make_look_variant(const PathNode & front, const std::string & slot, const LookOptions & o,
  const ArmModel * model, PathNode & out, std::string & fallback)
{
  fallback.clear();
  out = front;
  if (front.q.size() != 5) {fallback = "경로 look 관절 5개 필요"; return false;}
  const bool side = slot == "left" || slot == "right";
  if (!side && slot != "far" && slot != "near") {return true;}
  if (side) {
    if (!std::isfinite(o.side_deg) || o.side_deg <= 0.0 || o.side_deg > 90.0) {fallback = "look_side_deg 는 0 ~ 90°"; return false;}
    out.name = "look_" + slot;
    out.q[0] += (slot == "left" ? 1.0 : -1.0) * o.side_deg * M_PI / 180.0;
    if (model && !in_range(*model, out.q)) {fallback = out.name + ": J1 옆 보기 자세가 관절 범위 밖"; return false;}
  }
  const double tilt = side ? o.side_tilt_deg : slot == "far" ? o.far_tilt_deg : o.near_tilt_deg;
  if (tilt == 0.0) {return true;}
  // 새 옵션만 실패하면 원래 side 또는 front 를 씀 (새 거절 없음).
  if (!std::isfinite(tilt) || std::fabs(tilt) > 30.0 || !model) {
    fallback = "기울임이 유한한 ±30° 안이 아니거나 팔 모델 없음 → 기존 보기";
    return true;
  }
  PathNode tilted = out;
  tilted.name = "look_" + slot;
  const int joint = slot == "near" ? 2 : 3;
  tilted.q[joint] += (slot == "near" ? 1.0 : -1.0) * tilt * M_PI / 180.0;
  if (!in_range(*model, tilted.q)) {fallback = "기울인 보기 자세가 관절 범위 밖 → 기존 보기"; return true;}
  out = tilted;
  return true;
}

bool match_look_variant(const PathNode & front, const std::vector<PathNode> & variants,
  const std::vector<double> & now, double tol, PathNode & out)
{
  auto square_distance = [&now](const std::vector<double> & q) {
      if (!std::isfinite(distance(q, now))) {return std::numeric_limits<double>::infinity();}
      double d2 = 0.0;
      for (int j = 0; j < 5; ++j) {const double d = q[j] - now[j]; d2 += d * d;}
      return d2;
    };
  // 허용은 관절별 최대오차, 후보 선택은 제곱합. 다른 관절의 공통 추종오차가 작은 기울임 차이를 가리지 않게 함.
  double best = square_distance(front.q);
  bool found = false;
  for (const auto & n : variants) {
    const double d = distance(n.q, now);
    const double d2 = square_distance(n.q);
    if (d <= tol && d2 + 1e-12 < best) {out = n; best = d2; found = true;}
  }
  return found;
}

ArmPath look_home_bridge(const PathNode & variant, const PathNode & front, const ArmPath & front_home)
{
  ArmPath out = front_home;
  if (out.nodes.empty()) {return out;}
  if (out.nodes.front().q != front.q) {out.nodes.insert(out.nodes.begin(), front);}
  out.nodes.front().stand = true;   // 보기 사이 복귀도 기존 옆 보기와 같은 낮은 받침 속도
  out.nodes.front().stop = true;   // J1/J3/J4 를 정면으로 돌린 동작이 끝난 뒤 접기 시작
  PathNode start = variant;
  start.grip.clear(); start.stop = false; start.hold_s = 0.0;
  out.nodes.insert(out.nodes.begin(), start);
  return out;
}
}  // namespace manipulation
