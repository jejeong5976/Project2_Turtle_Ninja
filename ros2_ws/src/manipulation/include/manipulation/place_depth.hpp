// place_depth.hpp — RETRIEVE 칸별 파지 깊이 추정 오차를 PLACE 높이에만 반영함 (ROS 없음).
// 받는 것: 가정한 쥔 깊이 [mm] · RETRIEVE 출처 칸 (비면 그대로) · 가정보다 얕게 쥔 mm. 돌려주는 것: PLACE용 깊이 [mm].
#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace manipulation
{
inline double retrieve_place_depth_mm(double assumed_mm, const std::string & slot, double shallower_mm)
{
  if (slot.empty() || shallower_mm == 0.0 || !std::isfinite(shallower_mm) ||
    !std::isfinite(assumed_mm) || assumed_mm <= 0.0) {return assumed_mm;}
  const double corrected = assumed_mm - shallower_mm;
  // 숫자가 아니거나 쥔 깊이보다 큰 보정은 무시함 (거절 안 함).
  return std::isfinite(corrected) && corrected > 0.0 ? corrected : assumed_mm;
}

// 놓기 높이에 쓸 실제 쥔 깊이 [mm]: 박스 PICK 이 윗면을 top_bias_mm (pick_top_bias_mm) 높게 보고 수직으로 쥐면 손끝은 명령 깊이보다
//   그만큼 얕음 (명령 50 → 실제 40). 쥔 깊이 기록 (held_depth) 은 명령값 그대로 (STORE 칸 높이 맞춤 · 칸 경로 이름이 씀) → PLACE 만 뺌.
//   보정 0 · 숫자 아님 · 쥔 깊이 모름 · 빼면 0 이하 = 그대로 (거절 안 함)
inline double place_real_depth_mm(double held_mm, double top_bias_mm)
{
  if (!std::isfinite(held_mm) || held_mm <= 0.0 || !std::isfinite(top_bias_mm) || top_bias_mm == 0.0) {return held_mm;}
  const double real = held_mm - top_bias_mm;
  return real > 0.0 ? real : held_mm;
}

inline double place_drop_height_mm(
  double clear_mm, double height_mm, double depth_mm, double place_tilt, double held_tilt)
{
  const double d = std::abs(place_tilt - held_tilt) * M_PI / 180.0;
  return clear_mm - height_mm + (height_mm - depth_mm) * std::cos(d) + 17.4 * std::sin(d);
}
// 먼 자리 PLACE 다시 쥐기 점 [m]: 내려놓은 자리 (sx, sy) + place_far_putdown_offset_mm [x, y] (내려놓은 퍽이 명령 자리에서 밀린 만큼).
//   크기 2 아님 · 숫자 아님 = 보정 0 (거절 안 함). [0, 0] 이면 그대로
inline void far_regrasp_point(double sx, double sy, const std::vector<double> & off_mm, double & gx, double & gy)
{
  gx = sx;
  gy = sy;
  if (off_mm.size() != 2 || !std::isfinite(off_mm[0]) || !std::isfinite(off_mm[1])) {return;}
  if (off_mm[0] != 0.0) {gx = sx + off_mm[0] / 1000.0;}
  if (off_mm[1] != 0.0) {gy = sy + off_mm[1] / 1000.0;}
}

// 기울인 손으로 선 퍽을 다시 쥘 때 손끝을 퍽 축에서 반지름 바깥으로 옮길 양 [mm] = 손끝에서 손 축으로 pad_mm 인 손가락 면 점이
//   퍽 축 위에 오게: pad_mm · sin(기울임). 손가락 안쪽 면은 닫힘축에 수직인 평면 (반지름 · 위 평면) 이라 선 원기둥과는 세로 줄 하나로 닿음.
//   손끝이 퍽 축 위면 그 줄이 손가락 끝 모서리에서 시작해 손가락 면 (폭 ±9 mm) 을 비스듬히 반만 지남 (30°: 퍽 높이 18 mm) → 퍽이
//   그 줄을 축으로 손 안에서 돎. 옮기면 줄이 손가락 면 안쪽을 지나 닿는 길이가 늚 (30°: pad 8 → 27 mm, pad 16 → 33 mm). 네모 퍽은 면이 닿아 그대로.
//   수직 손 (0°) · 숫자 아님 · 0 이하 = 0 (옮기지 않음)
inline double tilted_regrip_shift_mm(double pad_mm, double tilt_deg)
{
  if (!std::isfinite(pad_mm) || !std::isfinite(tilt_deg) || pad_mm <= 0.0 || tilt_deg <= 0.0) {return 0.0;}
  return pad_mm * std::sin(tilt_deg * M_PI / 180.0);
}

// 먼 자리 PLACE 목표에 기울인 손으로 놓는 높이 [mm, 퍽 윗면 기준 손끝] — 다시 쥐기를 같은 박스 윗면 (같은 top) 에서
//   depth_mm 로 했으므로 퍽 바닥 = 손끝 − (높이 − 깊이) 가 정확히 알려짐 → 퍽 바닥이 면 위 clear_mm (떨어뜨리지 않고 내려놓음).
inline double far_place_drop_mm(double clear_mm, double depth_mm)
{
  return clear_mm - depth_mm;
}

// 놓은 뒤 손을 손 축 (손가락 길이 방향) 으로 곧게 빼는 점: 놓는 손끝 (x, y, 높이 from_mm) 에서 높이 to_mm 까지
//   손 축을 따라 올라가면 반지름 안쪽 (J1 쪽) 으로 (to − from) · tan(기울임) 만큼 옴. 기울임 0 · 숫자 아님 = (x, y) 그대로 (수직으로 올림)
inline void axis_retreat_xy(
  double x, double y, double j1x, double j1y, double tilt_deg, double from_mm, double to_mm, double & ox, double & oy)
{
  ox = x;
  oy = y;
  const double r = std::hypot(x - j1x, y - j1y);
  if (!std::isfinite(tilt_deg) || tilt_deg <= 0.0 || tilt_deg >= 89.0 || !(r > 1e-9) || !std::isfinite(from_mm) || !std::isfinite(to_mm)) {return;}
  const double d = (to_mm - from_mm) / 1000.0 * std::tan(tilt_deg * M_PI / 180.0);
  ox = x - (x - j1x) / r * d;
  oy = y - (y - j1y) / r * d;
}
}  // namespace manipulation
