// place_depth.hpp — RETRIEVE 칸별 파지 깊이 추정 오차를 PLACE 높이에만 반영함 (ROS 없음).
// 받는 것: 기존 쥔 깊이 · RETRIEVE 출처 칸 · 가정보다 얕게 쥔 mm. 보내는 것: PLACE용 깊이.
#pragma once

#include <cmath>
#include <string>

namespace manipulation
{
inline double retrieve_place_depth_mm(double assumed_mm, const std::string & slot, double shallower_mm)
{
  if (slot.empty() || shallower_mm == 0.0 || !std::isfinite(shallower_mm) ||
    !std::isfinite(assumed_mm) || assumed_mm <= 0.0) {return assumed_mm;}
  const double corrected = assumed_mm - shallower_mm;
  // 숫자가 아니거나 쥔 깊이보다 큰 보정은 무시함. 새 작업 거절을 만들지 않음.
  return std::isfinite(corrected) && corrected > 0.0 ? corrected : assumed_mm;
}

inline double place_drop_height_mm(
  double clear_mm, double height_mm, double depth_mm, double place_tilt, double held_tilt)
{
  const double d = std::abs(place_tilt - held_tilt) * M_PI / 180.0;
  return clear_mm - height_mm + (height_mm - depth_mm) * std::cos(d) + 17.4 * std::sin(d);
}
}  // namespace manipulation
