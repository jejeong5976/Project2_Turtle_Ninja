// pick_far.hpp — 먼 퍽 첫 쥐기 보정 (pick_offset_far_mm · pick_far_r_m) — 순수 C++, test/test_pick_far.cpp
// 팔 끝 근처 퍽은 수직 쥐기가 로봇 왼쪽 (+y) 으로 치우쳐 손가락이 퍽을 밂 (가까운 퍽은 보정 0 이 맞음)
//   → 쥐는 점이 J1 축에서 r_far_m 이상일 때만 [앞, 왼쪽] mm (base_link) 를 다른 보정 위에 더함 (control_master pick_plan, 서 있는 퍽 첫 쥐기)
// 반지름 = J1 축 (팔 모델 j1_xy) 에서 쥐는 점까지 수평 거리 — pick_poses 의 반지름 · 접선 (기울임 방향) 과 같은 기준

#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace manipulation
{

struct FarPickOffset
{
  bool valid{true};                          // offset_mm 이 [앞, 왼쪽] 두 숫자 (아니면 안 더함 — 부르는 쪽이 경고)
  bool applied{false};                       // 먼 퍽이고 보정이 0 이 아님 → d 를 더함
  double r_m{0.0};                           // J1 축에서 쥐는 점까지 수평 거리 [m]
  Eigen::Vector2d d{Eigen::Vector2d::Zero()};   // 더할 값 [m, base_link]
};

inline FarPickOffset far_pick_offset(const Eigen::Vector2d & target, const Eigen::Vector2d & j1_xy, double r_far_m,
  const std::vector<double> & offset_mm)
{
  FarPickOffset o;
  o.r_m = (target - j1_xy).norm();
  if (offset_mm.size() != 2 || !std::isfinite(offset_mm[0]) || !std::isfinite(offset_mm[1])) {
    o.valid = false;
    return o;
  }
  if ((offset_mm[0] == 0.0 && offset_mm[1] == 0.0) || !std::isfinite(r_far_m) || !std::isfinite(o.r_m) || o.r_m < r_far_m) {
    return o;
  }
  o.applied = true;
  o.d = Eigen::Vector2d(offset_mm[0], offset_mm[1]) / 1000.0;
  return o;
}

}  // namespace manipulation
