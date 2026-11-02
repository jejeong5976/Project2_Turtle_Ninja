// grasp_select.hpp — 이웃 퍽을 보고 집기 · 놓기 닫힘 방향과 벌림 폭 고르기 (순수 C++, test/test_grasp_select.cpp)
// =====================================================================
// 수직 손이 퍽 높이 안으로 내려가면 손가락 끝 두 개가 이웃 퍽과 같은 높이 → 위에서 본 2D 거리로 판단
//   손가락 끝 (URDF 충돌 메시, 손끝 tip5 기준 끝 35 mm): 닫힘축 방향 안쪽 면 9.1 mm + q · 바깥 면 27.3 mm + q, 폭 ±16.4 mm (q = 손가락 위치 [m])
//   이웃 퍽: 원기둥 = 원, 면 방향 아는 사각기둥 = 회전 사각, 모르는 사각기둥 = 외접원, 누운 · 방향 모름 = 반지름 30 mm 원. 놓인 위치 오차만큼 키움
//   목표 퍽: 손가락 안쪽 면이 목표 퍽 바깥보다 target_clear 이상 바깥이어야 함 (좁게 열 때 퍽 모서리 위로 내려가지 않게)
// 후보: 닫힘 방향 (네모 = 면 방향 2 개, 원기둥 · 모름 = 기본 방향부터 15° 간격 12 개) × 벌림 폭 (넓은 것부터)
// 순서: 넓은 벌림부터. 같은 폭 안에서 기본 방향 여유가 prefer_default 이상이면 기본 방향 먼저, 그다음 여유 큰 순 (같으면 기본 방향에 가까운 순)

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace manipulation
{

struct NeighborPuck
{
  Eigen::Vector2d c;       // 중심 (base_link xy) [m]
  bool square{false};      // 사각기둥 (모양 모르면 true 로 = 외접원)
  bool yaw_known{false};   // 면 방향을 앎 (사각기둥만)
  double yaw{0.0};         // 면 법선 방향 [rad]
  double radius{0.0};      // > 0 이면 이 반지름 원으로 봄 (누운 퍽 · 방향 모름)
};

struct TargetPuck
{
  bool square{true};       // 모르면 true (보수적으로)
  bool yaw_known{false};
  double yaw{0.0};
  double offset{0.0};      // 목표 점 ↔ 실제 검출 중심 차이 [m] (여유에서 뺌)
  bool gate{true};         // false = 목표 퍽 검사 안 함 (PLACE: 퍽이 이미 손가락 사이)
};

struct FingerTip
{
  double inner0{0.0091};   // q = 0 일 때 닫힘축 방향 안쪽 면 [m]
  double outer0{0.0273};   // 바깥 면
  double w_lo{-0.0164};    // 폭 방향 범위
  double w_hi{0.0164};
};

struct GraspChoice
{
  bool ok{false};
  double close_rad{0.0};   // 닫힘축 방향 (base_link xy)
  double grip{0.0};        // 손가락 위치 [m]
  double clearance{0.0};   // 이웃 퍽과 최소 여유 [m] (이웃 없으면 큰 값)
  std::string why;         // 실패 이유
};

struct GraspLimits
{
  double min_clear{0.002};        // 이웃 퍽과 최소 여유
  double target_clear{0.003};     // 목표 퍽 바깥 ↔ 손가락 안쪽 면 최소 여유
  double prefer_default{0.005};   // 기본 방향 여유가 이 이상이면 손목을 안 돌림
};

// 손가락 두 개 (닫힘 방향 close_rad, 손가락 위치 grip) ↔ 이웃 퍽 최소 거리 [m] (겹치면 0 이하)
double finger_clearance(
  const Eigen::Vector2d & center, double close_rad, double grip, const std::vector<NeighborPuck> & nb,
  const FingerTip & f = FingerTip{}, double puck_half = 0.015, double slop = 0.002);

// 목표 퍽이 닫힘축 방향으로 차지하는 반 길이 [m] (원기둥 15 mm, 면 정렬 네모 15 mm, 모르는 네모 = 외접 21.2 mm)
double target_half_along(const TargetPuck & t, double close_rad, double puck_half = 0.015);

// 닫힘 방향 후보: 네모(면 방향 앎) → yaw, yaw + 90°. 그 밖 → default_close 부터 15° 간격 12 개 (180° 주기)
std::vector<double> close_candidates(bool square_yaw_known, double yaw, double default_close);

// 되는 후보를 고르는 순서대로 (IK 가 안 되면 다음 후보). 비면 why 에 이유
std::vector<GraspChoice> rank_grasps(
  const Eigen::Vector2d & center, const TargetPuck & target, const std::vector<double> & closes, const std::vector<double> & grips,
  const std::vector<NeighborPuck> & nb, double default_close, std::string & why, const GraspLimits & lim = GraspLimits{},
  const FingerTip & f = FingerTip{});

// 첫 후보 (없으면 ok=false + why)
GraspChoice choose_grasp(
  const Eigen::Vector2d & center, const TargetPuck & target, const std::vector<double> & closes, const std::vector<double> & grips,
  const std::vector<NeighborPuck> & nb, double default_close, const GraspLimits & lim = GraspLimits{}, const FingerTip & f = FingerTip{});

}  // namespace manipulation
