// grasp_select.hpp — 이웃 퍽을 보고 집기 · 놓기 닫힘 방향과 벌림 폭 고르기 (순수 C++, test/test_grasp_select.cpp)
// =====================================================================
// 수직 손이 퍽 높이 안으로 내려가면 손가락 끝 두 개가 이웃 퍽과 같은 높이 → 위에서 본 2D 거리로 판단
//   손가락 끝 (URDF 충돌 메시, 손끝 tip5 기준 끝 35 mm): 닫힘축 방향 안쪽 면 9.1 mm + q · 바깥 면 27.3 mm + q, 폭 ±16.4 mm (q = 손가락 위치 [m])
//   이웃 퍽: 원기둥 = 반지름 15 mm 원, 면 방향 아는 사각기둥 = 회전 사각 (반폭 17.4 mm), 모르는 사각기둥 = 외접원 24.6 mm, 누운 · 방향 모름 = 반지름 30 mm 원. 놓인 위치 오차만큼 키움
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
  const FingerTip & f = FingerTip{}, double puck_half = 0.015, double slop = 0.002, double square_half = 0.0174);

// 놓을 퍽 발자국 (중심 center, 방향 axis_rad, 반 크기 half = (축 방향, 옆)) ↔ 이웃 퍽 최소 거리 [m] (이웃 없으면 무한)
//   누운 퍽을 놓을 때: 긴축 끝 (±30 mm) 이 손가락 폭 (±16.4 mm) 밖으로 나옴 → 손가락 여유만으로는 모자람
double footprint_clearance(
  const Eigen::Vector2d & center, double axis_rad, const Eigen::Vector2d & half, const std::vector<NeighborPuck> & nb,
  double puck_half = 0.015, double slop = 0.002, double square_half = 0.0174);

// 목표 퍽이 닫힘축 방향으로 차지하는 반 길이 [m] (원기둥 15 mm, 면 정렬 네모 17.4 mm, 기울면 17.4·(|cos δ|+|sin δ|), 모르는 네모 = 외접 24.6 mm)
double target_half_along(const TargetPuck & t, double close_rad, double puck_half = 0.015, double square_half = 0.0174);

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

// ---- 다시 쥘 자리 (regrasp spot): 원래 자리에 다시 내려놓으면 붙은 이웃을 누를 수 있음 ----
//   다시 쥐려고 내려놓는 자리마다 이웃 몸통과 사이 (spot_gap) · 박스 가장자리 여유 모자람 (spot_edge_short) 을 재고
//   order_regrasp_spots 순서로 IK 를 풂 (거절은 늘리지 않음: 사이를 채우는 자리가 없으면 가장 넓은 자리)
struct RegraspSpot
{
  Eigen::Vector2d p{Eigen::Vector2d::Zero()};   // 내려놓을 퍽 중심 (base_link xy) [m]
  double gap{1e9};          // 가장 가까운 이웃 몸통과 사이 [m] (이웃 없으면 1e9, 겹치면 음수)
  double edge_short{0.0};   // 박스 가장자리 여유가 모자란 양 [m] (0 = 안쪽)
  int pref{0};              // 지금 탐색 순서 (작을수록 먼저 = 팔에 가까운 자리)
  double reach{0.0};        // J1 에서 거리 [m] (같은 층 안에서 가까운 것부터 = 수직 손이 넉넉히 닿는 곳)
};

// 같은 색 이웃은 경계가 섞여 서로 쪽으로 겹쳐 잼. 목표 (p0, 반폭 half_t) 와 겹쳐 잰 이웃 (면 사이 < 0) 은
//   목표 반대쪽으로 겹친 만큼 (max_push 까지) 밀어 봄 → 그 이웃 너머 자리의 사이를 덜 낙관적으로 (면 사이 = 중심 거리 − 반폭 둘, PICK 과 같은 식)
std::vector<NeighborPuck> unbias_neighbors(const Eigen::Vector2d & p0, double half_t, std::vector<NeighborPuck> nb,
  double max_push = 0.005, double puck_half = 0.015, double square_half = 0.0174);

// 놓을 퍽 중심 s 에서 이웃 몸통까지 거리 − 놓을 퍽 반폭 half_t [m] (이웃 없으면 1e9, 겹치면 음수).
//   원으로 보는 이유: 다시 쥘 때 손목은 어느 방향으로든 돌 수 있음 (regrasp_align_yaw · ±90° 후보) → 열린 손가락이 어느 쪽이든 닿지 않아야 함
//   이웃: 면 방향 아는 네모 = 회전 사각 (반폭 square_half), 모르는 네모 = 외접원, 원기둥 = puck_half, 누운 · 모름 = radius
double spot_gap(const Eigen::Vector2d & s, double half_t, const std::vector<NeighborPuck> & nb,
  double puck_half = 0.015, double square_half = 0.0174);

// 박스 가장자리 여유 (박스가 몇 도 돌아 놓이면 regrasp_min_x · regrasp_max_abs_y 끝 자리가 벽 위일 수 있음):
//   앞 (x ≥ min_x + margin) · 옆 (|y| ≤ max_abs_y − margin) 까지 모자란 양의 합 [m] (0 = 안쪽).
//   퍽이 있는 · 있던 자리 known 보다 안쪽이면 그 가장자리는 모자람 0 (박스 안이 확실)
//   — 앞: x 가 그 퍽 x 이상이고 y 차 band 안, 옆: 같은 쪽 |y| 가 그 퍽 |y| 이하이고 x 차 band 안
double spot_edge_short(const Eigen::Vector2d & s, double min_x, double max_abs_y, double margin,
  const std::vector<Eigen::Vector2d> & known, double band = 0.02);

// 1 = 사이 ≥ need · 안쪽, 2 = 사이 ≥ need · 가장자리, 3 = 사이 모자람 (대체)
int spot_tier(const RegraspSpot & s, double need);

// IK 를 풀 순서: 1 → J1 거리 (5 mm 칸) 가까운 순, 2 → 모자란 가장자리 작은 순 (5 mm 칸: 0–5 · 5–10 …) · 그 안에서 J1 거리 (5 mm 칸) 가까운 순,
//   3 → (사이 − 모자란 가장자리) 큰 순 (1 mm 칸) · J1 거리 순. 같으면 pref. 가까운 것부터라 수직 손이 안 닿는 먼 자리는 한 번 실패한 뒤 IK 없이 넘길 수 있음
//   keep_first (store_recover · 쥔 퍽 영상 다시 쥐기의 원래 자리, origin_first): 그 자리가 층 1 · 2 (이웃과 사이 ≥ need) 면 맨 앞 그대로
//   (원래 자리는 이웃을 이미 본 곳 — 뒤로 밀리면 못 본 퍽 위에 놓을 수 있음)
std::vector<RegraspSpot> order_regrasp_spots(std::vector<RegraspSpot> spots, double need, const Eigen::Vector2d * keep_first = nullptr);

// regrasp_nodes 자리 후보 (이 순서 = pref): origin_first 면 p0 먼저 → 기본 후보 (J1 → 퍽 방향 반지름 r: x = min_x + 2 mm 가 되는 곳 ·
//   퍽 1 cm 앞 중 작은 것부터 1 cm 씩 퍽 1 cm 앞까지 × 옆 0 · ±4 · ±8 cm, 그다음 박스 앞쪽 끝 (x = min_x + 2 · 12 · 22 mm) 을 따라 가운데 쪽
//   1 cm 씩) → 넓힌 후보 (박스 앞쪽 끝 줄 · 퍽 너머 out_m 까지 = 박스 안쪽, 옆 2 cm 씩 ±16 cm, 옆으로 박스 밖이 되는 줄은 |y| = max_abs_y 점)
//   → p0 (origin_first 아니면 맨 끝). 박스 밖 (x < min_x · |y| > max_abs_y) 은 뺌. 넓힌 후보는 앞 후보와 5 mm 안이면 뺌 (기본 후보는 다 남음)
std::vector<Eigen::Vector2d> regrasp_spot_points(const Eigen::Vector2d & j1, const Eigen::Vector2d & p0, double min_x, double max_abs_y,
  bool origin_first, double out_m = 0.03);

}  // namespace manipulation
