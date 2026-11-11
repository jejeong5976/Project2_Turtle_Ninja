// visual_regrasp.hpp — 세워 놓은 퍽 다시 보고 수직으로 다시 쥐기 (regrasp_visual): 새 관측 고르기 · 관찰 대기 끼우기 · 수직 IK (ROS 없음)
//   기울여 쥔 퍽을 가까운 빈 자리에 세워 놓고 (regrasp_nodes) 손을 뺀 뒤 (re_place_clear) 곧게 더 올린 관찰 점 (re_observe) 에서
//   손목 카메라 새 장을 기다려 잰 중심 XY 로 수직 다시 쥐기 (re_above · re_down · re_grasp · re_up · re_lift) 를 다시 풂.
//   높이 · 닫힘 방향 · 벌림 · 속도는 원래 계획 그대로. 못 보거나 (시간 초과) · 모호하거나 · 너무 많이 옮겨졌거나 · IK 가 안 되면
//   원래 (보지 않고 쥐는) 경로 그대로. 거절 · 새 고장 없음
//   관찰 점이 re_place_clear 가 아닌 이유 (11-10 계산): 거기서는 카메라 ↔ 퍽 윗면 0.14–0.15 m 로 D435 848×480 깊이 최소 거리
//   (약 0.18 m) 안 → 팀 인지 puck_pose 가 누운 퍽 · 12 mm 치우침으로 봄. 윗면 위 110 mm · 손 20° 면 0.207 m · 오차 0.7 mm (합성 시험)
//   시험: test/test_visual_regrasp.cpp
#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include "manipulation/arm.hpp"
#include "manipulation/arm_path.hpp"
#include "manipulation/arm_kinematics.hpp"

namespace manipulation
{
// 계획 안 논리 대기 동작의 pose 이름 (모터 궤적 없음, control_master 가 가로챔)
inline constexpr const char * kRegraspObserve = "__regrasp_observe";
// 관찰 점 경로 점 이름 (regrasp_nodes 가 re_place_clear 와 re_above 사이에 멈춤으로 끼움)
inline constexpr const char * kRegraspObserveNode = "re_observe";

struct RegraspObservation
{
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};   // base_link 퍽 중심
  double stamp{0.0};                                  // 촬영 시각 [s]
  std::string id;                                     // AprilTag ID ("" = 못 읽음)
  bool upright{false}, exact_tf{false}, yaw_known{false};
  double yaw_rad{0.0};
};

// 새 관측 고르기: 대기 시작 + settle 뒤 촬영 · 촬영 시각 TF · 선 퍽 · 높이 ±40 mm · 오래되지 않음 (max_age)
//   id 있으면: 그 ID (expected 에서 id_radius 안), 없으면 ID 못 읽은 검출만 위치로 (25 mm 안). 다른 ID 는 쓰지 않음
//   id 없으면: 25 mm 안 아무 검출. 같은 조건 후보가 둘 이상 = 모호 → 안 씀 (why)
std::optional<RegraspObservation> select_regrasp_observation(
  const std::vector<RegraspObservation> & observations, const Eigen::Vector3d & expected,
  const std::string & id, double after, double now, double max_age, double id_radius, std::string * why = nullptr);

struct RegraspSplit
{
  std::vector<ArmAction> plan;
  ArmPath tail;   // 관찰 점 (멈춤) 부터 끝까지 — 교체할 원래 후반 경로. 비면 끼우지 않음
};
// 이미 stop 으로 끊은 관찰 점 (kRegraspObserveNode) 동작 뒤에 논리 대기 (kRegraspObserve, duration = timeout) 하나만 끼움. 원래 동작은 그대로
RegraspSplit split_regrasp_observation(const ArmPath & path, std::vector<ArmAction> plan, bool enabled, double timeout);

struct RegraspSolution
{
  std::vector<PickPose> poses;   // 3 개 (위 · 내림 · 쥐기) 또는 빔
  std::string grip, why;
};
// 값으로 받은 모델 · 새 XY · 원래 높이 (top) · 후보 (닫힘 방향, 벌림) 로 수직 손 IK 만 (기울인 손으로 바꾸지 않음)
RegraspSolution solve_visual_regrasp(const ArmModel & model, const Eigen::Vector2d & xy,
  double original_top, double yaw, const std::vector<double> & heights,
  const std::vector<std::pair<double, std::string>> & candidates, const std::vector<Q5> & seeds);

// 원래 후반 경로의 re_above · re_down · re_grasp · re_up · re_lift 관절값을 새 해로 바꿈 (다른 점 · 속도 · 멈춤 · 그리퍼 그대로).
//   바꾼 점이 정확히 5 개가 아니면 false (tail 그대로)
bool retarget_regrasp_tail(const ArmModel & model, const std::vector<PickPose> & poses, ArmPath & tail);

// 대기 한 주기의 판단 (control_master::advance_visual_regrasp 가 50 Hz 로 부름)
//   WAIT = 아직 쓸 새 관측 없음 (deadline 전), FALLBACK = 원래 (보지 않는) 경로로 (why), USE = tail 을 새 해로 바꿈
enum class RegraspDecision : uint8_t { WAIT, FALLBACK, USE };
struct RegraspTarget
{
  Eigen::Vector3d expected{Eigen::Vector3d::Zero()};   // 놓은 자리 (x, y, 퍽 중심 z) base_link
  std::string id;                                       // goal.puck_id ("" = 위치로만)
  double after{0.0}, max_age{0.5}, id_radius{0.06};     // select_regrasp_observation 과 같음
  double max_shift_m{0.015};                            // 잰 중심 ↔ 놓은 자리 한계 (넘으면 FALLBACK, NaN · 0 이하 = 한계 없음)
  double top{0.0}, close_deg{0.0};                      // 원래 수직 다시 쥐기의 높이 · 닫힘 방향 (그대로 씀)
  bool square{true};                                    // 네모 = 닫힘 방향 ±90° 도 같은 면 쥐기, 원기둥 = 아무 방향 (원래 방향 IK 가 안 되면 돌려서 다시 찾음)
  Eigen::Vector2d offset{Eigen::Vector2d::Zero()};       // 잰 중심에 더해 쥐는 점 [m, base xy] (regrasp_offset_mm, 보지 않고 다시 쥐기와 같은 값)
  std::vector<double> heights;                          // pick_heights_mm (위 · 내림 · 쥐기)
  Q5 seed{};                                            // 원래 해 (re_above) — IK 씨앗
};
struct RegraspPlan
{
  bool low_above{false};                                // 위 점을 낮춰서 (윗면 + 20 mm) 풀었음
  RegraspDecision decision{RegraspDecision::WAIT};
  ArmPath tail;              // USE: 새 해로 바꾼 후반 경로
  RegraspObservation obs;    // USE: 쓴 관측
  std::vector<PickPose> poses;
  std::string why;           // WAIT · FALLBACK: 이유
};
RegraspPlan decide_visual_regrasp(const std::vector<RegraspObservation> & observations, const RegraspTarget & target,
  double now, double deadline, const ArmModel & model, const ArmPath & tail);
}  // namespace manipulation
