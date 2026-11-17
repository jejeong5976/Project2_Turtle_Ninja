// visual_regrasp.hpp — 세워 놓은 퍽 다시 보고 수직으로 다시 쥐기 (regrasp_visual): 새 관측 고르기 · 관찰 대기 끼우기 · 수직 IK (ROS 없음)
//   기울여 쥔 퍽을 가까운 빈 자리에 세워 놓고 (regrasp_nodes) 손을 뺀 뒤 (re_place_clear) 곧게 더 올린 관찰 점 (re_observe) 에서
//   손목 카메라 새 장을 기다려 잰 중심 XY 로 수직 다시 쥐기 (re_above · re_down · re_grasp · re_up · re_lift) 를 다시 풂.
//   높이 · 벌림 · 속도는 원래 계획 그대로. 닫힘 방향은 원래 그대로, regrasp_align_yaw 면 잰 네모 면에 맞춤. 못 보거나 (시간 초과) · 모호하거나 ·
//   너무 많이 옮겨졌거나 · IK 가 안 되면 원래 (보지 않고 쥐는) 경로 그대로. 거절 · 새 고장 없음
//   관찰 점을 더 올리는 이유: re_place_clear 에서는 카메라 ↔ 퍽 윗면이 D435 깊이 최소 거리 (약 0.18 m) 안이라 잘못 잼
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
#include "manipulation/park_stand.hpp"

namespace manipulation
{
// 계획 안 논리 대기 동작의 pose 이름 (모터 궤적 없음, control_master 가 가로챔)
inline constexpr const char * kRegraspObserve = "__regrasp_observe";
// 관찰 점 경로 점 이름 (regrasp_nodes 가 re_place_clear 와 re_above 사이에 멈춤으로 끼움)
inline constexpr const char * kRegraspObserveNode = "re_observe";
// 옮겨 놓은 누운 퍽 다시 보기 (park_relook): 같은 대기 · 같은 값 (regrasp_visual_*). 관찰 점 = reloc_put_clear 뒤 (park_or_relocate_plan)
inline constexpr const char * kLyingObserve = "__lying_observe";
inline constexpr const char * kLyingObserveNode = "reloc_observe";
// 다시 본 누운 퍽으로 다시 계획한 받침 세우기의 첫 점 (= 지금 관찰 자세, 동작 없음)
inline constexpr const char * kRelookStartNode = "relook_start";

struct RegraspObservation
{
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};   // base_link 퍽 중심
  double stamp{0.0};                                  // 촬영 시각 [s]
  std::string id;                                     // AprilTag ID ("" = 못 읽음)
  bool upright{false}, exact_tf{false}, yaw_known{false};
  double yaw_rad{0.0};
  bool lying{false};                                  // 누운 퍽 (긴축이 수평에서 45° 안, 손목 인지 기준)
  double axis_rad{0.0};                               // 누운 퍽 긴축 수평 방향 [rad, base_link]
  bool tagged{false};                                 // 태그로 잰 중심 (누운 퍽 = 위 옆면 태그) — PICK 쥐는 점 보정이 따로
};

// 새 관측 고르기: 대기 시작 + settle 뒤 촬영 · 촬영 시각 TF · 선 퍽 · 높이 ±40 mm · 오래되지 않음 (max_age)
//   id 있으면: 그 ID (expected 에서 id_radius 안), 없으면 ID 못 읽은 검출만 위치로 (25 mm 안). 다른 ID 는 쓰지 않음
//   id 없으면: 25 mm 안 아무 검출. 같은 조건 후보가 둘 이상 = 모호 → 안 씀 (why)
std::optional<RegraspObservation> select_regrasp_observation(
  const std::vector<RegraspObservation> & observations, const Eigen::Vector3d & expected,
  const std::string & id, double after, double now, double max_age, double id_radius, std::string * why = nullptr);

// 누운 퍽 고르기 (park_relook 옮겨 놓은 누운 퍽 · regrasp_topple_lying 세워 놓은 퍽이 넘어짐): select_regrasp_observation 과 같은 새 장 ·
//   촬영 시각 TF · 높이 ±40 mm 조건에 누운 퍽만. id 있으면 그 ID (expected 에서 id_radius 안), 아니면 ID 못 읽은 검출을 위치로 (pos_radius 안).
//   avoid (계획 때 본 이웃 퍽 자리) 25 mm 안 검출은 그 이웃으로 보고 안 씀. 같은 조건 후보가 둘 이상 = 모호 → 안 씀 (why)
std::optional<RegraspObservation> select_lying_observation(
  const std::vector<RegraspObservation> & observations, const Eigen::Vector3d & expected, const std::string & id,
  double after, double now, double max_age, double id_radius, double pos_radius, const std::vector<Eigen::Vector2d> & avoid,
  std::string * why = nullptr);

struct RegraspSplit
{
  std::vector<ArmAction> plan;
  ArmPath tail;   // 관찰 점 (멈춤) 부터 끝까지 — 교체할 원래 후반 경로. 비면 끼우지 않음
};
// 이미 stop 으로 끊은 관찰 점 (node, 기본 kRegraspObserveNode) 동작 뒤에 논리 대기 (wait_pose, 기본 kRegraspObserve, duration = timeout) 하나만 끼움.
//   원래 동작은 그대로. 옮겨 놓은 누운 퍽 다시 보기는 node = kLyingObserveNode · wait_pose = kLyingObserve
RegraspSplit split_regrasp_observation(const ArmPath & path, std::vector<ArmAction> plan, bool enabled, double timeout,
  const char * node = kRegraspObserveNode, const char * wait_pose = kRegraspObserve);

struct RegraspSolution
{
  std::vector<PickPose> poses;   // 3 개 (위 · 내림 · 쥐기) 또는 빔
  std::string grip, why;
  double close{0.0};             // 풀린 후보의 닫힘 방향 [°]
};
// 값으로 받은 모델 · 새 XY · 원래 높이 (top) · 후보 (닫힘 방향, 벌림) 로 수직 손 IK 만 (기울인 손으로 바꾸지 않음)
RegraspSolution solve_visual_regrasp(const ArmModel & model, const Eigen::Vector2d & xy,
  double original_top, double yaw, const std::vector<double> & heights,
  const std::vector<std::pair<double, std::string>> & candidates, const std::vector<Q5> & seeds);

// 네모 퍽 면에 맞춘 닫힘 (regrasp_align_yaw): 원래 닫힘 close_deg 에서 잰 면 방향 yaw_deg (네모 = 90° 주기) 의 가장 가까운 면 법선까지
//   돌릴 각 [°] (−45 … 45). 입력이 NaN 이면 NaN
double regrasp_align_turn_deg(double close_deg, double yaw_deg);

// 손목 돌림 보정 [m, base xy] — control_master PICK 의 pick_offset_rot_mm · pick_offset_rot_robot_mm 와 같은 식:
//   (x, y) 를 쥐는 자세 q 의 닫힘축이 기본 (접선) 에서 rot 돈 만큼 (R(rot) − I) h (h = 90° 돌림 보정 rot_mm 에서) + |rot| ≥ 45° 면 robot_mm.
//   rot_deg 에 돌린 각 (±180, 손가락 방향 포함) 을 돌려줌
Eigen::Vector2d wrist_rot_offset(const ArmModel & m, double x, double y, const Q5 & q, const Eigen::Vector2d & rot_mm,
  const Eigen::Vector2d & robot_mm, double * rot_deg = nullptr);

// 다시 쥐기 손 지렛대 보정 [m, base xy] (regrasp_rot_hand_mm): 실제 손가락 가운데가 모델 손끝에서 손 좌표 g [mm] 만큼 떨어져 손목과 같이 돈다고 보고
//   자세 q (닫힘축이 접선에서 a) 로 쥘 때의 손끝 어긋남을 자세 q_ref (a_ref) 와 같게 하는 보정 = −R(az) (R(a) − R(a_ref)) g.
//   손 좌표 = tcp_hand_to_link5 와 같음: x = J5 기본 (닫힘축 = +접선) 에서 반지름 바깥, y = +닫힘축 (arm_grip close).
//   a 는 IK 해의 실제 닫힘축 (부호 포함) → J5 ±90° 에 따라 거울처럼 바뀜. q == q_ref 면 정확히 0. turn_deg = a − a_ref (±180)
Eigen::Vector2d wrist_lever_offset(const ArmModel & m, double x, double y, const Q5 & q, const Q5 & q_ref, const Eigen::Vector2d & hand_mm,
  double * turn_deg = nullptr);

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
  // 네모 면에 맞춰 닫기 (regrasp_align_yaw — 돈 네모를 처음 닫힘 그대로 쥐면 모서리를 잡아 칸 턱에 걸림)
  //   새 관측에 면 방향이 있으면 (네모 · yaw_known) 닫힘을 가장 가까운 면 법선으로 돌림 (|돌림| ≤ align_max_deg, 넘으면 그대로).
  //   원기둥 · 면 방향 모름 = 원래 방향 그대로. 켜면 손목 돌림 보정 (rot_mm · rot_robot_mm) 을 원래 자세 (seed) 와의 차이만큼 더함.
  //   false = 원래 닫힘 방향 그대로 · 돌림 보정 없음
  bool align_yaw{false};
  double align_max_deg{45.0};
  Eigen::Vector2d rot_mm{Eigen::Vector2d::Zero()};        // pick_offset_rot_mm [앞, 왼쪽] mm
  Eigen::Vector2d rot_robot_mm{Eigen::Vector2d::Zero()};  // pick_offset_rot_robot_mm [앞, 왼쪽] mm
  // 손목을 45° 넘게 돌려 (다른 면 쌍 · ±90°) 다시 쥘 때만 위 PICK 값 대신 쓰는 손 지렛대 g [손 x, 손 y] mm (regrasp_rot_hand_mm, wrist_lever_offset).
  //   NaN = 쓰지 않음 (모든 돌림에 PICK 값)
  Eigen::Vector2d rot_hand_mm{std::nan(""), std::nan("")};
  // 원기둥 손 안 앞뒤 보정 [손 x 앞, 손 y 왼손가락] mm (pick_offset_cyl_hand_mm): 원기둥 (square false) 이고 깊이 점 중심 관측 (tagged false) 일 때만
  //   새 해의 손 방향으로 base 에 옮겨 같은 닫힘 방향 · 같은 손 앞 방향으로 다시 풂 (안 되면 보정 없이). 0 = 보정 없음
  Eigen::Vector2d cyl_hand_mm{Eigen::Vector2d::Zero()};
};
struct RegraspPlan
{
  bool low_above{false};                                // 위 점을 낮춰서 (윗면 + 20 mm) 풀었음
  bool found{false};                                    // 놓은 선 퍽을 새 장에서 골랐음 (FALLBACK 이어도). false = 선 퍽이 안 보임 (넘어졌을 수 있음)
  RegraspDecision decision{RegraspDecision::WAIT};
  ArmPath tail;              // USE: 새 해로 바꾼 후반 경로
  RegraspObservation obs;    // USE: 쓴 관측
  std::vector<PickPose> poses;
  std::string why;           // WAIT · FALLBACK: 이유
  double close_deg{0.0};     // USE: 쓴 닫힘 방향 [°]
  bool aligned{false};       // USE: 잰 네모 면에 맞춰 닫힘을 돌림
  double puck_yaw_deg{0.0}, turn_deg{0.0};   // 잰 면 방향 · 원래 닫힘에서 면까지 돌림 [°] (align_yaw 이고 면 방향 있을 때)
  Eigen::Vector2d rot_d{Eigen::Vector2d::Zero()};   // USE: 더한 손목 돌림 보정 [m]
  double wrist_turn_deg{0.0};  // USE: 실제 손목 돌림 [°] = 새 해 닫힘축 − 원래 다시 쥐기 자세 닫힘축 (부호 = J5 가 실제로 돈 쪽, 닫힘 각 차이와 180° 다를 수 있음)
  bool rot_lever{false};       // USE: rot_d 를 손 지렛대 (rot_hand_mm) 로 구함 (false = PICK 값 pick_offset_rot_*)
  Eigen::Vector2d cyl_d{Eigen::Vector2d::Zero()};   // USE: 더한 원기둥 손 앞뒤 보정 [m, base xy] (cyl_hand_mm)
  std::string align_note;    // 맞추지 않은 이유 · 돌림 보정을 못 쓴 이유 ("" = 없음)
};
// 원기둥 손 안 앞뒤 보정 (pick_offset_cyl_hand_mm): 손 좌표 [앞 = 손 x, 왼손가락 = 손 y] mm → 쥐는 자세 q 에서 base xy [m]
//   (수직 손 · J5 기본이면 손 x = J1 에서 반지름 바깥 = 쥔 퍽 영상 '앞으로 쥠' 쪽). 손 좌표 정의 = tcp_hand_to_link5
Eigen::Vector2d cyl_hand_shift(const ArmModel & m, const Q5 & q, const Eigen::Vector2d & hand_mm);
// 두 자세의 손 앞 (손 x) 수평 방향이 tol_deg 안으로 같은지 — 같은 닫힘축이라도 J5 가 180° 돌면 손 앞이 뒤집혀 손 좌표 보정이 반대가 됨
bool same_hand_front(const ArmModel & m, const Q5 & a, const Q5 & b, double tol_deg);
RegraspPlan decide_visual_regrasp(const std::vector<RegraspObservation> & observations, const RegraspTarget & target,
  double now, double deadline, const ArmModel & model, const ArmPath & tail);

// ---- 받침에 놓은 원기둥 다시 보기 (control_master advance_park_relook) ----
//   pk_up 뒤 관찰 점 (re_observe, park_look_pose) 에서 같은 대기 (kRegraspObserve · regrasp_visual_*) → 새 장에서 받침 안 선 퍽 (select_regrasp_observation) →
//   잰 중심 − park_look_bias (카메라 치우침) 로 받침 수직 다시 쥐기 (park_re_above · park_re_down · park_re_lift) 를 다시 풂 (높이 · 깊이 · 닫힘 그대로).
//   못 봄 (시간 초과) · 모호 · 받침 가운데에서 max_shift 넘게 · IK 안 됨 = FALLBACK (원래 받침 가운데 쥐기, 새 거절 · 고장 없음)
struct ParkRegraspTarget
{
  Eigen::Vector3d expected{Eigen::Vector3d::Zero()};   // 받침 가운데 (x, y, 퍽 중심 z) base_link
  std::string id;                                       // goal.puck_id ("" = 위치로만)
  double after{0.0}, max_age{0.5}, id_radius{0.06};
  double max_shift_m{0.010};                            // 잰 중심 (치우침 뺀 뒤) ↔ 받침 가운데 한계 (넘으면 FALLBACK, NaN · 0 이하 = 한계 없음)
  Eigen::Vector2d bias{Eigen::Vector2d::Zero()};        // park_look_bias_mm: 받침 퍽을 잰 값 − 실제 [m, base xy] (빼서 씀)
  ParkCfg park;                                         // 받침 (중심 · 바닥 높이 · 다시 쥐는 깊이, 모델 z)
  UprightCfg upright;                                   // 퍽 높이
  Eigen::Vector3d close{Eigen::Vector3d::UnitY()};      // 원래 park_re_down 닫힘축 (그대로)
  Q5 seed{};                                            // 원래 park_re_above (IK 씨앗)
  double held_kg{0.017};                                // park_re_lift 처짐 앞먹임
  double tag_wait_s{0.0};                               // 태그 관측을 먼저: after 뒤 이만큼은 깊이 검출을 안 씀 (태그 장이 오면 바로). 0 = 깊이도 바로
};
struct ParkRegraspPlan
{
  RegraspDecision decision{RegraspDecision::WAIT};
  bool found{false};
  ArmPath tail;                                        // USE: park_re_* 를 바꾼 후반
  RegraspObservation obs;
  bool from_tag{false};                                 // found: 태그 모서리를 받침 퍽 윗면 높이에 내려 잰 중심
  Eigen::Vector2d xy{Eigen::Vector2d::Zero()};          // USE · 범위 넘음: 쥘 점 (잰 중심 − bias) [m]
  double shift_mm{0.0};                                 // 쥘 점 ↔ 받침 가운데
  std::string why;
};
// 후반 경로의 park_re_above · park_re_down · park_re_lift 관절값만 바꿈 (park_upright_again 과 같은 처짐 앞먹임: 빈손 · 빈손 · held_kg). 정확히 3 개가 아니면 false
bool retarget_park_tail(const ArmModel & model, const Q5 & q_above, const Q5 & q_down, double held_kg, ArmPath & tail);
// 받침 다시 보기 판단: tag_observations = 퍽 윗면 태그 모서리를 받침 퍽 윗면 높이에 내려 잰 중심 (한 장 · 크기 검사 통과만, 부르는 쪽이 만듦)
//   태그 관측이 고르기를 통과하면 그것, 아니면 after + tag_wait_s 뒤부터 깊이 검출 (observations, 받침 퍽 높이 ±40 mm). 둘 다 없으면 deadline 까지 WAIT
ParkRegraspPlan decide_park_regrasp(const std::vector<RegraspObservation> & observations,
  const std::vector<RegraspObservation> & tag_observations, const ParkRegraspTarget & target,
  double now, double deadline, const ArmModel & model, const ArmPath & tail);
ParkRegraspPlan decide_park_regrasp(const std::vector<RegraspObservation> & observations, const ParkRegraspTarget & target,
  double now, double deadline, const ArmModel & model, const ArmPath & tail);
}  // namespace manipulation
