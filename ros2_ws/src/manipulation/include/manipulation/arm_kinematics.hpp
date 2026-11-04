// arm_kinematics.hpp — 실물 집기 계산기와 같은 팔 기구학 (순수 C++, test/test_arm_kinematics.cpp)
// =====================================================================
// 모델 = 실물 시험에 쓴 기구학 (config/arm_kinematics.yaml, 10-27 배포 URDF 의 팔 사슬 + 손끝 · 틱 변환표 · 처짐 계수)
//   좌표: 모델 base (바닥 위 base_z_floor) — 로봇 base_link 와 x · y 같고 z 만 다름 (model_z = base_link_z + team_base_z_floor − base_z_floor)
//   관절 q [rad] 는 모델 관절 (팀 ROS 관절값과 다름) → 틱 → 팀 명령 rad = (틱 − 영점) · 2π/4096
// 계산
//   - FK: base → arm_base → 관절 5 개, 손끝 · 접근축 · 닫힘축 = link5 좌표 상수
//   - IK: 손끝 위치 · 접근축 · 닫힘축(90° 마다 같음) 잔차의 Levenberg–Marquardt, 관절 한계 안. 씨앗 여러 개
//   - 처짐 앞먹임: 실제 관절 = 엔코더 + c · τ (J2 · J3 · J4 중력 토크) → 명령 = 실제 − c · τ
//   - 퍽 집기 자세: 수직 손, 손끝이 퍽 윗면 위 h [mm] (위 40 · 10 · 쥐기 −40, control_master pick_heights_mm), 손목 롤은 받침 자세에 가까운 해
//     누운 네모 퍽: 같은 수직 손, 닫힘축 = lying_close_rad (긴축에 수직), 윗면 = 누운 높이 (control_master pick_lying_heights_mm)

#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace manipulation
{

struct JointMapEntry
{
  double ref_tick;
  double q_deg_at_ref;
  double sign;
};

struct ArmModel
{
  Eigen::Matrix4d T_base_armbase{Eigen::Matrix4d::Identity()};
  std::array<Eigen::Matrix4d, 5> joint_T{};
  std::array<Eigen::Vector3d, 5> axis{};
  Eigen::Vector3d tip5{Eigen::Vector3d::Zero()}, app5{Eigen::Vector3d::Zero()}, close5{Eigen::Vector3d::Zero()};
  Eigen::Vector2d j1_xy{Eigen::Vector2d::Zero()};
  std::array<double, 5> lo{}, hi{}, template_q{};
  double q5_rest{0.0};
  std::array<JointMapEntry, 5> jm{};
  std::array<std::array<double, 2>, 5> tick_range{};
  double sag_c{0.0};                  // [rad / N·m]
  double base_z_floor{0.072};         // 모델 base 높이 (바닥 위)
  double team_base_z_floor{0.033};    // 로봇 base_link 높이 (바닥 위)
  std::array<double, 5> zero_tick{{4088, 2048, 2048, 2048, 2048}};   // 팀 하드웨어 영점 (hardware_map zero_tick)
};

using Q5 = std::array<double, 5>;
using Ticks = std::array<int, 5>;

struct Grip
{
  Eigen::Vector3d tip, app, close;
};

struct IkSolution
{
  Q5 q;
  double pos_mm, app_deg, roll_deg;
};

// 관절 5 개 뒤 좌표계 (arm_link1 .. arm_link5_wrist_roll, 모델 base 기준)
std::array<Eigen::Matrix4d, 5> arm_frames(const ArmModel & m, const Q5 & q);
Grip arm_grip(const ArmModel & m, const Q5 & q);

// 목표 (손끝 p, 접근축 a, 닫힘축 c) — seeds 가 비면 J1 13 × J5 3 + 무작위 24 씨앗
//   roll_period: 닫힘축을 몇 ° 마다 같게 볼지 (90 = 네모 퍽 대칭, 실물 계산기와 같음 · 180 = 손가락 방향을 정확히)
std::vector<IkSolution> arm_ik(
  const ArmModel & m, const Eigen::Vector3d & p, const Eigen::Vector3d & a, const Eigen::Vector3d & c,
  const std::vector<Q5> & seeds = {}, double roll_period = 90.0);

Ticks q_to_ticks(const ArmModel & m, const Q5 & q);
Q5 ticks_to_q(const ArmModel & m, const Ticks & t);
bool ticks_in_range(const ArmModel & m, const Ticks & t);
// 팀 명령 rad (관절 1..5)
std::vector<double> ticks_to_command(const ArmModel & m, const Ticks & t);

// 중력 토크 τ2 · τ3 · τ4 [N·m] (held_kg = 손끝에 쥔 물체)
std::array<double, 3> sag_torques(const ArmModel & m, const Q5 & q, double held_kg);
// 원하는 실제 관절 → 엔코더가 가리켜야 할 관절 (처짐 앞먹임)
Q5 encoder_goal_q(const ArmModel & m, const Q5 & q_true, double held_kg);

struct PickPose
{
  std::string name;
  Ticks ticks;   // 실제 목표
  Ticks enc;     // 처짐 앞먹임 명령
  double tilt_deg{0.0};   // 손을 바깥쪽으로 기울인 각 (0 = 수직 손)
};

// 퍽 (모델 x, y, 윗면 z, 면 돌림 yaw [°]) 위 heights [mm] 마다 수직 손 자세 (앞 높이 해를 씨앗으로). 실패면 빈 목록 + why
//   명령(enc)은 실물 계산기처럼 쥔 물체 없이 계산
//   close_deg: 닫힘축 방향 [°] 을 직접 줌 (이웃 퍽을 보고 고른 방향, grasp_select). NaN 이면 위 규칙
//   max_tilt_deg: 손을 10° 씩 이만큼까지 바깥(J1 → 퍽 방향)으로 기울여 봄. 0 = 수직 손만 (예전)
//     기울여도 손끝은 모든 높이에서 퍽 중심축 위 (쥔 퍽을 수직으로 들고 내림 — 옆 퍽 쪽으로 쓸지 않게)
//   prefer_tilt_deg · max_dev_deg: 이 기울임에 가까운 것부터, 차이가 max_dev 안인 것만 (PLACE: 쥘 때 기울임과 맞춤)
//   해는 실제 틱 · 처짐 앞먹임 명령 틱 둘 다 관절 범위 안이어야 함
std::vector<PickPose> pick_poses(
  const ArmModel & m, double x, double y, double top_z, double yaw_deg, const std::vector<double> & heights_mm,
  std::string & why, double close_deg = std::nan(""), double max_tilt_deg = 0.0, double prefer_tilt_deg = 0.0,
  double max_dev_deg = 1e9);

// 기본 닫힘축 방향 [rad] (pick_poses 가 close_deg 없이 쓰는 방향): 원기둥 = J1 → 퍽 방향의 수직, 네모 = 그에 가장 가까운 면 법선
double default_close_rad(const ArmModel & m, double x, double y, double yaw_deg);

// 누운 네모 퍽 (긴축 수평 방향 axis_deg [°]) 의 닫힘축 [rad] = 긴축에 수직인 수평 방향 → 수직 손가락이 긴 옆면 두 개를 쥐고
//   손가락 폭 방향 = 긴축. 180° 주기 중 기본 닫힘축 (J1 → 퍽 방향의 수직) 에 가까운 쪽 (손목이 덜 돎). pick_poses 의 close_deg 로 줌
double lying_close_rad(const ArmModel & m, double x, double y, double axis_deg);

}  // namespace manipulation
