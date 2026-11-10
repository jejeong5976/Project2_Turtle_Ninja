// standup_table.hpp — 누운 네모 퍽 세우기 경로 표 (config/standup_paths.txt) 읽기 · 가장 가까운 경로 고르기 (순수 C++, test/test_standup_table.cpp)
// =====================================================================
// 표 = 퍽 자리 (한 변 · 중심 x y · 긴축 방향) 마다 Isaac 시뮬에서 강화학습 정책을 돌려 뽑은 관절 경로
//   (robot_sim_rl/rl/standup_openloop.py record → export_standup_table.py). 실물은 움직이는 동안 퍽을 못 재서
//   LOOK 으로 잰 퍽에 가장 가까운 칸의 경로를 그대로 재생 (시뮬 재생 시험: 퍽 ±4 mm · ±4° 어긋나도 87 % 섬)
// 한 줄 = 경로 하나: side x y axis rate n release end_x end_y end_face  above[5] grasp[5] q[n × 5] (팀 명령 틱, 처짐 앞먹임 없음)
#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "manipulation/arm_kinematics.hpp"

namespace manipulation
{

struct StandupPath
{
  double side{0.0};           // 누운 네모 퍽 한 변 [m]
  double x{0.0}, y{0.0};      // 누운 퍽 중심 [m, base_link]
  double axis_deg{0.0};       // 긴축 방향 [°] (180° 마다 같음)
  double rate{-1.0};          // 시뮬 재생 시험 성공률 (모르면 -1)
  int release{0};             // 이 점 (q 번호) 에 도착하면 그리퍼를 엶
  double end_x{0.0}, end_y{0.0};   // 시뮬에서 퍽이 선 자리 [m, base_link]
  double end_face_deg{0.0};   // 선 퍽 면 방향 [°]
  std::array<int, 5> above{}, grasp{};   // 퍽 위 40 mm (열림) · 쥐기 (−20 mm) [팀 명령 틱]
  std::vector<std::array<int, 5>> q;     // 세우기 경로 [팀 명령 틱]
};

// 표 파일을 읽음. '#' 줄 · 빈 줄은 건너뜀. 실패하면 false (why)
bool load_standup_table(const std::string & file, std::vector<StandupPath> & out, std::string & why);

// (x, y, 긴축, 한 변) 에 가장 가까운 경로. 한 변은 가장 가까운 크기만, 거리 = (dx / step_x)² + (dy / step_y)² + (d각 / step_deg)²
//   d각 은 180° 마다 같음. 없으면 -1. dx · dy · dyaw 에 차이를 돌려줌 (실제 − 표)
int nearest_standup(const std::vector<StandupPath> & table, double x, double y, double axis_deg, double side,
  double & dx, double & dy, double & dyaw_deg, double step_x = 0.0075, double step_y = 0.01, double step_deg = 7.5);

// 먼 · 옆 퍽 붙은 선 퍽 집기 표 (config/farpick/farpick_paths_ok75_nbr_<방향>.txt, 11-10 강화학습 — 같은 형식, load_standup_table 로 읽음)
//   이웃 퍽 방향 [°, base_link] → 가장 가까운 45° 표 이름 (000 … 315), 이웃 없으면 nbr_none
std::string farpick_table_name(bool has_nbr, double nbr_dir_deg);

// nearest_standup 과 같되 축 주기 period_deg (네모 90, 0 이하 = 축 무시 — 원기둥 · 면 모르는 네모)
int nearest_farpick(const std::vector<StandupPath> & table, double x, double y, double axis_deg, double side,
  double & dx, double & dy, double & dyaw_deg, double period_deg, double step_x = 0.0075, double step_y = 0.01, double step_deg = 7.5);

// 표 자세 하나를 실물에 맞게 옮김 (control_master 세우기 · 시뮬 재생 시험이 같은 계산)
//   1) 표 칸 퍽 (from) → 실제 퍽 (to): 수평 이동 + 퍽 둘레 da 회전 (손끝 · 접근축 · 닫힘축)
//   2) scale 이 켜진 점 (쥔 뒤 경로) 은 anchor (쥐는 손끝) 에서 수평 거리를 reach_scale 배로 (덜 뻗기)
//   3) 손끝을 dz 위아래로, tip_min_z 아래면 그 높이까지 올림 (표는 정책의 목표 관절이라 박스 윗면 아래를 가리키는 점이 있음)
//   그 자세에서 가장 가까운 IK 해. 5 축 팔은 기울인 손을 팔 평면 밖으로 못 돌려 접근축이 몇 ° 어긋날 수 있음 → 손끝 1 mm · 접근축 10° 까지
struct StandupRetarget
{
  Eigen::Vector2d from{Eigen::Vector2d::Zero()}, to{Eigen::Vector2d::Zero()};   // [m, 모델 xy = base_link xy]
  double da{0.0}, dz{0.0};             // [rad] · [m]
  double tip_min_z{-1e9};              // [m, 모델]
  Eigen::Vector2d anchor{Eigen::Vector2d::Zero()};   // 덜 뻗기 기준 (옮긴 뒤 쥐는 손끝 xy)
  double reach_scale{1.0};             // 1 = 그대로
};

struct RetargetResult
{
  Q5 q{};                    // 옮긴 실제 자세
  double app_err_deg{0.0};   // 접근축 · 닫힘축 어긋남 [°]
  double lifted_mm{0.0};     // tip_min_z 로 올린 높이 [mm]
  std::string fail;          // 실패 때 가장 가까운 해 설명
};

bool standup_retarget(const ArmModel & m, const Q5 & q_table, const StandupRetarget & r, bool scale, RetargetResult & out);

}  // namespace manipulation
