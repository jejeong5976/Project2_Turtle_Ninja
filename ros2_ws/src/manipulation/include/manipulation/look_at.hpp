// look_at.hpp — 손목 카메라로 한 점 보기 LOOK_AT (순수 계산 · test/test_look_at.cpp)
//   목표 점 t (base_link) 를 손목 컬러 카메라 광축이 지나고, 카메라가 t 에서 standoff 만큼 떨어지고,
//   화면이 수평 (roll_period 90° = 가로 · 세로 둘 다 됨) 이 되는 팔 자세를 찾음
//   관절 5 개로 6 조건 (위치 3 + 방향 2 + 화면 돌림 1) 을 다 못 맞춤 → 광축 · 거리 · 돌림을 세게, 정면 (보는 방향) 은 약하게 맞춤
//   - 모델 = arm_kinematics.hpp ArmModel (모델 link5 = URDF wrist_link, 모델 base z = base_link z − (base_z_floor − team_base_z_floor))
//   - 카메라 = link5 → d435_color_optical_frame 고정 변환 T5c (URDF camera_joint, control_master 는 TF 로 읽음)
//   - 잔차 (광축 · 거리 · 정면 · 돌림 · 씨앗에서 움직임) 의 Levenberg–Marquardt, 수치 미분, 관절 한계 안. 씨앗 여러 개 (J1 을 목표 쪽으로)
//   - 검사: 광축 오차 ≤ max_axis_err · 거리 [min, max] · 정면 각 ≤ max_obliquity · 돌림 ≤ max_roll · 틱 범위 · 차체 · 적재함 · 기둥 · 바닥 여유
//           광각 시야선 (선택): 광각 카메라 → 신호등 방향 선에서 팔 점이 occl_margin 넘게 떨어져야 함 (신호등 가림 금지)
//   - 경로: 시작 자세 → 해 관절 직선, path_step_deg 마다 같은 검사 (look_at_path_clear)
//   - 기준은 LookAtParams (control_master 파라미터 look_at_*). 상자는 URDF 충돌 상자 + 여유 (MoveIt 아님), 경기장 물체는 안 봄

#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "manipulation/arm_kinematics.hpp"

namespace manipulation
{

struct LookAtParams
{
  double standoff_m{0.30};          // 바라는 카메라 ↔ 목표 거리 [m] (slot d= 가 없을 때, control_master 기본 0.15)
  double standoff_min_m{0.20};      // 이 범위 밖이면 실패 [m] (control_master 기본 0.12)
  double standoff_max_m{0.45};
  double max_axis_err_deg{2.0};     // 광축 ↔ 목표 방향 최대 오차
  double max_obliquity_deg{45.0};   // 광축 ↔ 바라는 보는 방향 (벽 법선 반대) 최대 각
  double max_roll_deg{15.0};        // 화면 돌림 (roll_period 로 접은 값) 최대
  double roll_period_deg{90.0};     // 90 = 가로 · 세로 막대 둘 다 됨, 180 = 가로만
  double min_z_m{0.12};             // base_link z: 손목 · 카메라 · 손끝 점 최저 (바닥 위 0.153 m)
  // base_link 금지 상자 [x0, x1, y0, y1, z_top] (바닥부터 z_top 까지). 차체 = URDF 충돌 상자 (위 0.131) + 여유
  std::array<double, 5> body_box{{-0.14, 0.15, -0.14, 0.14, 0.17}};
  // 적재함 (판 z 0.138, 앞 끝 x 0.096) + 칸에 선 퍽 (0.06)
  std::array<double, 5> rack_box{{-0.13, 0.10, -0.14, 0.14, 0.21}};
  // 광각 기둥 (프로파일 20×20, URDF profile_column 충돌 상자) + 위 카메라 받침
  std::array<double, 5> mast_box{{-0.12, -0.08, -0.12, -0.08, 0.62}};
  double clear_pad_m{0.03};         // 팔 점 둘레 여유 (링크 반 굵기 0.025 + 0.005)
  double occl_margin_deg{6.0};      // 광각 → 신호등 시야선과 팔 점 사이 최소 각 (점 반지름 clear_pad 포함)
  double path_step_deg{2.0};        // 경로 검사 간격
  // 가중치 (광축 · 거리 · 돌림이 세고 정면은 약함)
  double w_axis{30.0}, w_dist{6.0}, w_obl{3.0}, w_roll{3.0}, w_move{0.05}, w_occl{20.0};
  // 팔을 접은 쪽으로 (뻗은 자세로 오래 있으면 J2 과부하). J2 중력 토크 [N·m] × w_tau 를 잔차에, max 넘으면 거절 (0 = 안 봄)
  double w_tau{1.0};
  double max_j2_torque_nm{0.0};
  // 손끝 · 카메라 ↔ 목표 벽 면 (목표를 지나고 보는 방향 가로 성분이 법선) 최소 거리, 바란 거리와 차이 상한
  double wall_margin_m{0.03};       // [m], 0 이하 = 안 봄
  double max_dist_err_m{0.03};      // |이룬 거리 − d| 상한 (0 이하 = 범위만)
  double w_wall{10.0};
  // 목표가 화면 (cx + aim_du, cy + aim_dv) [px] 에 오게 겨눔 — 손가락이 화면 오른쪽 아래 (x 420–640, y 290–480) 를 가림
  //   광축 오차 = 그 화소 광선 ↔ 목표 방향. 0, 0 = 가운데
  double aim_du_px{0.0}, aim_dv_px{0.0};
  double fx{606.32}, fy{606.19};
  int max_iter{80};
};

// slot 글: "d=0.30,vyaw=12.5,vpitch=0,dj1=4,dj4=-3,lyaw=-15,lpitch=5" (빈 값 · 모르는 키는 기본)
//   d       카메라 ↔ 목표 거리 [m]
//   vyaw    보는 방향 (카메라 → 목표) 수평각 [°, base_link, 반시계 +]. 없으면 J1 축 → 목표
//   vpitch  보는 방향 아래로 + [°]. 없으면 0 (수평으로 봄 — 벽 바코드)
//   dj1 dj4 해 자세에서 관절 1 · 4 를 이만큼 더 돌림 [°] (다시 읽기 흔들기, 모델 관절 · 실제 팀 관절 부호는 반대일 수 있음)
//   lyaw lpitch 광각 카메라 → 신호등 방향 [°, base_link] — 있으면 팔이 그 시야선을 가리지 않게
//   lcone   그 시야선 둘레 원뿔 반각 [°] (없으면 look_at_occl_margin_deg)
struct LookAtGoal
{
  Eigen::Vector3d target{Eigen::Vector3d::Zero()};   // base_link
  double standoff_m{std::numeric_limits<double>::quiet_NaN()};
  double view_yaw{std::numeric_limits<double>::quiet_NaN()};   // [rad]
  double view_pitch{std::numeric_limits<double>::quiet_NaN()}; // [rad] 아래 +
  double dj1{0.0}, dj4{0.0};                                   // [rad]
  bool has_light{false};
  double light_yaw{0.0}, light_pitch{0.0};                     // [rad] 위 + (광각 → 신호등)
  double light_cone_deg{std::numeric_limits<double>::quiet_NaN()};   // 시야 원뿔 반각 [°] (NaN = occl_margin_deg)
};

bool parse_look_at_slot(const std::string & slot, LookAtGoal & g, std::string & why);

struct LookAtSolution
{
  bool ok{false};
  Q5 q{};                      // 모델 관절 (실제, 처짐 전)
  double axis_err_deg{180.0};
  double dist_m{0.0};
  double obliquity_deg{180.0};
  double roll_deg{180.0};
  double cost{1e18};
  double j2_torque_nm{0.0};    // 이 자세 J2 중력 토크 (빈손)
  double wall_clear_m{0.0};    // 손끝 · 카메라 · 손목 중 벽 면에 가장 가까운 거리 (로봇 쪽 +)
  Eigen::Vector3d cam_pos{Eigen::Vector3d::Zero()}, cam_axis{Eigen::Vector3d::UnitX()};
  std::string why;
};

// 모델 base 와 base_link 높이 차 [m] (base_link z = 모델 z + 이 값)
inline double model_to_base_z(const ArmModel & m) {return m.base_z_floor - m.team_base_z_floor;}

// 자세 q 의 카메라 광학 프레임 (base_link). T5c = link5 → 광학 프레임
Eigen::Isometry3d look_camera_pose(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & q);

// 검사에 쓰는 팔 점 (base_link): 관절 3 · 4 · 5 뒤 원점, 손끝, 카메라, 관절 4 ~ 손끝 가운데
std::vector<Eigen::Vector3d> look_arm_points(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & q);

// 화면 돌림 [rad]: 광학 x 가 수평 · y 가 아래면 0. period 로 접어 [−period/2, period/2]
double look_roll(const Eigen::Isometry3d & cam, double period_rad);

// 실패 이유 글 앞 꼬리표 (미션이 읽음): [REACH] 관절 · 광축 · 거리 · 비스듬 · 돌림 · J2 토크 (몸을 옮기면 될 수 있음)
//   [WALL] 손끝이 벽에 너무 가까움 (몸을 뒤로) · [BODY] 차체 · 적재함 · 기둥 · 바닥 · [OCCL] 광각 → 신호등 원뿔을 가림 · [START] 시작 자세
// 한 자세 여유 검사: 바닥 · 차체 · 적재함 · 기둥 상자 · 목표 벽 면 · (light 면) 광각 시야선. wide_cam = 광각 광학 원점 (base_link)
bool look_at_clear(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & q, const LookAtParams & p,
  const LookAtGoal & g, const Eigen::Vector3d & wide_cam, std::string & why);

// 시야 원뿔만 (몸 · 벽 안 봄) — control_master 가 녹화 look_barcode 경로를 볼 때
bool look_at_cone_clear(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & q, const LookAtParams & p,
  const LookAtGoal & g, const Eigen::Vector3d & wide_cam, std::string & why);
// 손목 · 손끝 · 카메라 중 목표 벽 면에 가장 가까운 거리 [m] (로봇 쪽 +)
double look_at_wall_clear(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & q, const LookAtGoal & g);

// 관절 직선 경로 검사 (path_step_deg 마다, 끝점 포함). 벽 면은 끝 자세만 (지나는 길은 목표에서 멂).
//   시야 원뿔 (light): 지나는 길도 봄 — 원뿔 밖으로 나온 뒤 다시 들어가면 실패 (시작이 원뿔 안이면 나올 때까지는 봐줌)
bool look_at_path_clear(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & from, const Q5 & to,
  const LookAtParams & p, const LookAtGoal & g, const Eigen::Vector3d & wide_cam, std::string & why);

// 풀기: start = 지금 (또는 출발할) 자세 (모델 관절). 씨앗은 start · m.template_q · extra_seeds 각각 + J1 을 목표 쪽으로 돌린 것들
//   ok = 모든 검사 통과 (start → q 경로 포함, check_path 면). 못 풀면 가장 나은 해 + why
LookAtSolution solve_look_at(const ArmModel & m, const Eigen::Isometry3d & T5c, const LookAtGoal & g,
  const LookAtParams & p, const Q5 & start, const Eigen::Vector3d & wide_cam,
  const std::vector<Q5> & extra_seeds = {}, bool check_path = true);

// 해 평가만 (풀기 없이): 광축 · 거리 · 정면 · 돌림 값 채움, ok 는 기준 통과 여부 (여유 · 경로 제외)
LookAtSolution eval_look_at(const ArmModel & m, const Eigen::Isometry3d & T5c, const LookAtGoal & g,
  const LookAtParams & p, const Q5 & q);

}  // namespace manipulation
