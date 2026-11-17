// standup_table.hpp — 누운 네모 퍽 세우기 경로 표 (config/standup_paths.txt) 읽기 · 가장 가까운 경로 고르기 (순수 C++, test/test_standup_table.cpp)
// 표 = 퍽 자리 (한 변 · 중심 x y · 긴축 방향) 마다 Isaac 시뮬에서 강화학습 정책으로 뽑은 관절 경로 (export_standup_table.py).
//   실물은 움직이는 동안 퍽을 못 재서 LOOK 으로 잰 퍽에 가장 가까운 칸의 경로를 그대로 재생
// 한 줄 = 경로 하나: side x y axis rate n release end_x end_y end_face  above[5] grasp[5] q[n × 5] (팀 명령 틱, 처짐 앞먹임 없음)
#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "manipulation/arm_kinematics.hpp"
#include "manipulation/arm_path.hpp"

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

// 먼 · 옆 퍽 붙은 선 퍽 집기 표 (config/farpick/farpick_paths_ok75_nbr_<방향>.txt — 같은 형식, load_standup_table 로 읽음)
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
  // true: 접근축은 da 대신 손끝 방위 (J1 둘레) 가 바뀐 만큼만 돌림 (팔 평면 안 · 기울임 그대로 — 5 축 팔이 정확히 푸는 자세),
  //   닫힘축만 퍽 둘레 da 회전 (farpick_retarget). false = 접근축도 da 회전 (팔 평면 밖이면 IK 가 손끝 · 접근축을 나눠 어긋남)
  bool keep_plane{false};
};

struct RetargetResult
{
  Q5 q{};                    // 옮긴 실제 자세
  double app_err_deg{0.0};   // 접근축 · 닫힘축 어긋남 [°]
  double lifted_mm{0.0};     // tip_min_z 로 올린 높이 [mm]
  std::string fail;          // 실패 때 가장 가까운 해 설명
};

bool standup_retarget(const ArmModel & m, const Q5 & q_table, const StandupRetarget & r, bool scale, RetargetResult & out);

// 먼 누운 네모 퍽 가운데 쥐기 표 (config/lyingpick/lyingpick_paths_ok75.txt — farpick 과 같은 형식, load_standup_table 로 읽음):
//   side = 0.060 (누운 네모), axis = 긴축 [°] (180° 주기), release = 닫는 점. control_master park_or_relocate_plan ① 쥐기
constexpr double kLyingPickSide = 0.060;

// 표 파일: param 이 있으면 그것, 없으면 standup_paths_file 과 같은 폴더의 lyingpick/lyingpick_paths_ok75.txt (farpick_dir 과 같은 규칙), 둘 다 없으면 ""
std::string lyingpick_table_path(const std::string & param, const std::string & standup_file);

struct LyingPickChoice
{
  int row{-1};       // 고른 줄 (한도 안 줄이 없으면 -1)
  int nearest{-1};   // 한도와 상관없이 가장 가까운 줄 (로그, side 0.060 줄이 없으면 -1)
  double dx{0.0}, dy{0.0}, dyaw_deg{0.0};   // 실제 − 표 (row 가 없으면 nearest 기준), 긴축 차는 180° 주기 (−90 … 90)
  double dist_mm{0.0};
};

// 누운 퍽 (x, y, 긴축 axis_deg) 에 쓸 줄: side 0.060 줄 중 거리 max_dist_mm · 긴축 차 max_deg 안에서 가장 가까운 것
//   (가까움 = nearest_standup 과 같은 (dx / 7.5 mm)² + (dy / 10 mm)² + (d각 / 7.5°)²)
LyingPickChoice choose_lyingpick(const std::vector<StandupPath> & table, double x, double y, double axis_deg, double max_dist_mm, double max_deg);

// 표 한 줄 → 옮기기 쥐기 경로 점 (farpick_plan 과 같은 변환): 점 1 … 끝을 standup_retarget (덜 뻗기 없음) → 처짐 앞먹임 (닫는 점부터 held_kg) → 팀 명령.
//   점 1 = 위로 가며 엶 (open, grip_early = early_open) · 속도 시간표, 그 뒤 = 표 간격 dt_s (건너뛴 점만큼 길게).
//   이름: 닫기 전 "lp_in<i>" (빈손으로 내려온 길, lyingpick_free_node) · 닫는 점 "reloc_grasp" (close) ·
//   닫은 뒤 "lp<i>" · 마지막 "reloc_lift" (멈춤). IK 안 되는 점은 건너뜀 (닫는 점 · 닫은 뒤 전부가 안 되면 실패). 첫 점 (LOOK) 은 안 넣음
//   shift [m, base_link (x, y, z)]: 퍽 근처 구간만 손끝을 그만큼 더 옮김 (lyingpick_dxy_mm · lyingpick_dz_mm, 자세 방향 그대로 IK 다시).
//     닫기 전 점은 표 손끝이 닫는 점 손끝에서 kLyingShiftIn 안이면 다, kLyingShiftOut 밖이면 0, 사이는 직선.
//     닫는 점부터 끝 (들기) 까지 다 — 쥔 채 들기 시작에서 손끝이 튀지 않게
struct LyingPickPath
{
  std::vector<PathNode> nodes;
  std::vector<Q5> q;  // 점마다 실제 자세 (처짐 앞먹임 전, nodes 와 같은 순서)
  Q5 q_close{};      // 닫는 점 실제 자세 (처짐 앞먹임 전)
  int skipped{0};
};
constexpr double kLyingShiftIn = 0.050, kLyingShiftOut = 0.080;
// 닫기 전 멈춤 (hover): hover_m > 0 이면 닫는 점 바로 위 hover_m [m] (같은 손 방향, 수직) 에서 멈춰 hover_s 기다린 뒤 descend_s 동안 곧게 내려와 닫음
//   (닫는 점 손끝에서 hover_m + 3 mm 안의 표 점은 뺌 — 기울인 손이 비스듬히 끝까지 내려오며 퍽 옆면을 스치지 않게). 0 = 표 그대로
struct LyingHover
{
  double hover_m{0.0};
  double hover_s{0.0};
  double descend_s{0.0};
};
bool lyingpick_path(const ArmModel & m, const StandupPath & e, const StandupRetarget & rt, double held_kg, double dt_s, bool early_open,
  LyingPickPath & out, std::string & why, const Eigen::Vector3d & shift = Eigen::Vector3d::Zero(), const LyingHover & hover = LyingHover{});

// 먼 퍽 표 한 줄 → 재생 점 (control_master farpick_plan). 첫 점 (LOOK) 은 안 넣음. 이름: 표 점 "far<i>" · 닫는 점 "far_grasp" (close) · 끝 "far_lift" (멈춤)
//   첫 재생 점 = 위로 가며 엶 (open, grip_early = early_open) · 속도 시간표, 그 뒤 = 표 간격 dt_s (건너뛴 점만큼 길게). 처짐 앞먹임 = 닫는 점부터 held_kg
//     — 점 1 이 IK 안 돼 건너뛰어져도 그다음 첫 점에서 엶 (안 그러면 그리퍼가 닫힌 채 내려감)
//   local = false: 점 1 … 끝을 다 standup_retarget (rt 그대로: 이동 + 퍽 둘레 da 회전), IK 안 되는 점은 건너뜀 (닫는 점 · 닫은 뒤 전부가 안 되면 실패)
//   local = true (farpick_retarget): 퍽 근처 쥐기 구간만 옮김 — 닫기 전 점은 표 손끝이 닫는 점 손끝에서 kFarRetargetIn 안이면 다,
//     kFarRetargetOut 밖이면 0 (표 관절 그대로, IK 없음 — LOOK 쪽 먼 점을 퍽 둘레로 돌리면 IK 가 안 됨), 사이는 직선 (이동 · 회전 둘 다 그 비율).
//     닫는 점부터 끝 (들기) 까지 다. 접근축은 팔 평면 안 (keep_plane), 닫힘축은 da 회전.
//     거부 (false, why — control_master 는 local = false 로 다시): 다 옮기는 점 (닫기 전 kFarRetargetIn 안 · 닫는 점 · 닫은 뒤) 의 IK 실패,
//     이웃 두 재생 점 관절 변화가 표의 같은 구간 변화와 max_step_deg 넘게 다름 (IK 갈래 바뀜 · 튐, 부호 포함)
constexpr double kFarRetargetIn = 0.050, kFarRetargetOut = 0.080;
struct FarpickPath
{
  std::vector<PathNode> nodes;
  std::vector<Q5> q;      // 점마다 실제 자세 (처짐 앞먹임 전, nodes 와 같은 순서)
  std::vector<int> idx;   // 점마다 표 번호
  Q5 q_close{};           // 닫는 점 실제 자세
  int skipped{0};
  int first{-1};          // 첫 재생 점 표 번호
  int moved{0};           // (local) 옮긴 점 수 (나머지는 표 그대로)
  double close_dj_deg{0.0};   // 닫는 점 관절 변화 최대 (표 → 옮김) [°]
  double step_excess_deg{0.0};   // 이웃 두 점 관절 변화 − 표 변화 최대 [°]
};
bool farpick_path(const ArmModel & m, const StandupPath & e, const StandupRetarget & rt, double held_kg, double dt_s, bool early_open, bool local,
  double max_step_deg, FarpickPath & out, std::string & why);

// 먼 퍽 표 손목 돌림 보정: 보통 PICK 과 같이 손목을 기본 (접선) 에서 돌려 쥐면 쥐는 점을 옮김
//   (wrist_rot_offset: pick_offset_rot_mm, |돌림| ≥ 45° 면 + pick_offset_rot_robot_mm — 90° 돌리면 손가락 가운데가 약 15 mm 왼쪽).
//   표 닫는 점 (fp.q_close) 의 돌림으로 보정 d 를 재고 옮길 목표 rt.to + d 로 다시 풂 (local 은 처음 풀 때와 같게).
//   local 다시 풀기가 거부되면 (튐 · 쥐기 구간 IK) 경로 전체를 옮기는 방식 (local = false) 으로 한 번 더.
//   다시 풀기가 안 되거나 닫는 점 손목 갈래가 바뀌면 (|돌림| 45° 기준, PICK 과 같음) 보정 없이 (fp 그대로). d 가 0 이면 아무것도 안 함
struct FarpickRotShift
{
  Eigen::Vector2d d{Eigen::Vector2d::Zero()};   // 보정 [m, base xy]
  double rot_deg{0.0};     // 표 닫는 점 손목 돌림 (기본 = 접선에서) [°]
  bool used{false};        // 보정한 경로로 바꿈 (fp)
  bool whole{false};       // used 이고 local 이 거부돼 경로 전체를 옮겨 풂
  std::string note;        // 못 쓴 이유 ("" = 없음) · whole 이면 local 거부 이유
};
FarpickRotShift farpick_rot_shift(const ArmModel & m, const StandupPath & e, const StandupRetarget & rt, double held_kg, double dt_s,
  bool early_open, bool local, double max_step_deg, const Eigen::Vector2d & rot_mm, const Eigen::Vector2d & robot_mm, FarpickPath & fp);

// 누운 퍽 표 손목 돌림 보정 (farpick_rot_shift 와 같은 식 · 같은 값): 표 닫는 점 (lp.q_close) 의 손목 돌림으로 보정 d 를 재고
//   옮길 목표 rt.to + d 로 표 경로 전체를 다시 풂 (shift · hover 는 처음 풀 때와 같게). 다시 풀기가 안 되거나 닫는 점 손목 갈래가 바뀌거나
//   (|돌림| 45° 기준, PICK 과 같음) 표 점이 2 개 넘게 더 건너뛰어지면 (들기 · 내려가기가 짧아짐) 보정 없이 (lp 그대로). d 가 0 이면 아무것도 안 함
struct LyingRotShift
{
  Eigen::Vector2d d{Eigen::Vector2d::Zero()};   // 보정 [m, base xy] (쥐는 손끝을 퍽 중심 + d 로)
  double rot_deg{0.0};     // 표 닫는 점 손목 돌림 (기본 = 접선에서) [°]
  bool used{false};        // 보정한 경로로 바꿈 (lp)
  std::string note;        // 못 쓴 이유 ("" = 없음)
};
LyingRotShift lyingpick_rot_shift(const ArmModel & m, const StandupPath & e, const StandupRetarget & rt, double held_kg, double dt_s,
  bool early_open, const Eigen::Vector3d & shift, const LyingHover & hover, const Eigen::Vector2d & rot_mm, const Eigen::Vector2d & robot_mm,
  LyingPickPath & lp);

// 빈손 되돌아가기가 지나가도 되는 표 점 (닫기 전, 빈손으로 내려온 길) · 닫는 점 reloc_grasp (표 닫기가 빈손이면 위 점 → 닫는 점 →
//   내려온 길 거꾸로 → 관찰 자세, farpick 의 far_grasp 와 같음)
bool lyingpick_free_node(const std::string & name);
// 먼 퍽 표 (farpick_plan) 의 재생 점: "far<i>" (표 i 번째 점) · 닫는 점 "far_grasp". 들어 올린 끝 "far_lift" 는 아님
//   (빈손 되돌아가기: 위 점 → far_grasp → 내려온 표 점 거꾸로 → 관찰 자세)
bool farpick_free_node(const std::string & name);

// 쥔 퍽 중심의 손 안 자리 [m]: (접근축 성분, b = 접근축 × 닫힘축 성분). 닫힘축 성분은 손가락이 닫히며 0 으로 봄
//   (표 보정이 퍽 가운데에서 긴축 쪽으로 비켜 쥐면 그만큼 손 안에 남음 → 내려놓은 퍽 중심 = held_center_at)
Eigen::Vector2d held_center_in_hand(const Grip & g, const Eigen::Vector3d & center);
// 표 쥐기 손가락 사이 가운데 맞춤: 닫는 점 손끝이 퍽 중심에서 tip_off [m, base xy] 어긋났을 때, 닫힘축 (수평 성분) 방향 어긋남만 되돌리는 xy 옮김.
//   긴축 방향 (닫힘축에 수직) 은 그대로 (표가 고른 쥐는 자리). 닫힘축이 거의 수직이면 0
Eigen::Vector2d lying_recenter_shift(const Grip & g, const Eigen::Vector2d & tip_off);
Eigen::Vector3d held_center_at(const Grip & g, const Eigen::Vector2d & in_hand);

}  // namespace manipulation
