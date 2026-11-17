// 누운 네모 퍽 세우기 (그리퍼 받침, control_master stand_lying_mode park) 의 자세 계산 — ROS 없음 (시뮬 도구 · 테스트와 같은 코드)
//   1) lying_grasp: 누운 퍽을 (반지름 쪽이면 바깥 끝, 접선 쪽이면 J1 쪽 끝을) 손 기울임 0 · 10 · 20 · 30° 중 닿는 것으로 쥠.
//      닫힘축 = 손 축 × 긴축 (손가락이 긴 옆면 두 개). 손은 손 축을 따라 들어감
//   2) park_nodes: 쥔 퍽을 받침 위에서 긴 쪽 끝이 J1 쪽 아래로 elev_deg 가 되게 넣고 놓음 →
//      손 빼고 오므려 받침 바닥 위 push_h 로 밀어 세움 → 수직 손으로 다시 쥐고 듦
//   좌표 = 팔 모델 (xy = base_link, z = base_link − 0.039)
#pragma once

#include <array>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "manipulation/arm_kinematics.hpp"

namespace manipulation
{

// 길이는 모두 [m]
struct LyingGraspCfg
{
  double side{0.0348};           // 누운 높이 = 네모 단면 한 변
  double length{0.060};          // 긴축 길이
  double grip_offset{0.025};     // 쥐는 점 = 중심에서 쥐는 끝 쪽으로
  // 쥐는 깊이: 수직 손 손끝 = 윗면 − depth. 기울인 손도 손가락 가장 낮은 점이 이 높이 아래로 안 감
  double depth{0.045};
  double open_inner{0.0255};     // 열린 손가락 안쪽 면 (손 축에서, 닫힘 방향)
  double open_outer{0.0437};     // 바깥 면
  double finger_half_w{0.0164};  // 손가락 폭 반 (손 b 방향)
  double min_margin{0.0025};     // 들어갈 때 열린 손가락 ↔ 퍽 최소 여유 (닫힘 방향 한쪽)
  double min_depth{0.020};       // 굴린 퍽 윗면 → 손끝 최소
  double max_roll_deg{12.0};     // 퍽이 손가락에 맞으며 굴러도 되는 최대 (시뮬: 20° 는 들다 놓침)
  std::vector<double> tilts_deg{0.0, 10.0, 20.0, 30.0};
};

struct LyingGrasp
{
  std::vector<PickPose> poses;   // 위 · 내림 · 쥐기 (heights_mm 높이, 손 축을 따라). enc = 쥔 물체 없이 처짐 앞먹임
  Q5 q_grasp{};
  Eigen::Vector3d tip{Eigen::Vector3d::Zero()}, app{Eigen::Vector3d::Zero()}, close{Eigen::Vector3d::Zero()};   // 쥘 때 손
  Eigen::Vector3d c{Eigen::Vector3d::Zero()};       // 쥔 뒤 (굴린 뒤) 퍽 중심
  Eigen::Vector3d n{Eigen::Vector3d::Zero()};       // 쥔 뒤 퍽 단면의 위쪽 축
  Eigen::Vector3d a_end{Eigen::Vector3d::Zero()};   // 받침에 넣을 긴 쪽 끝 방향 (쥐는 끝의 반대)
  double tilt_deg{0.0}, roll_deg{0.0}, pitch_deg{0.0}, depth_mm{0.0}, margin_mm{0.0};
};

// 누운 퍽 (중심 x, y · 윗면 top_z [m] · 긴축 axis_deg) 쥐기. heights_mm = 위 · 내림 높이 (윗면 위, 손끝) 2 개 + 쥐는 깊이 (−depth, cfg.depth 대신)
//   accept: 이웃 여유 등 바깥 검사 (false 면 다음 기울임). 다 안 되면 false + why
bool lying_grasp(
  const ArmModel & m, double x, double y, double top_z, double axis_deg, const std::vector<double> & heights_mm,
  const LyingGraspCfg & cfg, LyingGrasp & out, std::string & why, double max_tilt_deg = 30.0,
  const std::function<bool(const LyingGrasp &, std::string &)> & accept = nullptr);

// 길이는 모두 [m], 각은 [°] (이름에 rad 표시가 있으면 [rad])
struct ParkCfg
{
  Eigen::Vector2d center{0.1008, 0.00115};   // 받침 중심
  double floor_z{0.0975};        // 받침 바닥 윗면
  double elev_deg{45.0};         // 넣을 때 퍽 긴축 기울기 (긴 쪽 끝이 J1 쪽 아래로). IK 안 되면 5° 씩 min_elev_deg 까지
  double min_elev_deg{40.0};
  double insert_clear{0.004};    // 넣을 때 긴 쪽 끝 가장 낮은 모서리 = 바닥 + 이만큼
  // 긴 쪽 끝면 중심 = 받침 중심에서 J1 쪽으로 이만큼. 밀어서 서려면 퍽이 약 49° 넘게 기대야 함 (시뮬: 4 mm 가 가장 잘 섬)
  double end_inset{0.004};
  double push_h{0.025};          // 미는 손끝 높이 = 바닥 위
  double push_dz{0.0};           // 밀기 시작 높이 = push_h + 이만큼 → 안쪽으로 가며 push_h 까지 내려감
  double push_r0{0.06};          // 밀기 시작 = 받침 중심에서 바깥으로
  double push_extra{0.0};        // 밀기 끝점을 J1 쪽 (몸 쪽) 으로 이만큼 더
  // 굴삭기처럼 밀기 (push_scoop_deg > 0): 손끝을 J1 쪽으로 이만큼 기울이고 push_low 만큼 낮게 시작해, 안쪽으로 밀면서 손을 세우고
  //   push_h 까지 올라감. IK 안 되면 5° 씩 줄임 (0 = 곧은 밀기)
  double push_scoop_deg{0.0};
  double push_low{0.0};
  // 밀기 동안 J3 를 이만큼 더 [rad, + = 손끝 위]. 시작 0 → 가운데 전부 → 끝점 0 (sin). IK 뒤에 더함 → 손끝 자리도 같이 움직임
  double push_dj3{0.0};
  // 밀기 동안 J2 를 이만큼 더 [rad, + = 손끝 위], push_dj3 와 같은 모양
  double push_dj2{0.0};
  // 꽂아 누르기 (press_mode, 밀기 대신): 놓은 손을 press_top 까지 곧게 올려 빼고 → 손을 수직으로 · 닫고
  //   → 받침 뒤 (J1 쪽) press_r 위 → 바닥 + press_h 까지 곧게 내려 뒤 벽에 걸린 퍽 뒷부분을 눌러 세움
  bool press_mode{false};
  double press_r{-0.016};        // 꽂는 손끝 자리 = 받침 중심에서 반지름 방향 (− = 뒤 · J1 쪽. 뒤 벽 안쪽 면 −23.5 mm, 손가락 반두께 6 mm)
  double press_top{0.066};       // 올리는 · 넘어가는 손끝 높이 = 바닥 위 (수직 손이 닿는 끝 66, 선 퍽 윗면 60)
  double press_h{0.010};         // 꽂는 가장 낮은 손끝 높이 = 바닥 위
  int press_steps{4};
  double regrasp_depth{0.030};   // 선 퍽 다시 쥐는 깊이 (윗면 아래)
  double regrasp_r{-0.006};      // 선 퍽 다시 쥐는 자리 = 받침 중심에서 반지름 방향 (− = J1 쪽)
  double insert_back{0.0};       // > 0: 다 기울인 자세를 이만큼 위에서 만들고 그 자세 그대로 곧게 내려 넣음 (턱을 쓸지 않게)
  double insert_extra{0.0};      // 넣는 끝점을 이만큼 더 아래로
  double insert_from_back{0.0};  // 곧게 넣기 출발점을 J1 쪽 (로봇 뒤) 으로 이만큼 → 대각선으로 들어감 (insert_back > 0 일 때)
  int loose_step{-1};           // 넣으며 기울이는 점 중 이 번째부터 손가락을 살짝 풂 (grip "loose", 팔은 안 멈춤). −1 = 안 풂
  double insert_dj3{0.0};        // 넣는 자세 (기울이는 점들) 의 J3 를 이만큼 더 [rad, + = 손끝 위]
  // > 0: 넣는 점에서 손가락만 살짝 풀고 (loose), 같은 높이 · 기울임으로 J1 쪽으로 이만큼 곧게 당기며 마지막에 놓음
  double release_pull{0.0};
  double release_pull_dz{0.0};   // 당기는 동안 위아래 (− = 내려가며)
  // 넣는 끝점을 손등 쪽 (손목 카메라 쪽, app × close) 으로 이만큼 — 넣는 움직임에 같이. 손 기울임 그대로
  double release_up{0.0};
  int release_pull_steps{3};
  double side{0.0348}, length{0.060};
  double held_kg{0.017};
  double inner_half{0.0235};     // 받침 안쪽 반 (반지름 방향)
  int tilt_steps{8}, push_steps{4};
  Eigen::Vector2d regrasp_xy{Eigen::Vector2d::Zero()};   // 선 퍽 다시 쥐는 자리 고정 보정 [m, base_link x · y]
};

struct ParkNode
{
  std::string name;
  Q5 q{};             // 실제 관절 (처짐 앞먹임 전)
  double held_kg{0.0};
  bool slow{true};    // 받침 근처 느린 구간 (false = 빈 공간 속도)
  std::string grip;   // "", "open", "close"
};

// lift 자세 q_from 뒤 받침 세우기 경유점. elev_used = 실제 넣은 기울기 [°]
bool park_nodes(
  const ArmModel & m, const ParkCfg & c, const LyingGrasp & g, const Q5 & q_from, std::vector<ParkNode> & out,
  double & elev_used, std::string & why);

// 기울여 쥔 선 퍽 → 받침에 세워 넣고 수직 손으로 다시 쥐기 (control_master park_upright_again)
//   1) 들기 (q_from) → 가운데 점 → 받침 위 (같은 손 기울임 · 수직축 둘레로만 돌림 = 퍽은 계속 수직)
//   2) 곧게 내려 퍽 바닥을 받침 바닥 위 b 에서 놓음 (b = 쥔 손가락 가장 낮은 곳이 벽 윗면 + finger_clear 가 되는 가장 낮은 높이)
//   3) 연 손을 곧게 올려 선 퍽 윗면 위로 → 수직 손으로 다시 쥐기 (park_nodes 의 park_re_* 와 같은 자리 · 깊이)
//   안 되면 false (부르는 쪽 = 박스 흐름). 좌표 = 팔 모델 (xy = base_link, z = base_link − 0.039)
struct UprightHeld
{
  Q5 q_grasp{};                                    // 쥘 때 실제 관절 (손 ↔ 퍽 관계)
  Eigen::Vector2d puck_xy{Eigen::Vector2d::Zero()};  // 쥔 퍽 중심 xy (쥘 때)
  double bottom_z{0.0};                            // 쥘 때 퍽 바닥 (박스 윗면, 모델 z)
  double width{0.0348};                            // 네모 한 변 · 원기둥 지름
  bool square{true};
  double held_kg{0.017};
};

struct UprightCfg
{
  double wall_h{0.0264};       // 받침 벽 높이 (바닥 위). CAD 메시 26.4 · 콜리전 상자 22.5 → 높은 쪽
  double cup_in_t{0.022};      // 받침 안쪽 반 (접선 = 닫힘 방향) · 바깥 반
  double cup_out_t{0.025};
  double cup_in_r{0.0233};     // 받침 안쪽 반 (반지름) · 바깥 반
  double cup_out_r{0.0265};
  double finger_clear{0.003};  // 놓는 점: 쥔 손가락 가장 낮은 곳 ↔ 벽 윗면
  double over_up{0.025};       // 받침 위 점 = 놓는 점 + 이만큼 위 (거기서 곧게 내려 넣음)
  int down_steps{4};
  double up_clear{0.003};      // 놓은 뒤: 연 손가락 가장 낮은 곳 ↔ 선 퍽 윗면
  double max_b{0.035};         // 놓는 퍽 바닥 높이 탐색 끝 (바닥 위)
  // 네모: 손가락 면 (= 닫힘축) ↔ 받침 접선 벽 틀어짐 한도 [°] (10° 면 한쪽 여유 1.5 mm)
  double max_face_deg{10.0};
  // 받침에 넣는 쥔 손 기울임 한도 [°]. 넘으면 false → 박스 흐름 (40° 는 놓는 자세 J2 가 한계에 걸림)
  double max_tilt_deg{30.0};
  double finger_inner0{0.0091}, finger_outer0{0.0273}, finger_half_w{0.0164};   // grasp_select FingerTip 과 같음
  double grip_open{0.0164};
  double puck_h{0.060};
};

struct UprightInfo
{
  double tilt_deg{0.0};    // 쥔 손 기울임
  double face_deg{0.0};    // 네모 면 ↔ 받침 벽 틀어짐
  double b_mm{0.0};        // 놓는 퍽 바닥 높이 (받침 바닥 위)
  double depth_mm{0.0};    // 쥔 깊이 (윗면 → 손끝, 수직)
  bool mid{false};         // 가운데 점을 씀
};

bool park_upright_nodes(
  const ArmModel & m, const ParkCfg & c, const UprightCfg & u, const UprightHeld & h, const Q5 & q_from, std::vector<ParkNode> & out,
  UprightInfo & info, std::string & why);

// ---- 받침에 놓은 원기둥 다시 보기 (control_master park_upright_again · advance_visual_regrasp) ----
// 원기둥은 받침에 놓으면 반지름 방향으로 굴러감 → pk_up 뒤 관찰 점에서 손목 카메라로 잰 중심으로 park_re_* 를 다시 풂
// 관찰 점: D435 깊이 최소 (약 0.18 m) 를 지키는 자세는 손끝을 적재함 쪽 위로 젖혀 카메라가 앞 아래로 받침을 보는 자세뿐
//   → 관절 격자 (거칠게 → 곱게) 를 훑어 화면 여유가 가장 큰 자세를 고름 (test_park_stand ParkLook*)
struct ParkLookCfg
{
  double min_depth{0.185};   // 카메라 → 퍽 윗면 원 가장 가까운 점 [m] (D435 848×480 깊이 최소 약 0.18)
  double max_depth{0.30};
  double fx{606.32}, fy{606.19}, cx{321.43}, cy{248.20};   // config/camera_wrist.yaml (640×480)
  double width{640.0}, height{480.0};
  double finger_u{410.0}, finger_v{280.0};   // 손가락이 가리는 화면 오른쪽 아래 (look_at: x 420–640 · y 290–480, 10 px 여유)
  double min_margin_px{12.0};   // 퍽 윗면 (어긋남 ±shift 포함) ↔ 화면 끝 · 손가락 최소 여유 [px]
  double depth_px_per_mm{10.0}; // 고를 때 깊이 여유 1 mm = 화면 여유 이만큼 [px] (둘 중 작은 쪽이 가장 큰 자세)
  double shift{0.007};         // 볼 때 퍽 중심이 받침 가운데에서 이만큼까지 (반지름 · 접선) 도 화면 안 [m]
  double tip_rise{0.005};      // 퍽 윗면 기울어 올라온 높이 여유 [m] (원기둥이 벽에 기대면 윗면 가장자리가 약 4 mm 오름)
  double finger_clear{0.010};  // 관찰 자세 열린 손가락 ↔ 퍽 윗면 (+ tip_rise) [m]
  double path_clear{0.004};    // 지나는 길 (관절 직선) 열린 손가락 ↔ 퍽 윗면 · 받침 벽 · 적재함 칸 퍽 [m]
  // base_link: x < rack_x 인 팔 점 (팔꿈치 · 손목 · 손끝 · 카메라) 은 z ≥ rack_z (적재함 판 0.138 + 칸 퍽 0.06 + 여유), x < body_x 는 z ≥ body_z
  double rack_x{0.13}, rack_z{0.24}, body_x{0.18}, body_z{0.20}, half_y{0.17};
  double rack_front_x{0.096};  // 적재함 앞 끝 (칸 퍽 위 지나는 길 검사)
  double rack_puck_top{0.198}; // 칸에 선 퍽 윗면 (base_link)
  double link_r{0.025};        // 팔 링크 반 굵기 (지나는 길 팔꿈치 · 손목 점 여유)
  std::array<double, 5> mast_box{{-0.12, -0.08, -0.12, -0.08, 0.62}};   // 광각 기둥 [x0, x1, y0, y1, z_top] base_link (look_at 과 같음)
  double mast_pad{0.03};
  double puck_r{0.01675};      // 원기둥 반지름 (지름 33.5)
  double lift{0.040};          // pk_up 에서 같은 손 자세로 곧게 올리는 높이 [m] (park_look_up, 손가락이 퍽에서 떨어지게). 0 = 안 올림
};

struct ParkLookInfo
{
  double depth{0.0}, dmin{0.0};            // 퍽 윗면 중심 깊이 · 가장 가까운 점 [m]
  double u0{0.0}, u1{0.0}, v0{0.0}, v1{0.0};   // 퍽 윗면 (± shift) 화면 범위 [px]
  double margin_px{0.0};                   // 화면 끝 · 손가락까지 가장 작은 여유
  double tilt_deg{0.0};                    // 손 기울임 (수직에서)
  double tip_h{0.0};                       // 손끝 ↔ 퍽 윗면 [m]
  double clear_up{0.0}, clear_obs{0.0}, clear_down{0.0};   // 지나는 길 손가락 여유 (pk_up → 올림 → 관찰 · 관찰 → park_re_above) [m]
  bool lifted{false};                      // park_look_up 을 씀
  bool from_hint{false};                   // hint 그대로 씀 (격자 안 훑음)
  bool at_lift{false};                     // 올린 점 (park_look_up 자리) 에서 봄 (park_lift_look) — 관찰 점 하나만 끼움
  double lift_m{0.0};                      // at_lift: pk_up 에서 곧게 올린 높이 [m]
  Q5 q_lift{};
  int evals{0};
};

// 자세 q 에서 받침 가운데 퍽 윗면 (± shift) 이 화면에 어떻게 보이나. T5c = link5 → 광학 프레임 (look_at 과 같음). 깊이 · 화면 · 손가락만 (몸 · 길 안 봄)
bool park_look_view(const ArmModel & m, const Eigen::Isometry3d & T5c, const ParkCfg & c, const UprightCfg & u, const ParkLookCfg & L,
  const Q5 & q, ParkLookInfo & info);
// 관절 직선 qa → qb 동안 열린 손가락 (grip_open) 가장 작은 여유 [m]: 받침 가운데 퍽 (반지름 + shift, 윗면 + tip_rise) · 받침 벽 윗면 · 적재함 칸 퍽 윗면
double park_look_path_clear(const ArmModel & m, const ParkCfg & c, const UprightCfg & u, const ParkLookCfg & L, const Q5 & qa, const Q5 & qb);
// 관찰 자세 고르기: q_up = pk_up (손 연 채 퍽 위), q_down = park_re_above (다시 쥐기 위 점). 못 찾으면 false + why (부르는 쪽 = 보지 않고 쥠)
//   hint: 앞에서 고른 관찰 자세 — 지금도 맞고 길이 비면 격자 없이 그대로 (계획 시간 줄이기)
bool park_look_pose(const ArmModel & m, const Eigen::Isometry3d & T5c, const ParkCfg & c, const UprightCfg & u, const ParkLookCfg & L,
  const Q5 & q_up, const Q5 & q_down, Q5 & q_obs, ParkLookInfo & info, std::string & why, const Q5 * hint = nullptr);
// 올린 점에서 보기: pk_up 에서 같은 손 자세로 곧게 올린 점 (park_look_pose 0 단계와 같은 lift) 을 관찰 점으로
//   (손을 많이 젖힌 격자 관찰 자세는 태그 · 깊이가 잘 안 나옴). 올림 IK 안 됨 · 길 여유가
//   park_look_pose 와 같은 기준에 못 미침 · 받침 가운데가 화면 밖 = false (부르는 쪽 = 격자 관찰 자세)
bool park_lift_look(const ArmModel & m, const Eigen::Isometry3d & T5c, const ParkCfg & c, const UprightCfg & u, const ParkLookCfg & L,
  const Q5 & q_up, const Q5 & q_down, Q5 & q_obs, ParkLookInfo & info, std::string & why);
// 관찰 점 끼우기: 원기둥 (square false) · enabled 일 때만 pk_up 뒤에 park_look_up (li.lifted) · observe_name (멈춤 점, 빈손 · 느린 구간) 을 끼움.
//   끼웠으면 true. 네모 · 꺼짐 · pk_up 없음 = pn 그대로 false
bool park_insert_look(std::vector<ParkNode> & pn, bool square, bool enabled, const Q5 & q_obs, const ParkLookInfo & li, const std::string & observe_name);
// 잰 중심 xy (모델) 로 받침 수직 다시 쥐기 (park_upright_nodes 의 park_re_above · park_re_down 과 같은 높이 · 깊이 · 닫힘 close). seed = 원래 park_re_above
bool park_regrasp_at(const ArmModel & m, const ParkCfg & c, const UprightCfg & u, const Eigen::Vector2d & xy, const Eigen::Vector3d & close,
  const Q5 & seed, Q5 & q_above, Q5 & q_down, std::string & why);

// 옮겨 놓기 (control_master park_relocate): 먼 · 옆 누운 퍽을 가까운 자리로 옮겨 세움 — 순수 계산
// 가까운 자리 (받침 세우기가 되는 자리) 인가 = J1 에서 거리 ≤ max_r [m]
bool park_near(const Eigen::Vector2d & j1, const Eigen::Vector2d & p, double max_r);
// 옮겨 놓을 자리 후보 (가까운 것부터): J1 → 퍽 선 위 x = min_x + 2 mm 부터 1 cm 씩 바깥으로 near_r 까지, 각각 옆으로 0 · ±4 · ±8 cm.
//   박스 밖 (x < min_x · |y| > max_abs_y) · 가까운 자리 밖은 뺌
std::vector<Eigen::Vector2d> park_relocate_spots(
  const Eigen::Vector2d & j1, const Eigen::Vector2d & puck, double min_x, double max_abs_y, double near_r);
// 옮겨 놓은 퍽 긴축 [°] = 그 자리의 반지름 (J1 → 자리) 방향 (lying_grasp 가 바깥 끝을 쥠, 닫힘축 = 접선)
double park_relocate_axis_deg(const Eigen::Vector2d & j1, const Eigen::Vector2d & spot);

// 비스듬한 가까운 누운 퍽 (park_relocate_diag_deg): 끝 쥐기는 비스듬하면 손가락이 퍽 위로 올라감
// 긴축이 0° · 90° (90° 주기) 중 가까운 쪽에서 벗어난 각 [0, 45] °. 숫자가 아니면 NaN
double park_axis_diag_deg(double axis_deg);
// 가까운 자리 누운 퍽도 옮겨 놓을지: max_dev_deg > 0 (≤ 0 = 끔) · park_relocate 켬 · 이 PICK 에서 아직 안 옮김 · 긴축이 max_dev_deg 넘게 비스듬
bool park_diag_relocate(double axis_deg, double max_dev_deg, bool relocate, bool already_relocated);
// diag 면 옮겨 놓기 (reloc) 먼저 — 빈 계획 (자리 없음 · 계획 안 됨) 이면 restore() 뒤 direct(). diag 가 아니면 direct() 만
template <class Plan, class Reloc, class Restore, class Direct>
Plan park_diag_or_direct(bool diag, Reloc && reloc, Restore && restore, Direct && direct)
{
  if (diag) {
    Plan p = reloc();
    if (!p.empty()) {return p;}
    restore();
  }
  return direct();
}

}  // namespace manipulation
