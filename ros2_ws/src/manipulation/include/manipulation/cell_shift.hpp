// cell_shift.hpp — 적재함이 움직였을 때 칸 경로의 칸 근처만 옮기기 (순수 C++, test/test_cell_shift.cpp)
// =====================================================================
// tools/arm_real/arm/shift_cell_paths.py 와 같은 규칙 (실물에서 S12 를 이 규칙으로 옮겨 넣기 · 빼기 확인)
//   - 손끝이 칸 중심에서 수평 35 mm 안이면 다 옮기고, 35~90 mm 는 비례해서 줄이고, 그 밖은 그대로
//   - STORE 는 둘째 점 ~ P0_복귀, RETRIEVE 는 P0 ~ 끝 앞 점 사이만, 양 끝 12 점에 걸쳐 0 → 1
//     → PICK 끝 · P0 · 받침 · 박스 위 기준점 자세는 그대로
//   - 점마다 관절 5 개로 손끝 위치 (무게 큼) · 접근축 · 닫힘축을 감쇠 최소제곱으로 맞춤 (관절 변화가 작은 해)
//   - 경로 값은 처짐 앞먹임이 들어간 명령값 — 몇 mm 옮기는 정도는 처짐 토크 변화를 무시함

#pragma once

#include <functional>
#include <string>

#include <Eigen/Dense>

#include "manipulation/arm_kinematics.hpp"
#include "manipulation/arm_path.hpp"

namespace manipulation
{

struct CellShiftReport
{
  int moved{0};                 // 옮긴 점 수
  double worst_pos_mm{0.0};     // 손끝이 원하는 만큼 못 옮긴 최대 [mm]
  double worst_app_deg{0.0};    // 접근축이 바뀐 최대 [°]
  double max_joint_deg{0.0};    // 관절이 바뀐 최대 [°]
};

// path (store_<칸> · retrieve_<칸>) 의 칸 근처 점을 base_link 수평 d [m] 만큼 옮김. 실패면 path 그대로 + why
//   실패: 경유점 이름 (P0_복귀 · P0) 없음, 손끝을 0.2 mm 안으로 못 옮김, 접근축이 1° 넘게 바뀜, 관절 범위 밖
// dz: 칸 근처를 위아래로도 옮김 [m] (− = 아래,: 퍽을 칸 위에서 떨어뜨리지 않고 더 내려 놓기)
bool shift_cell_path(
  const ArmModel & m, ArmPath & path, bool store, const Eigen::Vector2d & cell_xy, const Eigen::Vector2d & d,
  CellShiftReport & rep, std::string & why, double dz = 0.0);

// retrieve_* 경로: 쥐는 점 (close) 바로 뒤에 손끝을 곧게 clear_m 올린 점 (lift_clear) 을 넣고, 그 뒤 칸 근처 (수평 50 mm 안) 에서
//   그보다 낮은 점은 그 높이로 올림 — 쥔 퍽이 칸 안에서 옆으로 움직이며 벽을 긁지 않게 (11-10 실물: 깊게 쥔 R12 를 꺼낼 때 칸을 긁음).
//   raised = 올린 점 수. 실패 (쥐는 점 없음 · 못 올림 · 관절 범위 밖) 면 path 그대로 + why
bool retrieve_lift_clear(const ArmModel & m, ArmPath & path, const Eigen::Vector2d & cell_xy, double clear_m, int & raised, std::string & why);

// store_* 경로: 같은 규칙을 첫 놓는 점 (release) 뒤에 — 손끝을 곧게 clear_m 올린 점 (store_lift_clear, grip 없음) 을 넣고 그 뒤 칸 근처
//   그보다 낮은 점은 그 높이로 올림 (벌린 손가락이 놓은 퍽을 끌고 나오지 않게). 실패 (놓는 점 없음 · 못 올림 · 관절 범위 밖) 면 path 그대로 + why
bool store_lift_clear(const ArmModel & m, ArmPath & path, const Eigen::Vector2d & cell_xy, double clear_m, int & raised, std::string & why);

// store_* 경로의 칸 위 (마지막 내려가기 직전) 점: 첫 놓는 점 (grip release) 앞에서 손끝이 놓는 점보다 up_m [m] 이상 높은 마지막 점 번호
//   (11-09 실물: 흔들린 채 칸 위에 와서 그대로 내려감 → 거기서 멈춤, control_master store_settle_s). 손끝 높이는 팀 명령 rad → 팔 모델
//   첫 점 (출발 자세, path_actions 가 끊지 않는 점) 은 뺌. 놓는 점이 없거나 그런 점이 없으면 −1
int store_settle_index(const ArmModel & m, const ArmPath & path, double up_m);

// ---- 새 칸 경로 (storeheld, 11-10: config/arm_paths.yaml store_<칸>_d50 · _d30 · _d30_low · _d40_tilt10 · 20 · 30 · retrieve_<칸>_full) ----
// 경로 이름 → 칸 (store_S7 · retrieve_S7 · store_look_S7 · look_rack_S7 · store_S7_d30 · retrieve_S7_full → "S7"). 칸 경로가 아니면 ""
std::string path_cell(const std::string & name);
// 새 칸 경로인가 (이름에 _d30 · _d50 · _d40_tilt · _full). 칸 보정 (store_to_cell · store_offset · retrieve_offset · 세운 퍽 높이) 이 이미 들어 있음
bool storeheld_path(const std::string & name);
// STORE 새 경로 이름 (has = 그 이름의 경로가 있나). "" = 새 경로 없음 → 지금 이름 (store_<칸>) · 지금 흐름 (기울임 거절 등)
//   기울여 집은 퍽 (tilt_deg > 0.5) → _d40_tilt<10 · 20 · 30 중 가까운 값>, 쥔 깊이가 regrasp_depth_mm ± 5 → _d30 (low 면 _d30_low 우선), 그 밖 → _d50
std::string storeheld_store_name(const std::string & cell, double held_depth_mm, double regrasp_depth_mm, double tilt_deg, bool low,
  const std::function<bool(const std::string &)> & has);
// RETRIEVE: retrieve_<칸>_full 이 있으면 그것, 없으면 ""
std::string storeheld_retrieve_name(const std::string & cell, const std::function<bool(const std::string &)> & has);

}  // namespace manipulation
