// cell_shift.hpp — 적재함이 움직였을 때 칸 경로의 칸 근처만 옮기기 (순수 C++, test/test_cell_shift.cpp)
// =====================================================================
// tools/arm_real/arm/shift_cell_paths.py 와 같은 규칙 (실물에서 S12 를 이 규칙으로 옮겨 넣기 · 빼기 확인)
//   - 손끝이 칸 중심에서 수평 35 mm 안이면 다 옮기고, 35~90 mm 는 비례해서 줄이고, 그 밖은 그대로
//   - STORE 는 둘째 점 ~ P0_복귀, RETRIEVE 는 P0 ~ 끝 앞 점 사이만, 양 끝 12 점에 걸쳐 0 → 1
//     → PICK 끝 · P0 · 받침 · 박스 위 기준점 자세는 그대로
//   - 점마다 관절 5 개로 손끝 위치 (무게 큼) · 접근축 · 닫힘축을 감쇠 최소제곱으로 맞춤 (관절 변화가 작은 해)
//   - 경로 값은 처짐 앞먹임이 들어간 명령값 — 몇 mm 옮기는 정도는 처짐 토크 변화를 무시함

#pragma once

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
// dz: 칸 근처를 위아래로도 옮김 [m] (− = 아래, 11-07: 퍽을 칸 위에서 떨어뜨리지 않고 더 내려 놓기)
bool shift_cell_path(
  const ArmModel & m, ArmPath & path, bool store, const Eigen::Vector2d & cell_xy, const Eigen::Vector2d & d,
  CellShiftReport & rep, std::string & why, double dz = 0.0);

}  // namespace manipulation
