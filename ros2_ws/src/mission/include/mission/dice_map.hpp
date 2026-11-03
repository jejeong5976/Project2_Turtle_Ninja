// dice_map.hpp — 발제 v5 경기장 좌표 maps/dice_map.yaml 읽기 → map 프레임
//
// 역할
//   - 경기장 좌표 → map 프레임 변환
//       경기장 좌표 = 발제 그림 기준, 왼쪽 아래 기둥 = 원점, +X 오른쪽, +Y 위, m · rad
//       SLAM 지도 원점은 매핑 시작 자리 → map_from_arena 하나로 변환
// 입력: yaml 키
//   - map_from_arena: start | [x, y, yaw]
//       [x, y, yaw] = map 에서 본 경기장 원점 자세, 직접 잼
//       start       = 매핑을 출발 자세에서 시작했다고 보고 start 의 역변환을 씀
//   - start: [x, y, yaw]          출발 자세 = 복귀 지점
//   - barcode_view: [x, y, yaw]   바코드 읽는 자세, 출발 구역 안
//   - boxes:  B1 … B6: {center: [x, y], stop: [x, y, yaw]}
//   - places: L1 … L3: {center: [x, y], stop: [x, y, yaw], top_z: 0.07}
//       center = 주사위 격자 중심, 보는 방향 = stop yaw
//   - box_radius: 0.15            박스 LOOK 때 이 반경 안 퍽 = 그 박스 퍽, m
//   - box_size: 0.205 · box_approach_m: 0.10  박스 한 변 · 박스 앞 마지막 접근 거리 (0 = 끔), m
// 출력
//   - DiceMap: 정차 자세 · 장소 중심 · 박스 중심, map 프레임
// 실행
//   - mission_core 라이브러리, mission_manager 가 dice_map 파라미터 파일을 읽음. 시험 test_transfer
// note
//   - 정차 자세는 이동 시간표 maps/dice_travel_times.yaml 을 잰 자리와 같아야 함 = sim/robot_sim/results/dice_paths.json
//   - 노드 번호는 transfer_types.hpp 와 같음: 0 = S, 1–6 = B1–B6, 7–9 = L1–L3
//   - 맵 공개 · 매핑 뒤 지도 벽과 겹쳐서 확인할 것

#pragma once

#include <array>
#include <string>

#include "mission/common.hpp"

namespace mission
{

struct ArenaBox
{
  std::array<double, 2> center{};   // map [m]
  Pose2D stop;                      // map
};

struct ArenaPlace
{
  std::array<double, 2> center{};   // 주사위 격자 중심, map [m]
  Pose2D stop;                      // map
  std::array<double, 2> view_dir{{1.0, 0.0}};   // 보는 방향 단위 벡터 (stop yaw), map
  double top_z = 0.07;              // 놓는 면 높이 [m]
};

struct DiceMap
{
  Pose2D map_from_arena;            // map 에서 본 경기장 원점 자세
  Pose2D start;                     // 아래는 전부 map 프레임
  Pose2D barcode_view;
  std::array<ArenaBox, 6> boxes;    // B1–B6
  std::array<ArenaPlace, 3> places; // L1–L3
  double box_radius = 0.15;
  double box_size = 0.205;          // 박스 한 변 [m]
  double box_approach_m = 0.0;      // > 0 이면 박스 정차를 이만큼 앞에서 Nav2 로 멈춘 뒤 라이다로 똑바로 붙음 (planning approach)
};

// 실패하면 false + error. out 은 map 프레임으로 바꾼 값
bool load_dice_map(const std::string & path, DiceMap & out, std::string * error = nullptr);

// 경기장 → map
Pose2D arena_to_map(const Pose2D & map_from_arena, const Pose2D & p);
std::array<double, 2> arena_point_to_map(const Pose2D & map_from_arena, const std::array<double, 2> & p);

// 매핑을 a 자세에서 시작했을 때 map_from_arena (= a 의 역변환)
Pose2D inverse_pose(const Pose2D & a);

// 노드 번호 (0 = S, 1–6 = B, 7–9 = L) → 정차 자세. 범위 밖이면 start
Pose2D node_stop(const DiceMap & a, int node);

}  // namespace mission
