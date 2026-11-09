// place_box.hpp — 장소 박스 측정값 (손목 깊이) 으로 주사위 칸 기준을 고침
//
// 역할
//   - perception /perception/wrist/box 측정 (map 으로 바꾼 것) 을 모아 장소 하나의 박스 중심 · 회전 확정 (중앙값)
//   - 확정한 박스 기준으로 주사위 칸 점 = 측정 중심 + 회전한 눈 오프셋 (박스가 돌면 눈 모양도 같이 돎)
//   - 재검증용: 경기장 · 계획 사본에 측정 기준을 덮어씀
//   - 놓을 칸이 박스 테두리를 넘지 않게: 퍽 발자국 (반폭) + 여유가 박스 윗면 안에 들도록 박스 좌표에서 눈 오프셋을 자름
// 입력
//   - BoxSample: 촬영 시각 · map x · y · yaw
//   - 장소 지도 중심 · 지도상 박스 yaw (경기장 축 = map_from_arena yaw)
// 출력
//   - PlaceFrame: 측정 중심 · 지도 대비 회전 dyaw · 쓴 측정 수, 못 정하면 nullopt + 이유
// 실행
//   - mission_core, mission_manager 가 씀. 시험 test_transfer (PlaceBox)
// note
//   - 박스는 정사각형 → yaw 는 지도 yaw 에서 ±45° 로 접어서 비교
//   - 측정 중심이 지도 중심에서 max_shift_m 넘게 떨어지면 다른 물체 · 잘못 맞춘 것으로 보고 안 씀 (지도 값으로 감)
//   - 같은 정차 자리에서 재고 놓음 → map → base 변환 오차는 측정 · PLACE 에 같이 들어가 상쇄됨
//   - 테두리 자르기는 측정 박스 기준. 측정이 없으면 지도 박스 기준이라 실제 박스가 옮겨졌으면 보장 못 함

#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "mission/dice.hpp"
#include "mission/dice_map.hpp"

namespace mission
{

struct BoxSample
{
  double stamp = 0.0;   // 촬영 시각 [s]
  double x = 0.0, y = 0.0, yaw = 0.0;   // map
};

struct PlaceFrame
{
  double x = 0.0, y = 0.0;   // 박스 윗면 중심 map [m]
  double dyaw = 0.0;         // 지도 박스 방향 대비 회전 [rad], ±45° 안
  int n = 0;                 // 쓴 측정 수
};

// a − ref 를 (−45°, 45°] 로 접음
double fold_quarter_diff(double a, double ref);

// since 뒤 측정 중 지도 중심 max_shift_m 안 것 → 중앙값. min_n 개 못 되면 nullopt
std::optional<PlaceFrame> place_frame_from(
  const std::vector<BoxSample> & samples, double since, const std::array<double, 2> & nominal_center,
  double nominal_yaw, double max_shift_m, int min_n, std::string * why = nullptr);

// 칸 점 = 측정 중심 + R(dyaw) · 눈 오프셋 (오프셋은 지도 방향, apply_view_directions 거친 것)
std::array<double, 2> frame_point(const PlaceFrame & f, const std::array<double, 2> & offset);

// 박스 윗면 테두리 안 범위: 퍽 중심이 박스 중심에서 (box_half − puck_half − margin) 안이어야 퍽 전체가 윗면 위
struct PlaceLimits
{
  double box_half = 0.1025;    // 박스 윗면 반 변 [m] (205 mm)
  double puck_half = 0.0212;   // 퍽 발자국 반폭 [m] (네모 30 mm 대각선 반, 놓을 때 면 방향을 모르므로 가장 큰 값)
  double margin = 0.010;       // 테두리 여유 [m] (놓는 오차 · 측정 오차)
};

// 눈 오프셋 (지도 방향) 을 박스 좌표 (지도 박스 yaw = box_yaw) 로 돌려 각 축을 범위 안으로 자르고 다시 지도 방향으로. 잘랐으면 true
//   범위가 0 이하 (박스보다 퍽 + 여유가 큼) 면 가운데로
bool clamp_offset_in_box(std::array<double, 2> & offset, double box_yaw, const PlaceLimits & lim);

// 재검증용 사본: 장소 중심 = 측정 중심, 그 장소 눈 오프셋 = R(dyaw) 회전
void apply_place_frame(DiceMap & arena, DicePlan & plan, int place, const PlaceFrame & f);

}  // namespace mission
