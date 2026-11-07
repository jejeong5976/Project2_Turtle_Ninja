// place_box.hpp — 장소 박스 측정값 (손목 깊이) 으로 주사위 칸 기준을 고침
//
// 역할
//   - perception /perception/wrist/box 측정 (map 으로 바꾼 것) 을 모아 장소 하나의 박스 중심 · 회전 확정 (중앙값)
//   - 확정한 박스 기준으로 주사위 칸 점 = 측정 중심 + 회전한 눈 오프셋 (박스가 돌면 눈 모양도 같이 돎)
//   - 재검증용: 경기장 · 계획 사본에 측정 기준을 덮어씀
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

// 재검증용 사본: 장소 중심 = 측정 중심, 그 장소 눈 오프셋 = R(dyaw) 회전
void apply_place_frame(DiceMap & arena, DicePlan & plan, int place, const PlaceFrame & f);

}  // namespace mission
