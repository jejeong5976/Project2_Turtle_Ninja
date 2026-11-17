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
//   - 다시 붙기 (box_realign_*): 측정 박스가 이 정차에서 기대한 자리와 많이 다르면 측정 박스에 맞춘 정차점으로 다시 붙음
//     정차점 = 지도 정차가 지도 박스에 대해 가진 자리 (중심 → 정차 벡터 · 정차 yaw) 를 측정 박스 (중심 · dyaw) 로 옮기고 돌린 것
//   - 박스 다시 찾기 (box_relook): 정밀 접근 실패 뒤 LOOK 에서 잰 박스 (손목 깊이) · 본 퍽으로 새 중심 · 방향
//     → 네 면 중 로봇 쪽 면부터, 몸 사각형이 지도 (OccupancyGrid) 벽 · 다른 박스에 안 닿는 면의 정차점 (막힌 면만 있으면 가장 덜 막힌 면)

#pragma once

#include <array>
#include <cstdint>
#include <functional>
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

// 재검증 칸 기준: 재검증 LOOK 측정 (remeasured, 퍽 관측과 같은 때 위치 추정) 먼저 → 없으면 놓을 때 쓴 측정 (placed) → 둘 다 없으면 nullopt (지도 값)
std::optional<PlaceFrame> verify_place_frame(
  const std::optional<PlaceFrame> & placed, const std::optional<PlaceFrame> & remeasured);

// ---- 장소 박스 다시 붙기 (mission_manager box_realign_*)
// 실제로 선 정차 (used_stop) 에서 기대한 박스: 지도 정차 (map_stop) ↔ 지도 중심 (map_center) 의 상대 자리를 used_stop 으로 옮긴 것
//   지도 정차로 섰으면 지도 중심 · dyaw 0, 다시 붙은 정차로 섰으면 그때 쓴 측정 박스
PlaceFrame expected_box_at(const Pose2D & used_stop, const Pose2D & map_stop, const std::array<double, 2> & map_center);

// 측정 박스 f 에 지도 정차와 같은 상대 자리로 다시 붙을 정차점 (박스 중심까지 거리 · 보는 방향 그대로, yaw += dyaw)
Pose2D realigned_stop(const PlaceFrame & f, const Pose2D & map_stop, const std::array<double, 2> & map_center);

struct RealignCheck
{
  double shift_m = 0.0;   // 측정 중심 ↔ 기대 중심 [m]
  double dyaw = 0.0;      // 측정 ↔ 기대 방향 차 [rad], ±45° 로 접음 (정사각형)
  bool over = false;      // shift_m > max_shift_m 또는 |dyaw| > max_yaw
  bool realign = false;   // over 이고 done < max_n → 다시 붙기
};
// 측정 f 와 기대 e 비교. done = 이 장소에서 이번 task 에 이미 다시 붙은 횟수
RealignCheck realign_check(const PlaceFrame & f, const PlaceFrame & e, double max_shift_m, double max_yaw, int done, int max_n);

// ---- 박스 다시 찾기 (mission_manager box_relook_*)
// 점유 격자 (nav_msgs/OccupancyGrid 와 같은 배치: 행 = y, 원점 = 왼쪽 아래 칸 모서리, 회전 없음)
struct OccGrid
{
  double res = 0.0, ox = 0.0, oy = 0.0;
  int w = 0, h = 0;
  std::vector<int8_t> data;   // 0–100, −1 = 모름
  bool empty() const {return w <= 0 || h <= 0 || data.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h);}
  int at(double x, double y) const;   // 밖 = −1
};

// 정차 자세 몸 사각형 (앞뒤 ±half_x · 좌우 ±half_y, + margin) 안 막힌 점 수: 격자 칸 (≥ occ 또는 모름 −1) + extra 점 (다른 박스 테두리)
//   격자가 비었으면 격자는 안 봄
//   ignore = 막힘으로 안 셀 박스 (중심 x, y, 방향) — 그 윗면 (반 변 0.1025 + 3 cm) 안 격자 칸 (지도에 찍힌 목표 박스 자신)
int footprint_blocked(const OccGrid & g, const Pose2D & stop, double half_x, double half_y, double margin, int occ,
  const std::vector<std::array<double, 2>> & extra, const std::vector<std::array<double, 3>> & ignore = {});

// 박스 (중심 c, 한 변 size, 방향 yaw) 테두리 점 (step 간격) — footprint_blocked 의 extra
std::vector<std::array<double, 2>> box_outline(const std::array<double, 2> & c, double yaw, double size, double step = 0.02);

struct FaceStop
{
  Pose2D stop;        // 중심 − dist · (cos yaw, sin yaw), yaw = 면을 봄
  int face = 0;       // 0–3 (box_yaw + face · 90°)
  double turn = 0.0;  // |stop.yaw − ref_yaw| [rad]
  int blocked = 0;    // choose_face 가 채움
};
// 네 면 정차점, ref_yaw (로봇 → 박스 방향) 에 가까운 순 = 로봇 쪽 면부터
std::vector<FaceStop> box_face_stops(const std::array<double, 2> & c, double box_yaw, double dist, double ref_yaw);
// 순서대로 blocked(stop) == 0 인 첫 면. 없으면 가장 덜 막힌 면 (같으면 앞 순서). 후보 없으면 nullopt
std::optional<FaceStop> choose_face(std::vector<FaceStop> cands, const std::function<int(const Pose2D &)> & blocked);

struct BoxFind
{
  std::array<double, 2> center{};
  double yaw = 0.0;            // 박스 방향 (90° 주기). 퍽으로만 찾으면 ref_yaw 그대로
  bool from_detector = false;  // 손목 깊이 박스 측정으로 찾음 (방향도 잼)
  int n = 0;                   // 쓴 측정 수 · 퍽 수
  std::string note;            // 로그용 (측정을 안 쓴 이유 등)
};
// 다시 찾기 LOOK 뒤 결정. use_detector 이면 since 뒤 박스 측정 (map) 중 지도 중심 max_shift 안 중앙값 (min_n 개 이상) 먼저.
//   본 퍽이 있으면 모두 측정 박스 윗면 안 (반 변 + 2 cm) 이어야 씀 (다른 박스 · 틀린 맞춤 거르기). 아니면 퍽 평균 (방향 = ref_yaw)
//   퍽 평균도 max_shift 넘으면 nullopt (why = 이유)
std::optional<BoxFind> box_find_from(
  const std::vector<BoxSample> & samples, double since, const std::array<double, 2> & map_center, double ref_yaw,
  const std::vector<std::array<double, 2>> & pucks, bool use_detector, double box_size, double max_shift_m, int min_n,
  std::string * why = nullptr);

}  // namespace mission
