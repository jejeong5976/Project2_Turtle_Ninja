// dice_verify.hpp — 세 장소 재검증 판정, 발제 '이동 후 검증'
//
// 역할: 판정
//   - 묶기: 원시 관측 → 퍽 단위, 색 · 모양은 장 다수결 (표 갈리면 모름, 추정 안 함)
//   - 박스 밖 거르기: 박스 측정이 있으면 퍽 중심이 윗면 가장자리에서 ignore_inset_m 안쪽이 아닌 검출은 판정 · 개수에서 뺌 (ignored)
//     (이 장소에 놓은 퍽 ID 와 같은 ID 검출은 중심이 윗면 밖일 때만 뺌)
//     → 박스 모서리 · 로봇 몸 오검출이 개수를 늘리지 않게. 떨어진 퍽은 그 칸이 비어 불합격으로 나옴
//   - 모양 맞춤 (fit.enable 일 때만): 관측 퍽에 계획 눈 모양 (칸 배치) 을 평행이동 + 회전으로 맞춤 (기준에서 max_shift 안)
//     노드는 박스 측정이 없을 때만 켬 (측정 있으면 박스 중심 기준 칸 그대로 = 놓기 치우침을 숨기지 않음)
//   - 칸 ID 판정 (cell_ids 있음): 칸마다 놓은 퍽 ID 가 그 칸 중심 cell_tol_m 안 · 박스 위에 보여야 함
//     + 다시 본 색 · 모양으로 그룹 조건 (R 빨강 · G 초록 · Y 모양 통일, 모르면 그 퍽 불합격). cell_kinds 가 있으면 기록 색 · 모양을 이유에 같이
//     ID 못 읽은 퍽 · 계획에 없는 ID 는 불합격, 빈 칸 (안 놓음) 도 불합격
//   - 그룹 판정 (cell_ids 없음): 관측 위치가 칸 중심에서 cell_tol_m 안, 가장 가까운 칸, 한 칸에 하나
//   - 박스 위: 맞춘 모양이 박스 기준에서 max_shift 넘게 밀려야 하면 맞춤 안 함 (칸 밖으로 판정)
//     박스 측정 (VerifyBox) 이 있으면 맞춘 칸이 윗면 밖으로 나가는 해는 버리고, 퍽 중심이 윗면 밖이면 그 퍽 불합격 (떨어짐)
//   - 그룹: R = 빨강, G = 초록, Y = 그 장소 퍽이 모두 같은 모양 (모양 아는 퍽끼리 비교, 모르는 퍽은 그 퍽만 불합격)
//   - 장소: 본 수 = 눈 수, 모두 칸 위 · 그룹 맞음
//   - 전체: 세 장소를 다 봤을 때 ID 중복 · 누락 0–11
// 입력
//   - 장소별로 VERIFY 단계 LOOK 뒤 새로 본 퍽: ID · 색 · 모양 · map 위치
//   - 계획: 그룹 · 눈 수 · 칸 위치
// 출력
//   - 판정 결과 → 노드가 interfaces/VerifyRecord 로 발행
//   - verify_to_yaml: 사람 · 채점용 기록 파일 내용
// 실행
//   - mission_core 라이브러리, mission_manager 가 부름. 시험 test_transfer
// note
//   - 목표 상태를 복사하지 않음 → 관측이 없으면 그 칸은 빈 것으로 기록. 발제상 복사 로그는 인정 안 됨

#pragma once

#include "mission/common.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "mission/dice.hpp"
#include "mission/dice_map.hpp"

namespace mission
{

// 박스 윗면 (재검증 때 잰 것). 중심 · 방향 = map, 정사각형
struct VerifyBox
{
  double x = 0.0, y = 0.0, yaw = 0.0;
  double half = 0.1025;         // 윗면 반 변 [m] (205 mm)
  double margin = 0.0;          // 퍽 중심이 가장자리에서 이만큼 안쪽이어야 윗면 위 (0 = 중심이 윗면 안이면 안 떨어짐)
  std::string source;           // 측정 출처 (기록용)
};

// 점의 윗면 가장자리까지 안쪽 거리 [m] (음수 = 밖)
double box_inner_dist(const VerifyBox & b, double x, double y);

struct VerifyObs
{
  std::string id;               // "" = ID 못 읽음
  std::string color;
  std::string shape;
  double x = 0.0;               // map [m]
  double y = 0.0;
  int cell = -1;                // 판정 뒤 채움
  double cell_error_m = 0.0;
  int on_box = -1;              // 1 = 퍽 중심이 박스 윗면 안, 0 = 밖 (떨어짐), -1 = 박스 측정 없어 안 봄
  double edge_m = 0.0;          // 윗면 가장자리까지 안쪽 거리 [m] (음수 = 밖)
  bool ok = false;
  double stamp = -1.0;          // 촬영 시각 [s] (같은 장 구별용). < 0 = 모름 (같은 ID 는 한 퍽)
  std::array<float, 4> bbox{{0.0f, 0.0f, 0.0f, 0.0f}};   // 마지막 장 검출 상자 (원본 영상 픽셀 xyxy, 그림용), 모두 0 = 없음
  // 그 장 높이 · 카메라 원점 map [m] (NaN = 모름). group_place_obs 가 깊이 틀린 장을 가려냄
  double z = std::numeric_limits<double>::quiet_NaN();
  std::array<double, 3> cam{{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()}};
  int off_plane_n = 0;          // 묶은 뒤: 윗면 높이에서 벗어나 위치로 안 쓴 장 수
  bool pos_from_earlier = false;   // 묶은 뒤: 마지막 장이 벗어나 그 전 맞는 장 위치를 씀
  bool all_off_plane = false;      // 묶은 뒤: 맞는 장이 없음 (모든 장이 벗어남 → 마지막 장 그대로, 바닥 퍽이면 박스 밖)
};

struct PlaceVerify
{
  int place = -1;               // 0–2
  char group = '?';
  int expected = 0;             // 눈 수
  std::vector<VerifyObs> seen;
  std::string frame_source = "map";   // 칸 기준 출처: placed (놓을 때 박스 측정) · remeasured (재검증 LOOK 측정) · map
  std::array<double, 2> ref_center{{0.0, 0.0}};   // 그 기준의 장소 중심 map [m]
  bool fitted = false;          // 모양 맞춤 씀
  double fit_dx = 0.0, fit_dy = 0.0, fit_dyaw = 0.0;   // 맞춘 모양 = 기준 칸을 중심 둘레로 fit_dyaw 돌리고 (dx, dy) 옮긴 것
  std::vector<std::array<double, 2>> cells;   // 판정에 쓴 칸 중심 map [m] (맞춤 뒤)
  // 칸마다 로봇이 볼 때 자리 이름 (cells 와 같은 길이): 한글 "가까운 줄 왼쪽" · 영문 "near-left" (문구 · 그림 · 기록용)
  std::vector<std::string> cell_spots, cell_spots_en;
  std::optional<VerifyBox> box; // 박스 위 확인에 쓴 윗면 (없으면 안 봄)
  std::vector<std::string> cell_ids;   // 칸마다 놓은 퍽 ID ("" = 안 놓음). 비면 그룹 판정
  std::string y_shape;          // Y 장소 기준 모양 (판정에 쓴 것, "" = 없음 · 섞임)
  std::vector<VerifyObs> ignored;      // 박스 윗면 밖이라 판정에서 뺀 검출 (기록 · 그림용)
  bool ok = false;
  std::vector<std::string> issues;
  double stamp_s = 0.0;         // 본 시각
  double cell_tol_m = 0.05;     // 판정에 쓴 칸 허용 거리 (그림용)
  // 판정에 실제로 쓴 칸 기준: 놓을 때 · 다시 잰 박스 측정 · 모양 맞춤을 반영한 중심 (map). judge_place 가 채움
  //   기록 파일 center · cells 는 이 값 (지도 원래 값 아님). has_frame false = 측정 없음 (지도 값으로 씀)
  bool has_frame = false;
  std::array<double, 2> center{{0.0, 0.0}};
};

struct VerifyResult
{
  std::vector<PlaceVerify> places;          // 본 순서
  bool complete = false;                    // 세 장소 다 봄
  bool pass = false;
  std::vector<std::string> duplicate_ids;
  std::vector<std::string> missing_ids;
  std::vector<std::string> issues;          // 장소 문제 + 전체 문제
};

// 원시 관측 (한 장 = 한 개, 시간 순) → 퍽 단위로 묶음
//   ID 있는 관측: 같은 ID 끼리 한 퍽. 단 같은 장 (stamp 같음, ≥ 0) 에서 merge_m 넘게 떨어진 같은 ID 는 다른 퍽
//     → 따로 남겨 summarize 의 duplicate_ids 에 나옴. 다음 장부터는 가장 가까운 쪽에 붙임 (장이 늘어도 퍽 수 안 늘어남)
//   ID 없는 관측: merge_m 안끼리 한 퍽, ID 있는 퍽 merge_m 안이면 그 퍽에 합침 (같은 퍽을 태그 못 읽은 장)
//   위치 = 가장 최근 장. 색 · 모양 = 읽힌 장 ("" 아닌 장) 다수결, 1 등 비율 < vote_ratio 거나 읽힌 장 없으면 "" (모름)
//   plane_tol_m > 0: 윗면 높이 = 이 장소 장들 z 중앙값. z 가 이보다 plane_tol_m 넘게 벗어난 장 = 깊이 틀린 장
//     (깊이 점이 거의 없으면 광선 따라 크게 밀린 위치가 나옴)
//     → 묶기 (같은 퍽 찾기) 는 카메라 원점 → 그 점 광선과 윗면 높이 교점으로, 위치는 그 퍽의 마지막 '맞는' 장
//     맞는 장이 하나도 없으면 (정말 바닥에 떨어진 퍽: 모든 장이 낮음) 마지막 장 그대로 → 박스 밖 판정 유지
//     z 모름 (NaN) · 장 3 개 미만이면 이 거르기 안 함
std::vector<VerifyObs> group_place_obs(
  const std::vector<VerifyObs> & raw, double merge_m = 0.03, double vote_ratio = 0.7, double plane_tol_m = 0.0);

// 모양 맞춤 범위 (enable = false 면 기준 칸 그대로 판정)
struct VerifyFitParams
{
  bool enable = true;
  double max_shift_m = 0.10;    // 기준 중심에서 모양을 옮길 수 있는 거리 (박스 윗면 반폭 0.1025 → 모양 중심이 박스 위)
  double max_rot_rad = M_PI;     // 기준 방향에서 돌릴 수 있는 각. π = 방향 무관 (눈 방향은 채점과 무관, 모양만)
};

// 관측 점에 칸 모양 (center + offsets) 을 맞춤. cell_tol_m 안에 든 짝 수 최대 → 남은 오차 제곱합 최소
//   시작 가설 = 회전 15° 간격 × ("점 하나가 칸 하나 위" 평행이동 + 기준 그대로). 첫 짝 찾기만 pair_tol_m 로 넓게
//   범위 (max_shift · max_rot) 밖이거나 맞는 짝이 없으면 nullopt
struct PatternFit
{
  double dx = 0.0, dy = 0.0, dyaw = 0.0;
  int matched = 0;
};
std::optional<PatternFit> fit_pattern(
  const std::array<double, 2> & center, const std::vector<std::array<double, 2>> & offsets,
  const std::vector<std::array<double, 2>> & points, double cell_tol_m, double pair_tol_m, const VerifyFitParams & fp,
  const VerifyBox * box = nullptr);   // 있으면 칸이 윗면 (margin 안쪽) 밖으로 나가는 해는 버림

// 장소 하나 판정. obs = 장소 반경 안 새 관측 (group_place_obs 로 묶은 것). plan 은 apply_view_directions 를 거친 것
//   cell_ids = 칸마다 놓은 퍽 ID (pip_offsets 와 같은 길이, "" = 안 놓음). 비면 그룹 판정
//   ignore_inset_m = 박스 측정 있을 때 윗면 가장자리에서 이만큼 안쪽이 아닌 검출은 뺌 (0 = 안 뺌)
//     cell_ids 에 있는 ID 검출은 중심이 윗면 밖일 때만 뺌
//   cell_kinds = 칸마다 놓은 퍽의 기록 {색, 모양} (이유 글에만 씀, 판정은 다시 본 값). 비어도 됨
PlaceVerify judge_place(
  int place, const DicePlan & plan, const DiceMap & arena, std::vector<VerifyObs> obs,
  double cell_tol_m = 0.05, double stamp_s = 0.0, const VerifyFitParams & fit = {},
  const std::optional<VerifyBox> & box = std::nullopt, const std::vector<std::string> & cell_ids = {},
  double ignore_inset_m = 0.0, const std::vector<std::array<std::string, 2>> & cell_kinds = {});

// 칸 자리 이름: 칸 중심 − 장소 중심을 보는 방향 (view_dir, 로봇 → 박스) 으로 나눔
//   보는 방향 성분 > +half_m = 먼 줄, < −half_m = 가까운 줄, 그 사이 = 가운데 줄. 왼쪽 성분도 같은 식 (half_m = 눈 간격 반)
//   en = true 면 "far-left" 꼴 (그림용, 한글 글꼴 없음)
std::string cell_spot_name(
  const std::array<double, 2> & center, const std::array<double, 2> & view_dir, const std::array<double, 2> & cell,
  bool en = false, double half_m = 0.03);

// 장소 결과 모아 전체 판정 (세 장소 다 있으면 complete, ID 중복 · 누락 · pass)
VerifyResult summarize(const std::vector<PlaceVerify> & places, int total_ids = 12);

// 기록 파일 내용 (yaml)
std::string verify_to_yaml(const VerifyResult & r, const DicePlan & plan, const DiceMap & arena,
  const std::string & barcode, const std::string & signal_sequence);

}  // namespace mission
