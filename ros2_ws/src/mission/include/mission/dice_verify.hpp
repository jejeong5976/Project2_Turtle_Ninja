// dice_verify.hpp — 세 장소 재검증 판정, 발제 v5 '이동 후 검증'
//
// 역할: 판정
//   - 묶기: 원시 관측 → 퍽 단위, 색 · 모양은 장 다수결 (표 갈리면 모름, 추정 안 함)
//   - 칸: 관측 위치가 계획 칸 중심에서 cell_tol_m 안, 가장 가까운 칸, 한 칸에 하나
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
#include <optional>
#include <string>
#include <vector>

#include "mission/dice.hpp"
#include "mission/dice_map.hpp"

namespace mission
{

struct VerifyObs
{
  std::string id;               // "" = ID 못 읽음
  std::string color;
  std::string shape;
  double x = 0.0;               // map [m]
  double y = 0.0;
  int cell = -1;                // 판정 뒤 채움
  double cell_error_m = 0.0;
  bool ok = false;
};

struct PlaceVerify
{
  int place = -1;               // 0–2
  char group = '?';
  int expected = 0;             // 눈 수
  std::vector<VerifyObs> seen;
  bool ok = false;
  std::vector<std::string> issues;
  double stamp_s = 0.0;         // 본 시각
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
//   ID 있는 관측: 같은 ID 끼리 한 퍽
//   ID 없는 관측: merge_m 안끼리 한 퍽, ID 있는 퍽 merge_m 안이면 그 퍽에 합침 (같은 퍽을 태그 못 읽은 장)
//   위치 = 가장 최근 장. 색 · 모양 = 읽힌 장 ("" 아닌 장) 다수결, 1 등 비율 < vote_ratio 거나 읽힌 장 없으면 "" (모름)
std::vector<VerifyObs> group_place_obs(
  const std::vector<VerifyObs> & raw, double merge_m = 0.03, double vote_ratio = 0.7);

// 장소 하나 판정. obs = 장소 반경 안 새 관측 (group_place_obs 로 묶은 것). plan 은 apply_view_directions 를 거친 것
PlaceVerify judge_place(
  int place, const DicePlan & plan, const DiceMap & arena, std::vector<VerifyObs> obs,
  double cell_tol_m = 0.03, double stamp_s = 0.0);

// 장소 결과 모아 전체 판정 (세 장소 다 있으면 complete, ID 중복 · 누락 · pass)
VerifyResult summarize(const std::vector<PlaceVerify> & places, int total_ids = 12);

// 기록 파일 내용 (yaml)
std::string verify_to_yaml(const VerifyResult & r, const DicePlan & plan, const DiceMap & arena,
  const std::string & barcode, const std::string & signal_sequence);

}  // namespace mission
