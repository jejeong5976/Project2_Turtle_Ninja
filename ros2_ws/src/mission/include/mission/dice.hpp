// dice.hpp — 주사위 기억 미션 계산
//
// 역할
//   1. 바코드 해석: Code128 원문 → 정렬 방향, 표 = 레포 maps/dice_barcodes.json (2026-11-04)
//        발제 v5: P2-01 · P2-03 오름차순, P2-02 · P2-04 내림차순
//   2. 그룹 정렬: 신호등 R · Y · G 횟수 → L1 · L2 · L3 에 놓을 그룹, 동률이면 R → Y → G 순서 유지
//   3. 주사위 눈: 눈 수 → 장소 중심 기준 놓을 점, 3×3 격자 간격 60 mm
//        그림 좌표 → apply_view_directions 로 관찰 원에서 본 방향에 맞춤
// 입력
//   - 신호 횟수 SignalRecorder::count, 바코드 원문, 바코드 표 (load_barcode_table)
// 출력
//   - DicePlan: 장소별 그룹 · 눈 수 · 놓을 점
// 실행
//   - mission_core 라이브러리. mission_manager 는 dice_layout · apply_view_directions 를 부름,
//     바코드 해석은 task_parser 를 거쳐 parse_dice_barcode
//   - 시험 test_mission_core · test_transfer
// note
//   - 퍽 할당 (R = 빨강만, G = 초록만, Y = 한 모양 통일) 은 여기서 안 함
//     → transfer_solver 가 두 모양을 다 풀어 시간이 짧은 쪽으로 정함, Y 에 이미 놓은 퍽이 있으면 재계획 때 그 모양으로 고정
//   - 순수 C++, ROS 없음

#pragma once

#include "mission/common.hpp"

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace mission
{

// 바코드 정렬 방향
enum class DiceOrder { Ascending, Descending };

// 신호 그룹 횟수 (R · Y · G)
struct SignalCounts
{
  int red = 0;
  int yellow = 0;
  int green = 0;

  int of(char group) const;     // 'R' · 'Y' · 'G' (그 밖은 0)
  int total() const {return red + yellow + green;}
};

// 장소 하나 (L1 · L2 · L3)
struct DicePlace
{
  char group = '?';                         // 'R' · 'Y' · 'G'
  int pips = 0;                             // 주사위 눈 수 = 그 그룹 신호 횟수
  std::vector<std::array<double, 2>> pip_offsets;   // 장소 중심 기준 (dx, dy) [m], 지도 프레임
};

struct DicePlan
{
  DiceOrder order = DiceOrder::Ascending;
  std::array<DicePlace, 3> places;          // L1 · L2 · L3
};

struct DiceConfig
{
  int total_pucks = 12;                     // 발제: 퍽 12개, 신호 합 12
  int min_count = 1;                        // 그룹별 횟수 범위 (발제 각 1–6)
  int max_count = 6;
  double pip_pitch_m = 0.060;               // 격자 간격 (발제 60 mm)
};

// 바코드 원문 → 정렬 방향 표
using BarcodeTable = std::map<std::string, DiceOrder>;

// 바코드 표 파일 읽기 (JSON, yaml-cpp 로 읽음)
//   {"barcodes": [{"barcode": "P2-01", "sortOrder": "ASC" | "DESC", "sortLabel": "오름차순"}, …]}
//   sortLabel 은 사람용이라 안 씀. 빈 목록 · 빈 바코드 · ASC/DESC 아닌 값 · 같은 바코드 두 번이면 실패
bool load_barcode_table(const std::string & path, BarcodeTable & out, std::string * error = nullptr);

// 1. 바코드 원문 → 정렬 방향. 표에 없는 코드면 nullopt + error (발제: 미등록 바코드 = 오류 정지)
std::optional<DiceOrder> parse_dice_barcode(
  const std::string & raw_text, const BarcodeTable & table, std::string * error = nullptr);

// 신호 횟수가 규칙에 맞는가 (각 min–max, 합 total_pucks)
bool valid_signal_counts(const SignalCounts & c, const DiceConfig & cfg = {}, std::string * error = nullptr);

// 2. 그룹 정렬 → L1 · L2 · L3 그룹 글자. 동률이면 R → Y → G 순서 유지
std::array<char, 3> sort_dice_groups(const SignalCounts & c, DiceOrder order);

// 3. 눈 수 (1–6) → 장소 중심 기준 놓을 점 (dx, dy) [m]. 발제 p.4 그림 그대로 (2 · 3 = 왼쪽 위 → 오른쪽 아래 대각선, 6 = 좌우 두 열)
//    범위 밖이면 빈 목록
std::vector<std::array<double, 2>> dice_pip_offsets(int pips, double pitch_m = 0.060);

// 장소 배치: 횟수 검사 → 그룹 정렬 → 그림 좌표 눈 위치 (퍽 할당은 운반 계획기가 함)
//   횟수 규칙이 틀리면 nullopt + error
std::optional<DicePlan> dice_layout(
  const SignalCounts & counts, DiceOrder order, const DiceConfig & cfg = {}, std::string * error = nullptr);

// 보는 방향 기준 눈 배치 (2026-10-30 사용자 결정: 배치대 앞 관찰 원에서 봤을 때 주사위 모양)
//   picture : dice_pip_offsets 결과 = 그림 좌표 (x 오른쪽, y 위 = 보는 사람에서 먼 쪽)
//   (fx, fy): 보는 방향 = 관찰 원 → 장소 중심 단위 벡터 (지도 프레임). 오른쪽 = (fy, −fx)
//   지도 좌표 = x · 오른쪽 + y · 보는 방향.  (0, 1) (남쪽에서 북쪽을 봄) 이면 그림 = 지도 그대로
//   주의: 발제 p.4 문구는 "지도 위쪽이 +Y, 모든 장소 같은 방향" → 보는 방향을 전부 (0, 1) 로 주면 그 해석이 됨
std::vector<std::array<double, 2>> pips_view_to_map(
  const std::vector<std::array<double, 2>> & picture, double fx, double fy);

// DicePlan 의 장소마다 보는 방향 적용 (dice_layout 결과는 그림 좌표 = 보는 방향 (0, 1)) → 먼 칸부터 정렬까지 함
//   view_dir[i] = L(i+1) 관찰 원 → 장소 중심 단위 벡터. 접근 방향(장소 → 로봇) = −view_dir 로 order_far_first
void apply_view_directions(DicePlan & plan, const std::array<std::array<double, 2>, 3> & view_dir);
// 그림 위쪽 방향 up[i] (map, 발제 p.4 = 경기장 +Y 를 세 곳 다) 으로 눈을 놓고, 놓는 순서만 보는 방향 view_dir 기준 먼 칸부터
void apply_place_directions(DicePlan & plan, const std::array<std::array<double, 2>, 3> & up,
  const std::array<std::array<double, 2>, 3> & view_dir);

// 놓는 순서: 접근 방향 (장소 중심 → 로봇 쪽 단위 벡터 (ax, ay)) 기준 먼 점부터
//   → 먼저 놓은 퍽을 그리퍼가 건드리지 않게. pip_offsets 만 재정렬 (그룹 안 퍽은 아무 점에 놓아도 됨)
void order_far_first(DicePlace & place, double ax, double ay);

const char * dice_order_name(DiceOrder o);

}  // namespace mission
