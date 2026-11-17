// signal_recorder.hpp — 신호등 점멸 순서 기록
//
// 역할
//   - 발제 신호등 과제
//       출발 구역에서 신호등이 12 번 점멸: 한 색 1 s 켜짐 + 1 s 소등, 같은 색이 이어져도 소등으로 구별
//       12 번째 점멸 뒤 3 s 연속 소등 = 끝, 별도 출발 신호 없음
//       R · Y · G 각 1–6 회, 합 12. 소등은 횟수에 안 넣음. 누락 · 불명 색은 추정 금지
//   - 판단
//       점멸 1 개 = 색 관측 시작 ~ Off 가 min_off_s 0.25 s 이상 이어질 때까지. 켜짐 < min_on_s 0.3 s 면 깜빡임으로 버림
//       켜짐 길이 = 첫 켜짐 관측 ~ 마지막 켜짐 관측 (관측 시각만). 다음 Off 도착까지로 재면 관측 끊김 (엔진 막 켜짐 등)
//         만큼 길어져 0.14-0.38 s 가짜 점멸이 세어짐
//       점멸 색 = 그 안 색 관측 다수결. 다수 비율 < majority 0.8 또는 애매 관측이 많으면 '?'
//       기록 시작 = 첫 '쓸 만한 점멸' (색 R · Y · G 확실 + 켜짐 ≥ min_on_s). 그 전 '?' · 짧은 점멸은 세지 않음 (notes 에만)
//       소등 없이 다른 색으로 바로 바뀌면 점멸을 나누고 문제로 기록
//       NotSeen 은 켜짐 · 소등 어느 쪽으로도 안 셈. 기록 중 max_gap_s 0.8 s 이상이면 불완전
//       끝 = expected 12 개 이상 뒤 Off 가 end_off_s 2.5 s 이상 → Done (또는 문제 있으면 Incomplete)
//       12 개 전에 긴 소등 = 중간부터 본 시퀀스 (또는 놓침) → 끝내지 않고 따로 남긴 뒤 (partial) 다음 시퀀스를 처음부터 기다림
//       give_up = 대기 시간 초과 (mission 의 signal_wait_s) → Incomplete. 지금 기록이 비었으면 마지막 partial 을 결과로 남김
//       Done 이어도 '?' · 끊김 · 횟수 규칙 위반이 있으면 Incomplete
// 입력
//   - 관측 하나씩: 시각 + 종류, /perception/traffic_light 에서 바꿈
//       Off       = TrafficLight OFF
//       Red · Yellow · Green
//       Ambiguous = UNKNOWN + visible
//       NotSeen   = UNKNOWN + !visible, 토픽 끊김 포함
// 출력
//   - 상태 Waiting · Recording · Done · Incomplete, 점멸 순서 · 색별 횟수 · 점멸 시각 · 문제 목록
//   - 노드가 interfaces/SignalRecord 로 바꿔 발행
// 실행
//   - mission_core 라이브러리, mission_manager 가 씀. 시험 test_mission_core
// note
//   - end_off_s 2.5 s 는 발제 3 s 에서 관측 지연 여유를 뺀 값

#pragma once

#include "mission/common.hpp"

#include <string>
#include <vector>

namespace mission
{

enum class SignalObs { Off, Red, Yellow, Green, Ambiguous, NotSeen };

enum class SignalStatus
{
  Waiting,      // 첫 쓸 만한 점멸 전 (중간부터 본 시퀀스를 버린 뒤 포함)
  Recording,    // 점멸 기록 중
  Done,         // 12 개 · 끝 소등 확인 · 색 모두 확실 · 횟수 규칙 맞음
  Incomplete,   // 끝났지만 믿을 수 없음 (issues 참고). 색을 추정해 채우지 않음
};

struct SignalRecorderConfig
{
  int expected = 12;            // 점멸 수
  double min_on_s = 0.3;        // 이보다 짧은 켜짐은 깜빡임 (오검출)
  double min_off_s = 0.25;      // 점멸 사이 소등으로 인정하는 최소 길이
  double end_off_s = 2.5;       // 끝 소등 (발제 3 s, 관측 지연 여유)
  double max_gap_s = 0.8;       // 이보다 긴 NotSeen 은 점멸 하나를 가릴 수 있음 (켜짐 1 s)
  double majority = 0.8;        // 점멸 색 다수결 최소 비율
  double max_ambiguous = 0.3;   // 점멸 안 애매 관측 비율 상한
  int min_count = 1;            // 색별 횟수 범위 (발제 각 1–6)
  int max_count = 6;
};

struct SignalBlink
{
  char color = '?';             // 'R' · 'Y' · 'G' · '?'
  double start = 0.0;           // 첫 색 관측 시각
  double end = 0.0;             // 마지막 색 관측 시각
};

// 12 개 전에 끝 소등이 와 남겨 둔 시퀀스 하나 (본 그대로 보존, 결과에 이어 붙이지 않음)
struct SignalPartial
{
  int n = 0;                    // 몇 번째 partial (1 부터, reset 전까지)
  double parked = 0.0;          // 끝 소등 시작 시각 (남긴 시각)
  std::string sequence;
  std::vector<SignalBlink> blinks;
  std::vector<std::string> issues;
};

class SignalRecorder
{
public:
  explicit SignalRecorder(SignalRecorderConfig cfg = {}) : cfg_(cfg) {}

  // 관측 하나 (시각 순서대로). 끝난 뒤(Done · Incomplete)에는 무시
  void observe(double t, SignalObs o);
  void reset() {*this = SignalRecorder(cfg_);}
  // 기다림 끝 (시간 초과): 끝나지 않았으면 Incomplete. 지금 점멸이 없으면 마지막 partial 을 결과로
  void give_up(double t, const std::string & why);

  SignalStatus status() const {return status_;}
  bool finished() const {return status_ == SignalStatus::Done || status_ == SignalStatus::Incomplete;}
  const std::vector<SignalBlink> & blinks() const {return blinks_;}
  std::string sequence() const;                  // 예: "RGRRGRGGYGYG" ('?' 포함 가능)
  int count(char c) const;                       // 'R' · 'Y' · 'G' · '?'
  const std::vector<std::string> & issues() const {return issues_;}   // 결과를 못 믿게 하는 문제 (있으면 Incomplete)
  const std::vector<std::string> & notes() const {return notes_;}     // 참고 (짧은 깜빡임 버림 등)
  int partials() const {return n_partial_;}                           // 12 개 전에 끝 소등이 와 버린 시퀀스 수
  const std::string & last_partial() const {return partial_seq_;}     // 그 마지막 순서
  const std::vector<SignalPartial> & partial_history() const {return partial_history_;}   // 모든 partial (기록 파일용, 판정에 안 씀)

private:
  void close_blink(double t);
  void finish(double t);
  void park_partial(double t);
  void issue(const std::string & s) {issues_.push_back(s);}

  SignalRecorderConfig cfg_;
  SignalStatus status_ = SignalStatus::Waiting;
  std::vector<SignalBlink> blinks_;
  std::vector<std::string> issues_;
  std::vector<std::string> notes_;

  // 지금 점멸 (켜짐 구간)
  bool in_blink_ = false;
  double blink_start_ = 0.0, blink_last_on_ = 0.0;   // 첫 · 마지막 켜짐 관측 시각
  int n_red_ = 0, n_yellow_ = 0, n_green_ = 0, n_amb_ = 0;
  char cur_color_ = 0;                           // 지금 점멸에서 마지막으로 본 색 (색 바뀜 감지)

  double off_since_ = -1.0;                      // Off 연속 시작 (-1 = 아님)
  double gap_since_ = -1.0;                      // NotSeen 연속 시작
  bool gap_flagged_ = false;
  double last_t_ = -1.0;

  // 12 개 전에 끝 소등이 와 버린 시퀀스 (중간부터 봄 · 놓침). 시간 초과 때 지금 기록이 비었으면 이것을 결과로
  int n_partial_ = 0;
  std::string partial_seq_;
  std::vector<SignalBlink> partial_blinks_;
  std::vector<std::string> partial_issues_;
  std::vector<SignalPartial> partial_history_;   // 덮어쓰지 않고 쌓음 (B2). 판정 · 결과에는 안 씀
};

const char * signal_status_name(SignalStatus s);

}  // namespace mission
