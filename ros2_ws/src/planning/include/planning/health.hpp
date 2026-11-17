// health.hpp — 실행 조건 감시: 센서 끊김 · 위치 추정 품질
//
// 역할
//   - ConditionMonitor: 매 주기 나쁨 · 좋음을 넣으면 대기 원인을 켜고 끄고 FAULT 를 판정
//       켜짐  = 나쁨이 들어오면 바로, 그 순간부터 해당 동작 차단
//       꺼짐  = 좋음이 recover_s 동안 이어지면 자동 해제
//       FAULT = 켜진 뒤 fault_after_s 가 지나도 안 꺼짐, 또는 자동 해제를 max_recoveries 번 쓴 뒤 다시 나빠짐
//   - covariance_std: PoseWithCovariance 6×6 행 우선 36 칸 → x · y · yaw 표준편차, AMCL 쓸 때
//   - loc_health_lost · loc_health_ready: pose_guard 의 /localization_health level 판정, slam_toolbox 쓸 때
//   - arm_goal_uses_map · arm_start_blocked: 대기 원인이 켜졌을 때 새 팔 목표 시작을 막을지
//   - LocRecovery: 위치 추정 상실을 FAULT 없이 단계로 처리 (대기 → 마지막 정상 자리로 물러나기 → 조건부 계속)
//       Wait     = 상실 직후. 이동 · map 의존 팔 목표 차단, 정상이 ok_hold_s 이어지면 Ok
//       Backoff  = 대기 wait_s 넘음 + 물러나기 가능 → 실행기가 마지막 정상 자리 쪽으로 짧게 이동 (최대 max_backoff 번)
//       Degraded = 상실 degrade_s 넘음 + map → odom 안정 (또는 물러나기 다 씀), 또는 force_s 넘음 → 차단 풀고 계속
//       무한 대기 없음: force_s 넘으면 안정 여부와 상관없이 Degraded
// 입력
//   - 매 주기 나쁨 · 좋음과 시각, 공분산 배열, 위치 감시 level 과 받은 시각, 팔 작업 번호 · 목표 종류
// 출력
//   - 대기 원인 켜짐 여부, FAULT 여부, 표준편차, 위치 상실 · 준비 여부, 팔 시작 차단 여부
// 실행
//   - planning_core 라이브러리, Executor 가 씀. 시험 test_planning_core
// note
//   - FAULT 는 reset() 전까지 유지
//   - max_recoveries < 0 = 횟수 제한 없음

#pragma once

#include <cstdint>
#include <optional>

#include "planning/types.hpp"

namespace planning
{

enum class Health
{
  Ok,        // 문제 없음
  Waiting,   // 대기 원인 켜짐 (차단 중)
  Fault,     // 회복 못 함
};

class ConditionMonitor
{
public:
  explicit ConditionMonitor(double recover_s = 2.0, int max_recoveries = 2, double fault_after_s = 10.0)
  : recover_s_(recover_s), fault_after_s_(fault_after_s), max_recoveries_(max_recoveries) {}

  Health update(bool bad, double now);

  bool active() const {return active_ || faulted_;}
  void reset();   // 새로 시작 (FAULT 복구 · start 때)

private:
  double recover_s_;
  double fault_after_s_;
  int max_recoveries_;
  bool active_ = false;
  bool faulted_ = false;
  double bad_since_ = 0.0;
  double good_since_ = -1.0;
  int recoveries_ = 0;
};

// 검출 끊김 구간: perception 이 멈춰 respawn 하는 동안 LOOK 결과가 빈 것을 알아보려고 끊김 시작 · 끝을 남김
//   켜짐 = 나쁨이 들어오면 바로 (시작 시각 = 그때), 꺼짐 = 좋음이 recover_s 이어지면 (끝 시각 = 좋음이 시작된 때)
//   overlaps(since) = [since, 지금] 에 끊김이 있었나 (지금 끊김 포함). 끊김 한 번 = count +1
class DetectionGap
{
public:
  explicit DetectionGap(double recover_s = 1.0) : recover_s_(recover_s) {}

  // 반환: 이번 주기에 끊김이 시작됨 (+1) · 끝남 (-1) · 그대로 (0) → 부르는 쪽 로그용
  int update(bool bad, double now);
  bool active() const {return active_;}
  bool overlaps(double since) const {return active_ || (count_ > 0 && end_ >= since);}
  double started() const {return start_;}
  double ended() const {return end_;}
  int count() const {return count_;}
  void reset();

private:
  double recover_s_;
  bool active_ = false;
  double start_ = -1.0, end_ = -1.0, good_since_ = -1.0;
  int count_ = 0;
};

// 검출 끊김이 LOOK 결과를 쓰는 단계와 겹쳤을 때 할 일
//   look_since < 0 (이 정차에 LOOK 없음) · 안 겹침 → Proceed, 아직 끊김 → Wait (회복 · respawn 기다림, FAULT 는 감시가 정함)
//   끝난 끊김과 겹침 → redo_used < redo_max 면 Redo (같은 LOOK 다시 뒤 이 단계 다시), 다 썼으면 Proceed (본 만큼으로)
enum class GapAction
{
  Proceed,
  Wait,
  Redo,
};
GapAction gap_action(const DetectionGap & g, double look_since, int redo_used, int redo_max, bool redo_allowed = true);

// 위치 표준편차 [m, m, rad]
struct PoseStd
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// 공분산 36칸 → 표준편차. 대각 값이 유한 · 0 이상이 아니면 nullopt (못 믿는 값)
std::optional<PoseStd> covariance_std(const double * cov36);

// pose_guard 의 /localization_health (diagnostic_msgs/DiagnosticStatus) 마지막 값
//   level 0 정상 · 1 경고 (느리게) · 2 정지 · 3 STALE
struct LocHealth
{
  int level = -1;            // -1 = 아직 못 받음
  double received_s = -1.0;  // 받은 시각 (DiagnosticStatus 에는 header 가 없음)
};

// 상실 = 못 받음 · timeout_s 넘게 끊김 · level 2 이상. 경고 level 1 은 계속 감
bool loc_health_lost(const LocHealth & h, double now, double timeout_s);

// 출발 준비 = 신선하고 level 0
bool loc_health_ready(const LocHealth & h, double now, double timeout_s);

// 팔 작업 target 이 위치 추정에 기대는가 (Executor::arm_target 과 같은 구분)
//   HOME · LOOK 류 · LOOK_BARCODE · LOOK_RACK · STORE · RETRIEVE = 로봇 자체 기준 (자세 · 칸 이름 · base_link 칸 측정) → false
//   그 밖 (PICK · PLACE) 중 Target::Puck · Target::Custom → true
//     Puck   = odom 최근 검출 또는 퍽 기록부 map 위치 → 최신 TF 로 base_link
//     Custom = 미션 TargetResolver 의 점 → 최신 TF 로 base_link. 프레임은 resolver 가 정해 실행기가 미리 모름
//              (미션 PLACE 는 map) → base_link 점이어도 보수적으로 위치 의존으로 봄
bool arm_goal_uses_map(uint8_t task, Target target);

// 새 팔 목표를 보내기 전에 막나 (보낸 뒤 진행 중인 작업은 안 건드림, 회복 · FAULT 판단은 감시가 함)
//   DETECTION_LOST    : PICK 시작 차단
//   LOCALIZATION_LOST : PICK 시작 + arm_goal_uses_map 인 팔 작업 시작 차단
bool arm_start_blocked(uint8_t task, Target target, bool detection_lost, bool localization_lost);

// 위치 추정 상실 처리 단계
enum class LocMode
{
  Ok,        // 정상
  Wait,      // 상실, 제자리 대기 (차단)
  Backoff,   // 상실, 마지막 정상 자리 쪽으로 물러나는 중 (차단, 실행기가 이동)
  Degraded,  // 상실이 이어지지만 차단 풀고 계속 (위치를 그대로 믿음)
};

const char * loc_mode_name(LocMode m);

struct LocRecoveryConfig
{
  double ok_hold_s = 3.0;    // 정상이 이만큼 이어지면 Ok
  double wait_s = 10.0;      // 대기 (상실 직후 · 물러나기 뒤) 이만큼 지나면 물러나기
  double degrade_s = 30.0;   // 상실 이만큼 + map → odom 안정 (또는 물러나기 다 씀) 이면 Degraded
  double force_s = 60.0;     // 상실 이만큼이면 무조건 Degraded (무한 대기 막음)
  int max_backoff = 2;       // 상실 한 번에 물러나기 최대 횟수
};

class LocRecovery
{
public:
  explicit LocRecovery(LocRecoveryConfig c = {}) : c_(c) {}

  // lost: 위치 추정 상실 판정, stable: map → odom 이 최근 안정, can_backoff: 지금 물러나기 이동 가능
  // 반환: 이번 주기에 물러나기를 시작해야 함 (mode 가 Backoff 로 바뀜)
  bool update(bool lost, bool stable, bool can_backoff, double now);
  void backoff_done(double now);   // 물러나기 끝 (성공 · 실패 · 건너뜀) → 다시 Wait

  LocMode mode() const {return mode_;}
  bool blocks() const {return mode_ == LocMode::Wait || mode_ == LocMode::Backoff;}
  double lost_for(double now) const {return mode_ == LocMode::Ok ? 0.0 : now - lost_since_;}
  int backoffs() const {return backoffs_;}
  void reset();

private:
  LocRecoveryConfig c_;
  LocMode mode_ = LocMode::Ok;
  double lost_since_ = 0.0;     // 이번 상실 시작
  double wait_since_ = 0.0;     // 지금 대기 시작 (상실 · 물러나기 끝)
  double good_since_ = -1.0;    // 정상 연속 시작
  int backoffs_ = 0;
};

}  // namespace planning
