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

}  // namespace planning
