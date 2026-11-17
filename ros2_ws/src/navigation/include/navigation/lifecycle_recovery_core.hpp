// lifecycle_recovery_core.hpp — 시작 lifecycle 순서·복구 계산임. ROS/스레드/벽시계 의존 없음.
// scripts/lifecycle_recovery_core.py 의 C++ 판 (같은 상태·같은 문구). startup_lifecycle_watchdog 이 씀.
// 받음: 단조 시각, 실제 준비 조건, 서비스·노드 상태 관측, 요청 결과.
// 보냄: 한 번에 하나의 STARTUP/RESET/노드 전환 (recover_only 면 + lifecycle_manager 프로세스 재시작 요청).
//   속도·hold·FAULT는 다루지 않음.
// recover_only (기본): 처음 켜기는 Nav2 lifecycle_manager autostart 가 함. 이 계산은 autostart 가 기회를 가진 뒤
//   실패한 manager 만 고침 — is_active false (Aborting bringup) → RESET→정합→STARTUP, is_active 무응답 (서비스 응답 유실로
//   manager 가 change_state 를 끝없이 기다림) → manager 프로세스 재시작 (launch respawn 이 다시 띄움, autostart 다시).
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace navigation
{
namespace lifecycle
{

constexpr int UNCONFIGURED = 1;
constexpr int INACTIVE = 2;
constexpr int ACTIVE = 3;
constexpr int STARTUP = 0;
constexpr int RESET = 3;
constexpr int CLEANUP = 2;
constexpr int DEACTIVATE = 4;

// active 관측은 value 1 = True, 0 = False. 노드 상태 관측은 lifecycle state id
struct Observation
{
  int value = 0;
  double requested = 0.0;
  double received = 0.0;
};

struct Snapshot
{
  bool inputs_ready = false;
  bool services_ready = false;
  // recover_only: manager 의 manage_nodes · is_active 서비스가 보임 (autostart 판단을 시작하는 문)
  bool manager_ready = false;
  // recover_only + output_timeout_s > 0: 그룹이 ACTIVE 일 때 나와야 할 출력 (localization = map→odom TF)
  bool output_ok = true;
  std::string output_reason;
  std::string input_reason = "입력 기다림";
  std::optional<Observation> active;
  std::map<std::string, Observation> states;

  const Observation * state(const std::string & n) const
  {
    auto it = states.find(n);
    return it == states.end() ? nullptr : &it->second;
  }
};

struct Request
{
  int64_t token = 0;
  std::string kind;     // manager / node / process (recover_only: lifecycle_manager 프로세스 재시작)
  std::string target;
  int command = 0;
  double sent = 0.0;
  double deadline = 0.0;
  std::string purpose;
  std::optional<double> timed_out_at;
};

struct RecoveryOptions
{
  int64_t max_recoveries = 3;
  double command_timeout = 90.0;
  double node_timeout = 20.0;
  double drain_timeout = 120.0;
  double verify_timeout = 45.0;
  double stable_s = 16.0;
  double backoff_s = 1.0;
  double freshness_s = 3.0;
  double cleanup_timeout = 90.0;
  double native_reconnect_s = 11.0;
  // ---- recover_only ----
  bool recover_only = false;
  double autostart_grace_s = 30.0;   // 문 (manager 서비스 · 입력) 열린 뒤 이만큼 지나도 준비 안 되면 실패
  double abort_confirm_s = 6.0;      // is_active false 가 이만큼 이어지면 (autostart 가 끝났는데 비활성 = Aborting bringup) 바로 실패
  double hang_s = 20.0;              // 보낸 전환이 기한을 넘고 상태 조회도 이만큼 무응답이면 manager 멈춤 → 프로세스 재시작
  double output_timeout_s = 0.0;     // > 0: lifecycle 은 ACTIVE 인데 출력 (Snapshot::output_ok) 이 이만큼 없으면 RESET
  int64_t max_restarts = 2;          // manager 프로세스 재시작 한도 (RESET 복구 max_recoveries 와 따로 셈)
  // autostart 판단 시간을 입력 준비 (inputs_ready) 뒤부터 셈. navigation = true (localization · 지도 · map→odom 전엔
  //   planner activate 가 TF 를 기다리는 게 정상). localization = false (AMCL activate 는 스캔이 필요 없음)
  bool autostart_needs_inputs = true;
};

// 초기 STARTUP + 실패 시 RESET→정합→STARTUP 최대 max_recoveries회임.
// ROS service timeout은 원격 취소가 아님. timeout 뒤에도 원래 응답 또는
// 그 뒤 요청한 같은 대상 상태 조회 응답(직렬 callback 장벽)을 기다림.
// 어느 쪽도 안 오면 복구 상태만 exhausted로 보고하며 다음 전환을 겹치지 않음.
class LifecycleRecovery
{
public:
  LifecycleRecovery(std::string name, std::vector<std::string> nodes, RecoveryOptions options = {});

  bool healthy(const Snapshot & s, double now) const;
  // 늦은/이전 세대 응답은 상태를 바꾸지 않음.
  bool complete(int64_t token, bool success, double now, const std::string & detail = "");
  std::optional<Request> tick(double now, const Snapshot & s);
  // 다른 그룹 시작 중 먼저 켜진 그룹이 다시 내려간 경우만 복구함.
  void recheck_before_handoff(double now, const Snapshot & s);

  std::string name;
  std::vector<std::string> nodes;
  RecoveryOptions opt;
  std::string phase = "waiting_inputs";
  std::optional<Request> pending;
  int64_t recoveries = 0;
  int64_t restarts = 0;   // recover_only: manager 프로세스 재시작 횟수
  std::string last_error;
  std::string reason = "시작 입력 기다림";

private:
  bool fresh(const Observation * obs, double now, double after = -1.0) const;
  bool fresh(const std::optional<Observation> & obs, double now, double after = -1.0) const
  {
    return fresh(obs ? &*obs : nullptr, now, after);
  }
  Request & request(double now, const std::string & kind, const std::string & target, int command,
    const std::string & purpose);
  void recover(double now, const std::string & why);
  void schedule_reset(double now, const std::string & why);
  void cleanup(double now);
  // recover_only
  void enter_autostart(double now, const std::string & why);
  std::optional<Request> restart(double now, const std::string & why);
  void fail_autostart(double now, const std::string & why);

  bool native_possible_ = false;
  std::optional<double> native_until_;
  double native_entered_ = 0.0;
  std::optional<bool> native_observed_active_;
  int64_t token_ = 0;
  double next_at_ = 0.0;
  double verify_started_ = 0.0;
  std::optional<double> stable_since_;
  double cleanup_started_ = 0.0;
  std::size_t cleanup_index_ = 0;
  double after_ = 0.0;
  std::map<std::pair<std::string, int>, int> changes_;
  // recover_only
  double autostart_since_ = -1.0;            // 이 시각 뒤에 요청한 is_active 응답만 봄 (재시작 전 이전 manager 응답 무시)
  std::optional<double> gate_since_;         // 문이 열린 시각
  std::optional<double> false_since_;        // is_active false 가 이어진 시작
  std::optional<double> output_missing_since_;
  bool force_reset_ = false;                 // lifecycle 은 ACTIVE 지만 출력이 없어 RESET 이 필요 (waiting_reset 의 정상 지름길 안 씀)
};

// startup_lifecycle_watchdog 의 /startup/lifecycle_status JSON
// (파이썬 json.dumps(status, ensure_ascii=False, sort_keys=True) 와 같은 글자)
struct WatchdogStatus
{
  bool ready = false;
  bool finished = false;
  bool localization_ready = false;
  bool navigation_ready = false;
  bool scan = false;
  bool odom_tf = false;
  bool map = false;
  bool map_odom_tf = false;
  // 그룹 이름 순 (localization, navigation)
  std::vector<std::pair<std::string, const LifecycleRecovery *>> groups;
};
void write_watchdog_status(std::string & out, const WatchdogStatus & st);

// ---- lifecycle_manager 프로세스 찾기 (recover_only 재시작, ROS 없음 · 시험 대상) ----
// /proc/<pid>/cmdline (NUL 구분) 에 노드 이름 remap "__node:=<node>" 이 따로 한 칸으로 있나 (launch_ros Node 의 -r __node:=)
bool cmdline_has_node(const std::string & cmdline, const std::string & node);
// /proc/<pid>/environ (NUL 구분) 의 ROS_DOMAIN_ID. 없거나 빈 값이면 "0" (ROS 기본)
std::string environ_domain(const std::string & environ);
// getenv("ROS_DOMAIN_ID") 와 같은 꼴로: nullptr · 빈 값 → "0", 앞뒤 공백 뺌
std::string normalize_domain(const char * value);

}  // namespace lifecycle
}  // namespace navigation
