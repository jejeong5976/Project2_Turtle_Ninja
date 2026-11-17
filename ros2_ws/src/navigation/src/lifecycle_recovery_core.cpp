// lifecycle_recovery_core.cpp — scripts/lifecycle_recovery_core.py 의 C++ 판 (같은 분기·같은 문구).
#include "navigation/lifecycle_recovery_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "navigation/py_json.hpp"

namespace navigation
{
namespace lifecycle
{

LifecycleRecovery::LifecycleRecovery(std::string name_in, std::vector<std::string> nodes_in, RecoveryOptions options)
: name(std::move(name_in)), nodes(std::move(nodes_in)), opt(options)
{
  if (nodes.empty() || opt.max_recoveries < 0) {
    throw std::invalid_argument("관리 노드와 0 이상 복구 횟수 필요");
  }
  if (opt.recover_only) {
    phase = "waiting_autostart";
    reason = "Nav2 autostart 기다림";
  }
}

bool LifecycleRecovery::fresh(const Observation * obs, double now, double after) const
{
  return obs != nullptr && obs->requested >= after && 0 <= now - obs->received &&
         now - obs->received <= opt.freshness_s;
}

bool LifecycleRecovery::healthy(const Snapshot & s, double now) const
{
  if (!(fresh(s.active, now) && s.active->value == 1)) {
    return false;
  }
  for (const auto & n : nodes) {
    const Observation * o = s.state(n);
    if (opt.recover_only) {
      // manager is_active true 를 믿음. 감시기가 못 보는 노드 (조회 응답 없음) 때문에 정상 manager 를 RESET 하지 않음.
      // 최근에 본 노드가 ACTIVE 아님이면 실패.
      if (fresh(o, now) && o->value != ACTIVE) {
        return false;
      }
    } else if (!(fresh(o, now) && o->value == ACTIVE)) {
      return false;
    }
  }
  return true;
}

Request & LifecycleRecovery::request(double now, const std::string & kind, const std::string & target,
  int command, const std::string & purpose)
{
  ++token_;
  const double timeout = kind == "manager" ? opt.command_timeout : opt.node_timeout;
  Request r;
  r.token = token_;
  r.kind = kind;
  r.target = target;
  r.command = command;
  r.sent = now;
  r.deadline = now + timeout;
  r.purpose = purpose;
  pending = r;
  reason = purpose;
  return *pending;
}

void LifecycleRecovery::recover(double now, const std::string & why)
{
  last_error = why;
  stable_since_.reset();
  if (native_possible_) {
    // STARTUP 성공 뒤 bond 하락은 Nav2의 기존 재연결(10초 창)을 먼저 존중함.
    // 창 뒤에 보낸 is_active 응답이 같은 manager callback queue의 장벽임.
    phase = "waiting_native";
    reason = "Nav2 기존 bond 재연결 먼저 기다림";
    native_entered_ = now;
    native_until_.reset();
    native_observed_active_.reset();
    return;
  }
  schedule_reset(now, why);
}

void LifecycleRecovery::schedule_reset(double now, const std::string & why)
{
  if (recoveries >= opt.max_recoveries) {
    phase = "exhausted";
    reason = opt.max_recoveries == 3 ? "lifecycle 자동 복구 3회 한도 소진" : "lifecycle 복구 한도 소진";
  } else {
    phase = "waiting_reset";
    reason = why;
    next_at_ = now + opt.backoff_s;
  }
}

void LifecycleRecovery::cleanup(double now)
{
  phase = "cleanup";
  cleanup_started_ = after_ = now;
  cleanup_index_ = 0;
  changes_.clear();
  reason = "RESET 뒤 노드별 실제 상태 정합";
}

bool LifecycleRecovery::complete(int64_t token, bool success, double now, const std::string & detail)
{
  if (!pending || pending->token != token) {
    return false;
  }
  const Request r = *pending;
  pending.reset();
  if (phase == "exhausted") {
    return false;
  }
  if (r.purpose == "RESTART") {
    if (success) {
      enter_autostart(now, "lifecycle_manager 다시 띄움 (launch respawn): autostart 기다림");
    } else {
      phase = "exhausted";
      last_error = detail.empty() ? std::string("lifecycle_manager 프로세스 재시작 실패") : detail;
      reason = "lifecycle_manager 프로세스 재시작 실패: 상태 보고만";
    }
  } else if (r.purpose == "STARTUP") {
    if (success) {
      native_possible_ = true;
      phase = "verifying";
      verify_started_ = now;
      stable_since_.reset();
      reason = "manager·노드 ACTIVE 및 bond 안정 확인";
    } else {
      recover(now, detail.empty() ? std::string("STARTUP 실패") : detail);
    }
  } else if (r.purpose == "RESET") {
    if (!success) {
      last_error = detail.empty() ? std::string("RESET 일부 실패: 노드별 정합으로 이어 감") : detail;
    }
    cleanup(now);
  } else {
    // false여도 노드가 바뀌었을 수 있음. 새 GetState로 판단함.
    after_ = now;
    if (!success) {
      last_error = detail.empty() ? r.target + " 전환 응답 실패" : detail;
    }
  }
  return true;
}

std::optional<Request> LifecycleRecovery::tick(double now, const Snapshot & s)
{
  if (phase == "ready" || phase == "exhausted") {
    return std::nullopt;
  }
  if (pending) {
    Request & r = *pending;
    if (now < r.deadline) {
      return std::nullopt;
    }
    if (!r.timed_out_at) {
      r.timed_out_at = now;
      reason = r.purpose + " 응답 기한 지남: 원격 완료 확인 중";
    }
    const Observation * barrier = r.kind == "manager" ? (s.active ? &*s.active : nullptr) : s.state(r.target);
    if (fresh(barrier, now, *r.timed_out_at)) {
      // 상태 조회는 timeout 뒤에 보낸 것만 인정함. 오래된 관측은 장벽 아님.
      const std::string purpose = r.purpose;
      complete(r.token, false, now, purpose + " 응답 timeout 뒤 상태 조회 완료");
    } else if (opt.recover_only && now - *r.timed_out_at >= opt.hang_s) {
      // manager (또는 대상 노드) 가 응답을 잃고 멈춤: 같은 manager 에 새 전환을 쌓지 않고 프로세스를 다시 띄움.
      // pending 을 지워 늦게 온 이전 응답 (토큰 다름) 은 무시됨.
      const std::string why = r.purpose + " 응답 · 뒤 상태 조회 모두 무응답 (manager 멈춤)";
      pending.reset();
      return restart(now, why);
    } else if (now - *r.timed_out_at >= opt.drain_timeout) {
      phase = "exhausted";
      reason = "원격 lifecycle 응답 미완료: 전환 중복 없이 상태 보고";
      last_error = r.purpose + " 및 뒤 상태 조회 모두 무응답";
      // pending을 남겨 같은 대상에 새 전환을 보내지 않음.
    }
    return std::nullopt;
  }
  if (phase == "ready" || phase == "exhausted") {
    return std::nullopt;
  }
  auto all_fresh = [&](double after) {
      for (const auto & n : nodes) {
        if (!fresh(s.state(n), now, after)) {
          return false;
        }
      }
      return true;
    };
  auto any_not_unconfigured = [&]() {
      for (const auto & n : nodes) {
        if (s.state(n)->value != UNCONFIGURED) {
          return true;
        }
      }
      return false;
    };
  if (phase == "waiting_autostart") {
    // recover_only: autostart 가 스스로 켜기를 기다림. 감시기는 STARTUP 을 보내지 않음.
    if (healthy(s, now)) {
      native_possible_ = true;
      phase = "verifying";
      verify_started_ = now;
      stable_since_.reset();
      gate_since_.reset();
      false_since_.reset();
      reason = "autostart ACTIVE: 안정 확인";
      return std::nullopt;
    }
    if (!s.manager_ready || (opt.autostart_needs_inputs && !s.inputs_ready)) {
      // navigation 은 localization 준비 · 지도 · map→odom 전에는 activate 가 TF 를 기다리는 게 정상 → 시간 안 셈
      gate_since_.reset();
      false_since_.reset();
      reason = !s.manager_ready ? std::string("lifecycle_manager 서비스 기다림") : s.input_reason;
      return std::nullopt;
    }
    if (!gate_since_) {
      gate_since_ = now;
    }
    const bool answered = fresh(s.active, now, autostart_since_);
    if (answered && s.active->value == 0) {
      // autostart 가 도는 동안 manager 는 is_active 에 답하지 않음 (같은 callback group) → false 응답 = autostart 끝남 · 비활성
      if (!false_since_) {
        false_since_ = std::max(s.active->requested, *gate_since_);
      }
      if (now - *false_since_ >= opt.abort_confirm_s) {
        fail_autostart(now, "autostart 실패 (is_active false, Aborting bringup)");
        return std::nullopt;
      }
    } else if (answered) {
      false_since_.reset();
    }
    if (now - *gate_since_ < opt.autostart_grace_s) {
      reason = "Nav2 autostart 기다림";
      return std::nullopt;
    }
    if (answered) {
      fail_autostart(now, "autostart 시간 초과 (manager 는 응답, 노드 ACTIVE 아님)");
      return std::nullopt;
    }
    char buf[96];
    std::snprintf(buf, sizeof(buf), "lifecycle_manager is_active 무응답 %.0f s (서비스 응답 유실로 멈춤)", now - *gate_since_);
    return restart(now, buf);
  } else if (phase == "waiting_inputs") {
    if (!s.inputs_ready || !s.services_ready) {
      reason = !s.inputs_ready ? s.input_reason : std::string("lifecycle 서비스 준비 기다림");
      return std::nullopt;
    }
    if (!fresh(s.active, now) || !all_fresh(-1.0)) {
      reason = "lifecycle 초기 상태 확인 중";
      return std::nullopt;
    }
    if (healthy(s, now)) {
      native_possible_ = true;
      phase = "verifying";
      verify_started_ = now;
      stable_since_.reset();
    } else if (any_not_unconfigured()) {
      recover(now, "시작 전 일부 노드가 이미 configure/active 상태");
    } else {
      phase = "starting";
      return request(now, "manager", name, STARTUP, "STARTUP");
    }
  } else if (phase == "verifying") {
    if (opt.recover_only && opt.output_timeout_s > 0 && s.inputs_ready && healthy(s, now) && !s.output_ok) {
      // lifecycle 은 ACTIVE 인데 출력이 안 나옴 (AMCL 이 /map · /scan 을 못 받아 map→odom 없음 등) → RESET 이 구독을 새로 만듦
      stable_since_.reset();
      if (!output_missing_since_) {
        output_missing_since_ = now;
      }
      if (now - *output_missing_since_ >= opt.output_timeout_s) {
        output_missing_since_.reset();
        native_possible_ = false;   // bond 는 정상이라 Nav2 재연결을 기다릴 일이 아님
        force_reset_ = true;        // lifecycle 은 정상으로 보이므로 waiting_reset 에서 바로 verifying 으로 돌아가지 않게
        recover(now, "ACTIVE 인데 출력 없음: " + s.output_reason);
      } else {
        reason = "ACTIVE, 출력 기다림: " + s.output_reason;
      }
      return std::nullopt;
    }
    output_missing_since_.reset();
    if (s.inputs_ready && healthy(s, now)) {
      if (!stable_since_) {
        stable_since_ = now;
      }
      if (now - *stable_since_ >= opt.stable_s) {
        phase = "ready";
        reason = "lifecycle 시작 준비 완료";
      }
    } else if (healthy(s, now)) {
      stable_since_.reset();
      reason = s.input_reason;  // 센서 준비를 기다림. 정상 lifecycle을 RESET하지 않음.
      verify_started_ = now;
    } else {
      // bond가 잠깐 붙었다 다시 끊긴 경우도 검증 창 안에서 복구함.
      const bool had_active = stable_since_.has_value();
      stable_since_.reset();
      if (had_active || now - verify_started_ >= opt.verify_timeout) {
        recover(now, "STARTUP 뒤 ACTIVE/bond/입력 준비 확인 실패");
      }
    }
  } else if (phase == "waiting_native") {
    if (healthy(s, now)) {
      phase = "verifying";
      verify_started_ = now;
      stable_since_.reset();
      reason = "Nav2 기존 재연결 성공: ACTIVE 안정 다시 확인";
    } else if (now >= native_entered_ + opt.native_reconnect_s + opt.drain_timeout) {
      phase = "exhausted";
      reason = "Nav2 재연결 뒤 manager 상태 응답 없음: 전환 중복 없이 상태 보고";
      last_error = reason;
    } else if (fresh(s.active, now, native_entered_)) {
      // hard RESET이 오래 걸리면 native 재연결 타이머도 늦게 시작함.
      // 첫 최신 응답, 또는 ACTIVE→inactive 응답을 받은 시점부터 창을 셈.
      const bool active_value = s.active->value == 1;
      if (!native_until_ || (native_observed_active_ == true && !active_value)) {
        native_until_ = now + opt.native_reconnect_s;
      }
      native_observed_active_ = active_value;
      if (now >= *native_until_ && fresh(s.active, now, *native_until_) && all_fresh(*native_until_)) {
        // 같은 manager queue에서 창 뒤에 요청한 상태 조회가 끝났음.
        native_possible_ = false;
        schedule_reset(now, "Nav2 기존 재연결 창 및 최신 상태 조회 뒤에도 준비 미완료");
      }
    }
  } else if (phase == "waiting_reset") {
    if (healthy(s, now) && !force_reset_) {
      native_possible_ = true;
      phase = "verifying";
      verify_started_ = now;
      stable_since_.reset();
    } else if (now >= next_at_ && s.services_ready) {
      recoveries += 1;
      force_reset_ = false;
      phase = "resetting";
      return request(now, "manager", name, RESET, "RESET");
    }
    reason = "해당 lifecycle RESET 준비 기다림";
  } else if (phase == "cleanup") {
    if (now - cleanup_started_ >= opt.cleanup_timeout) {
      recover(now, "노드별 cleanup 기한 초과");
      return std::nullopt;
    }
    if (cleanup_index_ >= nodes.size()) {
      phase = "restart";
      next_at_ = now + opt.backoff_s;
      return std::nullopt;
    }
    const std::string node = nodes[nodes.size() - 1 - cleanup_index_];
    const Observation * obs = s.state(node);
    if (!fresh(obs, now, after_)) {
      reason = node + " RESET 뒤 상태 기다림";
      return std::nullopt;
    }
    if (obs->value == UNCONFIGURED) {
      cleanup_index_ += 1;
      return std::nullopt;
    }
    if (obs->value == ACTIVE || obs->value == INACTIVE) {
      const int command = obs->value == ACTIVE ? DEACTIVATE : CLEANUP;
      const auto key = std::make_pair(node, command);
      auto it = changes_.find(key);
      const int count = it == changes_.end() ? 0 : it->second;
      if (count >= 2) {
        recover(now, node + " 노드 전환 재시도 소진");
        return std::nullopt;
      }
      changes_[key] = count + 1;
      return request(now, "node", node, command, "노드별 cleanup");
    }
    reason = node + " 전환 중/UNKNOWN 상태 기다림";
  } else if (phase == "restart") {
    if (now < next_at_ || !s.inputs_ready || !s.services_ready) {
      reason = !s.inputs_ready ? s.input_reason : std::string("STARTUP 재시도 준비 기다림");
      return std::nullopt;
    }
    if (!all_fresh(-1.0)) {
      return std::nullopt;
    }
    if (any_not_unconfigured()) {
      recover(now, "cleanup 뒤 노드 상태가 다시 바뀜");
      return std::nullopt;
    }
    phase = "starting";
    return request(now, "manager", name, STARTUP, "STARTUP");
  }
  return std::nullopt;
}

void LifecycleRecovery::enter_autostart(double now, const std::string & why)
{
  phase = "waiting_autostart";
  reason = why;
  autostart_since_ = now;
  gate_since_.reset();
  false_since_.reset();
  stable_since_.reset();
  output_missing_since_.reset();
  native_possible_ = false;
}

std::optional<Request> LifecycleRecovery::restart(double now, const std::string & why)
{
  last_error = why;
  stable_since_.reset();
  if (restarts >= opt.max_restarts) {
    phase = "exhausted";
    reason = "lifecycle_manager 재시작 한도 소진: 상태 보고만";
    return std::nullopt;
  }
  restarts += 1;
  phase = "restarting";
  return request(now, "process", name, 0, "RESTART");
}

void LifecycleRecovery::fail_autostart(double now, const std::string & why)
{
  // autostart 가 성공한 적 없음 → Nav2 bond 재연결을 기다릴 일 없이 바로 RESET 차례
  native_possible_ = false;
  gate_since_.reset();
  false_since_.reset();
  recover(now, why);
}

void LifecycleRecovery::recheck_before_handoff(double now, const Snapshot & s)
{
  // 둘 다 준비된 뒤에는 adapter가 호출하지 않아 운전 중 개입하지 않음.
  // 센서 미수신 자체는 lifecycle 실패로 바꾸지 않음.
  if (phase != "ready") {
    return;
  }
  bool failed = fresh(s.active, now) && s.active->value == 0;
  if (!failed) {
    for (const auto & n : nodes) {
      const Observation * o = s.state(n);
      if (fresh(o, now) && o->value != ACTIVE) {
        failed = true;
        break;
      }
    }
  }
  if (failed) {
    recover(now, "전체 시작 완료 전 먼저 켠 lifecycle 상태가 내려감");
  }
}

namespace
{
void append_pending(std::string & out, const std::optional<Request> & r)
{
  if (!r) {
    out += "null";
    return;
  }
  out += "{\"command\": ";
  pyjson::append_int(out, r->command);
  out += ", \"kind\": ";
  pyjson::append_string(out, r->kind);
  out += ", \"target\": ";
  pyjson::append_string(out, r->target);
  out += ", \"timed_out\": ";
  pyjson::append_bool(out, r->timed_out_at.has_value());
  out += ", \"token\": ";
  pyjson::append_int(out, r->token);
  out += "}";
}

template<typename F>
void append_group_map(std::string & out, const WatchdogStatus & st, F && value)
{
  out += "{";
  bool first = true;
  for (const auto & [key, core] : st.groups) {
    if (!first) {
      out += ", ";
    }
    first = false;
    pyjson::append_string(out, key);
    out += ": ";
    value(*core);
  }
  out += "}";
}
}  // namespace

void write_watchdog_status(std::string & out, const WatchdogStatus & st)
{
  bool any_exhausted = false;
  for (const auto & g : st.groups) {
    any_exhausted = any_exhausted || g.second->phase == "exhausted";
  }
  const char * phase = st.ready ? "ready" : st.finished ? "monitoring" : any_exhausted ? "exhausted" : "starting";
  out.clear();
  out += "{\"groups\": ";
  append_group_map(out, st, [&out](const LifecycleRecovery & c) {
      out += "{\"last_error\": ";
      pyjson::append_string(out, c.last_error);
      out += ", \"pending\": ";
      append_pending(out, c.pending);
      out += ", \"phase\": ";
      pyjson::append_string(out, c.phase);
      out += ", \"ready\": ";
      pyjson::append_bool(out, c.phase == "ready");
      out += ", \"reason\": ";
      pyjson::append_string(out, c.reason);
      out += ", \"recovery_attempts\": ";
      pyjson::append_int(out, c.recoveries);
      if (c.opt.recover_only) {
        out += ", \"restarts\": ";
        pyjson::append_int(out, c.restarts);
      }
      out += "}";
    });
  out += ", \"inputs\": {\"map\": ";
  pyjson::append_bool(out, st.map);
  out += ", \"map_odom_tf\": ";
  pyjson::append_bool(out, st.map_odom_tf);
  out += ", \"odom_tf\": ";
  pyjson::append_bool(out, st.odom_tf);
  out += ", \"scan\": ";
  pyjson::append_bool(out, st.scan);
  out += "}, \"last_error\": ";
  append_group_map(out, st, [&out](const LifecycleRecovery & c) {pyjson::append_string(out, c.last_error);});
  out += ", \"localization_ready\": ";
  pyjson::append_bool(out, st.localization_ready);
  out += ", \"monitor_only_after_startup\": ";
  pyjson::append_bool(out, st.finished);
  out += ", \"navigation_ready\": ";
  pyjson::append_bool(out, st.navigation_ready);
  out += ", \"pending\": ";
  append_group_map(out, st, [&out](const LifecycleRecovery & c) {append_pending(out, c.pending);});
  out += ", \"phase\": ";
  pyjson::append_string(out, phase);
  out += ", \"ready\": ";
  pyjson::append_bool(out, st.ready);
  out += ", \"recovery_attempts\": ";
  append_group_map(out, st, [&out](const LifecycleRecovery & c) {pyjson::append_int(out, c.recoveries);});
  out += ", \"startup_completed\": ";
  pyjson::append_bool(out, st.finished);
  out += "}";
}

namespace
{
// NUL 구분 목록을 한 칸씩
template<typename F>
void each_field(const std::string & buf, F && f)
{
  std::size_t b = 0;
  while (b < buf.size()) {
    std::size_t e = buf.find('\0', b);
    if (e == std::string::npos) {
      e = buf.size();
    }
    if (e > b && f(buf.substr(b, e - b))) {
      return;
    }
    b = e + 1;
  }
}

std::string trim(const std::string & v)
{
  const auto b = v.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {
    return "";
  }
  const auto e = v.find_last_not_of(" \t\r\n");
  return v.substr(b, e - b + 1);
}
}  // namespace

bool cmdline_has_node(const std::string & cmdline, const std::string & node)
{
  if (node.empty()) {
    return false;
  }
  const std::string want = "__node:=" + node;
  bool found = false;
  each_field(cmdline, [&](const std::string & a) {
      found = a == want;
      return found;
    });
  return found;
}

std::string environ_domain(const std::string & environ)
{
  std::string value;
  each_field(environ, [&](const std::string & a) {
      if (a.rfind("ROS_DOMAIN_ID=", 0) == 0) {
        value = a.substr(14);
        return true;
      }
      return false;
    });
  return normalize_domain(value.c_str());
}

std::string normalize_domain(const char * value)
{
  const std::string v = value ? trim(value) : std::string();
  return v.empty() ? std::string("0") : v;
}

}  // namespace lifecycle
}  // namespace navigation
