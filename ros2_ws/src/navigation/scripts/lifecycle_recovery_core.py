#!/usr/bin/env python3
"""시작 lifecycle 순서·복구 계산임. ROS/스레드/벽시계 의존 없음.
받음: 단조 시각, 실제 준비 조건, 서비스·노드 상태 관측, 요청 결과.
보냄: 한 번에 하나의 STARTUP/RESET/노드 전환. 속도·hold·FAULT는 다루지 않음.
"""
from dataclasses import dataclass, field
from typing import Dict, Optional

UNCONFIGURED, INACTIVE, ACTIVE = 1, 2, 3
STARTUP, RESET, CLEANUP, DEACTIVATE = 0, 3, 2, 4


@dataclass(frozen=True)
class Observation:
    value: object
    requested: float
    received: float


@dataclass
class Snapshot:
    inputs_ready: bool = False
    services_ready: bool = False
    input_reason: str = "입력 기다림"
    active: Optional[Observation] = None
    states: Dict[str, Observation] = field(default_factory=dict)


@dataclass
class Request:
    token: int
    kind: str                 # manager / node
    target: str
    command: int
    sent: float
    deadline: float
    purpose: str
    timed_out_at: Optional[float] = None


class LifecycleRecovery:
    """초기 STARTUP + 실패 시 RESET→정합→STARTUP 최대 max_recoveries회임.

    ROS service timeout은 원격 취소가 아님. timeout 뒤에도 원래 응답 또는
    그 뒤 요청한 같은 대상 상태 조회 응답(직렬 callback 장벽)을 기다림.
    어느 쪽도 안 오면 복구 상태만 exhausted로 보고하며 다음 전환을 겹치지 않음.
    """
    def __init__(self, name, nodes, *, max_recoveries=3, command_timeout=90.0,
                 node_timeout=20.0, drain_timeout=120.0, verify_timeout=45.0,
                 stable_s=16.0, backoff_s=1.0, freshness_s=3.0,
                 cleanup_timeout=90.0, native_reconnect_s=11.0):
        if not nodes or max_recoveries < 0:
            raise ValueError("관리 노드와 0 이상 복구 횟수 필요")
        self.name, self.nodes = name, list(nodes)
        self.max_recoveries = max_recoveries
        self.command_timeout, self.node_timeout = command_timeout, node_timeout
        self.drain_timeout, self.verify_timeout = drain_timeout, verify_timeout
        self.stable_s, self.backoff_s, self.freshness_s = stable_s, backoff_s, freshness_s
        self.cleanup_timeout = cleanup_timeout
        self.native_reconnect_s = native_reconnect_s
        self._native_possible = False
        self._native_until = None
        self._native_entered = 0.0
        self._native_observed_active = None
        self.phase = "waiting_inputs"
        self.pending = None
        self.recoveries = 0
        self.last_error = ""
        self.reason = "시작 입력 기다림"
        self._token = 0
        self._next_at = 0.0
        self._verify_started = 0.0
        self._stable_since = None
        self._cleanup_started = 0.0
        self._cleanup_index = 0
        self._after = 0.0
        self._changes = {}

    def _fresh(self, obs, now, after=-1.0):
        return obs is not None and obs.requested >= after and 0 <= now - obs.received <= self.freshness_s

    def healthy(self, s, now):
        return (self._fresh(s.active, now) and s.active.value is True and
                all(self._fresh(s.states.get(n), now) and s.states[n].value == ACTIVE for n in self.nodes))

    def _request(self, now, kind, target, command, purpose):
        self._token += 1
        timeout = self.command_timeout if kind == "manager" else self.node_timeout
        self.pending = Request(self._token, kind, target, command, now, now + timeout, purpose)
        self.reason = purpose
        return self.pending

    def _recover(self, now, why):
        self.last_error = why
        self._stable_since = None
        if self._native_possible:
            # STARTUP 성공 뒤 bond 하락은 Nav2의 기존 재연결(10초 창)을 먼저 존중함.
            # 창 뒤에 보낸 is_active 응답이 같은 manager callback queue의 장벽임.
            self.phase, self.reason = "waiting_native", "Nav2 기존 bond 재연결 먼저 기다림"
            self._native_entered = now
            self._native_until = None
            self._native_observed_active = None
            return
        self._schedule_reset(now, why)

    def _schedule_reset(self, now, why):
        if self.recoveries >= self.max_recoveries:
            self.phase, self.reason = "exhausted", "lifecycle 자동 복구 3회 한도 소진" if self.max_recoveries == 3 else "lifecycle 복구 한도 소진"
        else:
            self.phase, self.reason = "waiting_reset", why
            self._next_at = now + self.backoff_s

    def _cleanup(self, now):
        self.phase = "cleanup"
        self._cleanup_started = self._after = now
        self._cleanup_index = 0
        self._changes = {}
        self.reason = "RESET 뒤 노드별 실제 상태 정합"

    def complete(self, token, success, now, detail=""):
        """늦은/이전 세대 응답은 상태를 바꾸지 않음."""
        r = self.pending
        if r is None or r.token != token:
            return False
        self.pending = None
        if self.phase == "exhausted":
            return False
        if r.purpose == "STARTUP":
            if success:
                self._native_possible = True
                self.phase = "verifying"
                self._verify_started = now
                self._stable_since = None
                self.reason = "manager·노드 ACTIVE 및 bond 안정 확인"
            else:
                self._recover(now, detail or "STARTUP 실패")
        elif r.purpose == "RESET":
            if not success:
                self.last_error = detail or "RESET 일부 실패: 노드별 정합으로 이어 감"
            self._cleanup(now)
        else:
            # false여도 노드가 바뀌었을 수 있음. 새 GetState로 판단함.
            self._after = now
            if not success:
                self.last_error = detail or (r.target + " 전환 응답 실패")
        return True

    def tick(self, now, s):
        if self.phase in ("ready", "exhausted"):
            return None
        if self.pending:
            r = self.pending
            if now < r.deadline:
                return None
            if r.timed_out_at is None:
                r.timed_out_at = now
                self.reason = r.purpose + " 응답 기한 지남: 원격 완료 확인 중"
            barrier = s.active if r.kind == "manager" else s.states.get(r.target)
            if self._fresh(barrier, now, r.timed_out_at):
                # 상태 조회는 timeout 뒤에 보낸 것만 인정함. 오래된 관측은 장벽 아님.
                self.complete(r.token, False, now, r.purpose + " 응답 timeout 뒤 상태 조회 완료")
            elif now - r.timed_out_at >= self.drain_timeout:
                self.phase = "exhausted"
                self.reason = "원격 lifecycle 응답 미완료: 전환 중복 없이 상태 보고"
                self.last_error = r.purpose + " 및 뒤 상태 조회 모두 무응답"
                # pending을 남겨 같은 대상에 새 전환을 보내지 않음.
            return None
        if self.phase in ("ready", "exhausted"):
            return None
        if self.phase == "waiting_inputs":
            if not s.inputs_ready or not s.services_ready:
                self.reason = s.input_reason if not s.inputs_ready else "lifecycle 서비스 준비 기다림"
                return None
            if not self._fresh(s.active, now) or not all(self._fresh(s.states.get(n), now) for n in self.nodes):
                self.reason = "lifecycle 초기 상태 확인 중"
                return None
            if self.healthy(s, now):
                self._native_possible = True
                self.phase, self._verify_started = "verifying", now
                self._stable_since = None
            elif any(s.states[n].value != UNCONFIGURED for n in self.nodes):
                self._recover(now, "시작 전 일부 노드가 이미 configure/active 상태")
            else:
                self.phase = "starting"
                return self._request(now, "manager", self.name, STARTUP, "STARTUP")
        elif self.phase == "verifying":
            if s.inputs_ready and self.healthy(s, now):
                if self._stable_since is None:
                    self._stable_since = now
                if now - self._stable_since >= self.stable_s:
                    self.phase, self.reason = "ready", "lifecycle 시작 준비 완료"
            elif self.healthy(s, now):
                self._stable_since = None
                self.reason = s.input_reason  # 센서 준비를 기다림. 정상 lifecycle을 RESET하지 않음.
                self._verify_started = now
            else:
                # bond가 잠깐 붙었다 다시 끊긴 경우도 검증 창 안에서 복구함.
                had_active = self._stable_since is not None
                self._stable_since = None
                if had_active or now - self._verify_started >= self.verify_timeout:
                    self._recover(now, "STARTUP 뒤 ACTIVE/bond/입력 준비 확인 실패")
        elif self.phase == "waiting_native":
            if self.healthy(s, now):
                self.phase, self._verify_started = "verifying", now
                self._stable_since = None
                self.reason = "Nav2 기존 재연결 성공: ACTIVE 안정 다시 확인"
            elif now >= self._native_entered + self.native_reconnect_s + self.drain_timeout:
                self.phase = "exhausted"
                self.reason = "Nav2 재연결 뒤 manager 상태 응답 없음: 전환 중복 없이 상태 보고"
                self.last_error = self.reason
            elif self._fresh(s.active, now, self._native_entered):
                # hard RESET이 오래 걸리면 native 재연결 타이머도 늦게 시작함.
                # 첫 최신 응답, 또는 ACTIVE→inactive 응답을 받은 시점부터 창을 셈.
                if (self._native_until is None or
                        (self._native_observed_active is True and s.active.value is False)):
                    self._native_until = now + self.native_reconnect_s
                self._native_observed_active = s.active.value
                if (now >= self._native_until and self._fresh(s.active, now, self._native_until) and
                        all(self._fresh(s.states.get(n), now, self._native_until) for n in self.nodes)):
                    # 같은 manager queue에서 창 뒤에 요청한 상태 조회가 끝났음.
                    self._native_possible = False
                    self._schedule_reset(now, "Nav2 기존 재연결 창 및 최신 상태 조회 뒤에도 준비 미완료")
        elif self.phase == "waiting_reset":
            if self.healthy(s, now):
                self._native_possible = True
                self.phase, self._verify_started = "verifying", now
                self._stable_since = None
            elif now >= self._next_at and s.services_ready:
                self.recoveries += 1
                self.phase = "resetting"
                return self._request(now, "manager", self.name, RESET, "RESET")
            self.reason = "해당 lifecycle RESET 준비 기다림"
        elif self.phase == "cleanup":
            if now - self._cleanup_started >= self.cleanup_timeout:
                self._recover(now, "노드별 cleanup 기한 초과")
                return None
            if self._cleanup_index >= len(self.nodes):
                self.phase, self._next_at = "restart", now + self.backoff_s
                return None
            node = self.nodes[-1 - self._cleanup_index]
            obs = s.states.get(node)
            if not self._fresh(obs, now, self._after):
                self.reason = node + " RESET 뒤 상태 기다림"
                return None
            if obs.value == UNCONFIGURED:
                self._cleanup_index += 1
                return None
            if obs.value in (ACTIVE, INACTIVE):
                command = DEACTIVATE if obs.value == ACTIVE else CLEANUP
                key = (node, command)
                count = self._changes.get(key, 0)
                if count >= 2:
                    self._recover(now, node + " 노드 전환 재시도 소진")
                    return None
                self._changes[key] = count + 1
                return self._request(now, "node", node, command, "노드별 cleanup")
            self.reason = node + " 전환 중/UNKNOWN 상태 기다림"
        elif self.phase == "restart":
            if now < self._next_at or not s.inputs_ready or not s.services_ready:
                self.reason = s.input_reason if not s.inputs_ready else "STARTUP 재시도 준비 기다림"
                return None
            if not all(self._fresh(s.states.get(n), now) for n in self.nodes):
                return None
            if any(s.states[n].value != UNCONFIGURED for n in self.nodes):
                self._recover(now, "cleanup 뒤 노드 상태가 다시 바뀜")
                return None
            self.phase = "starting"
            return self._request(now, "manager", self.name, STARTUP, "STARTUP")
        return None

    def recheck_before_handoff(self, now, s):
        """다른 그룹 시작 중 먼저 켜진 그룹이 다시 내려간 경우만 복구함.
        둘 다 준비된 뒤에는 adapter가 호출하지 않아 운전 중 개입하지 않음.
        센서 미수신 자체는 lifecycle 실패로 바꾸지 않음.
        """
        if self.phase != "ready":
            return
        failed = self._fresh(s.active, now) and s.active.value is False
        failed = failed or any(self._fresh(s.states.get(n), now) and s.states[n].value != ACTIVE for n in self.nodes)
        if failed:
            self._recover(now, "전체 시작 완료 전 먼저 켠 lifecycle 상태가 내려감")

    def status(self):
        r = self.pending
        return {"phase": self.phase, "ready": self.phase == "ready", "recovery_attempts": self.recoveries,
                "reason": self.reason, "last_error": self.last_error,
                "pending": None if r is None else {"token": r.token, "kind": r.kind, "target": r.target,
                                                    "command": r.command, "timed_out": r.timed_out_at is not None}}
