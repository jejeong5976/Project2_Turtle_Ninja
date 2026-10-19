"""ArmSession 시험 — ROS 없이 (가짜 future · goal handle). 2026-10-16 R2 회귀 시험 포함.

colcon test --packages-select nt_planning   또는   python3 -m pytest ros2_ws/src/nt_planning/test
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nt_planning.action_session import ArmSession  # noqa: E402


class Log:
    def __init__(self):
        self.lines = []

    def info(self, t):
        self.lines.append(("I", t))

    def warning(self, t):
        self.lines.append(("W", t))

    def error(self, t):
        self.lines.append(("E", t))


class Fut:
    def __init__(self):
        self._done, self._v, self._cbs, self.exc = False, None, [], None

    def done(self):
        return self._done

    def result(self):
        if self.exc:
            raise self.exc
        return self._v

    def set(self, v):
        self._done, self._v = True, v
        for cb in self._cbs:
            cb(self)

    def add_done_callback(self, cb):
        self._cbs.append(cb)
        if self._done:
            cb(self)


class CancelResp:
    def __init__(self, code):
        self.return_code = code   # 0 받아들임 · 1 거절 · 2 모르는 goal · 3 이미 끝남 (action_msgs/CancelGoal)


class Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancels = 0
        self.cancel_futs = []     # 보낸 취소 요청들 (응답을 기다리는 중인 것 = done 아님)
        self.res = Fut()

    def get_result_async(self):
        return self.res

    def cancel_goal_async(self):
        self.cancels += 1
        f = Fut()
        self.cancel_futs.append(f)
        return f

    def pending_cancel_requests(self):   # rclpy ActionClient._pending_cancel_requests 와 같은 뜻
        return sum(1 for f in self.cancel_futs if not f.done())


class Res:
    def __init__(self, ok, msg=""):
        class R:
            pass
        self.result = R()
        self.result.success, self.result.message = ok, msg


def new():
    sent = []

    def send():
        f = Fut()
        sent.append(f)
        return f
    return ArmSession(Log(), retry_s=15, accept_timeout_s=5, result_timeout_s=60), sent, send


def test_success_path():
    s, sent, send = new()
    assert s.poll(0.0, 0, send, True)[0] == ArmSession.PENDING
    h = Handle()
    sent[0].set(h)
    assert s.poll(0.1, 0, send, True)[0] == ArmSession.PENDING
    h.res.set(Res(True, "완료"))
    assert s.poll(0.2, 0, send, True) == (ArmSession.OK, "완료")
    assert not s.active and s.pending_cancels == 0


# ★ R2: 수락 답(future)은 도착했지만 다음 poll 이 handle 을 꺼내기 전에 FAULT → 그 goal 을 취소해야 함
def test_cancel_after_accept_response_before_poll():
    s, sent, send = new()
    s.poll(0.0, 1, send, True)
    h = Handle()
    sent[0].set(h)          # 수락 답 도착 (아직 poll 안 함 → handle 미저장)
    s.cancel("FAULT", 0.05)
    assert h.cancels == 1   # 예전 코드: 0 (두 조건을 다 건너뛰고 참조만 지움)
    assert s.pending_cancels == 1 and not s.active


def test_cancel_while_waiting_accept_then_accepted_later():
    s, sent, send = new()
    s.poll(0.0, 1, send, True)
    s.cancel("FAULT", 0.1)
    h = Handle()
    sent[0].set(h)          # 늦게 받아들여짐 → 콜백이 바로 취소
    assert h.cancels == 1
    h.res.set(Res(False, "canceled"))
    s.service_orphans(0.3)
    assert s.pending_cancels == 0


def test_cancel_while_waiting_accept_then_rejected():
    s, sent, send = new()
    s.poll(0.0, 1, send, True)
    s.cancel("FAULT", 0.1)
    sent[0].set(Handle(accepted=False))
    s.service_orphans(0.2)
    assert s.pending_cancels == 0


def test_cancel_running_keeps_tracking_until_result():
    s, sent, send = new()
    s.poll(0.0, 1, send, True)
    h = Handle()
    sent[0].set(h)
    s.poll(0.1, 1, send, True)       # 실행 중
    s.cancel("FAULT", 1.0)
    assert h.cancels == 1
    s.service_orphans(2.1)
    assert h.cancels == 1            # (R8) 앞 취소 요청의 응답이 없으면 새로 안 보냄
    assert s.pending_cancels == 1    # 그래도 잊지 않고 추적
    h.res.set(Res(False, "canceled"))
    s.service_orphans(2.2)
    assert s.pending_cancels == 0 and s.unresolved_cancels == 0


# ★ 재검증 R8: 응답 없는 서버에 수천 tick 동안 취소 요청이 쌓이지 않음 (Codex 재현: 20 번에 1 → 21)
def test_unanswered_cancel_requests_do_not_pile_up():
    s, sent, send = new()
    s.poll(0.0, 1, send, True)
    h = Handle()
    sent[0].set(h)
    s.poll(0.1, 1, send, True)
    s.cancel("FAULT", 0.0)
    for k in range(1, 5001):         # 50 Hz 로 100 s
        s.service_orphans(k * 0.02)
    assert h.pending_cancel_requests() == 1 and h.cancels == 1
    assert s.pending_cancels == 0 and s.unresolved_cancels == 1   # 10 s 뒤 포기 + 기록
    assert any(level == "E" and "확인하지 못함" in text for level, text in s.log.lines)


def test_rejected_cancel_is_retried_with_limit():
    s, sent, send = new()
    s.poll(0.0, 1, send, True)
    h = Handle()
    sent[0].set(h)
    s.poll(0.1, 1, send, True)
    s.cancel("FAULT", 0.0)
    t = 0.0
    for _ in range(400):
        t += 0.05
        for f in h.cancel_futs:
            if not f.done():
                f.set(CancelResp(1))   # 서버가 매번 거절
        s.service_orphans(t)
    assert h.cancels == 5            # max_cancel_tries
    assert h.pending_cancel_requests() == 0
    assert s.unresolved_cancels == 1


def test_accepted_cancel_waits_for_result_without_resend():
    s, sent, send = new()
    s.poll(0.0, 1, send, True)
    h = Handle()
    sent[0].set(h)
    s.poll(0.1, 1, send, True)
    s.cancel("FAULT", 0.0)
    h.cancel_futs[0].set(CancelResp(0))   # 받아들임 (취소 중)
    for k in range(1, 100):
        s.service_orphans(k * 0.05)
    assert h.cancels == 1
    h.res.set(Res(False, "canceled"))     # 뒤늦은 결과도 그 goal 기준으로 정리
    s.service_orphans(5.0)
    assert s.pending_cancels == 0 and s.unresolved_cancels == 0


def test_never_accepted_goal_is_given_up_too():
    s, sent, send = new()
    s.poll(0.0, 1, send, True)
    s.cancel("FAULT", 0.0)           # 수락 답이 영영 안 옴
    for k in range(1, 300):
        s.service_orphans(k * 0.05)
    assert s.pending_cancels == 0 and s.unresolved_cancels == 1


def test_orphan_list_is_bounded():
    s, sent, send = new()
    for i in range(20):
        s.poll(i * 0.01, 1, send, True)
        h = Handle()
        sent[-1].set(h)
        s.poll(i * 0.01 + 0.001, 1, send, True)
        s.cancel("FAULT", i * 0.01)
    assert s.pending_cancels <= 8
    assert s.unresolved_cancels == 12


def test_cancel_only_that_goal_not_the_next_one():
    s, sent, send = new()
    s.poll(0.0, 1, send, True)
    old = Handle()
    sent[0].set(old)
    s.cancel("FAULT", 0.1)
    # 새 작업
    s.poll(0.2, 0, send, True)
    new_h = Handle()
    sent[1].set(new_h)
    s.poll(0.3, 0, send, True)
    old.cancel_futs[0].set(CancelResp(1))   # 옛 goal 취소가 거절됨 → 다시 보낼 때도 옛 goal 에만
    s.service_orphans(5.0)
    assert old.cancels == 2 and new_h.cancels == 0
    new_h.res.set(Res(True))
    assert s.poll(0.4, 0, send, True)[0] == ArmSession.OK


def test_accept_timeout_fails_and_cancels_late_accept():
    s, sent, send = new()
    s.poll(0.0, 1, send, True)
    kind, why = s.poll(5.1, 1, send, True)
    assert kind == ArmSession.FAIL and "받았다는 답" in why
    h = Handle()
    sent[0].set(h)
    assert h.cancels == 1


def test_result_timeout_fails_and_cancels():
    s, sent, send = new()
    s.poll(0.0, 1, send, True)
    h = Handle()
    sent[0].set(h)
    s.poll(1.0, 1, send, True)
    kind, _ = s.poll(61.5, 1, send, True)
    assert kind == ArmSession.FAIL and h.cancels == 1


def test_server_missing_fails_after_retry_window():
    s, sent, send = new()
    for k in range(150):
        assert s.poll(k * 0.1, 0, send, False)[0] == ArmSession.PENDING
    assert s.poll(15.1, 0, send, False)[0] == ArmSession.FAIL
    assert not sent


def test_rejected_retries_then_fails():
    s, sent, send = new()
    t = 0.0
    kind = ArmSession.PENDING
    while kind == ArmSession.PENDING and t < 30:
        kind, _ = s.poll(t, 0, send, True)
        if sent and not sent[-1].done():
            sent[-1].set(Handle(accepted=False))
        t += 0.1
    assert kind == ArmSession.FAIL and len(sent) >= 10


def test_accept_response_exception_is_not_accepted():
    s, sent, send = new()
    s.poll(0.0, 0, send, True)
    sent[0].exc = RuntimeError("server died")
    sent[0].set(None)
    assert s.poll(0.1, 0, send, True)[0] == ArmSession.PENDING   # 거절처럼 → 다시 보낼 때를 기다림
