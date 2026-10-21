"""
action_session.py — 팔 작업(ArmTask) 한 건의 "보내기 → 받아 줬나 → 끝났나 → 취소" 수명 관리
=====================================================================================
2026-10-16 검토 R2 반영. ROS 를 import 하지 않는 순수 Python → test/test_action_session.py 로 시험.

왜 따로 뺐나
  예전에는 planning_master 안에 arm_future · arm_handle · arm_result 세 변수가 흩어져 있었고, 취소할 때
  "handle 이 있으면 취소, future 가 아직이면 콜백" 두 경우만 봤다. 그런데 **수락 응답(future)은 이미 도착했지만
  다음 tick 이 아직 handle 을 꺼내 저장하기 전**에 FAULT 가 나면 두 조건을 모두 건너뛰고 참조만 지웠다
  → 이미 받아들여진 팔 작업이 취소되지 않고 계속 돎 (Codex 분기 재현).
  액션은 "보냄 → 수락 답 → 실행 → 결과" 가 전부 늦게 도착할 수 있어서, 상태 하나하나를 한 객체가 책임지게 했다.

이 객체가 지키는 약속
  1. cancel() 은 어느 단계에서 불려도 해당 goal 하나만 취소한다
       보내기 전           → 할 일 없음
       수락 답 대기 중     → 답이 오면(받아들여졌으면) 그 goal 취소
       수락 답 도착 · 미처리 → future 에서 handle 을 꺼내 바로 취소   ← R2 의 빠졌던 경우
       실행 중             → handle 로 취소
  2. 취소한 goal 은 "결과가 올 때까지" 목록(orphans)에 남겨 둔다 — 참조만 지우고 잊지 않음.
     pending_cancels 로 몇 개 남았는지 볼 수 있음
  2-1. (2026-10-16 재검증 R8) 취소 요청은 goal 하나당 **응답을 기다리는 것 하나만**.
     예전: 결과가 안 오면 1 s 마다 cancel_goal_async() 를 새로 불렀는데, rclpy 는 응답이 올 때까지 요청을 하나씩
     보관함(_pending_cancel_requests) → 서버가 죽어 있으면 끝없이 쌓임 (Codex 재현: 20 번 재시도에 1 → 21 개)
     지금: ① 앞 취소 요청의 응답이 아직이면 새로 안 보냄 ② 서버가 "거절" 이라고 답했을 때만 cancel_retry_s 뒤 다시,
          최대 max_cancel_tries 번 ③ cancel_give_up_s 안에 결과가 안 오면 추적을 그만두고 "취소 완료를 알 수 없음" 으로
          기록(unresolved_cancels) — planning 은 이걸 보면 FAULT 를 유지 ④ 추적 목록도 max_orphans 개까지
     ⚠ 여기서 추적을 그만둔다 ≠ 팔이 멈췄다. 실제 팔 정지는 control_master · JTC · 하드웨어 쪽 안전장치가 맡고,
        이 기록은 사람에게 "직접 확인하라" 는 신호
  3. 새 작업은 이전 작업의 늦은 답에 영향받지 않는다 (작업마다 future · handle 을 따로 가짐)

쓰는 법 (planning_master)
  s = ArmSession(log, retry_s=15, accept_timeout_s=5, result_timeout_s=60)
  매 tick:  r = s.poll(now, task, send_fn, server_ready)   # r = ("pending", detail) | ("ok", msg) | ("fail", why)
  FAULT 때: s.cancel("FAULT")
  매 tick (FAULT · DONE 이어도): s.service_orphans(now)
"""


class ArmSession:
    PENDING = "pending"
    OK = "ok"
    FAIL = "fail"

    def __init__(self, log, retry_s=15.0, accept_timeout_s=5.0, result_timeout_s=60.0, cancel_retry_s=1.0,
                 max_cancel_tries=5, cancel_give_up_s=10.0, max_orphans=8):
        self.log = log                      # .info / .warning(.warn) / .error 를 가진 것 (rclpy logger 등)
        self.retry_s = retry_s
        self.accept_timeout_s = accept_timeout_s
        self.result_timeout_s = result_timeout_s
        self.cancel_retry_s = cancel_retry_s
        self.max_cancel_tries = max_cancel_tries   # goal 하나에 취소 요청을 보내는 최대 횟수 (거절됐을 때만 다시)
        self.cancel_give_up_s = cancel_give_up_s   # 첫 취소부터 이만큼 결과가 없으면 추적 포기 + unresolved
        self.max_orphans = max_orphans
        self.orphans = []                   # 취소했지만 아직 결과를 못 받은 goal 들 (dict)
        self.unresolved_cancels = 0         # 취소 완료를 끝내 확인 못 한 goal 수 (0 이 아니면 FAULT 유지 · 사람 확인)
        self._clear()

    # ------------------------------------------------------------------ 상태
    def _clear(self):
        self.task = None
        self.future = None        # send_goal_async 의 future (수락 답)
        self.handle = None        # 받아들여진 goal handle
        self.result = None        # get_result_async 의 future
        self.first_try = None     # 이번 작업을 처음 시도한 시각 (거절 · 서버 없음 → 다시 보내기 기준)
        self.sent_at = 0.0        # 보낸 시각 / 받아 준 시각 (시간 제한 기준)
        self.retry_at = 0.0

    @property
    def active(self):
        """지금 진행 중인 작업이 있나 (보냈거나, 다시 보내려고 기다리는 중)."""
        return self.task is not None

    @property
    def pending_cancels(self):
        return len(self.orphans)

    # ------------------------------------------------------------------ 진행
    def poll(self, now, task, send_fn, server_ready):
        """작업 task 를 진행. send_fn() 은 send_goal_async 를 불러 future 를 돌려줌.

        돌려줌: (PENDING, 설명) 진행 중 / (OK, 메시지) 성공 / (FAIL, 이유) 실패 — FAIL 이면 이미 cancel 까지 함
        """
        if self.task is not None and self.task != task:
            # 다른 작업으로 바뀜 (호출하는 쪽 실수 방지): 이전 것은 취소하고 새로
            self.cancel("작업 종류가 바뀜", now)
        if self.task is None:
            self.task = task
            self.first_try = now

        # ① 아직 안 보냄 (또는 거절돼서 다시 보낼 차례)
        if self.future is None:
            if now < self.retry_at:
                return self.PENDING, "거절됨 → 다시 보낼 때를 기다림"
            if not server_ready:
                if now - self.first_try > self.retry_s:
                    return self._fail(f"/arm_task 서버가 {self.retry_s:.0f} s 동안 안 보임", now)
                return self.PENDING, "control_master(/arm_task) 기다리는 중"
            self.future = send_fn()
            self.sent_at = now
            return self.PENDING, "팔 작업 보냄"

        # ② 수락 답 기다림 / 도착
        if self.handle is None:
            if not self.future.done():
                if now - self.sent_at > self.accept_timeout_s:
                    return self._fail("팔 작업을 받았다는 답이 없음", now)
                return self.PENDING, "수락 답 기다리는 중"
            h = self._future_value(self.future)
            if h is None or not getattr(h, "accepted", False):
                self.future = None
                if now - self.first_try < self.retry_s:
                    self.retry_at = now + 1.0
                    return self.PENDING, "팔 작업 거절됨 (팔 준비 중?) → 1 s 뒤 다시"
                return self._fail(f"팔 작업이 {self.retry_s:.0f} s 동안 계속 거절됨", now)
            self.handle = h
            self.result = h.get_result_async()
            self.sent_at = now
            return self.PENDING, "팔 작업 실행 중"

        # ③ 결과 기다림
        if not self.result.done():
            if now - self.sent_at > self.result_timeout_s:
                return self._fail("팔 작업이 끝나지 않음 (시간 초과)", now)
            return self.PENDING, "팔 작업 실행 중"
        wrapped = self._future_value(self.result)
        res = getattr(wrapped, "result", None)
        ok = bool(getattr(res, "success", False))
        msg = getattr(res, "message", "")
        self._clear()
        return (self.OK, msg) if ok else (self.FAIL, f"팔 작업 실패/취소: {msg}")

    def _warn(self, text):
        (getattr(self.log, "warning", None) or self.log.warn)(text)

    def _fail(self, why, now):
        self.log.error(why)
        self.cancel(why, now)
        return self.FAIL, why

    # ------------------------------------------------------------------ 취소
    def cancel(self, why, now=None):
        """지금 작업을 (어느 단계든) 취소하고 결과가 올 때까지 orphans 로 추적. 작업이 없으면 아무것도 안 함."""
        if self.task is None and self.future is None:
            return
        entry = {"future": self.future, "handle": self.handle, "result": self.result, "why": why,
                 "cancel_future": None,      # 응답을 기다리는 취소 요청 (goal 하나에 하나만)
                 "cancel_sent_at": None, "tries": 0,
                 "since": now}               # 첫 취소 시각 (포기 기준). now 를 모르면 다음 service 때 정함
        self._clear()
        if entry["future"] is None:
            return   # 보내기 전 — 취소할 goal 이 없음
        if len(self.orphans) >= self.max_orphans:
            self._give_up(self.orphans.pop(0), "추적 목록이 가득 참")
        self.orphans.append(entry)
        if self._service(entry, now, force=True):
            self.orphans.remove(entry)

    def service_orphans(self, now):
        """취소한 goal 들 뒷정리. 매 tick 불러 줄 것 (FAULT · DONE 상태여도)."""
        self.orphans = [e for e in self.orphans if not self._service(e, now)]

    def _give_up(self, e, reason):
        self.unresolved_cancels += 1
        self.log.error(f"팔 작업 취소 완료를 확인하지 못함 ({e['why']}, {reason}) — 추적 중단. "
                       f"팔이 실제로 멈췄는지 직접 확인할 것 (취소 요청 {e['tries']} 번)")

    def _service(self, e, now, force=False):
        """돌려줌: True = 이 항목은 끝남 (목록에서 뺌)."""
        if e["since"] is None and now is not None:
            e["since"] = now
        if e["handle"] is None:
            f = e["future"]
            if not f.done():
                if not e.get("cb"):
                    e["cb"] = True   # 수락 답이 오는 즉시 처리 (다음 tick 을 기다리지 않음)
                    f.add_done_callback(lambda _f, e=e: self._service(e, e.get("last_now", now), force=True))
                if self._expired(e, now):
                    self._give_up(e, f"{self.cancel_give_up_s:.0f} s 동안 수락 답도 없음")
                    return True
                return False
            h = self._future_value(f)
            if h is None or not getattr(h, "accepted", False):
                return True   # 거절됐거나 실패 — 돌 goal 이 없음
            e["handle"] = h
            e["result"] = h.get_result_async()
            self._warn(f"취소한 팔 작업이 받아들여져 있었음 → 그 goal 취소 ({e['why']})")
        if e["result"] is not None and e["result"].done():
            return True   # 결과까지 받음 = 정말 끝남
        if now is not None:
            e["last_now"] = now
        if self._should_send_cancel(e, now, force):
            if e["tries"] > 0:
                self._warn(f"팔 작업 취소가 거절됨 → 다시 보냄 ({e['tries'] + 1}/{self.max_cancel_tries}, {e['why']})")
            e["cancel_future"] = e["handle"].cancel_goal_async()
            e["cancel_sent_at"] = now
            e["tries"] += 1
        if self._expired(e, now):
            self._give_up(e, f"{self.cancel_give_up_s:.0f} s 동안 결과 없음")
            return True
        return False

    def _should_send_cancel(self, e, now, force):
        """새 취소 요청을 보낼 차례인가. 응답 대기 중인 요청이 있으면 절대 새로 안 만듦 (R8)."""
        if e["tries"] >= self.max_cancel_tries:
            return False
        cf = e["cancel_future"]
        if cf is None:
            return True   # 아직 한 번도 안 보냄
        if not cf.done():
            return False  # 앞 요청의 응답을 기다리는 중
        if not self._cancel_rejected(cf):
            return False  # 서버가 받아들였거나(취소 중) 이미 끝난 goal → 결과만 기다림
        if force:
            return True
        last = e["cancel_sent_at"]
        return now is not None and (last is None or now - last >= self.cancel_retry_s)

    def _expired(self, e, now):
        return now is not None and e["since"] is not None and now - e["since"] >= self.cancel_give_up_s

    @staticmethod
    def _cancel_rejected(cf):
        """취소 응답이 "거절" 인가. action_msgs/CancelGoal: return_code 1 = ERROR_REJECTED.
        응답이 예외(서버가 죽는 중 등)여도 다시 보낼 대상으로 봄. 0(받아들임) · 2(모르는 goal) · 3(이미 끝남)은 다시 안 보냄."""
        resp = ArmSession._future_value(cf)
        if resp is None:
            return True
        return getattr(resp, "return_code", 0) == 1

    @staticmethod
    def _future_value(f):
        try:
            return f.result()
        except Exception:   # 응답 예외 (서버가 죽는 중 등) → "받아들여지지 않음" 으로 취급
            return None
