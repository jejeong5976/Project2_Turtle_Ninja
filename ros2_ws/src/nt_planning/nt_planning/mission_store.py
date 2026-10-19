"""
mission_store.py — 읽은 QR 지시를 보관하는 곳 (상한 · 중복 제거)
=====================================================================
2026-10-16 검토: 예전에는 valid 한 QR 메시지를 받을 때마다 list 에 append 만 했다 → 지우기 · 중복 제거 · 상한이 없어서
  같은 QR 을 A → B → A 로 번갈아 보거나 perception 이 재시작하면 같은 지시가 계속 쌓임 (100,000 번 → 100,000 개, Codex 확인)
  "누수" 라기보다 **필요 없는 데이터를 계속 들고 있는 문제** + 같은 지시를 두 번 수행할 위험.

규칙 (QR 포맷이 아직 미정이라 최소한으로)
  - 같은 지시 = 같은 raw_text. 이미 있으면 추가하지 않음 (보관은 상태 대기 · 진행 · 완료로 구분)
  - 서로 다른 지시를 "색 · 선반이 같다" 는 이유만으로 합치지 않음 (실제로 같은 색 퍽 두 개를 옮기라는 지시일 수 있음)
  - 보관 상한 max_items, 글자 상한 max_text. 넘으면 버리고 dropped 로 셈 (진단용)
  TODO QR 포맷이 정해지면 raw_text 대신 지시 ID(작업 번호)로 같은지 판단

ROS 를 import 하지 않음 → test/test_mission_store.py
"""


class MissionStore:
    WAITING, ACTIVE, DONE = "waiting", "active", "done"

    def __init__(self, max_items=32, max_text=256):
        self.max_items = max_items
        self.max_text = max_text
        self.items = {}      # raw_text → {"msg": 메시지, "state": WAITING|ACTIVE|DONE}
        self.order = []      # 받은 순서 (raw_text)
        self.dropped = 0     # 상한 · 너무 긴 글자로 버린 수

    def add(self, msg):
        """돌려줌: True = 새 지시로 보관, False = 이미 있음 · 버림."""
        key = getattr(msg, "raw_text", "")
        if not getattr(msg, "valid", False):
            return False
        if len(key) > self.max_text:
            self.dropped += 1
            return False
        if key in self.items:
            return False
        if len(self.items) >= self.max_items:
            self.dropped += 1
            return False
        self.items[key] = {"msg": msg, "state": self.WAITING}
        self.order.append(key)
        return True

    def next_waiting(self):
        for k in self.order:
            if self.items[k]["state"] == self.WAITING:
                return self.items[k]["msg"]
        return None

    def mark(self, msg, state):
        k = getattr(msg, "raw_text", "")
        if k in self.items:
            self.items[k]["state"] = state

    def count(self, state=None):
        if state is None:
            return len(self.items)
        return sum(1 for v in self.items.values() if v["state"] == state)

    def __len__(self):
        return len(self.items)
