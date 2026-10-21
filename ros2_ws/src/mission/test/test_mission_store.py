"""MissionStore 시험 — QR 지시 보관 상한 · 중복 제거 (2026-10-16 검토)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from mission.mission_store import MissionStore  # noqa: E402


class Qr:
    def __init__(self, text, valid=True, color="red", shelf="A"):
        self.raw_text, self.valid, self.puck_color, self.target_shelf_id = text, valid, color, shelf


def test_same_qr_many_times_is_stored_once():
    m = MissionStore()
    for _ in range(100000):
        m.add(Qr("R-A-1"))
    assert len(m) == 1   # 예전: 100,000


def test_alternating_qr_does_not_grow():
    m = MissionStore()
    for _ in range(1000):
        m.add(Qr("A"))
        m.add(Qr("B"))
    assert len(m) == 2


def test_cap_and_long_text_are_dropped_and_counted():
    m = MissionStore(max_items=3, max_text=10)
    for k in range(5):
        m.add(Qr(f"Q{k}"))
    m.add(Qr("x" * 11))
    assert len(m) == 3 and m.dropped == 3


def test_invalid_is_ignored_and_states():
    m = MissionStore()
    assert not m.add(Qr("bad", valid=False))
    a, b = Qr("A"), Qr("B", color="red", shelf="A")   # 색 · 선반이 같아도 다른 지시면 따로
    m.add(a)
    m.add(b)
    assert len(m) == 2
    assert m.next_waiting() is a
    m.mark(a, MissionStore.DONE)
    assert m.next_waiting() is b
    assert m.count(MissionStore.DONE) == 1
