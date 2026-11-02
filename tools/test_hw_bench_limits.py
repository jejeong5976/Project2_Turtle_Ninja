"""
test_hw_bench_limits.py — hw_bench.py limits 시험 (보드 없이): dynamixel_sdk 를 가짜로 바꿔 버스에 나가는 호출이 ping · 읽기뿐인지
  python3 -m pytest tools/test_hw_bench_limits.py -q
"""
import importlib.util
import json
import types
from pathlib import Path

import pytest
import yaml

spec = importlib.util.spec_from_file_location("hw_bench", Path(__file__).with_name("hw_bench.py"))
hb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hb)
DRIVE_ONLY = Path(__file__).parents[1] / "config/hardware_map_drive_only.yaml"
ZERO = {11: 3066, 12: 1552, 13: 2577, 14: 3589}   # hardware_map_drive_only.yaml 의 실측 영점 (새 전진 방향)


def bench_map(tmp_path, zeros=ZERO):
    import yaml
    hw = yaml.safe_load(DRIVE_ONLY.read_text())
    for m in hw["motors"].values():
        if m["id"] in zeros:
            m["zero_tick"] = zeros[m["id"]]
    p = tmp_path / "map.yaml"
    p.write_text(yaml.safe_dump(hw))
    return str(p)


def fake_sdk(regs, calls):
    """regs[id][addr] = 값 (부호 없는 정수). calls 에 (함수 이름, id, addr) 를 남김"""
    class Port:
        def __init__(self, name):
            pass

        def openPort(self):
            return True

        def setBaudRate(self, b):
            return True

        def closePort(self):
            calls.append(("closePort", None, None))

    class Pkt:
        def __init__(self, proto):
            pass

        def ping(self, port, i):
            calls.append(("ping", i, None))
            return (regs[i][0] if i in regs else 0), (0 if i in regs else -3001), 0

        def _read(self, name, port, i, addr):
            calls.append((name, i, addr))
            return regs[i].get(addr, 0), 0, 0

        def read1ByteTxRx(self, port, i, addr):
            return self._read("read1", port, i, addr)

        def read2ByteTxRx(self, port, i, addr):
            return self._read("read2", port, i, addr)

        def read4ByteTxRx(self, port, i, addr):
            return self._read("read4", port, i, addr)

        def __getattr__(self, name):   # write1ByteTxRx 등 다른 것은 부르는 순간 기록 + 예외
            def f(*a, **k):
                calls.append((name, None, None))
                raise AssertionError(f"limits 가 {name} 을 불렀음")
            return f

        def getTxRxResult(self, r):
            return str(r)

    return types.SimpleNamespace(COMM_SUCCESS=0, PortHandler=Port, PacketHandler=Pkt)


def u32(v):
    return v & 0xFFFFFFFF


def motor(present, homing=0, lo=0, hi=4095, drive_mode=0, mode=3, model=1020):
    return {0: model, 6: 48, 10: drive_mode, 11: mode, 20: u32(homing), 48: hi, 52: lo, 64: 0, 70: 0, 132: u32(present)}


def run(monkeypatch, tmp_path, regs, ids=("11", "12", "13", "14"), map_path=None):
    calls = []
    monkeypatch.setattr(hb, "sdk", lambda: fake_sdk(regs, calls))
    out = tmp_path / "limits.json"
    args = hb.argparse.Namespace(port="FAKE", baud=1_000_000, ids=[int(i) for i in ids],
                                 map=map_path or bench_map(tmp_path), json=str(out))
    hb.cmd_limits(args)
    return calls, json.loads(out.read_text())


def test_only_ping_and_reads_go_to_bus(monkeypatch, tmp_path, capsys):
    regs = {i: motor(z) for i, z in ZERO.items()}
    calls, rows = run(monkeypatch, tmp_path, regs)
    kinds = {c[0] for c in calls}
    assert kinds <= {"ping", "read1", "read2", "read4", "closePort"}       # 쓰기 호출 없음
    by = {r["id"]: r for r in rows}
    assert by[14]["interpret"]["cw_deg"] == pytest.approx((4095 - 3589) * 360 / 4096)    # 14: 시계 쪽 약 44° (여유 20 눈금 빼면 −42.7°)
    assert by[14]["interpret"]["ccw_deg"] == pytest.approx(3589 * 360 / 4096)
    assert by[11]["interpret"]["cw_deg"] == pytest.approx((4095 - 3066) * 360 / 4096)
    assert all(r["interpret"]["limits_default"] and r["homing_offset"] == 0 for r in rows)
    out = capsys.readouterr().out
    assert "읽기 전용" in out and "Homing Offset ≠ 0" not in out


def test_settings_explained_when_not_default(monkeypatch, tmp_path, capsys):
    """Homing Offset · 좁혀진 한계 · 방향 반대가 있으면 그대로 보여 주고 해석에 반영"""
    regs = {14: motor(3589, homing=1024, lo=1000, hi=3800, drive_mode=1)}
    calls, rows = run(monkeypatch, tmp_path, regs, ids=("14",))
    r = rows[0]
    assert r["homing_offset"] == 1024 and r["interpret"]["actual_estimate"] == 3589 - 1024
    assert r["interpret"]["reverse"] is True and r["interpret"]["limits_default"] is False
    assert r["interpret"]["cw_deg"] == pytest.approx((3800 - 3589) * 360 / 4096)
    out = capsys.readouterr().out
    assert "Homing Offset ≠ 0" in out and "Min/Max 가 기본" in out and "Drive Mode 방향 반대" in out


def test_negative_homing_and_zero_outside_limits(monkeypatch, tmp_path, capsys):
    regs = {14: motor(3589, homing=-500, lo=0, hi=2000)}
    _, rows = run(monkeypatch, tmp_path, regs, ids=("14",))
    assert rows[0]["homing_offset"] == -500 and rows[0]["interpret"]["zero_inside"] is False
    assert "직진 눈금이 Min~Max 밖" in capsys.readouterr().out


def test_missing_motor_reported_not_crash(monkeypatch, tmp_path):
    calls, rows = run(monkeypatch, tmp_path, {11: motor(3066)}, ids=("11", "12"))
    assert rows[1]["id"] == 12 and "응답 없음" in rows[1]["problems"][0]


def test_read_only_bus_blocks_write():
    class B:
        def close(self):
            pass
    with pytest.raises(RuntimeError):
        hb.ReadOnlyBus(B()).write(11, "homing_offset", 0)


def test_drive_only_map_reads_ticks_and_identifies_drive_board(monkeypatch, tmp_path, capsys):
    """설치 지도 그대로: 눈금을 읽고 지도 영점과 나란히 보여 줌. ID 1 · 2 · 41 · 42 가 W210 · 속도 모드로 대답 = 주행 보드"""
    regs = {1: motor(0, mode=1, model=1030), 2: motor(0, mode=1, model=1030), 41: motor(0, mode=1, model=1030),
            42: motor(0, mode=1, model=1030), 11: motor(2050), 12: motor(2566), 13: motor(3590), 14: motor(2561)}
    calls, rows = run(monkeypatch, tmp_path, regs, ids=("1", "2", "41", "42", "11", "12", "13", "14"), map_path=str(DRIVE_ONLY))
    assert {c[0] for c in calls} <= {"ping", "read1", "read2", "read4", "closePort"}
    by = {r["id"]: r for r in rows}
    assert [by[i]["present_position"] for i in (11, 12, 13, 14)] == [2050, 2566, 3590, 2561]
    motors = yaml.safe_load(DRIVE_ONLY.read_text())["motors"]          # 지도 영점 그대로 보여 주는지 (10-30 Homing 뒤 값이 바뀜)
    assert [by[i]["zero_tick"] for i in (11, 12, 13, 14)] == [motors[f"steer_{n}_joint"]["zero_tick"] for n in ("fl", "fr", "rl", "rr")]
    assert all(by[i]["operating_mode"] == 1 for i in (1, 2, 41, 42))
