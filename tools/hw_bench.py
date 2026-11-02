#!/usr/bin/env python3
"""
hw_bench.py — 실제 다이나믹셀 책상 시험 (ROS 없이, dynamixel_sdk 로 버스에 직접)
==========================================================================================
D3 A안(nt_opencr 펌웨어) 에서도 의미 있는 질문만 남김:
  Q3 버스 하나(OpenCR 1장)에 모터 14개를 다 물려도 한 바퀴(전부 읽기 + 쓰기)가 10 ms(100 Hz) 안에 끝나나 → timing
  Q4 모터 자체 경비견(Bus Watchdog, 주소 98)이 300 ms 끊김에서 정확히 멈추나 (안전장치 ③)          → watchdog
  + 보드가 몇 장이든 이름을 고정할 수 있나 (udev)                                                       → ports
  + 모터 청진기: 모드 · 토크 · 워치독 · 에러 · 전압 · 온도                                               → inspect
  + 조향 범위 원인 확인 (2026-10-16): Homing Offset · Min/Max Position Limit · Drive Mode · 지금 위치 — **읽기만**   → limits

⚠ 이 도구는 다이나믹셀 패킷을 버스에 바로 보냄 → U2D2, 또는 OpenCR 에 ROBOTIS usb_to_dxl 예제를 올린 상태에서 씀
  (nt_opencr 펌웨어가 올라가 있으면 nt_packet 패킷만 알아들음). ros2_control_node 가 같은 포트를 잡고 있으면 못 엶.
  pip install dynamixel-sdk

  python3 tools/hw_bench.py ports                                          # 꽂힌 OpenCR / U2D2 목록 + udev 판정
  python3 tools/hw_bench.py inspect --scan                                 # ID 0~30 찾기
  python3 tools/hw_bench.py inspect --ids 1 11 [--clear-watchdog]
  python3 tools/hw_bench.py limits --ids 1 2 41 42 11 12 13 14 --map config/hardware_map_drive_only.yaml --json out.json   # 쓰기 없음
  python3 tools/hw_bench.py timing --ids 1 11 [--set-rdt 0] [--csv t.csv]   # 모터 수를 늘려 가며 재고 8 · 14개 예측
  python3 tools/hw_bench.py watchdog --id 1 [--watchdog 15 --gaps 0.1 0.25 0.35 0.5]   # ⚠ 바퀴를 공중에 띄우고
  python3 tools/hw_bench.py power --id 2 [--speeds 50 100 200 --accs 0 20 --csv p.csv]  # 무부하 전력 M0~M3 ⚠ 축이 자유롭게

"""

import argparse
import csv
import glob
import json
import statistics
import subprocess
import sys
import time
from pathlib import Path

# =============================================================================== 공통
PROTOCOL = 2.0
# XM430-W210 / W350 컨트롤 테이블 (두 모델 주소 동일)   이름: (주소, 크기 byte)
ADDR = {
    "model_number":      (0, 2),
    "firmware_version":  (6, 1),
    "drive_mode":        (10, 1),  # EEPROM, 비트 0 = 방향 반대 (Reverse)
    "homing_offset":     (20, 4),  # EEPROM, 부호 있음 [눈금]. Present Position = 실제 위치 + 이 값 (ROBOTIS e-Manual)
    "max_position_limit": (48, 4), # EEPROM, 위치 모드 목표 상한 [눈금]
    "min_position_limit": (52, 4), # EEPROM, 위치 모드 목표 하한 [눈금]
    "return_delay_time": (9, 1),   # EEPROM, 단위 2 us (기본 250 = 500 us)
    "operating_mode":    (11, 1),  # EEPROM, 1 = 속도, 3 = 위치
    "torque_enable":     (64, 1),
    "led":               (65, 1),
    "hw_error":          (70, 1),
    "bus_watchdog":      (98, 1),  # 단위 20 ms, 0 = 끔, -1(=255) = 워치독 걸림
    "goal_velocity":     (104, 4), # 단위 0.229 rpm
    "profile_acceleration": (108, 4),  # 속도 모드: 단위 214.577 rev/min², 0 = 제한 없음
    "goal_position":     (116, 4), # 0~4095, 2048 = 가운데
    "present_current":   (126, 2), # 단위 2.69 mA
    "present_velocity":  (128, 4),
    "present_position":  (132, 4),
    "present_voltage":   (144, 2), # 단위 0.1 V
    "present_temp":      (146, 1), # 단위 1 C
}
OP_MODE_NAME = {0: "전류", 1: "속도", 3: "위치", 4: "확장위치", 5: "전류기반위치", 16: "PWM"}
MODEL_NAME = {1020: "XM430-W350", 1030: "XM430-W210"}
RPM_PER_UNIT = 0.229          # goal/present velocity 1칸
MA_PER_UNIT = 2.69            # present current 1칸


def sdk():
    try:
        import dynamixel_sdk
    except ImportError:
        sys.exit("dynamixel_sdk 가 없습니다 → pip install dynamixel-sdk")
    return dynamixel_sdk


def to_signed(value, size):
    """모터가 보내는 부호 없는 숫자를 부호 있는 숫자로 (2의 보수)."""
    bits = size * 8
    return value - (1 << bits) if value >= 1 << (bits - 1) else value


def to_unsigned(value, size):
    """음수를 모터에 보낼 때 (예: 역방향 속도)."""
    return value & ((1 << (size * 8)) - 1)


class Bus:
    """포트 하나 = 다이나믹셀 버스 하나. OpenCR(usb_to_dxl) 이든 U2D2 든 똑같이 보인다."""

    def __init__(self, port, baud):
        d = sdk()
        self.ok = d.COMM_SUCCESS
        self.port = d.PortHandler(port)
        self.pkt = d.PacketHandler(PROTOCOL)
        if not self.port.openPort():
            sys.exit(f"포트를 열 수 없음: {port}  (ros2_control_node 가 켜져 있으면 먼저 끄기)")
        if not self.port.setBaudRate(baud):
            sys.exit(f"통신 속도 설정 실패: {baud}")

    def close(self):
        self.port.closePort()

    def ping(self, dxl_id):
        model, res, _ = self.pkt.ping(self.port, dxl_id)
        return model if res == self.ok else None

    def read(self, dxl_id, name, signed=False):
        addr, size = ADDR[name]
        fn = {1: self.pkt.read1ByteTxRx, 2: self.pkt.read2ByteTxRx, 4: self.pkt.read4ByteTxRx}[size]
        value, res, _ = fn(self.port, dxl_id, addr)
        if res != self.ok:
            raise IOError(f"ID {dxl_id} {name} 읽기 실패: {self.pkt.getTxRxResult(res)}")
        return to_signed(value, size) if signed else value

    def write(self, dxl_id, name, value):
        addr, size = ADDR[name]
        fn = {1: self.pkt.write1ByteTxRx, 2: self.pkt.write2ByteTxRx, 4: self.pkt.write4ByteTxRx}[size]
        res, err = fn(self.port, dxl_id, addr, to_unsigned(int(value), size))
        if res != self.ok:
            raise IOError(f"ID {dxl_id} {name}={value} 쓰기 실패: {self.pkt.getTxRxResult(res)}")
        return err  # 0 이 아니면 모터가 "받긴 했는데 문제 있음" 을 알려 준 것 (예: 범위 밖)


def wd_value(raw):
    return raw - 256 if raw > 127 else raw


# =============================================================================== ports
def cmd_ports(_args):
    """OpenCR 여러 장을 구분할 수 있나: 시리얼번호가 보드마다 다르면 udev 규칙을 시리얼번호로, 같으면 USB 자리로."""
    devs = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))   # OpenCR = ACM, U2D2 = USB
    if not devs:
        sys.exit("시리얼 장치가 하나도 없음. USB 를 꽂고 'dmesg | tail' 로 인식됐는지 확인.")
    print(f"{'장치':<14} {'VID:PID':<10} {'시리얼번호(SERIAL)':<24} {'USB 위치(ID_PATH)':<28} 이름")
    serials = []
    for d in devs:
        out = subprocess.run(['udevadm', 'info', '--query=property', f'--name={d}'],
                             capture_output=True, text=True).stdout
        prop = dict(line.split('=', 1) for line in out.splitlines() if '=' in line)
        serials.append(prop.get('ID_SERIAL_SHORT', ''))
        print(f"{d:<14} {prop.get('ID_VENDOR_ID', '')}:{prop.get('ID_MODEL_ID', ''):<5} {serials[-1]:<24} "
              f"{prop.get('ID_PATH', '').replace('platform-', ''):<28} {prop.get('ID_MODEL', '')}")
    print("\n판정:")
    if len(serials) <= 1:
        print("  장치가 1개뿐 → 보드를 하나 더 꽂고 다시 실행해야 비교 가능")
    elif len(set(serials)) == len(serials):
        print("  시리얼번호가 모두 다름 → udev 규칙을 시리얼번호로 만들면 됨 (재부팅해도 이름 고정)")
    else:
        print("  시리얼번호가 겹침 → USB 위치(ID_PATH)로 규칙을 만들어야 함 (꽂는 구멍이 바뀌면 이름도 바뀜 → 케이블 라벨)")


# =============================================================================== inspect
def cmd_inspect(args):
    """모터 상태 한 줄씩. ros2_control_node 를 kill -9 한 뒤 워치독이 걸렸는지(-1) 보는 데도 씀"""
    bus = Bus(args.port, args.baud)
    ids = list(range(0, 31)) if args.scan else args.ids
    if not ids:
        sys.exit("--ids 또는 --scan 중 하나는 필요")
    found = []
    for i in ids:
        if bus.ping(i) is not None:
            found.append(i)
        elif not args.scan:
            print(f"ID {i:3d}: 응답 없음 (ID·통신속도·전원·배선 확인)")
    if args.scan:
        print(f"찾은 ID: {found if found else '없음'}")
    if args.clear_watchdog:
        for i in found:
            bus.write(i, "bus_watchdog", 0)
            print(f"ID {i}: 워치독 0 으로 지움")
    if found:
        print(f"\n{'ID':>3} {'모델':<11} {'모드':<6} {'토크':<4} {'워치독':<10} {'에러':<5} "
              f"{'전압V':>5} {'온도C':>5} {'위치':>6} {'속도rpm':>8} {'전류mA':>7}")
    for i in found:
        wd = wd_value(bus.read(i, 'bus_watchdog'))
        wd_txt = "꺼짐" if wd == 0 else ("★걸림(-1)" if wd < 0 else f"{wd * 20} ms")
        print(f"{i:3d} {MODEL_NAME.get(bus.read(i, 'model_number'), '?'):<11} "
              f"{OP_MODE_NAME.get(bus.read(i, 'operating_mode'), '?'):<6} "
              f"{'ON' if bus.read(i, 'torque_enable') else 'off':<4} {wd_txt:<10} "
              f"{bus.read(i, 'hw_error'):#04x}  {bus.read(i, 'present_voltage') * 0.1:5.1f} "
              f"{bus.read(i, 'present_temp'):5d} {bus.read(i, 'present_position', signed=True):6d} "
              f"{bus.read(i, 'present_velocity', signed=True) * RPM_PER_UNIT:8.1f} "
              f"{bus.read(i, 'present_current', signed=True) * MA_PER_UNIT:7.0f}")
    bus.close()


# =============================================================================== limits — 조향 범위 원인 (읽기 전용)
# 왜: 조향 13 의 직진 눈금이 3596 이라 위치 모드에서 시계 쪽이 약 44° 뿐. 이게 설정(Homing Offset · Min/Max Position Limit ·
#     Drive Mode) 때문인지 조립 기준(혼을 끼운 각도) 때문인지 가르려면 먼저 그 값들을 **쓰지 않고** 읽어야 함
# nt_opencr 펌웨어로는 못 함: 레지스터를 읽는 상자가 없고, 위치도 CONFIG(설정 쓰기) 뒤에야 옴 → usb_to_dxl(또는 U2D2)로 읽음
LIMIT_ITEMS = ["model_number", "firmware_version", "drive_mode", "operating_mode", "homing_offset",
               "min_position_limit", "max_position_limit", "torque_enable", "hw_error", "present_position"]
SIGNED_ITEMS = {"homing_offset", "present_position"}
DEG_PER_TICK = 360.0 / 4096


class ReadOnlyBus:
    """Bus 를 감싸 읽기 · ping 만 허용. write 를 부르면 바로 예외 (limits 가 실수로라도 쓰지 않게)"""

    def __init__(self, bus):
        self._bus = bus

    def ping(self, dxl_id):
        return self._bus.ping(dxl_id)

    def read(self, dxl_id, name, signed=False):
        return self._bus.read(dxl_id, name, signed=signed)

    def write(self, *a, **k):
        raise RuntimeError("limits 는 읽기 전용 — 쓰기 시도")

    def close(self):
        self._bus.close()


def read_limits(bus, dxl_id):
    """한 모터의 설정 · 위치를 읽기만 → dict (읽기 실패는 그 칸에 None + problems)"""
    out, problems = {"id": dxl_id}, []
    for name in LIMIT_ITEMS:
        try:
            out[name] = bus.read(dxl_id, name, signed=name in SIGNED_ITEMS)
        except IOError as e:
            out[name] = None
            problems.append(str(e))
    out["problems"] = problems
    return out


def judge_limits(r, zero=None, sign=None):
    """읽은 값 해석: 위치 모드에서 명령할 수 있는 눈금 = [Min, Max] (0~4095 안). 직진(zero)에서 양쪽으로 몇 도인지.
    sign = 시험 설정의 scale (조향 −1 = 모터 + 가 위에서 시계). 돌려줌: 해석 dict"""
    lo, hi = r.get("min_position_limit"), r.get("max_position_limit")
    j = {"reverse": None if r.get("drive_mode") is None else bool(r["drive_mode"] & 1)}
    if r.get("present_position") is not None and r.get("homing_offset") is not None:
        j["actual_estimate"] = r["present_position"] - r["homing_offset"]   # e-Manual 식으로 되돌린 값 (실측 확인 전)
    if lo is not None and hi is not None:
        lo_c, hi_c = max(0, lo), min(4095, hi)
        j["command_range"] = [lo_c, hi_c]
        j["limits_default"] = (lo, hi) == (0, 4095)
        if zero is not None:
            j["to_max_deg"] = (hi_c - zero) * DEG_PER_TICK   # 눈금 증가 쪽
            j["to_min_deg"] = (zero - lo_c) * DEG_PER_TICK   # 눈금 감소 쪽
            if sign is not None:
                cw, ccw = (j["to_max_deg"], j["to_min_deg"]) if sign < 0 else (j["to_min_deg"], j["to_max_deg"])
                j["cw_deg"], j["ccw_deg"] = cw, ccw            # 위에서 볼 때 (시험 설정의 방향 기준)
            j["zero_inside"] = lo_c <= zero <= hi_c
    return j


def cmd_limits(args):
    """조향 Homing Offset · Min/Max Position Limit · Drive Mode · 지금 위치를 **읽기만** (토크 · 설정 · 목표를 안 씀)"""
    zeros, signs = {}, {}
    if args.map:
        import yaml
        hw = yaml.safe_load(Path(args.map).read_text())
        for m in hw["motors"].values():
            if m.get("zero_tick") is not None:   # null = 아직 안 잰 영점 (hardware_map_drive_only.yaml) → 해석만 생략
                zeros[m["id"]] = int(m["zero_tick"])
            if "scale" in m:
                signs[m["id"]] = float(m["scale"])
    print("읽기 전용: ping · 읽기만 (토크 · 모드 · Homing Offset · 한계 · 목표를 쓰지 않음). usb_to_dxl 또는 U2D2 로")
    bus = ReadOnlyBus(Bus(args.port, args.baud))
    rows = []
    try:
        for i in args.ids:
            if bus.ping(i) is None:
                rows.append({"id": i, "problems": ["응답 없음 (ID · 통신 속도 · 전원 · 배선)"]})
                continue
            r = read_limits(bus, i)
            r["interpret"] = judge_limits(r, zeros.get(i), signs.get(i))
            r["zero_tick"] = zeros.get(i)
            rows.append(r)
    finally:
        bus.close()
    print(f"\n{'ID':>3} {'모델':<11} {'모드':<10} {'DriveMode':>9} {'HomingOff':>9} {'Min':>6} {'Max':>6} {'지금위치':>8} "
          f"{'직진':>6} {'시계쪽°':>8} {'반시계쪽°':>9} 메모")
    for r in rows:
        if "present_position" not in r:
            print(f"{r['id']:3d}  {' · '.join(r['problems'])}")
            continue
        j = r["interpret"]
        memo = []
        if r["homing_offset"]:
            memo.append(f"Homing Offset ≠ 0 (지금 위치 − Offset = {j.get('actual_estimate')})")
        if j.get("limits_default") is False:
            memo.append("Min/Max 가 기본(0 · 4095) 아님")
        if j.get("reverse"):
            memo.append("Drive Mode 방향 반대")
        if j.get("zero_inside") is False:
            memo.append("⚠ 직진 눈금이 Min~Max 밖")
        if r["torque_enable"]:
            memo.append("⚠ 토크 켜짐")
        memo += r["problems"]
        cw = f"{j['cw_deg']:8.1f}" if "cw_deg" in j else f"{'-':>8}"
        ccw = f"{j['ccw_deg']:9.1f}" if "ccw_deg" in j else f"{'-':>9}"
        print(f"{r['id']:3d} {MODEL_NAME.get(r['model_number'], '?'):<11} {OP_MODE_NAME.get(r['operating_mode'], '?'):<10} "
              f"{r['drive_mode']:>9} {r['homing_offset']:>9} {r['min_position_limit']:>6} {r['max_position_limit']:>6} "
              f"{r['present_position']:>8} {r['zero_tick'] if r['zero_tick'] is not None else '-':>6} {cw} {ccw} {' · '.join(memo)}")
    print("\n해석: 위치 모드 목표는 Min~Max(0~4095 안)로 잘림. 시계/반시계 = 직진에서 그 끝까지 (시험 설정 scale 기준, 위에서 볼 때)")
    print("⚠ 명령할 수 있는 범위일 뿐 — 케이블 · 기구 간섭으로 실제로 갈 수 있는 범위는 따로 확인. Homing Offset 허용 범위는 모드마다 다름 (e-Manual)")
    if args.json:
        Path(args.json).write_text(json.dumps(rows, ensure_ascii=False, indent=2))
        print(f"저장: {args.json}")


# =============================================================================== timing — Q3
# 한 바퀴 = Sync Read 전류(126)·속도(128)·위치(132) 10바이트 + Sync Write LED(65) 1바이트 (모터를 안 움직이려고 LED.
#   실제 목표값은 4바이트라 모터당 3바이트 더 김 → 1 Mbps 에서 약 0.03 ms 차이)
# 결과를 좌우하는 것: Return Delay Time(주소 9) 기본 250 = 500 us — 모터 하나마다 0.5 ms 버림. OpenManipulator 공식은 0
#   (--set-rdt 0. EEPROM 이라 전원 꺼도 유지, 토크 꺼진 상태에서만 바뀜)
READ_START, READ_LEN = 126, 10


def _measure(bus, ids, loops):
    d = sdk()
    sr = d.GroupSyncRead(bus.port, bus.pkt, READ_START, READ_LEN)
    sw = d.GroupSyncWrite(bus.port, bus.pkt, ADDR["led"][0], 1)
    for i in ids:
        sr.addParam(i)
        sw.addParam(i, [0])
    times, fails = [], 0
    for _ in range(loops):
        t0 = time.perf_counter()
        res = sr.txRxPacket()
        ok = res == d.COMM_SUCCESS and all(sr.isAvailable(i, READ_START, READ_LEN) for i in ids)
        sw.txPacket()
        dt = (time.perf_counter() - t0) * 1000.0
        if ok:
            times.append(dt)
        else:
            fails += 1
    times.sort()
    if not times:
        return None
    return {"n": len(ids), "mean": statistics.mean(times), "p95": times[int(len(times) * 0.95) - 1],
            "max": times[-1], "fail": fails}


def cmd_timing(args):
    """모터 1개, 2개 … 로 늘려 가며 재고, 직선(고정비용 + 모터당비용 × 개수)으로 8 · 14개를 예측 (2개로 잰 예측은 참고값)"""
    bus = Bus(args.port, args.baud)
    for i in args.ids:
        if bus.ping(i) is None:
            sys.exit(f"ID {i} 응답 없음 → inspect --scan 으로 먼저 확인")
    if args.set_rdt is not None:
        for i in args.ids:
            bus.write(i, "torque_enable", 0)
            bus.write(i, "return_delay_time", args.set_rdt)
    rdts = {i: bus.read(i, "return_delay_time") for i in args.ids}
    print("Return Delay Time: " + ", ".join(f"ID{i}={v}({v * 2}us)" for i, v in rdts.items()))
    print(f"통신 속도 {args.baud} bps, 반복 {args.loops}회\n")
    rows = []
    print(f"{'모터수':>5} {'평균ms':>8} {'95%ms':>8} {'최대ms':>8} {'실패':>5}")
    for k in range(1, len(args.ids) + 1):
        r = _measure(bus, args.ids[:k], args.loops)
        if r is None:
            print(f"{k:5d}  전부 실패 — 배선·전원 확인")
            continue
        rows.append(r)
        print(f"{r['n']:5d} {r['mean']:8.2f} {r['p95']:8.2f} {r['max']:8.2f} {r['fail']:5d}")
    bus.close()
    if not rows:
        return
    xs, ys = [r["n"] for r in rows], [r["p95"] for r in rows]
    # ⚠ (2026-10-16 실측) PC 에서 USB(usb_to_dxl · U2D2)를 거쳐 재면 모터마다 응답 패킷을 USB 로 받느라
    #   약 1 ms 단위 계단이 생김 (모터 1개 → 2개: +0.97 ms, 버스 계산은 +0.25 ms 남짓).
    #   그래서 여기 숫자는 **PC 에서 직접 버스를 돌릴 때(이 도구 · U2D2)** 의 시간이고, 펌웨어가 보드 안에서
    #   Sync Read 하는 시간(nt_opencr)이 아님 → 보드 수(D4)를 이 예측으로 정하지 말 것
    if len(xs) < 2:
        print("\n모터 1개만 재서 직선을 그을 수 없음 → 모터 수별 예측 안 함 (2개 이상으로 다시)")
    else:
        mx, my = statistics.mean(xs), statistics.mean(ys)
        b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
        a = my - b * mx
        print(f"\n직선 맞추기 (95% 값 기준, PC↔USB 포함): 시간 ≈ {a:.2f} ms + {b:.2f} ms × 모터수")
        for n in args.predict:
            t = a + b * n
            verdict = "100 Hz 가능" if t < 10 else ("50 Hz 가능" if t < 20 else "50 Hz 도 빠듯")
            print(f"  모터 {n:2d}개 → 한 바퀴 약 {t:5.2f} ms → {verdict}  (PC 에서 직접 돌릴 때)")
        if len(rows) < 3:
            print(f"  ※ 모터 {len(rows)}개 점으로 그은 직선이라 참고값. 개수를 늘려 다시 재면 정확해짐.")
        print("  ※ USB 왕복 · 1 ms 계단 포함 — 보드 안(nt_opencr) 시간은 이보다 짧음. 보드 수 결정에 그대로 쓰지 말 것")
    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["n", "mean", "p95", "max", "fail"])
            w.writeheader()
            w.writerows(rows)
        print(f"\n저장: {args.csv}")


# =============================================================================== watchdog — Q4
# 워치독은 "아무 패킷이든" 오면 시간을 다시 셈 → 중간에 들여다볼 수 없음. 그래서:
#   바퀴를 천천히 돌림 → 정해 둔 침묵 시간 동안 아무것도 안 보냄 → 침묵이 끝나자마자 워치독 값을 읽음 (-1 = 그 사이에 걸림)
#   침묵 시간을 바꿔 가며 경계를 찾음. 기대: 워치독 15(=300 ms) 면 0.25 s 는 안 걸리고 0.35 s 는 걸림
def _arm_motor(bus, i, watchdog, speed):
    """워치독을 지우고 → 다시 켜고 → 바퀴를 돌린 뒤 0.5초간 '말을 걸며' 속도가 붙는지 확인."""
    bus.write(i, "bus_watchdog", 0)                # 걸려 있던 에러(-1) 해제
    if not bus.read(i, "torque_enable"):
        bus.write(i, "torque_enable", 1)
    bus.write(i, "bus_watchdog", watchdog)
    bus.write(i, "goal_velocity", speed)
    t_end = time.perf_counter() + 0.5
    v = 0
    while time.perf_counter() < t_end:             # 이 동안은 계속 읽어서 워치독이 안 걸리게
        time.sleep(0.05)
        v = bus.read(i, "present_velocity", signed=True)
    # ↑ 마지막 동작이 반드시 "읽기" 로 끝나야 함. 반대(읽기→쉬기)면 침묵 시간에 0.05 s 가 몰래 더해짐 (가짜 모터 시험에서 발견)
    return v


def cmd_watchdog(args):
    i = args.id
    bus = Bus(args.port, args.baud)
    if bus.ping(i) is None:
        sys.exit(f"ID {i} 응답 없음")
    input(f"ID {i} 바퀴가 공중에 떠 있나요? 맞으면 Enter (중단: Ctrl+C) ")
    bus.write(i, "torque_enable", 0)   # 속도 모드로 (EEPROM 이라 토크를 꺼야 바뀜)
    bus.write(i, "operating_mode", 1)
    timeout_ms = args.watchdog * 20
    print(f"\n워치독 {args.watchdog} = {timeout_ms} ms, 속도 {args.speed}칸 ({args.speed * RPM_PER_UNIT:.1f} rpm)\n")
    print(f"{'침묵(ms)':>8} {'실제침묵':>8} {'침묵 전 속도':>12} {'침묵 후 워치독':>14} {'침묵 후 속도':>12}  판정")
    results = []
    try:
        for gap in args.gaps:
            v_before = _arm_motor(bus, i, args.watchdog, args.speed)
            t0 = time.perf_counter()
            time.sleep(gap)                                  # ← 여기서 아무 패킷도 안 보냄
            real_ms = (time.perf_counter() - t0) * 1000
            wd = wd_value(bus.read(i, "bus_watchdog"))
            v_after = bus.read(i, "present_velocity", signed=True)
            tripped, expect = wd == -1, real_ms > timeout_ms
            print(f"{gap * 1000:8.0f} {real_ms:8.1f} {v_before * RPM_PER_UNIT:9.1f}rpm "
                  f"{('걸림(-1)' if tripped else str(wd)):>14} {v_after * RPM_PER_UNIT:9.1f}rpm  "
                  f"{'기대대로' if tripped == expect else '★기대와 다름'}")
            results.append((tripped, expect))
    finally:
        bus.write(i, "bus_watchdog", 0)
        bus.write(i, "goal_velocity", 0)
        bus.write(i, "torque_enable", 0)
        bus.close()
        print("\n정리 완료: 워치독 0, 속도 0, 토크 끔")
    if results and all(t == e for t, e in results):
        print(f"결론: {timeout_ms} ms 경계에서 정확히 멈춤 → 안전장치 ③ 근거 확보")
    elif results:
        print("결론: 기대와 다른 칸이 있음 → 모터 펌웨어 버전(v38 이상) · 설정을 확인하고 표를 그대로 기록")


# =============================================================================== power — 무부하 전력 (2026-10-16 2단계)
# 모터 하나를 속도 모드로 M0 대기 → M1 버티기 → M2 일정 속도 → M3 출발 · 정지 반복, 단계마다 모터가 재는
#   Present Voltage(전압) · Present Current(전류)를 계속 읽어 요약. 전원 장치 표시 · 멀티미터 값과 나란히 적을 것
# ⚠ 모터 안 센서 값 = 그 모터가 먹는 전류. 전원 전체(OpenCR · 다른 모터 포함)는 전원 장치 쪽에서 따로 잼
# ⚠ 읽기 간격이 USB 왕복 때문에 수 ms 라서 출발 순간의 짧은 피크는 놓칠 수 있음 → "관측 최대" 로만 기록
POWER_BLOCK = (126, 21)   # 126 전류(2) · 128 속도(4) · 132 위치(4) · 136~143 궤적 · 144 전압(2) · 146 온도(1)


def _power_sample(bus, i):
    data, res, _ = bus.pkt.readTxRx(bus.port, i, *POWER_BLOCK)
    if res != bus.ok or len(data) < POWER_BLOCK[1]:
        return None
    cur = to_signed(data[0] | data[1] << 8, 2) * MA_PER_UNIT / 1000.0
    vel = to_signed(data[2] | data[3] << 8 | data[4] << 16 | data[5] << 24, 4) * RPM_PER_UNIT
    volt = (data[18] | data[19] << 8) * 0.1
    return time.perf_counter(), volt, cur, vel, data[20]


def _power_phase(bus, i, name, seconds, rows, events=None, settle=0.0):
    """seconds 동안 계속 읽기. events = [(시작 뒤 초, 함수)] 를 그 시각에 실행 (출발 · 정지 명령)"""
    print(f"  ▶ {name} ({seconds:.0f} s) — 지금 전원 장치 / 멀티미터 값을 보세요", flush=True)
    t0, fails, pending = time.perf_counter(), 0, sorted(events or [], key=lambda e: e[0])
    samples = []
    while time.perf_counter() - t0 < seconds:
        while pending and time.perf_counter() - t0 >= pending[0][0]:
            pending.pop(0)[1]()
        s = _power_sample(bus, i)
        if s is None:
            fails += 1
            continue
        samples.append(s)
        rows.append((name, s[0] - t0) + s[1:])
    hw_err = bus.read(i, "hw_error")
    steady = [s for s in samples if s[0] - t0 >= settle] or samples
    if not samples:
        print(f"    읽기 전부 실패 ({fails} 회) — 통신 · 전원 확인")
        return
    vmin = min(s[1] for s in samples)
    imean = statistics.mean(abs(s[2]) for s in steady)
    imax = max(abs(s[2]) for s in samples)
    vel = statistics.mean(s[3] for s in steady)
    dt = (samples[-1][0] - samples[0][0]) / max(1, len(samples) - 1) * 1000
    print(f"    전압 최저 {vmin:5.1f} V · 평균 {statistics.mean(s[1] for s in samples):5.1f} V | "
          f"전류 평균 {imean:5.3f} A · 관측 최대 {imax:5.3f} A | 속도 평균 {vel:6.1f} rpm | "
          f"읽기 {len(samples)} 회 (약 {dt:.1f} ms 간격) 실패 {fails} | 온도 {samples[-1][4]} C | "
          f"Hardware Error {hw_err:#04x}{'  ★' if hw_err else ''}")


def cmd_power(args):
    i = args.id
    bus = Bus(args.port, args.baud)
    if bus.ping(i) is None:
        sys.exit(f"ID {i} 응답 없음")
    print(f"ID {i} 무부하 전력 측정: M0 대기 → M1 버티기 → M2 속도 {args.speeds} → M3 출발·정지 {args.cycles} 번 × 가속 {args.accs}")
    input("⚠ 모터가 고정돼 있고 축(바퀴)이 자유롭게 돌 수 있나요? 손 · 선이 닿지 않나요? 맞으면 Enter (중단: Ctrl+C) ")
    orig_mode = bus.read(i, "operating_mode")
    orig_acc = bus.read(i, "profile_acceleration")
    rows = []
    try:
        bus.write(i, "torque_enable", 0)
        _power_phase(bus, i, "M0 대기 (토크 끔)", args.hold, rows)
        bus.write(i, "operating_mode", 1)   # 속도 모드 (EEPROM — 끝나면 원래대로)
        bus.write(i, "profile_acceleration", 0)
        bus.write(i, "goal_velocity", 0)
        bus.write(i, "torque_enable", 1)
        _power_phase(bus, i, "M1 버티기 (토크 켬, 속도 0)", args.hold, rows)
        for v in args.speeds:
            bus.write(i, "goal_velocity", v)
            _power_phase(bus, i, f"M2 일정 속도 {v} 칸 ({v * RPM_PER_UNIT:.1f} rpm)", args.hold, rows, settle=1.0)
        bus.write(i, "goal_velocity", 0)
        time.sleep(1.0)
        top = max(args.speeds)
        for acc in args.accs:
            bus.write(i, "profile_acceleration", acc)
            ev = []
            for k in range(args.cycles):
                ev += [(k * 3.0, lambda: bus.write(i, "goal_velocity", top)),
                       (k * 3.0 + 1.5, lambda: bus.write(i, "goal_velocity", 0))]
            _power_phase(bus, i, f"M3 출발·정지 0↔{top} 칸 × {args.cycles} (가속 {acc}{' = 제한 없음' if acc == 0 else ''})",
                         args.cycles * 3.0, rows, events=ev)
    finally:
        try:
            bus.write(i, "goal_velocity", 0)
            bus.write(i, "torque_enable", 0)
            bus.write(i, "profile_acceleration", orig_acc)
            bus.write(i, "operating_mode", orig_mode)
        finally:
            bus.close()
        print(f"\n정리 완료: 속도 0, 토크 끔, 모드 {OP_MODE_NAME.get(orig_mode, orig_mode)} · 가속 {orig_acc} 로 되돌림")
    if args.csv and rows:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["phase", "t_s", "voltage_V", "current_A", "velocity_rpm", "temp_C"])
            w.writerows((r[0], f"{r[1]:.4f}", f"{r[2]:.1f}", f"{r[3]:.4f}", f"{r[4]:.1f}", r[5]) for r in rows)
        print(f"원자료: {args.csv} ({len(rows)} 줄)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def with_port(p):
        p.add_argument("--port", default="/dev/ttyACM0")
        p.add_argument("--baud", type=int, default=1_000_000)
        return p

    sub.add_parser("ports").set_defaults(fn=cmd_ports)
    p = with_port(sub.add_parser("inspect"))
    p.add_argument("--ids", type=int, nargs="*", default=[])
    p.add_argument("--scan", action="store_true", help="ID 0~30 를 ping 해서 찾기")
    p.add_argument("--clear-watchdog", action="store_true", help="워치독 값을 0 으로 (에러 해제)")
    p.set_defaults(fn=cmd_inspect)
    p = with_port(sub.add_parser("limits", help="조향 Homing Offset · Min/Max Position Limit · Drive Mode · 지금 위치 읽기 (쓰기 없음)"))
    p.add_argument("--ids", type=int, nargs="+", default=[11, 12, 13, 14])
    p.add_argument("--map", default=None, help="직진 눈금(zero_tick) · scale 이 든 시험 설정 (있으면 양쪽 각도까지 계산)")
    p.add_argument("--json", default=None)
    p.set_defaults(fn=cmd_limits)
    p = with_port(sub.add_parser("timing"))
    p.add_argument("--ids", type=int, nargs="+", required=True)
    p.add_argument("--loops", type=int, default=500)
    p.add_argument("--set-rdt", type=int, default=None, metavar="값", help="Return Delay Time 을 바꾼 뒤 잼 (단위 2 us)")
    p.add_argument("--predict", type=int, nargs="+", default=[8, 14])
    p.add_argument("--csv", default=None)
    p.set_defaults(fn=cmd_timing)
    p = with_port(sub.add_parser("watchdog"))
    p.add_argument("--id", type=int, required=True, help="바퀴(속도 모드로 쓸) 모터 ID")
    p.add_argument("--watchdog", type=int, default=15, help="워치독 값 (단위 20 ms, 15 = 300 ms)")
    p.add_argument("--speed", type=int, default=30, help="속도 (단위 0.229 rpm)")
    p.add_argument("--gaps", type=float, nargs="+", default=[0.1, 0.2, 0.25, 0.35, 0.5], help="침묵 시간들 (초)")
    p.set_defaults(fn=cmd_watchdog)
    p = with_port(sub.add_parser("power", help="무부하 전력 M0 대기 · M1 버티기 · M2 일정 속도 · M3 출발·정지"))
    p.add_argument("--id", type=int, required=True, help="속도 모드로 돌릴 모터 ID (바퀴)")
    p.add_argument("--speeds", type=int, nargs="+", default=[50, 100, 200], help="M2 속도들 (단위 0.229 rpm)")
    p.add_argument("--accs", type=int, nargs="+", default=[0, 20], help="M3 가속 한계들 (0 = 제한 없음, 20 = 우리 바퀴 설정)")
    p.add_argument("--cycles", type=int, default=5, help="M3 출발·정지 횟수")
    p.add_argument("--hold", type=float, default=10.0, help="M0 · M1 · M2 단계마다 초")
    p.add_argument("--csv", default=None, help="원자료 저장 (단계 · 시각 · 전압 · 전류 · 속도 · 온도)")
    p.set_defaults(fn=cmd_power)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
