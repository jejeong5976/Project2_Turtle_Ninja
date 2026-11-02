#!/usr/bin/env python3
"""OpenCR 포트 이름 고정 규칙 만들기 (2026-10-29) — 허브에 여러 장치를 꽂아도 부팅마다 /dev/opencr_drive · /dev/opencr_arm 이 같은 보드.
규칙 파일만 만들고 설치는 안 함 (설치는 아래 출력의 sudo 명령을 직접).

순서 (보드를 하나씩 꽂으며 물어봄 — 어느 보드인지 사람이 정함):
  python3 tools/jetson/make_udev_rules.py
  1) 모든 OpenCR 을 뺀 상태에서 시작 → 2) 주행 보드만 꽂고 Enter → 3) 팔 보드도 꽂고 Enter
  → 99-ninja-turtle-opencr.rules 생성. 시리얼번호가 두 보드에서 다르면 시리얼번호로, 같으면 USB 자리(ID_PATH)로 (그땐 꽂는 구멍 고정 · 라벨)
이미 이름이 있으면(/dev/opencr_*) 그 규칙도 보여 줌. ModemManager 가 꽂을 때 AT 명령으로 건드리지 않게 막음 (터틀봇3 OpenCR 규칙과 같음).
"""
import glob
import os
import subprocess
import sys

RULE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "99-ninja-turtle-opencr.rules")


def devices():
    out = {}
    for d in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
        txt = subprocess.run(["udevadm", "info", "--query=property", f"--name={d}"], capture_output=True, text=True).stdout
        out[d] = dict(l.split("=", 1) for l in txt.splitlines() if "=" in l)
    return out


def wait_new(before, what):
    input(f"{what} 를 꽂고 2~3 초 뒤 Enter ")
    now = devices()
    new = [d for d in now if d not in before]
    if len(new) != 1:
        sys.exit(f"새 장치가 {len(new)}개 ({new}) — 하나씩 꽂아야 함. 처음부터 다시")
    p = now[new[0]]
    print(f"  {what}: {new[0]} · {p.get('ID_VENDOR_ID')}:{p.get('ID_MODEL_ID')} · 시리얼 {p.get('ID_SERIAL_SHORT', '(없음)')} · 자리 {p.get('ID_PATH')}")
    return now, p


def main():
    for n in ("opencr_drive", "opencr_arm"):
        if os.path.exists(f"/dev/{n}"):
            print(f"이미 있음: /dev/{n} → {os.path.realpath('/dev/' + n)}")
    start = devices()
    if any(p.get("ID_VENDOR_ID") == "0483" for p in start.values()):
        print("⚠ 지금 꽂힌 OpenCR(0483) 이 있음 — OpenCR 을 모두 빼고 다시 실행해야 어느 보드인지 구분 가능")
        print("  (라이다 · 카메라 등 다른 장치는 꽂혀 있어도 됨)")
        if input("그래도 진행? (y/N) ").strip().lower() != "y":
            return
    now, drive = wait_new(start, "주행(바퀴) OpenCR")
    _, arm = wait_new(now, "팔 OpenCR")
    s1, s2 = drive.get("ID_SERIAL_SHORT"), arm.get("ID_SERIAL_SHORT")
    if s1 and s2 and s1 != s2:
        match = lambda p: f'ATTRS{{idVendor}}=="{p["ID_VENDOR_ID"]}", ATTRS{{idProduct}}=="{p["ID_MODEL_ID"]}", ATTRS{{serial}}=="{p["ID_SERIAL_SHORT"]}"'
        how = "시리얼번호 (꽂는 구멍이 바뀌어도 같은 이름)"
    else:
        match = lambda p: f'ENV{{ID_PATH}}=="{p["ID_PATH"]}"'
        how = "USB 자리 (시리얼번호가 같거나 없음 → 꽂는 구멍을 바꾸면 안 됨, 케이블에 라벨)"
    text = ("# 닌자터틀 OpenCR 포트 이름 고정 (tools/jetson/make_udev_rules.py 로 생성) — 기준: " + how + "\n"
            f'SUBSYSTEM=="tty", {match(drive)}, SYMLINK+="opencr_drive", GROUP="dialout", MODE="0660", ENV{{ID_MM_DEVICE_IGNORE}}="1"\n'
            f'SUBSYSTEM=="tty", {match(arm)}, SYMLINK+="opencr_arm", GROUP="dialout", MODE="0660", ENV{{ID_MM_DEVICE_IGNORE}}="1"\n')
    open(RULE, "w").write(text)
    print(f"\n만듦: {RULE}  (기준: {how})\n{text}")
    print("설치 (직접):")
    print(f"  sudo cp {RULE} /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger")
    print("확인: ls -l /dev/opencr_drive /dev/opencr_arm  (보드를 뺐다 꽂아도 같은 쪽을 가리켜야 함)")


if __name__ == "__main__":
    main()
