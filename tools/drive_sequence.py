#!/usr/bin/env python3
"""
drive_sequence.py — 바퀴 전용 동작 시험: 전후진 · 좌우 · 대각선 · 제자리 회전 · 이동+회전 · 정지 (ROS, /cmd_vel)
================================================================================================
robot.launch.py arm:=false 로 띄운 뒤 실행. mock · 가짜 보드(fw_sim) · 실물 모두 같은 순서 · 같은 판정.

구간마다: /cmd_vel 을 --seconds 동안 20 Hz 로 보냄 → 0 을 --stop-seconds 동안 보냄. /joint_states 로 판정
  ① 조향 범위: 조향 관절이 모듈별 허용 범위(하드웨어 지도 영점 · 방향으로 계산, robot.launch.py 와 같은 식) 안
  ② 동작: 구간 마지막 --check-seconds 동안 모듈마다 (바퀴 선속도 + ω·offset)·(cos θ, sin θ) 가 명령의 모듈 속도와
     ±(--tol × 명령 크기) 안 (조향이 다 돌기 전 · 가속 중은 판정 구간에서 뺌). 모터 보고값 기준 — 바닥 이동 거리 · 방향 판정 아님
  ③ 정지: 0 을 보낸 뒤 --stop-within 초 안에 바퀴 4 개 실측 |v| < 0.05 rad/s
마지막 구간 "명령 끊김": 전진을 보내다가 아무것도 안 보냄 → 같은 시간 안에 정지 (swerve_controller cmd_timeout 0.3 s)
속도 상한(바꿀 수 없음): |v| ≤ 0.05 m/s, |ω| ≤ 0.3 rad/s. 실패를 숨기지 않음 — 하나라도 실패면 종료 1, 기록 JSON 에 전부

  python3 tools/drive_sequence.py --hardware-map <hardware_map_drive_only.yaml> --out /tmp/drive_seq.json
⚠ 실물: 바퀴를 띄운 상태에서 먼저, 전원 끌 사람 · 비상 정지 준비. 시작 전에 범위 · 순서를 보여 주고 Enter 를 받음 (--yes 로 생략)

--mode (기본 sequence = 위 순서). 나머지 모드도 같은 속도 상한 · 같은 판정 · 측정 못 한 값은 "미측정"
  stop        정지 시험. ① 명령 끊김 (--reps 회): 전진 중 /cmd_vel 발행을 멈춤
              ② 프로그램 종료 (마지막 1 회): 전진 명령을 계속 보내는 중에 터미널 A 를 Ctrl+C
              시점 구분: 마지막 명령 발행 · 구동 명령 0 (/swerve_controller/commands) · 실측 바퀴 정지 (/joint_states).
              종료 뒤에는 /joint_states 가 없어 바퀴 정지 · 거리는 영상 + 눈금으로만. 시점마다 삑 소리 (--beep, 영상 맞추기)
  distance    주행 정확도: 전진 · 후진 · 좌 · 우 를 --distance 씩 --reps 회. 끝날 때마다 바닥 실측(거리 · 옆 흐름 · 방향 오차)을
              입력받음 — 비우면 미측정. 엔코더(odom) 값은 따로 적고 실측 대신 쓰지 않음
  transitions 12 번(fr) 안쪽 자세(−180°)를 만든 뒤 후진 · 좌우 · 대각 · 회전 · 전진으로 전환. 안쪽 자세 확인과 fr 바퀴 속도 오차를
              따로 판정 (안쪽 자세 ≠ 구동 불능). 구간마다 간섭 · 들림 · 끌림을 물음 (입력하면 중단)
  park        정지 후 바깥쪽 복귀 (launch park_on_idle:=true 필요): ① 안쪽 자세 → 0 명령 유지 → 1 s 뒤 0° 복귀
              ② 0 명령 뒤 명령 끊김 → 복귀 안 함 ③ 복귀 도중 전진 명령 → 전환
"""
import argparse
import json
import math
import os
import struct
import subprocess
import sys
import tempfile
import time
import wave

import rclpy
import yaml
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

MAX_LIN, MAX_ANG = 0.05, 0.3
STOP_RAD_S = 0.05
STEER = ["steer_fl_joint", "steer_fr_joint", "steer_rl_joint", "steer_rr_joint"]
DRIVE = ["drive_fl_joint", "drive_fr_joint", "drive_rl_joint", "drive_rr_joint"]
STEER_MARGIN_TICKS, TICK_RAD = 20, 2.0 * math.pi / 4096

D = 0.035   # 대각선 성분 (크기 ≈ 0.05)
SEQUENCE = [("전진", 0.05, 0, 0), ("후진", -0.05, 0, 0), ("좌", 0, 0.05, 0), ("우", 0, -0.05, 0),
            ("대각 앞왼", D, D, 0), ("대각 앞오른", D, -D, 0), ("대각 뒤왼", -D, D, 0), ("대각 뒤오른", -D, -D, 0),
            ("제자리 반시계", 0, 0, 0.3), ("제자리 시계", 0, 0, -0.3), ("전진+반시계", 0.05, 0, 0.3)]


def steer_ranges(hw):
    """robot.launch.py steer_ranges 와 같은 식. 영점 없으면 None"""
    out = []
    for j in STEER:
        m = hw["motors"][j]
        if m.get("zero_tick") is None:
            return None
        z, sc = int(m["zero_tick"]), float(m.get("scale", 1.0))
        if m.get("extended"):   # 여러 바퀴 모드 (모의 전용) — robot.launch.py 와 같게
            out.append((math.radians(float(m["cable_min_deg"])), math.radians(float(m["cable_max_deg"]))))
            continue
        a, b = sc * (STEER_MARGIN_TICKS - z) * TICK_RAD, sc * (4095 - STEER_MARGIN_TICKS - z) * TICK_RAD
        # 선(케이블) 한계 cable_min_deg / cable_max_deg (2026-10-28) — robot.launch.py 와 같게
        out.append((max(min(a, b), math.radians(float(m.get("cable_min_deg", -math.inf)))),
                    min(max(a, b), math.radians(float(m.get("cable_max_deg", math.inf))))))
    return out


class Probe(Node):
    def __init__(self):
        super().__init__("drive_sequence")
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.rows = []
        self.odom = []      # (t, x, y, yaw) — 엔코더 · 자이로 계산값 (실측 아님)
        self.cmds = []      # (t, {관절: 명령}) — swerve_controller 가 하드웨어에 쓴 명령 (~/commands)
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(JointState, "/swerve_controller/commands", self.on_cmd, 50)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.odom.append((time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y,
                          2.0 * math.atan2(q.z, q.w)))

    def on_cmd(self, m):
        d = {j: (m.position[k] if j.startswith("steer") else m.velocity[k]) for k, j in enumerate(m.name)}
        if all(j in d for j in STEER + DRIVE):
            self.cmds.append((time.monotonic(), d))

    def on_js(self, m):
        d = dict(zip(m.name, zip(m.position, m.velocity)))
        if all(j in d for j in STEER + DRIVE):
            self.rows.append((time.monotonic(), d))

    def send(self, vx, vy, wz):
        if math.hypot(vx, vy) > MAX_LIN + 1e-9 or abs(wz) > MAX_ANG + 1e-9:   # 어떤 모드도 상한을 못 넘음
            raise ValueError(f"속도 상한 초과 ({vx}, {vy}, {wz})")
        self.last_pub = time.monotonic()
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(m)

    def run(self, secs, cmd):
        """cmd = (vx, vy, wz) 또는 None(아무것도 안 보냄)"""
        t0 = time.monotonic()
        nxt = t0
        while time.monotonic() - t0 < secs:
            if cmd is not None and time.monotonic() >= nxt:
                self.send(*cmd)
                nxt += 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
        return t0


def module_errors(rows, t_from, t_to, cmd, geo, r_wheel):
    """구간 [t_from, t_to] 에서 모듈마다 최악 속도 오차 비율 (바퀴 선속도 + ω·offset 의 바닥 속도 vs 명령의 모듈 속도). 표본 부족 = None"""
    vx, vy, wz = cmd
    sel = [d for t, d in rows if t_from <= t <= t_to]
    if len(sel) < 5:
        return None
    per = [0.0] * len(geo)
    for d in sel:
        for i, (x, y, off) in enumerate(geo):
            th, lin = d[STEER[i]][0], d[DRIVE[i]][1] * r_wheel + wz * off
            mvx, mvy = vx - wz * y, vy + wz * x
            mag = max(math.hypot(mvx, mvy), 1e-6)
            per[i] = max(per[i], math.hypot(lin * math.cos(th) - mvx, lin * math.sin(th) - mvy) / mag)
    return per


def judge(rows, t_from, t_to, cmd, geo, r_wheel, tol):
    """구간 [t_from, t_to] 상태로 ② 동작 판정 → (ok, 최악 오차 비율, 문구)"""
    per = module_errors(rows, t_from, t_to, cmd, geo, r_wheel)
    if per is None:
        return False, None, "판정 구간 상태가 5 개 미만"
    worst = max(per)
    return worst <= tol, worst, f"모듈 속도 오차 최대 {worst:.0%} (허용 {tol:.0%})"


def angle_off(steer, heading):
    """조향 steer 가 heading 방향(정 · 역회전 둘 다)에서 벗어난 각 [rad]"""
    return abs((steer - heading + math.pi / 2) % math.pi - math.pi / 2)


def stop_time(rows, t_from):
    return next((t - t_from for t, d in rows if t >= t_from and all(abs(d[j][1]) < STOP_RAD_S for j in DRIVE)), None)


def first_cmd_zero(cmds, t_from):
    """t_from 뒤 처음으로 구동 명령 넷 다 0 인 시점까지 [s]. 명령 기록이 없으면 None"""
    return next((t - t_from for t, d in cmds if t >= t_from and all(abs(d[j]) < 1e-6 for j in DRIVE)), None)


def odom_delta(odom, t0, t1):
    """odom(엔코더 · 자이로 계산) 으로 t0 → t1 이동을 t0 의 몸 기준으로: (앞 [m], 왼쪽 [m], 회전 [rad]). 없으면 None"""
    a = next((o for o in odom if o[0] >= t0), None)
    b = next((o for o in reversed(odom) if o[0] <= t1), None)
    if a is None or b is None or b[0] <= a[0]:
        return None
    dx, dy = b[1] - a[1], b[2] - a[2]
    c, s_ = math.cos(a[3]), math.sin(a[3])
    return (c * dx + s_ * dy, -s_ * dx + c * dy, (b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi)


def num(text):
    try:
        return float(text)
    except ValueError:
        return None


class Ctx:
    """모드 공통: 노드 · 기하 · 인자 · 묻기 · 삑"""

    def __init__(self, n, a, geo, r_wheel):
        self.n, self.a, self.geo, self.r = n, a, geo, r_wheel
        self.wav = None
        if a.beep:
            self.wav = os.path.join(tempfile.gettempdir(), "drive_sequence_beep.wav")
            with wave.open(self.wav, "w") as w:
                w.setnchannels(1), w.setsampwidth(2), w.setframerate(16000)
                w.writeframes(b"".join(struct.pack("<h", int(12000 * math.sin(2 * math.pi * 1000 * k / 16000)))
                                       for k in range(16000 * 3 // 10)))

    def beep(self):
        if self.wav:
            try:   # 소리는 있으면 좋은 것 — aplay 없는 컴퓨터(젯슨 등) 에서 예외로 주행 도구가 멈추면 안 됨 (10-31)
                subprocess.Popen(["aplay", "-q", self.wav], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError:
                self.wav = None

    def ask(self, prompt):
        """사람에게 묻기 (그동안 /cmd_vel 은 안 나감 → 0.3 s 뒤 컨트롤러가 정지). --yes 면 빈 답"""
        return "" if self.a.yes else input(prompt).strip()

    def headings(self, cmd):
        vx, vy, wz = cmd
        out = []
        for x, y, _ in self.geo:
            mvx, mvy = vx - wz * y, vy + wz * x
            out.append(math.atan2(mvy, mvx) if math.hypot(mvx, mvy) > 1e-9 else None)
        return out

    def align(self, cmd, timeout=8.0, hold=0.3):
        """cmd 방향으로 조향만 맞춤: 2 % 속도(≤ 1 mm/s)로 보내며 네 조향이 0.07 rad 안 · 멈춤이 0.3 s 이어질 때까지 → 0 을 hold 초.
        허용 0.07 rad (2026-10-30, 예전 0.05): 차체 방향이 1° 쯤 틀어져 있으면 정면 유지가 섞는 회전 때문에 느린 속도에서 바퀴가 ±3° 벌어짐
        (Homing 뒤 모의 대각 왕복에서 2.9° 허용을 넘어 정렬 시간 초과). 주행 동작과는 무관한 시험 도구 판정."""
        small = tuple(c * 0.02 for c in cmd)
        hs = self.headings(cmd)
        t0 = time.monotonic()
        nxt, ok_since = t0, None
        while time.monotonic() - t0 < timeout:
            if time.monotonic() >= nxt:
                self.n.send(*small)
                nxt += 0.05
            rclpy.spin_once(self.n, timeout_sec=0.01)
            if not self.n.rows:
                continue
            d = self.n.rows[-1][1]
            ok = all(h is None or (angle_off(d[j][0], h) < 0.07 and abs(d[j][1]) < 0.05) for j, h in zip(STEER, hs))
            ok_since = (ok_since or time.monotonic()) if ok else None
            if ok_since and time.monotonic() - ok_since > 0.3:
                self.n.run(hold, (0, 0, 0))
                return True
        last = self.n.rows[-1][1] if self.n.rows else None   # 2026-10-30: 원인 찾기용으로 마지막 상태를 남김
        info = "" if last is None else (" · 마지막 조향 [" + ", ".join(f"{math.degrees(last[j][0]):.1f}" for j in STEER) + "]° · 조향 속도 ["
                                        + ", ".join(f"{last[j][1]:.2f}" for j in STEER) + "] rad/s · 목표 방향 ["
                                        + ", ".join("-" if h is None else f"{math.degrees(h):.1f}" for h in hs) + "]°")
        raise RuntimeError("조향 정렬 시간 초과 — 발행 중단, 추가 구간/복귀 실행 안 함" + info)

    def steer(self):
        return [self.n.rows[-1][1][j][0] for j in STEER]

    def make_inward(self):
        """12 번(fr) 을 −180° (바퀴 안쪽) 로 (1 mm/s 이하 이동). 컨트롤러는 지금 각도에서 가까운 쪽(90° 안)만 고르므로
        먼저 전진 방향으로 맞춰 fr 을 0° 또는 −180° 로 만들고, 0° 면 −80° → −160° → 0° 방향 순서로 돌림"""
        def go(deg):
            self.align((MAX_LIN * math.cos(math.radians(deg)), MAX_LIN * math.sin(math.radians(deg)), 0))
        go(0)
        if abs(self.steer()[1] + math.pi) >= 0.1:
            for deg in (-80, -160, 0):
                go(deg)
        return abs(self.steer()[1] + math.pi) < 0.1


def segment(c, name, cmd, seconds, stop_seconds, stop_cmd=(0, 0, 0)):
    """cmd 를 seconds 동안 → stop_cmd(None = 발행 멈춤) 를 stop_seconds 동안. 판정 dict"""
    n, a = c.n, c.a
    t0 = n.run(seconds, cmd)
    t_end = time.monotonic()
    ts = n.run(stop_seconds, stop_cmd)
    seg = [(t, d) for t, d in n.rows if t0 <= t <= time.monotonic()]
    rng = c.ranges
    out_rng = sorted({STEER[i] for t, d in seg for i, (lo, hi) in enumerate(rng) if not lo - 1e-3 <= d[STEER[i]][0] <= hi + 1e-3})
    per = module_errors(n.rows, t_end - a.check_seconds, t_end, cmd, c.geo, c.r)
    st = stop_time(n.rows, ts)
    r = {"name": name, "cmd": list(cmd), "steer_in_range": not out_rng, "steer_out_of_range": out_rng,
         "module_rel_err": per, "motion_ok": per is not None and max(per) <= a.tol,
         "stop_s": st, "stop_ok": st is not None and st <= a.stop_within,
         "steer_deg_start": [round(math.degrees(seg[0][1][j][0]), 1) for j in STEER] if seg else None,
         "steer_deg_end": [round(math.degrees(seg[-1][1][j][0]), 1) for j in STEER] if seg else None,
         "odom_enc": odom_delta(n.odom, t0, time.monotonic())}
    r["ok"] = r["steer_in_range"] and r["motion_ok"] and r["stop_ok"]
    return r


def fmt_err(per):
    return "미측정" if per is None else " · ".join(f"{j.split('_')[1]} {e:.0%}" for j, e in zip(STEER, per))


# ------------------------------------------------------------------ 모드
def mode_stop(c):
    n, a = c.n, c.a
    res = c.res
    for k in range(a.reps):
        c.align((MAX_LIN, 0, 0))  # Timeout raises; never continue into a new motion.
        c.ask(f"[명령 끊김 {k + 1}/{a.reps}] 기준선에 맞췄으면 Enter → 전진 {a.run_seconds} s 뒤 발행 멈춤 (삑) ")
        n.run(a.run_seconds, (MAX_LIN, 0, 0))
        t_last = n.last_pub
        c.beep()
        n.run(3.0, None)
        enc = odom_delta(n.odom, t_last, t_last + 3.0)
        r = {"case": "명령 끊김", "rep": k + 1, "t_last_cmd": t_last,
             "cmd_zero_after_s": first_cmd_zero(n.cmds, t_last) if n.cmds else None,
             "wheel_stop_after_s": stop_time(n.rows, t_last),
             "encoder_stop_dist_m": enc[0] if enc else None,
             "wheel_rad_s_at_last_cmd": next(([round(d[j][1], 3) for j in DRIVE] for t, d in reversed(n.rows) if t <= t_last), None)}
        r["floor_stop_dist_mm"] = num(c.ask("  실측 정지 거리 [mm] (명령 끊김 = 마지막 발행(삑) 시점 위치 → 완전히 멈춘 위치, 영상 · 눈금). 모르면 Enter: "))
        r["ok"] = r["wheel_stop_after_s"] is not None and r["wheel_stop_after_s"] <= a.stop_within
        res.append(r)
        if not r["ok"]:
            return res
        print(f"{'✓' if r['ok'] else '✗'} 명령 끊김 {k + 1}: 구동 명령 0 까지 {opt(r['cmd_zero_after_s'], '%.2f s')} · "
              f"실측 바퀴 정지 {opt(r['wheel_stop_after_s'], '%.2f s')} · 엔코더 정지 거리 {opt(r['encoder_stop_dist_m'], '%.0f mm', 1000)} · "
              f"바닥 실측 {opt(r['floor_stop_dist_mm'], '%.0f mm')}", flush=True)
    if a.skip_shutdown:
        return res
    c.align((MAX_LIN, 0, 0))
    c.ask(f"[프로그램 종료] 기준선에 맞췄으면 Enter → 전진 {a.run_seconds} s 뒤 '지금 Ctrl+C' (삑) 가 나오면 터미널 A 에서 Ctrl+C ")
    n.run(a.run_seconds, (MAX_LIN, 0, 0))
    t_say = time.monotonic()
    c.beep()
    print("▶ 지금 터미널 A 에서 Ctrl+C  (전진 명령은 계속 보냄)", flush=True)
    t_lim, nxt = t_say + 20.0, t_say
    while time.monotonic() < t_lim:
        if time.monotonic() >= nxt:
            n.send(MAX_LIN, 0, 0)
            nxt += 0.05
        rclpy.spin_once(n, timeout_sec=0.01)
        if n.rows and time.monotonic() - n.rows[-1][0] > 1.0 and n.rows[-1][0] > t_say:
            break
    t_js_end = n.rows[-1][0] if n.rows else None
    gone = t_js_end is not None and time.monotonic() - t_js_end > 1.0
    last_c = next(((t, d) for t, d in reversed(n.cmds)), None)
    r = {"case": "프로그램 종료", "t_prompt": t_say, "joint_states_ended": gone,
         "joint_states_end_after_prompt_s": (t_js_end - t_say) if gone else None,
         "wheel_rad_s_last_seen": [round(n.rows[-1][1][j][1], 3) for j in DRIVE] if n.rows else None,
         "drive_cmd_last_seen": [round(last_c[1][j], 3) for j in DRIVE] if last_c else None,
         "drive_cmd_last_seen_before_js_end_s": (t_js_end - last_c[0]) if (last_c and gone) else None,
         "note": "종료 뒤 /joint_states · /swerve_controller/commands 가 없어 실제 바퀴 정지 · 거리는 영상 · 눈금으로만"}
    if not gone:
        print("✗ 20 s 안에 /joint_states 가 멈추지 않음 — 종료가 안 됐거나 다른 launch 가 있음")
    r["floor_stop_after_ctrl_c_s"] = num(c.ask("  영상: Ctrl+C 누른 순간 → 바퀴가 완전히 멈춤 [s]. 모르면 Enter: "))
    r["floor_stop_dist_mm"] = num(c.ask("  영상 · 눈금: Ctrl+C 누른 순간의 위치 → 완전히 멈춘 위치 [mm] (안내 삑 시점 아님). 모르면 Enter: "))
    r["ok"] = gone
    res.append(r)
    if not r["ok"]:
        return res
    print(f"{'✓' if gone else '✗'} 프로그램 종료: 안내 뒤 /joint_states 끊김 {opt(r['joint_states_end_after_prompt_s'], '%.2f s')} · "
          f"끊기기 전 마지막 바퀴 {r['wheel_rad_s_last_seen']} rad/s · 바닥 실측 {opt(r['floor_stop_dist_mm'], '%.0f mm')}"
          " — 이제 터미널 A 를 다시 띄워야 함", flush=True)
    return res


def mode_distance(c):
    a = c.a
    secs = a.distance / MAX_LIN
    dirs = [("전진", (MAX_LIN, 0, 0)), ("후진", (-MAX_LIN, 0, 0)), ("좌", (0, MAX_LIN, 0)), ("우", (0, -MAX_LIN, 0))]
    res = c.res
    for k in range(a.reps):
        for name, cmd in dirs:
            c.align(cmd)
            ans = c.ask(f"[{name} {k + 1}/{a.reps}] 기준선에 맞췄으면 Enter (s 건너뜀 · q 끝): ")
            if ans == "q":
                return res
            if ans == "s":
                continue
            r = segment(c, name, cmd, secs, a.stop_seconds)
            r["rep"] = k + 1
            r["cmd_dist_m"] = MAX_LIN * secs   # 가속 · 감속 램프가 같아 v·t 와 같음
            hx, hy = cmd[0] / MAX_LIN, cmd[1] / MAX_LIN
            e = r.pop("odom_enc")
            r["encoder_along_m"] = (e[0] * hx + e[1] * hy) if e else None
            r["encoder_side_m"] = (-e[0] * hy + e[1] * hx) if e else None   # + = 진행 방향 왼쪽
            r["encoder_yaw_deg"] = math.degrees(e[2]) if e else None
            r["floor_dist_mm"] = num(c.ask("  실측 이동 거리 [mm] (모르면 Enter): "))
            r["floor_side_mm"] = num(c.ask("  실측 옆 흐름 [mm] (+ = 진행 방향 왼쪽): "))
            r["floor_yaw_deg"] = num(c.ask("  실측 차체 방향 오차 [deg] (+ = 반시계): "))
            res.append(r)
            if not r["ok"]:
                return res
            print(f"{'✓' if r['ok'] else '✗'} {name} {k + 1}: 명령 {r['cmd_dist_m'] * 1000:.0f} mm · "
                  f"바닥 {opt(r['floor_dist_mm'], '%.0f mm')} 옆 {opt(r['floor_side_mm'], '%+.0f mm')} 방향 {opt(r['floor_yaw_deg'], '%+.1f°')} · "
                  f"(엔코더 {opt(r['encoder_along_m'], '%.0f mm', 1000)} 옆 {opt(r['encoder_side_m'], '%+.0f mm', 1000)} "
                  f"방향 {opt(r['encoder_yaw_deg'], '%+.1f°')}) · {fmt_err(r['module_rel_err'])}", flush=True)
    return res


def mode_transitions(c):
    a = c.a
    res = c.res
    for name, vx, vy, wz in [s for s in SEQUENCE if s[0] not in ("전진+반시계",)][1:] + [SEQUENCE[0]]:
        inward = c.make_inward()
        fr0 = c.steer()[1]
        r = segment(c, name, (vx, vy, wz), a.seconds, a.stop_seconds)
        r["inward_before"] = inward
        r["fr_deg_before"] = round(math.degrees(fr0), 1)
        per = r["module_rel_err"]
        if not inward:
            r["verdict"] = "안쪽 자세를 못 만듦 (전환 시험 아님)"
        elif per is None:
            r["verdict"] = "판정 표본 부족"
        elif per[1] > a.tol and max(per[:1] + per[2:]) <= a.tol:
            r["verdict"] = "12 번만 명령을 못 따름 — 구동 이상 의심"
        elif max(per) > a.tol:
            r["verdict"] = "여러 모듈 오차 초과"
        else:
            r["verdict"] = "안쪽 자세에서 정상 전환 · 구동"
        note = c.ask(f"  {name}: 판 · 모듈 · 케이블 간섭, 바퀴 들림 · 끌림? 없으면 Enter, 있으면 내용 (입력하면 중단): ")
        r["observed"] = note or ("미관찰 (--yes)" if a.yes else "이상 없음")
        r["ok"] = r["ok"] and inward and not note
        res.append(r)
        if not r["ok"]:
            return res
        print(f"{'✓' if r['ok'] else '✗'} {name:10s} 시작 fr {r['fr_deg_before']}° → 끝 {r['steer_deg_end']} · {fmt_err(per)} · "
              f"정지 {opt(r['stop_s'], '%.2f s')} · {r['verdict']}", flush=True)
        if note:
            print("중단: 관찰된 이상 — " + note)
            break
    return res


def mode_park(c):
    n, a = c.n, c.a
    res = c.res
    # ① 복귀
    inward = c.make_inward()
    n.run(0.5, None)   # 정렬 끝의 0 명령과 이어지지 않게 한 번 끊음 → 1 s 를 여기서부터
    s0 = c.steer()
    t0 = time.monotonic()
    n.run(7.0, (0, 0, 0))
    t_start = next((t - t0 for t, d in n.cmds if t >= t0 and abs(d["steer_fr_joint"] - s0[1]) > 0.05), None) if n.cmds else None
    t_done = next((t - t0 for t, d in n.rows if t >= t0 and all(abs(d[j][0]) < 0.03 for j in STEER)), None)
    enc = odom_delta(n.odom, t0, time.monotonic())
    wheel_max = max((abs(d[j][1]) for t, d in n.rows if t >= t0 for j in DRIVE), default=None)
    r = {"case": "복귀", "inward_before": inward, "steer_deg_before": [round(math.degrees(x), 1) for x in s0],
         "steer_deg_end": [round(math.degrees(x), 1) for x in c.steer()],
         "park_start_after_zero_s": t_start, "all_zero_after_s": t_done, "wheel_max_rad_s": wheel_max,
         "encoder_move": enc}
    r["outward_at_zero"] = c.ask("  네 바퀴가 모두 바깥쪽인가요? (y / n / 모르면 Enter): ") or None
    r["observed"] = c.ask("  간섭 · 들림 · 끌림? 없으면 Enter: ") or None
    r["ok"] = (inward and t_start is not None and t_start >= 0.95 and t_done is not None
               and r["outward_at_zero"] != "n" and not r["observed"])
    res.append(r)
    if not r["ok"]:
        return res
    print(f"{'✓' if r['ok'] else '✗'} 복귀: 안쪽 {inward} · 0 명령 뒤 복귀 시작 {opt(t_start, '%.2f s')} · 네 조향 0° {opt(t_done, '%.2f s')} · "
          f"끝 {r['steer_deg_end']} · 바퀴 최대 {opt(wheel_max, '%.2f rad/s')} (offset 보상) · "
          f"엔코더 이동 {opt(enc and math.hypot(enc[0], enc[1]), '%.1f mm', 1000)} · 바깥쪽 {r['outward_at_zero'] or '미확인'}", flush=True)
    if r["observed"]:
        return res
    # ② 0 명령 0.3 s 뒤 명령 끊김 → 복귀 시작 안 함
    inward = c.make_inward()
    n.run(0.5, None)
    s0 = c.steer()
    n.run(0.3, (0, 0, 0))
    n.run(4.0, None)
    moved = max(abs(a_ - b_) for a_, b_ in zip(c.steer(), s0))
    r = {"case": "끊김 뒤 복귀 안 함", "inward_before": inward, "steer_change_deg": round(math.degrees(moved), 1)}
    r["ok"] = inward and moved < 0.03
    res.append(r)
    if not r["ok"]:
        return res
    print(f"{'✓' if r['ok'] else '✗'} 끊김: 4 s 동안 조향 변화 {r['steer_change_deg']}°", flush=True)
    # ③ 복귀 도중 전진
    inward = c.make_inward()
    n.run(0.5, None)
    n.run(2.0, (0, 0, 0))   # 1 s 뒤 복귀 시작 → 약 1 s 동안 돌다가
    mid = c.steer()
    r = segment(c, "복귀 중 전진", (MAX_LIN, 0, 0), a.seconds, 0.9)
    n.run(0.5, None)
    r.update(case="복귀 중 새 명령", inward_before=inward, steer_deg_at_new_cmd=[round(math.degrees(x), 1) for x in mid])
    r["observed"] = c.ask("  간섭 · 들림 · 끌림? 없으면 Enter: ") or None
    r["ok"] = r["ok"] and inward and not r["observed"]
    res.append(r)
    if not r["ok"]:
        return res
    print(f"{'✓' if r['ok'] else '✗'} 복귀 중 전진: 새 명령 때 조향 {r['steer_deg_at_new_cmd']} → 끝 {r['steer_deg_end']} · "
          f"{fmt_err(r['module_rel_err'])} · 정지 {opt(r['stop_s'], '%.2f s')}", flush=True)
    return res


def opt(v, f, scale=1.0):
    return "미측정" if v is None else f % (v * scale)


def ros_param(name):
    try:
        out = subprocess.run(["ros2", "param", "get", "/swerve_controller", name], capture_output=True, text=True, timeout=15).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.split(":")[-1].strip() if "value is" in out else None


MODE_TEXT = {
    "stop": "정지 시험: 명령 끊김 {reps} 회 → 프로그램 종료 1 회 (전진 {run} s 뒤)",
    "distance": "주행 정확도: 전진 · 후진 · 좌 · 우 {dist:.2f} m 씩 {reps} 회 (끝날 때마다 바닥 실측 입력)",
    "transitions": "12 번 안쪽 자세(−180°)에서 후진 · 좌 · 우 · 대각 4 · 회전 2 · 전진 전환 (구간 {sec} s)",
    "park": "정지 후 바깥쪽 복귀: 복귀 · 끊김 뒤 복귀 안 함 · 복귀 중 전진",
}


def finish_park(n, timeout=10.0):
    """Explicit normal completion only. Fresh telemetry must confirm a settled zero pose.
    Never call from finally, interrupt, fault, or a failed test.
    """
    start = time.monotonic()
    settled = None
    while time.monotonic() - start < timeout:
        now = time.monotonic()
        if (not n.rows or not n.cmds or now - n.rows[-1][0] > 0.3
                or now - n.cmds[-1][0] > 0.3):
            return {"case": "정상 완료 정리", "ok": False, "reason": "상태/명령 통신 끊김", "elapsed_s": now-start}
        n.run(0.05, (0, 0, 0))
        now = time.monotonic()
        if now - n.rows[-1][0] > 0.3 or now - n.cmds[-1][0] > 0.3:
            return {"case": "정상 완료 정리", "ok": False, "reason": "상태/명령 통신 끊김", "elapsed_s": now-start}
        d, c = n.rows[-1][1], n.cmds[-1][1]
        ready = (all(abs(d[j][0]) < 0.03 and abs(c[j]) < 0.03 for j in STEER)
                 and all(abs(d[j][1]) < STOP_RAD_S and abs(c[j]) < STOP_RAD_S for j in DRIVE))
        settled = (settled if settled is not None else now) if ready else None
        if settled is not None and now - settled >= 0.3:
            return {"case": "정상 완료 정리", "ok": True, "elapsed_s": now-start}
    return {"case": "정상 완료 정리", "ok": False, "reason": "완료 확인 시간 초과", "elapsed_s": timeout}


def check_finish_option(a, park):
    if a.finish_park and (park != "True" or a.mode == "stop"):
        sys.exit("--finish-park 는 park_on_idle:=true 와 정상 동작 시험에서만 사용 (stop 모드 제외)")


def run_outcome(results, expected, finish_requested, completed, abort_reason=None):
    """A subset of successful rows is not a completed run."""
    failed = [r.get("name") or r.get("case") for r in results if not r.get("ok")]
    required = expected + int(finish_requested)
    if not completed or len(results) != required:
        status = "incomplete"
    elif failed:
        status = "failed"
    else:
        status = "passed"
    return {"status": status, "expected_records": required, "recorded": len(results),
            "failed_cases": failed, "abort_reason": abort_reason}


def print_outcome(outcome):
    label = {"passed": "모두 통과", "failed": "실패", "incomplete": "미완료"}[outcome["status"]]
    print(f"결과: {label} (기록 {outcome['recorded']}/{outcome['expected_records']}) · "
          f"실패 항목 {outcome['failed_cases']} · 중단 {outcome['abort_reason'] or '없음'}")


def run_mode(a, hw, ranges, geo, r_wheel):
    text = MODE_TEXT[a.mode].format(reps=a.reps, run=a.run_seconds, dist=a.distance, sec=a.seconds)
    if a.mode == "stop" and a.skip_shutdown:
        text = text.replace(" → 프로그램 종료 1 회", "")
    print(text + f". |v| ≤ {MAX_LIN} m/s · |ω| ≤ {MAX_ANG} rad/s")
    park = ros_param("park_on_idle")
    if park is None:
        sys.exit("/swerve_controller 의 park_on_idle 을 못 읽음 — robot.launch.py arm:=false 가 떠 있나?")
    check_finish_option(a, park)
    if not a.finish_park and (park == "True") != (a.mode == "park"):
        sys.exit("park 모드는 launch park_on_idle:=true, 다른 모드는 park_on_idle:=false 로 띄울 것 (지금 " + park + ")")
    if not a.yes:
        input("⚠ USB 선이 바퀴에서 떨어져 고정 · 시험 공간 확보 · 즉시 정지(Ctrl+C · 전원) 준비됐나요? Enter (중단 Ctrl+C) ")
    rclpy.init()
    n = Probe()
    n.last_pub = None
    c = Ctx(n, a, geo, r_wheel)
    c.ranges = ranges
    c.res = res = []   # 모드 함수가 여기에 바로 쌓음 → 중간에 멈춰도 그때까지 저장
    expected = {"distance": 4 * a.reps, "transitions": 10, "park": 3,
                "stop": a.reps + int(not a.skip_shutdown)}[a.mode]
    completed, abort_reason = False, None
    try:
        n.run(1.0, None)
        if not n.rows:
            sys.exit("/joint_states 가 안 옴 — robot.launch.py arm:=false 가 떠 있나?")
        if not n.cmds:
            print("⚠ /swerve_controller/commands 가 안 옴 → 구동 명령 0 시점 · 복귀 시작 시점은 미측정")
        {"stop": mode_stop, "distance": mode_distance, "transitions": mode_transitions, "park": mode_park}[a.mode](c)
        if a.finish_park and len(res) == expected and all(r.get("ok") for r in res):
            res.append(finish_park(n))
        completed = True
    except KeyboardInterrupt:
        abort_reason = "KeyboardInterrupt"
        print("사용자 중단 → 발행 중단, 컨트롤러 timeout 정지 (복귀 시작 안 함)", file=sys.stderr)
    except BaseException as exc:
        abort_reason = type(exc).__name__ + ": " + str(exc)
        raise
    finally:
        outcome = run_outcome(res, expected, a.finish_park, completed, abort_reason)
        rec = {"outcome": outcome, "mode": a.mode, "hardware_map": a.hardware_map, "ranges_deg": [[math.degrees(x) for x in r] for r in ranges],
               "args": vars(a), "results": res}
        if a.raw:
            rec["raw"] = [{"t": round(t, 4), "steer": [d[j][0] for j in STEER], "wheel": [d[j][1] for j in DRIVE]} for t, d in n.rows]
            rec["raw_cmd"] = [{"t": round(t, 4), "steer": [d[j] for j in STEER], "wheel": [d[j] for j in DRIVE]} for t, d in n.cmds]
            rec["raw_odom"] = [list(o) for o in n.odom]
        with open(a.out, "w") as output_file:
            json.dump(rec, output_file, ensure_ascii=False, indent=1)
        print(f"저장: {a.out}")
        n.destroy_node()
        rclpy.try_shutdown()
    print_outcome(outcome)
    sys.exit(0 if outcome["status"] == "passed" else 1)


def select_sequence(only):
    if only is None:
        return SEQUENCE
    selected = set(only.split(","))
    unknown = selected - {s[0] for s in SEQUENCE}
    if unknown:
        raise ValueError("알 수 없는 --only 구간: " + ", ".join(sorted(unknown)))
    return [s for s in SEQUENCE if s[0] in selected]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--finish-park", action="store_true",
                    help="정상 시험 통과 후 명시적 0 유지 + 네 조향 0/바퀴 정지 확인. park_on_idle:=true 필요; 오류/중단에서는 실행 안 함")
    ap.add_argument("--hardware-map", required=True)
    ap.add_argument("--controllers", nargs="+", default=None,
                    help="module_x · module_y · module_offset · wheel_radius 를 읽을 파일들, 뒤가 앞을 덮음 "
                         "(기본: 설치된 bringup 의 controllers.yaml + drive_only.yaml — launch arm:=false 와 같은 순서)")
    ap.add_argument("--seconds", type=float, default=6.0, help="구간 길이 (조향 180° ≈ 3.3 s 포함)")
    ap.add_argument("--check-seconds", type=float, default=1.0)
    ap.add_argument("--stop-seconds", type=float, default=2.0)
    ap.add_argument("--stop-within", type=float, default=1.0, help="0 명령 · 명령 끊김 뒤 실측 정지까지 허용 [s]")
    ap.add_argument("--tol", type=float, default=0.2)
    ap.add_argument("--only", default=None, help="쉼표로 구간 이름만 (예: 전진,후진)")
    ap.add_argument("--out", default="/tmp/drive_sequence.json")
    ap.add_argument("--yes", action="store_true", help="시작 확인 생략 (mock · fw_sim 용)")
    ap.add_argument("--raw", action="store_true", help="모든 /joint_states 표본을 JSON 에 같이 저장 (원인 분석용)")
    ap.add_argument("--mode", default="sequence", choices=["sequence", "stop", "distance", "transitions", "park"])
    ap.add_argument("--reps", type=int, default=3, help="stop 의 명령 끊김 · distance 의 방향마다 반복 횟수")
    ap.add_argument("--run-seconds", type=float, default=4.0, help="stop: 끊기 · 종료 전 전진 시간")
    ap.add_argument("--distance", type=float, default=0.30, help="distance: 한 번 이동 [m] (0.05 m/s 로 거리/속도 초)")
    ap.add_argument("--beep", action="store_true", help="stop: 발행 멈춤 · Ctrl+C 안내 때 삑 (영상 시각 맞추기)")
    ap.add_argument("--skip-shutdown", action="store_true", help="stop: 프로그램 종료 시험은 빼기")
    a = ap.parse_args()

    hw = yaml.safe_load(open(a.hardware_map))
    ranges = steer_ranges(hw)
    if ranges is None:
        sys.exit("하드웨어 지도에 조향 zero_tick 이 없음 — 범위를 모르고는 시험 안 함")
    if a.controllers is None:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("bringup") + "/config/"
        a.controllers = [share + "controllers.yaml", share + "drive_only.yaml"]
    sc = {}
    for f in a.controllers:
        sc.update(yaml.safe_load(open(f))["swerve_controller"]["ros__parameters"])
    geo = list(zip(sc["module_x"], sc["module_y"], sc["module_offset"]))
    r_wheel = float(sc["wheel_radius"])
    try:
        seq = select_sequence(a.only)
    except ValueError as exc:
        sys.exit(str(exc))
    for name, vx, vy, wz in seq:
        if math.hypot(vx, vy) > MAX_LIN + 1e-9 or abs(wz) > MAX_ANG:
            sys.exit(f"{name}: 속도 상한 초과")

    print(f"모듈 기하 (몸체 좌표): x {sc['module_x']} · y {sc['module_y']} · offset {sc['module_offset']} · 바퀴 반지름 {r_wheel}")
    print("모듈별 조향 허용 범위 [deg]: " + " · ".join(f"{j.split('_')[1]} {math.degrees(lo):+.1f}~{math.degrees(hi):+.1f}"
                                              for j, (lo, hi) in zip(STEER, ranges)))
    if a.mode in ("transitions", "park") and ros_param("steering_policy") == "coherent":
        sys.exit("기존 transitions/park 의 -180° 준비 동작은 nearest 전용. coherent 에서는 sequence 및 별도 혼합 자세 회귀를 사용할 것")
    if a.mode != "sequence":
        return run_mode(a, hw, ranges, geo, r_wheel)
    park = ros_param("park_on_idle")
    check_finish_option(a, park)
    if park == "True" and not a.finish_park:
        sys.exit("sequence 는 park_on_idle:=false 로 띄울 것 (정지 2 s 동안 복귀가 끼어듦)")
    print("순서: " + " → ".join(s[0] for s in seq) + f" → 명령 끊김. 구간 {a.seconds} s + 정지 {a.stop_seconds} s, "
          f"|v| ≤ {MAX_LIN} m/s · |ω| ≤ {MAX_ANG} rad/s")
    if not a.yes:
        input("⚠ 바퀴를 띄웠거나 시험 공간을 비웠고, USB 선 고정 · 전원을 끌 사람이 있나요? 진행하려면 Enter (중단 Ctrl+C) ")

    rclpy.init()
    n = Probe()
    results = []
    completed, abort_reason = False, None
    try:
        n.run(1.0, None)
        if not n.rows:
            sys.exit("/joint_states 가 안 옴 — robot.launch.py arm:=false 가 떠 있나?")
        for name, vx, vy, wz in seq + [("명령 끊김", 0.05, 0, 0)]:
            t0 = n.run(a.seconds if name != "명령 끊김" else 2.0, (vx, vy, wz))
            t_end = time.monotonic()
            ts = n.run(a.stop_seconds, None if name == "명령 끊김" else (0, 0, 0))
            seg = [(t, d) for t, d in n.rows if t0 <= t <= time.monotonic()]
            out_rng = sorted({STEER[i] for t, d in seg for i, (lo, hi) in enumerate(ranges)
                              if not lo - 1e-3 <= d[STEER[i]][0] <= hi + 1e-3})
            ok_m, worst, why = (True, None, "끊김 구간은 동작 판정 안 함") if name == "명령 끊김" else \
                judge(n.rows, t_end - a.check_seconds, t_end, (vx, vy, wz), geo, r_wheel, a.tol)
            stop_t = stop_time(n.rows, ts)
            ok_s = stop_t is not None and stop_t <= a.stop_within
            r = {"name": name, "cmd": [vx, vy, wz], "steer_in_range": not out_rng, "steer_out_of_range": out_rng,
                 "motion_ok": ok_m, "motion_worst_rel_err": worst, "motion_note": why,
                 "stop_s": stop_t, "stop_ok": ok_s,
                 "steer_deg_end": [round(math.degrees(seg[-1][1][j][0]), 1) for j in STEER] if seg else None,
                 "wheel_rad_s_end_of_motion": [round(d[j][1], 3) for t, d in seg if t <= t_end for j in DRIVE][-4:]}
            r["ok"] = r["steer_in_range"] and ok_m and ok_s
            results.append(r)
            print(f"{'✓' if r['ok'] else '✗'} {name:10s} 조향 범위 {'안' if not out_rng else '밖 ' + str(out_rng)} · {why} · "
                  f"정지 {('%.2f s' % stop_t) if stop_t is not None else '확인 안 됨'} · 끝 조향 {r['steer_deg_end']}", flush=True)
            if not r["ok"]:
                break  # No further steering commands after a failed segment.
        if a.finish_park and len(results) == len(seq) + 1 and all(r["ok"] for r in results):
            results.append(dict(name="정상 완료 정리", **finish_park(n)))
        completed = True
    except KeyboardInterrupt:
        abort_reason = "KeyboardInterrupt"
        print("사용자 중단 → 발행 중단, 컨트롤러 timeout 정지 (복귀 시작 안 함)", file=sys.stderr)
    except BaseException as exc:
        abort_reason = type(exc).__name__ + ": " + str(exc)
        raise
    finally:
        outcome = run_outcome(results, len(seq) + 1, a.finish_park, completed, abort_reason)
        rec = {"outcome": outcome, "hardware_map": a.hardware_map, "ranges_deg": [[math.degrees(x) for x in r] for r in ranges], "results": results}
        if a.raw:
            rec["raw"] = [{"t": round(t, 4), "steer": [d[j][0] for j in STEER], "wheel": [d[j][1] for j in DRIVE]} for t, d in n.rows]
        with open(a.out, "w") as output_file:
            json.dump(rec, output_file, ensure_ascii=False, indent=1)
        print(f"저장: {a.out}")
        n.destroy_node()
        rclpy.shutdown()
    print_outcome(outcome)
    sys.exit(0 if outcome["status"] == "passed" else 1)


if __name__ == "__main__":
    main()
