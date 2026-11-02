#!/usr/bin/env python3
"""여러 모양 경로 만들고 따라가기 (2026-10-28). path_follow_2m.py(직진, 실물 2회 확인)를 구간 여러 개로 넓힌 것.
run_shape.sh 가 run_drive.sh(NT_TOOL=shape) 로 띄움.

모양 (--shape, 시작 순간 odom 자세 기준: 앞 = +x, 왼쪽 = +y, 차체 방향은 계속 시작 방향 유지)
  line  앞 L            back  뒤 L            left  왼쪽 L          right  오른쪽 L
  diag  앞왼 대각 L     L     앞 L → 왼쪽 L (꺾을 때 멈춤)          square  L × L 사각형 한 바퀴 (앞 → 왼 → 뒤 → 오른)
  rot90 · rot180 · rot-90  제자리에서 자이로 보며 회전 (닫힌 고리)
  star  전방향: 0° 부터 --step-deg(기본 10°) 씩 방향마다 L 갔다가 출발점으로 돌아옴 (10° 면 36 방향 · 72 구간, L 0.5 m 에 약 10 분)
        끝에 방향별 표 · 그림(<out>_star.png). 각도: 0° = 앞, 90° = 왼쪽, 180° = 뒤, 270° = 오른쪽
  repeat  한 방향(--deg)으로 L 갔다가 돌아오기를 --minutes(기본 10 분) 동안 계속 (시간이 되면 하던 왕복만 마치고 끝)
        왕복마다 돌아온 위치의 odom 오차 기록 → 표 · 그림(<out>_repeat.png). 목표가 시작 자세 기준 고정이라 odom 오차는 쌓이지 않음
        (바닥 미끄러짐 누적은 줄자로). 36 방향 왕복 실험은 dir_calib.py (run_dirs.sh)
  (L = --length, 기본 line · back 2.0 m, star · repeat 0.5 m, 나머지 1.0 m)
둥근 경로 (2026-10-29, smooth_path.py): --round R [m] 이면 이동 모양의 모서리를 원호로 (꺾임 150° 넘으면 멈춤), --waypoints "x,y;x,y" 로 임의 경로.
  원호 속도 ≤ √(a_lat·R) · w_dir·R, 끝 · 원호 앞 미리 감속, 50 Hz. 모의 사각형 1 m: 0.10 m/s 53.8 → 43.5 s · 0.20 m/s 38.6 → 26.0 s
  ⚠ 이동 방향이 바퀴 범위 끝을 지나는 원호(예 90° → 180°, rl 선 한계 170°)는 도중에 바퀴가 뒤집혀 잠깐 섬 → 옆 오차 최대 약 12 mm
직선 구간 추종 (10-28 실물 2 m 직진 2 회 확인한 식): 경로 방향 속도(끝 0.3 m 감속, 최저 0.02 m/s) + 옆 되돌림 −k_e·옆 오차(≤ 0.3·속도, 2 mm 무시)
  + 방향 유지 k_ψ·방향 오차(≤ 0.2 rad/s, 0.5° 무시) → 몸 좌표 /cmd_vel. 구간 끝(남은 ≤ 5 mm)마다 0 명령 0.5 s 로 멈춘 뒤 다음 구간.
회전: wz = k_rot·남은 각(0.05~0.6 rad/s), 남은 각 ≤ 0.5° 면 끝.
출력: 구간마다 끝 오차 · 최대 옆 오차 · 걸린 시간, 마지막에 목표점 · 출발점 대비 오차 → --out JSON, 경로 /path_follow/path
⚠ --frame odom(기본): 바퀴 + 자이로 기준 — 바퀴가 못 느끼는 옆 미끄러짐(카펫, 10-30 36 방향 뒤 1~2 m)은 못 고침.
지도 좌표 (2026-10-31, p2_main tools/ 로 옮김): --frame map 이면 위치 = SLAM 의 map→odom (slam_toolbox, 스캔마다 갱신) ∘ 최신 /odom (50 Hz, controllers.yaml odom_rate_hz)
  → 추종 주기는 그대로 (SLAM 갱신을 기다리지 않음), 밀림은 SLAM 이 map→odom 을 고칠 때마다 되돌림. map→odom 이 1 s 넘게 안 오면 0 명령 · 끝.
  --abs: --waypoints 를 그 좌표계(map/odom) 의 절대 좌표로 (미션 L 박스 앞 등). 없으면 예전처럼 시작 자세 기준.
위치 감시 (2026-11-01, tools/jetson/pose_guard.py 의 /localization_health): 오면 따름 — 경고 = 속도 반, 정지 = 0 명령으로 기다림 (10 s 넘으면 끝).
  판정이 오다가 1 s 끊기면 정지로 봄. --guard 면 판정이 처음부터 안 와도 출발 안 함.
앞 물체 정지 (2026-11-01, 실물: 사람이 경로에 들어가도 안 멈춤): /scan(몸 기준) 에서 이번 주기에 보낼 방향으로 몸 끝 + --stop-margin 안,
  몸 폭 + --side-margin 안에 점 → 0 명령으로 기다림, 치우면 다시 감 (30 s 넘으면 끝). 제자리 회전이면 몸 모서리 원 + 1 cm 안.
  몸 모양 · 검사 식은 body_footprint.py (sensor_check · wall_follow 와 같이 씀).
  /scan 을 한 번도 못 받으면 출발 안 함 (--no-stop 일 때만 검사 없이).
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # 같은 폴더 (p2_main tools/) 의 drive_sequence · smooth_path
import numpy as np            # noqa: E402
import drive_sequence as ds   # noqa: E402
import smooth_path as sp      # noqa: E402
from body_footprint import obstacle_hit, self_mask   # noqa: E402
import rclpy                  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus   # noqa: E402
from geometry_msgs.msg import PoseStamped   # noqa: E402
from sensor_msgs.msg import LaserScan   # noqa: E402
from rclpy.qos import qos_profile_sensor_data   # noqa: E402
from nav_msgs.msg import Path   # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile   # noqa: E402

SHAPES = ("line", "back", "left", "right", "diag", "L", "square", "rot90", "rot180", "rot-90", "star", "repeat")


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def plan(shape, L, step_deg=10.0, deg=0.0):
    """구간 목록 (시작 자세 좌표): ("move", (x, y)) 또는 ("rot", 각도 rad)"""
    if shape == "repeat":   # 시간으로 끊으므로 넉넉히 (왕복 1000 번)
        t = math.radians(deg)
        return [("move", (L * math.cos(t), L * math.sin(t))), ("move", (0.0, 0.0))] * 1000
    if shape == "star":
        out = []
        for k in range(int(round(360.0 / step_deg))):
            t = math.radians(k * step_deg)
            out += [("move", (L * math.cos(t), L * math.sin(t))), ("move", (0.0, 0.0))]
        return out
    d = L / math.sqrt(2)
    return {"line": [("move", (L, 0))], "back": [("move", (-L, 0))], "left": [("move", (0, L))],
            "right": [("move", (0, -L))], "diag": [("move", (d, d))], "L": [("move", (L, 0)), ("move", (L, L))],
            "square": [("move", (L, 0)), ("move", (L, L)), ("move", (0, L)), ("move", (0, 0))],
            "rot90": [("rot", math.pi / 2)], "rot180": [("rot", math.pi)], "rot-90": [("rot", -math.pi / 2)]}[shape]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hardware-map", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--raw", action="store_true")
    ap.add_argument("--yes", action="store_true", help="모의 전용: 시작 확인 생략")
    ap.add_argument("--shape", default="line", choices=SHAPES)
    ap.add_argument("--length", type=float, default=None, help="구간 길이 [m] (기본 line · back 2.0, star · repeat 0.5, 나머지 1.0)")
    ap.add_argument("--speed", type=float, default=0.10, help="최고 속도 [m/s] (0.03~0.26)")
    ap.add_argument("--step-deg", type=float, default=10.0, help="star 방향 간격 [deg] (5~90)")
    ap.add_argument("--deg", type=float, default=0.0, help="repeat 방향 [deg] (0 앞, 90 왼, 180 뒤, 270 오른)")
    ap.add_argument("--minutes", type=float, default=10.0, help="repeat 시간 [분] (0.5~30)")
    ap.add_argument("--k-e", type=float, default=1.0)
    ap.add_argument("--k-psi", type=float, default=1.5)
    ap.add_argument("--k-rot", type=float, default=1.5)
    # 2026-10-29 둥근 경로 (smooth_path.py): 주면 모서리를 원호로 이어 멈추지 않고 지나감 (꺾임 150° 넘으면 거기서 멈춤)
    ap.add_argument("--round", type=float, default=0.0, help="모서리 반지름 [m] (0 = 예전처럼 모서리마다 멈춤, 0.05~0.5)")
    ap.add_argument("--waypoints", default="", help='경유점 "x,y;x,y;..." [m] 시작 자세 기준 (앞 +x · 왼 +y, 시작점 자동). 주면 --shape 대신')
    ap.add_argument("--a-lat", type=float, default=0.5, help="둥근 경로: 옆 가속 상한 [m/s²]")
    ap.add_argument("--w-dir", type=float, default=2.0, help="둥근 경로: 이동 방향이 도는 빠르기 상한 [rad/s]")
    ap.add_argument("--hz", type=float, default=50.0, help="둥근 경로 추종 주기 [Hz] (직선 구간 추종은 예전 20 Hz)")
    ap.add_argument("--frame", default="odom", choices=("odom", "map"), help="위치 좌표계 (map = SLAM 위치, slam · locate 떠 있을 때)")
    ap.add_argument("--stop-margin", type=float, default=0.09, help="앞 물체 정지: 가는 방향 몸 끝에서 [m]")
    ap.add_argument("--side-margin", type=float, default=0.0, help="앞 물체 정지: 몸 폭 바깥 여유 [m] (벽 27 cm 옆 경로라 0)")
    ap.add_argument("--no-stop", action="store_true", help="앞 물체 정지 끔")
    ap.add_argument("--guard", action="store_true", help="pose_guard 판정(/localization_health) 이 꼭 있어야 움직임")
    ap.add_argument("--abs", action="store_true", help="--waypoints 가 --frame 의 절대 좌표 (시작 자세 기준 아님)")
    a = ap.parse_args()
    if a.yes and os.environ.get("NT_HW", "opencr") != "mock":
        ap.error("--yes 는 모의(NT_HW=mock)에서만")
    L = a.length if a.length is not None else (2.0 if a.shape in ("line", "back") else 0.5 if a.shape in ("star", "repeat") else 1.0)
    if not 0.5 <= a.minutes <= 30:
        sys.exit("--minutes 0.5~30")
    if not 5.0 <= a.step_deg <= 90.0:
        sys.exit("--step-deg 5~90")
    if not (0.2 <= L <= 3.0 and 0.03 <= a.speed <= 0.26):
        sys.exit("--length 0.2~3 m, --speed 0.03~0.26 m/s")
    smooth = a.round > 0 or bool(a.waypoints)
    if smooth and not (0.0 <= a.round <= 0.5 and 0.1 <= a.a_lat <= 1.0 and 0.5 <= a.w_dir <= 4.5 and 10 <= a.hz <= 100):
        sys.exit("--round 0~0.5 m · --a-lat 0.1~1 · --w-dir 0.5~4.5 · --hz 10~100")
    if smooth and not a.waypoints and a.shape in ("star", "repeat", "rot90", "rot180", "rot-90"):
        sys.exit("둥근 경로는 이동 모양(line back left right diag L square) 또는 --waypoints 만")
    try:
        wps = [tuple(float(v) for v in p.split(",")) for p in a.waypoints.split(";") if p.strip()]
    except ValueError:
        sys.exit('--waypoints 형식: "x,y;x,y"')
    if any(len(p) != 2 or max(abs(p[0]), abs(p[1])) > 7.0 for p in wps):
        sys.exit("--waypoints: 점마다 x,y 두 숫자, 각 7 m 이내")
    if a.abs and not wps:
        sys.exit("--abs 는 --waypoints 와 같이")
    ds.MAX_LIN, ds.MAX_ANG = max(ds.MAX_LIN, a.speed * 1.31), max(ds.MAX_ANG, 0.6)

    rclpy.init()
    n = ds.Probe()
    if a.frame == "map":
        import tf2_ros
        from rclpy.time import Time
        tf_buf = tf2_ros.Buffer()
        tf_lis = tf2_ros.TransformListener(tf_buf, n, spin_thread=False)   # noqa: F841 — 아래 spin_once 가 /tf 를 받음
    path_pub = n.create_publisher(Path, "/path_follow/path", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    health, hold = {"t": None, "level": 0, "msg": ""}, {"t": None, "why": ""}
    scan = {"t": None, "p": None}

    def on_scan(m):
        r = np.asarray(m.ranges, float)
        ang = m.angle_min + m.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        pts = np.c_[r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])]
        scan.update(t=time.monotonic(), p=self_mask(pts))

    n.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)

    def blocked(now, cmd, remain=None):
        """이번 주기에 보낼 몸 속도 cmd 로 가면 닿을 물체 (사람 · 선 · 박스). 스캔을 아직 못 받았거나 0.5 s 끊기면 막힘"""
        if a.no_stop:
            return ""
        if scan["t"] is None:
            return "스캔 기다림 (/scan — 라이다 · scan_merger 떠 있나?)"
        if now - scan["t"] > 0.5:
            return "스캔 끊김"
        d = obstacle_hit(scan["p"], *cmd, remain=remain, margin=a.stop_margin, side=a.side_margin)
        return "" if d is None else f"앞에 물체 {d:.2f} m"

    def on_health(m):
        lv = m.level if isinstance(m.level, int) else m.level[0]
        if lv != health["level"]:
            print(f"\n  ■ 위치 감시: {['정상', '경고 (속도 반)', '정지'][min(lv, 2)]} — {m.message}", flush=True)
        health.update(t=time.monotonic(), level=lv, msg=m.message)

    n.create_subscription(DiagnosticStatus, "/localization_health", on_health, 10)

    def guard_step(now, t0, cmd, remain=None):
        """위치 감시 판정 → (속도 배율, 고친 t0, 끝낼 이유). 정지면 0 명령 보내고 기다림 — 기다린 시간은 구간 시간 초과에서 뺌"""
        if health["t"] is None:
            sc = 0.0 if a.guard else 1.0
        elif now - health["t"] > 1.0:
            sc = 0.0
        else:
            sc = (1.0, 0.5, 0.0)[min(health["level"], 2)]
        why = "" if sc > 0 else f"위치 감시 정지 ({health['msg'] or '판정 안 옴 — pose_guard 떠 있나?'})"
        obj = blocked(now, cmd, remain)
        if obj:
            sc, why = 0.0, obj
        if sc > 0:
            if hold["t"] is not None:
                t0 += now - hold["t"]; hold["t"] = None
                print("\n  ■ 다시 감", flush=True)
            return sc, t0, None
        if hold["t"] is None or why != hold["why"]:
            print(f"\n  ■ 멈춤 — 기다림: {why}", flush=True)
            hold["t"] = hold["t"] or now; hold["why"] = why
        n.send(0.0, 0.0, 0.0)
        return 0.0, t0, (why if now - hold["t"] > (30.0 if obj else 10.0) else None)
    log, segs_out, result = [], [], {"time": time.strftime("%Y-%m-%dT%H:%M:%S"), "args": vars(a), "length": L}

    def spin_for(sec):   # 쌓인 메시지를 다 처리 → 늘 최신 odom
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.005)

    def pose():
        _, x, y, yaw = n.odom[-1]
        if a.frame == "odom":
            return x, y, yaw
        try:   # map→odom (가장 최근) ∘ odom 자세
            t = tf_buf.lookup_transform("map", "odom", Time())
        except Exception as ex:
            raise RuntimeError(f"map→odom 없음 ({type(ex).__name__}) — slam · locate 가 떠 있나?")
        st = Time.from_msg(t.header.stamp)
        age = 0.0 if st.nanoseconds == 0 else (n.get_clock().now() - st).nanoseconds * 1e-9   # 시각 0 = 정적 TF (시험 · 고정) → 끊김 검사 안 함
        if age > 1.0:
            raise RuntimeError(f"map→odom 끊김 ({age:.1f} s)")
        q, p = t.transform.rotation, t.transform.translation
        ym = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s_ = math.cos(ym), math.sin(ym)
        return p.x + c * x - s_ * y, p.y + s_ * x + c * y, wrap(ym + yaw)

    try:
        spin_for(1.0)
        if not n.odom:
            sys.exit("/odom 이 안 옴 — launch 가 떠 있나?")
        x0, y0, yaw0 = pose()
        if a.abs:   # 절대 좌표 경유점 → 시작 자세 기준으로 (아래 계획 · 표시는 시작 자세 기준)
            c0_, s0_ = math.cos(-yaw0), math.sin(-yaw0)
            wps = [(c0_ * (px - x0) - s0_ * (py - y0), s0_ * (px - x0) + c0_ * (py - y0)) for px, py in wps]
        c0, s0 = math.cos(yaw0), math.sin(yaw0)
        to_world = lambda px, py: (x0 + c0 * px - s0 * py, y0 + s0 * px + c0 * py)
        segments = plan(a.shape, L, a.step_deg, a.deg)
        if smooth:   # 경유점 → 둥근 조각들 ("path", (시작 자세 좌표 점들, odom 좌표 점들))
            wps = [(0.0, 0.0)] + (wps or [t for k, t in segments if k == "move"])
            try:
                pieces = sp.round_corners(wps, a.round)
            except ValueError as ex:
                sys.exit(f"경로: {ex}")
            segments = [("path", (pc, [(*to_world(x, y), r) for x, y, r in pc])) for pc in pieces]
        # 경로 표시 (회전만이면 시작점 하나)
        msg = Path()
        msg.header.frame_id = a.frame
        msg.header.stamp = n.get_clock().now().to_msg()
        prev = (0.0, 0.0)
        pts = [to_world(0, 0)]
        for kind, tgt in (segments[:2] if a.shape == "repeat" else segments):
            if kind == "move":
                seg_len = math.hypot(tgt[0] - prev[0], tgt[1] - prev[1])
                k = max(1, int(seg_len / 0.05))
                pts += [to_world(prev[0] + (tgt[0] - prev[0]) * i / k, prev[1] + (tgt[1] - prev[1]) * i / k) for i in range(1, k + 1)]
                prev = tgt
            elif kind == "path":
                pts += [(x, y) for x, y, _ in tgt[1][::5]] + [tgt[1][-1][:2]]
        for px, py in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = px, py
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw0 / 2), math.cos(yaw0 / 2)
            msg.poses.append(ps)
        path_pub.publish(msg)
        desc = (f"{len(segments) // 2} 방향 × ({L:.2f} m 갔다가 돌아옴), {a.step_deg:g}° 간격" if a.shape == "star" else
                f"{a.deg:g}° 방향으로 {L:.2f} m 왔다 갔다 {a.minutes:g} 분" if a.shape == "repeat" else
                f"경유점 {' → '.join(f'({x:+.2f}, {y:+.2f})' for x, y in wps)} · 모서리 R {a.round:g} m · 조각 {len(segments)}개 · {a.hz:g} Hz" if smooth else None) or " → ".join(f"({t[0]:+.2f}, {t[1]:+.2f})" if k == "move" else f"회전 {math.degrees(t):+.0f}°" for k, t in segments)
        print(f"시작 위치 ({a.frame}): x {x0:+.3f} · y {y0:+.3f} m · 방향 {math.degrees(yaw0):+.1f}°")
        print(f"모양 {'경유점' if a.waypoints else a.shape} (구간 {len(segments)}개, 시작 기준 앞 +x · 왼 +y): {desc}")
        print(f"추종: 최고 {a.speed:.2f} m/s · 옆 되돌림 {a.k_e} · 방향 되돌림 {a.k_psi} · 회전 {a.k_rot} /s")
        if not a.yes:
            input("경로 주변 30 cm 이상 비었는지 확인 후 Enter (중단 Ctrl+C) ")
        result["start"] = [x0, y0, math.degrees(yaw0)]
        t_all = time.monotonic()
        prev, yaw_target = (0.0, 0.0), yaw0
        for idx, (kind, tgt) in enumerate(segments, 1):
            t0 = time.monotonic()
            why, max_e, last_print = "도착", 0.0, 0.0
            if kind == "move":
                ax, ay = to_world(*prev)
                bx, by = to_world(*tgt)
                seg_len = math.hypot(bx - ax, by - ay)
                tx, ty = (bx - ax) / seg_len, (by - ay) / seg_len
                nx, ny = -ty, tx
                timeout = seg_len / a.speed * 3 + 5
                print(f"\n[구간 {idx}] 직선 {seg_len:.2f} m → 시작 기준 ({tgt[0]:+.2f}, {tgt[1]:+.2f})")
                while True:
                    spin_for(0.05)
                    now = time.monotonic()
                    if not n.odom or now - n.odom[-1][0] > 0.5:
                        why = "odom 끊김"
                        break
                    x, y, yaw = pose()
                    s = (x - ax) * tx + (y - ay) * ty
                    e = (x - ax) * nx + (y - ay) * ny
                    epsi = wrap(yaw_target - yaw)
                    remain = seg_len - s
                    max_e = max(max_e, abs(e))
                    if remain <= 0.005:
                        break
                    v = max(0.02, min(a.speed, 0.6 * remain))
                    vc = 0.0 if abs(e) < 0.002 else max(-0.3 * v, min(0.3 * v, -a.k_e * e))
                    wz = 0.0 if abs(epsi) < math.radians(0.5) else max(-0.2, min(0.2, a.k_psi * epsi))
                    wx, wy = v * tx + vc * nx, v * ty + vc * ny
                    c, sn = math.cos(yaw), math.sin(yaw)
                    bvx, bvy = c * wx + sn * wy, -sn * wx + c * wy
                    sc, t0, gwhy = guard_step(now, t0, (bvx, bvy, wz), remain)   # 이번에 보낼 방향으로 검사 (11-02 GPT I6)
                    if gwhy:
                        why = gwhy
                        break
                    if sc == 0.0:
                        continue
                    if now - t0 > timeout:
                        why = "시간 초과"
                        break
                    bvx, bvy, wz = sc * bvx, sc * bvy, sc * wz
                    n.send(bvx, bvy, wz)
                    log.append({"seg": idx, "t": round(now - t_all, 3), "s": round(s, 4), "e_mm": round(e * 1000, 1),
                                "epsi_deg": round(math.degrees(epsi), 2), "cmd": [round(bvx, 4), round(bvy, 4), round(wz, 4)]})
                    if now - last_print > 0.5:
                        last_print = now
                        print(f"  {now - t_all:5.1f} s · 진행 {s:5.3f}/{seg_len:.2f} m · 옆 오차 {e * 1000:+6.1f} mm · "
                              f"방향 오차 {math.degrees(epsi):+5.2f}°", flush=True)
                n.run(0.5, (0.0, 0.0, 0.0))
                x, y, yaw = pose()
                s, e = (x - ax) * tx + (y - ay) * ty, (x - ax) * nx + (y - ay) * ny
                r = {"seg": idx, "kind": "move", "outcome": why, "len_m": round(seg_len, 3), "along_err_mm": round((s - seg_len) * 1000, 1),
                     "cross_mm": round(e * 1000, 1), "max_cross_mm": round(max_e * 1000, 1),
                     "heading_err_deg": round(math.degrees(wrap(yaw_target - yaw)), 2), "secs": round(time.monotonic() - t0, 2),
                     "pos_from_start_mm": [round((math.cos(yaw0) * (x - x0) + math.sin(yaw0) * (y - y0)) * 1000, 1),
                                           round((-math.sin(yaw0) * (x - x0) + math.cos(yaw0) * (y - y0)) * 1000, 1)],
                     "t_end": round(time.monotonic() - t_all, 1)}
                prev = tgt
            elif kind == "path":   # 둥근 경로 (2026-10-29): 경로 위 가장 가까운 점 · 접선 기준, 속도는 곡률 · 끝 감속 계획
                pc_s, pc = tgt
                vplan = sp.plan_speed(pc, a.speed, a.a_lat, a.w_dir)
                s_cum = sp.arc_lengths(pc)
                total, hint = s_cum[-1], 0
                timeout = sum((s_cum[k + 1] - s_cum[k]) / vplan[k + 1] for k in range(len(pc) - 1)) * 3 + 10   # 계획 시간 (감속 포함) 기준
                print(f"\n[구간 {idx}] 둥근 경로 {total:.2f} m (점 {len(pc)}개, 계획 최저 {min(vplan[:-1] or vplan):.3f} m/s)")
                while True:
                    spin_for(1.0 / a.hz)
                    now = time.monotonic()
                    if not n.odom or now - n.odom[-1][0] > 0.5:
                        why = "odom 끊김"
                        break
                    x, y, yaw = pose()
                    i, tx, ty, e, d_along = sp.locate(pc, x, y, hint)
                    hint = i
                    remain = total - (s_cum[i] + d_along)
                    epsi = wrap(yaw_target - yaw)
                    max_e = max(max_e, abs(e))
                    if remain <= 0.005 and i >= len(pc) - 3:
                        break
                    v = max(0.02, min(min(vplan[i:i + 6]), 0.6 * remain))
                    vc = 0.0 if abs(e) < 0.002 else max(-0.3 * v, min(0.3 * v, -a.k_e * e))
                    wz = 0.0 if abs(epsi) < math.radians(0.5) else max(-0.2, min(0.2, a.k_psi * epsi))
                    wx, wy = v * tx - vc * ty, v * ty + vc * tx
                    c, sn = math.cos(yaw), math.sin(yaw)
                    bvx, bvy = c * wx + sn * wy, -sn * wx + c * wy
                    sc, t0, gwhy = guard_step(now, t0, (bvx, bvy, wz), remain)   # 이번에 보낼 방향으로 검사 (11-02 GPT I6)
                    if gwhy:
                        why = gwhy
                        break
                    if sc == 0.0:
                        continue
                    if now - t0 > timeout:
                        why = "시간 초과"
                        break
                    bvx, bvy, wz = sc * bvx, sc * bvy, sc * wz
                    n.send(bvx, bvy, wz)
                    log.append({"seg": idx, "t": round(now - t_all, 3), "s": round(total - remain, 4), "e_mm": round(e * 1000, 1),
                                "epsi_deg": round(math.degrees(epsi), 2), "v": round(v, 3), "cmd": [round(bvx, 4), round(bvy, 4), round(wz, 4)]})
                    if now - last_print > 0.5:
                        last_print = now
                        print(f"  {now - t_all:5.1f} s · 진행 {total - remain:5.3f}/{total:.2f} m · 속도 {v:.3f} · 옆 오차 {e * 1000:+6.1f} mm · "
                              f"방향 오차 {math.degrees(epsi):+5.2f}°", flush=True)
                n.run(0.5, (0.0, 0.0, 0.0))
                x, y, yaw = pose()
                ex, ey = pc[-1][0], pc[-1][1]
                _, tx, ty, _, _ = sp.locate(pc, ex, ey, len(pc) - 2)
                r = {"seg": idx, "kind": "path", "outcome": why, "len_m": round(total, 3),
                     "along_err_mm": round(((x - ex) * tx + (y - ey) * ty) * 1000, 1), "cross_mm": round((-(x - ex) * ty + (y - ey) * tx) * 1000, 1),
                     "max_cross_mm": round(max_e * 1000, 1), "heading_err_deg": round(math.degrees(wrap(yaw_target - yaw)), 2),
                     "secs": round(time.monotonic() - t0, 2),
                     "pos_from_start_mm": [round((math.cos(yaw0) * (x - x0) + math.sin(yaw0) * (y - y0)) * 1000, 1),
                                           round((-math.sin(yaw0) * (x - x0) + math.cos(yaw0) * (y - y0)) * 1000, 1)],
                     "t_end": round(time.monotonic() - t_all, 1)}
                prev = pc_s[-1][:2]
            else:
                yaw_target = wrap(yaw_target + tgt)
                print(f"\n[구간 {idx}] 제자리 회전 {math.degrees(tgt):+.0f}° → 방향 {math.degrees(yaw_target):+.1f}°")
                timeout = abs(tgt) / 0.3 * 3 + 5
                while True:
                    spin_for(0.05)
                    now = time.monotonic()
                    if not n.odom or now - n.odom[-1][0] > 0.5:
                        why = "odom 끊김"
                        break
                    _, _, yaw = pose()
                    err = wrap(yaw_target - yaw)
                    if abs(err) <= math.radians(0.5):
                        break
                    wz = math.copysign(max(0.05, min(0.6, a.k_rot * abs(err))), err)
                    sc, t0, gwhy = guard_step(now, t0, (0.0, 0.0, wz))
                    if gwhy:
                        why = gwhy
                        break
                    if sc == 0.0:
                        continue
                    if now - t0 > timeout:
                        why = "시간 초과"
                        break
                    wz *= sc
                    n.send(0.0, 0.0, wz)
                    log.append({"seg": idx, "t": round(now - t_all, 3), "err_deg": round(math.degrees(err), 2), "cmd": [0, 0, round(wz, 4)]})
                    if now - last_print > 0.5:
                        last_print = now
                        print(f"  {now - t_all:5.1f} s · 남은 각 {math.degrees(err):+6.1f}° · 회전 명령 {wz:+.2f} rad/s", flush=True)
                n.run(0.5, (0.0, 0.0, 0.0))
                _, _, yaw = pose()
                r = {"seg": idx, "kind": "rot", "outcome": why, "cmd_deg": round(math.degrees(tgt), 1),
                     "heading_err_deg": round(math.degrees(wrap(yaw_target - yaw)), 2), "secs": round(time.monotonic() - t0, 2)}
            segs_out.append(r)
            print(f"  → {r['outcome']} · " + " · ".join(f"{k} {v}" for k, v in r.items() if k not in ("seg", "kind", "outcome")), flush=True)
            if why != "도착":
                break
            if a.shape == "repeat" and idx % 2 == 0:
                n.rows[:] = n.rows[-5:]   # 관절 100 Hz 기록이 30 분이면 수십만 개 → 왕복마다 비움 (odom 은 --raw 용으로 둠)
                n.cmds[:] = n.cmds[-5:]
                el = time.monotonic() - t_all
                print(f"  ■ 왕복 {idx // 2} 끝 · {el / 60:.1f}/{a.minutes:g} 분 · 출발점에서 앞 {r['pos_from_start_mm'][0]:+.1f} · 왼 {r['pos_from_start_mm'][1]:+.1f} mm", flush=True)
                if el >= a.minutes * 60:
                    break
        # 최종: 마지막 목표점 · 출발점 대비 (시작 자세 좌표)
        x, y, yaw = pose()
        dx, dy = x - x0, y - y0
        fx, fy = c0 * dx + s0 * dy, -s0 * dx + c0 * dy
        goal = prev
        done = len(segs_out) == len(segments) or (a.shape == "repeat" and len(segs_out) % 2 == 0 and len(segs_out) > 0)
        result.update(outcome=all(r["outcome"] == "도착" for r in segs_out) and done and "도착" or "중단",
                      segments=segs_out, end_from_start_mm=[round(fx * 1000, 1), round(fy * 1000, 1)],
                      end_err_mm=[round((fx - goal[0]) * 1000, 1), round((fy - goal[1]) * 1000, 1)],
                      end_heading_err_deg=round(math.degrees(wrap(yaw_target - yaw)), 2), secs=round(time.monotonic() - t_all, 2))
        print(f"\n결과: {result['outcome']} · {result['secs']} s")
        print(f"  {a.frame} 기준 마지막 목표점 오차: 앞 {result['end_err_mm'][0]:+.1f} · 왼 {result['end_err_mm'][1]:+.1f} mm · "
              f"방향 {result['end_heading_err_deg']:+.2f}° (출발점에서 앞 {result['end_from_start_mm'][0]:+.0f} · 왼 {result['end_from_start_mm'][1]:+.0f} mm)")
        if a.shape == "repeat" and segs_out:
            cyc = []
            for k in range(0, len(segs_out) - 1, 2):
                o, b = segs_out[k], segs_out[k + 1]
                cyc.append({"n": k // 2 + 1, "t_min": round(b["t_end"] / 60, 2), "out_along_mm": o["along_err_mm"], "out_cross_mm": o["cross_mm"],
                            "back_pos_mm": b["pos_from_start_mm"], "max_cross_mm": max(o["max_cross_mm"], b["max_cross_mm"]),
                            "heading_err_deg": b["heading_err_deg"]})
            result["repeat"] = {"deg": a.deg, "cycles": cyc}
            print(f"\n{a.deg:g}° 왕복 {len(cyc)} 번 · 왕복마다 출발점에서 (앞 · 왼) · 최대 옆 · 방향 오차")
            for c in cyc[:: max(1, len(cyc) // 12)] + ([cyc[-1]] if len(cyc) > 12 else []):
                print(f"  {c['n']:3d} 번 ({c['t_min']:4.1f} 분) · {c['back_pos_mm'][0]:+6.1f} {c['back_pos_mm'][1]:+6.1f} mm · {c['max_cross_mm']:4.1f} mm · {c['heading_err_deg']:+5.2f}°")
            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
                k = [c["n"] for c in cyc]
                ax[0].plot(k, [c["back_pos_mm"][0] for c in cyc], "o-", ms=3, label="back at start: front [mm]")
                ax[0].plot(k, [c["back_pos_mm"][1] for c in cyc], "s-", ms=3, label="back at start: left [mm]")
                ax[0].plot(k, [c["max_cross_mm"] for c in cyc], "^:", ms=3, label="max |cross| [mm]")
                ax[0].set_ylabel(f"{a.frame} [mm]"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=8)
                ax[1].plot(k, [c["heading_err_deg"] for c in cyc], "o-", ms=3)
                ax[1].set_ylabel("heading err [deg]"); ax[1].set_xlabel("cycle"); ax[1].grid(alpha=.3)
                fig.suptitle(f"repeat {a.deg:g} deg, {L:.2f} m out/back x {len(cyc)} ({result['secs'] / 60:.1f} min, {a.frame})")
                fig.tight_layout()
                png = a.out.rsplit(".", 1)[0] + "_repeat.png"
                fig.savefig(png, dpi=110)
                print("  그림:", png)
            except Exception as ex:
                print("  그림 실패:", ex)
        if a.shape == "star" and len(segs_out) >= 2:   # 첫 나감이 실패하면 표로 만들 왕복이 없음
            rows = []
            for k in range(0, len(segs_out) - 1, 2):
                o, b = segs_out[k], segs_out[k + 1]
                rows.append({"deg": round(k // 2 * a.step_deg, 1), "out_along_mm": o.get("along_err_mm"), "out_cross_mm": o.get("cross_mm"),
                             "out_max_cross_mm": o.get("max_cross_mm"), "back_along_mm": b.get("along_err_mm"), "back_cross_mm": b.get("cross_mm"),
                             "max_cross_mm": max(o.get("max_cross_mm", 0), b.get("max_cross_mm", 0)),
                             "heading_err_deg": b.get("heading_err_deg"), "secs": round(o.get("secs", 0) + b.get("secs", 0), 1)})
            result["star"] = rows
            print("\n방향 · 나간 끝(앞 · 옆) · 돌아온 끝(앞 · 옆) · 최대 옆 · 방향 오차 · 시간")
            for r in rows:
                print(f"  {r['deg']:5.0f}° · {r['out_along_mm']:+6.1f} {r['out_cross_mm']:+6.1f} mm · {r['back_along_mm']:+6.1f} {r['back_cross_mm']:+6.1f} mm"
                      f" · {r['max_cross_mm']:5.1f} mm · {r['heading_err_deg']:+5.2f}° · {r['secs']:4.1f} s")
            worst = max(rows, key=lambda r: r["max_cross_mm"])
            slow = max(rows, key=lambda r: r["secs"])
            print(f"  가장 크게 벗어난 방향 {worst['deg']:.0f}° ({worst['max_cross_mm']:.1f} mm) · 가장 오래 걸린 방향 {slow['deg']:.0f}° ({slow['secs']} s)")
            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                deg = [r["deg"] for r in rows]
                fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
                ax[0].plot(deg, [r["out_cross_mm"] for r in rows], "o-", label="out: cross err")
                ax[0].plot(deg, [r["back_cross_mm"] for r in rows], "s-", label="back: cross err")
                ax[0].plot(deg, [r["max_cross_mm"] for r in rows], "^:", label="max |cross| (out/back)")
                ax[0].plot(deg, [r["out_along_mm"] for r in rows], "x--", label="out: along err")
                ax[0].set_ylabel(f"{a.frame} error [mm]"); ax[0].grid(alpha=.3); ax[0].legend(ncol=4, fontsize=8)
                ax[1].bar(deg, [r["secs"] for r in rows], width=a.step_deg * 0.8)
                ax[1].set_ylabel("out+back time [s]"); ax[1].set_xlabel("travel direction [deg] (0 front, 90 left, 180 back, 270 right)")
                ax[1].grid(alpha=.3)
                fig.suptitle(f"star {L:.2f} m x {len(rows)} directions @ {a.speed} m/s ({a.frame})")
                fig.tight_layout()
                png = a.out.rsplit(".", 1)[0] + "_star.png"
                fig.savefig(png, dpi=110)
                print("  그림:", png)
            except Exception as ex:
                print("  그림 실패:", ex)
        if a.frame == "odom":
            print("  ⚠ 실제 바닥 위치는 줄자로 (odom 은 옆 미끄러짐을 못 봄 — --frame map 이면 SLAM 위치)")
    except KeyboardInterrupt:
        result["outcome"] = "Ctrl+C"
    except RuntimeError as ex:   # map→odom 없음 · 끊김 (--frame map) — 아래 finally 가 0 명령
        result["outcome"] = str(ex)
        print("✗ 중단:", ex)
    finally:
        try:   # 어떤 예외로 끝나도 0 명령 (2026-10-29). Ctrl+C 로 rclpy 가 먼저 꺼졌으면 발행 실패 → 컨트롤러 cmd_timeout 0.3 s 가 정지
            n.run(0.5, (0.0, 0.0, 0.0))
        except Exception:
            pass
        result["log"] = log
        result.setdefault("segments", segs_out)
        if a.raw:
            result["raw_odom"] = [list(o) for o in n.odom]
        with open(a.out, "w") as f:
            json.dump(result, f, ensure_ascii=False)
        print("저장:", a.out)
        n.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if result.get("outcome") == "도착" else 1)


if __name__ == "__main__":
    main()
