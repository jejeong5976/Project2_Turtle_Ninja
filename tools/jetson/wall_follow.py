#!/usr/bin/env python3
"""벽 따라가기 자동 주행 (2026-10-31, SLAM 지도 만들기용) — run_robot.sh slam 이 떠 있을 때 다른 터미널에서.

  python3 tools/jetson/wall_follow.py [--side right] [--dist 0.30] [--speed 0.08] [--minutes 10] [--out ~/wall_follow.json]

한쪽 벽(기본 오른쪽)을 --dist 거리로 따라감. --minutes 가 지나면 멈춤. 처음 벽 자리로 돌아오면(섬 둘레 한 바퀴) 끝내지 않고 벽 반대쪽으로 벗어나 다른 벽을 찾음.
몸 방향은 그대로 두고(정면 유지는 컨트롤러가 함) 진행 방향만 바꿈 — 스워브 · 라이다 360° 라 몸을 돌릴 필요 없음.
  처음: 진행 방향 90° 격자를 스캔의 벽 방향에 맞춤(로봇이 벽과 비스듬히 서 있어도 됨). 앞으로 가다가 막히면 왼쪽으로 꺾어 벽을 오른쪽에 둠. 앞이 막히면 반대쪽으로 90°, 옆 벽이 끊기면 몸 하나만큼 더 가서 벽 쪽으로 90°.
안전: 진행 경로 앞 (몸 끝 + --stop) 이나 몸 옆에 닿기 직전(3 cm) 에 점(사람 · 선 포함) 이 있으면 0 (치우면 다시 감). 스캔 · odom 0.5 s 끊김 = 0. 아무 키 = 정지 후 끝.
  ⚠ 라이다 방향(URDF lidar_front_yaw) 이 맞는지 먼저 확인 (진행표 C-2) — 틀리면 벽을 반대쪽으로 봄.
진행 방향은 초당 90° 이하로 바꾸고 방향이 많이 다르면 속도를 줄임 (컨트롤러 '조향 먼저' 게이트 10° 에 덜 걸리게).
"""
import argparse
import json
import math
import os
import select
import sys
import termios
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # tools/ (body_footprint)
from body_footprint import BODY_HX, BODY_HY, obstacle_hit, self_mask   # noqa: E402

RATE = 20.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wall_grid(p):
    """스캔 점(각도 순) 5 칸 건너 선분 방향의 90° 주기 평균 [rad, 몸 기준, −45° ~ 45°] (이웃끼리는 라이다 잡음 1 ~ 2 cm 에 묻힘). 벽이 뚜렷하지 않으면 0."""
    d = p[5:] - p[:-5]
    L = np.hypot(d[:, 0], d[:, 1])
    m = (L > 0.03) & (L < 0.25)
    c = np.sum(L[m] * np.exp(4j * np.arctan2(d[m, 1], d[m, 0])))
    return float(np.angle(c) / 4) if abs(c) > 0.3 * np.sum(L[m]) else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--side', choices=('right', 'left'), default='right')
    ap.add_argument('--dist', type=float, default=0.30, help='벽까지 목표 거리 [m] (몸 중심 기준)')
    ap.add_argument('--speed', type=float, default=0.08, help='m/s (최대 0.10)')
    ap.add_argument('--front', type=float, default=0.35, help='진행 방향 이 거리 안이 막히면 꺾음 [m]')
    ap.add_argument('--body', type=float, default=0.16, help='옆 벽 · 앞 막힘을 재는 띠의 반폭 [m] (자기 몸 점은 body_footprint 가 버림)')
    ap.add_argument('--stop', type=float, default=0.08, help='몸 상자 밖 이 거리 안에 점이 있으면 정지 [m]')
    ap.add_argument('--minutes', type=float, default=10.0)
    ap.add_argument('--base', default='base_footprint')
    ap.add_argument('--scan', default='/scan', help='앞 · 뒤 합친 360° (scan_merger). 앞만이면 /lidar_front/scan')
    ap.add_argument('--out', default=os.path.expanduser('~/wall_follow.json'))
    a = ap.parse_args()
    a.speed = min(a.speed, 0.10)
    sgn = 1.0 if a.side == 'right' else -1.0       # 벽 쪽 = 진행 방향의 오른쪽(+1) / 왼쪽(−1)

    rclpy.init()
    n = rclpy.create_node('wall_follow')
    pub = n.create_publisher(TwistStamped, '/cmd_vel', 10)
    tfb = Buffer(); TransformListener(tfb, n)
    st = {'pts': None, 'scan_t': 0.0, 'odom': None, 'odom_t': 0.0, 'T': None}

    def on_scan(m):
        if st['T'] is None:                                         # 라이다 장착은 고정 → 한 번만
            try:
                tr = tfb.lookup_transform(a.base, m.header.frame_id, Time()).transform
            except Exception:
                return
            st['T'] = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        tx, ty, tyaw = st['T']
        r = np.asarray(m.ranges, float)
        ang = m.angle_min + m.angle_increment * np.arange(len(r)) + tyaw
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        p = np.c_[tx + r[ok] * np.cos(ang[ok]), ty + r[ok] * np.sin(ang[ok])]
        st['pts'] = self_mask(p)   # 자기 몸(바퀴 · 기둥 · 선) 빔 버림 — 11-01 실물 왼앞 (0.10, 0.17) 점 포함, path_follow 와 같은 몸
        st['scan_t'] = time.monotonic()

    def on_odom(m):
        p = m.pose.pose
        st['odom'] = (p.position.x, p.position.y, yaw_of(p.orientation)); st['odom_t'] = time.monotonic()

    n.create_subscription(LaserScan, a.scan, on_scan, qos_profile_sensor_data)
    n.create_subscription(Odometry, '/odom', on_odom, 10)

    def send(vx, vy):
        m = TwistStamped()
        m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = a.base
        m.twist.linear.x, m.twist.linear.y = vx, vy
        pub.publish(m)

    tty_in = sys.stdin.isatty()                                         # 터미널이 아니면(시험 · 백그라운드) 키 정지 없음

    def key():
        if not tty_in:
            return None
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if r else None

    theta = theta_cmd = None          # 진행 방향 [rad, odom 기준] — 목표 · 지금 명령 (초당 90° 까지만 따라감)
    v_f = np.zeros(2)                 # 저역 통과한 몸 기준 속도
    have_wall, open_from, home, dist, last_xy = False, None, None, 0.0, None
    frame = None                      # 'map' (SLAM) 또는 'odom'
    trail, escapes, escaping = [], 0, False   # 지나간 자리 (x, y, 진행 방향, 거리) 0.2 m 마다 · 섬 탈출 횟수
    why, log, t0, last_print, last_ctrl = '', [], time.monotonic(), 0.0, 0.0
    old = termios.tcgetattr(sys.stdin) if tty_in else None
    try:
        if tty_in:
            tty.setcbreak(sys.stdin.fileno())
            termios.tcflush(sys.stdin, termios.TCIFLUSH)                  # 앞 프로그램에서 남은 키 버림 (11-01: 남은 키로 3 s 만에 끝남)
        print(__doc__.split('\n')[2].strip()); print('아무 키 = 정지 후 끝 · slam 모드(라이다)로 띄운 뒤에 쓸 것')
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.01)
            now = time.monotonic()
            if now - last_ctrl < 1.0 / RATE:                              # 메시지마다 돌지 않게 — 계산 · 명령은 정확히 20 Hz (10-31 시뮬: 빨리 돌아 꺾기 · 거르기가 몇 배 빨랐음)
                continue
            last_ctrl = now
            if key() is not None:
                why = '키'; break
            if now - t0 > a.minutes * 60:
                why = '시간 제한'; break
            if st['pts'] is None or st['odom'] is None or now - st['scan_t'] > 0.5 or now - st['odom_t'] > 0.5:
                send(0.0, 0.0); v_f[:] = 0
                if now - t0 > 3 and now - last_print > 1:
                    miss = ' · '.join(n_ for n_, bad in ((f'스캔({a.scan})', st['pts'] is None or now - st['scan_t'] > 0.5),
                                                          ('odom', st['odom'] is None or now - st['odom_t'] > 0.5)) if bad)
                    print(f'\r  {miss} 기다림 — run_robot.sh slam 으로 라이다까지 띄웠는지 확인        ', flush=True); last_print = now
                continue
            x, y, yaw = st['odom']
            # 위치 · 방향 = SLAM 지도 기준 (11-01 실물: odom 방향이 6 분에 12° 바뀌어 진행 격자가 벽과 틀어짐 · odom 경로가 벽을 뚫고 지나감)
            #   처음에 map 이 있으면 끝까지 map, 없으면(slam 안 띄움) 끝까지 odom
            if frame is None:
                frame = 'map' if tfb.can_transform('map', a.base, Time()) else 'odom'
                print(f'\r  위치 기준: {frame}        ')
            if frame == 'map':
                try:
                    mo = tfb.lookup_transform('map', 'odom', Time())
                    tr = tfb.lookup_transform('map', a.base, Time()).transform
                except Exception:
                    send(0.0, 0.0); v_f[:] = 0
                    continue
                stamp = Time.from_msg(mo.header.stamp)                    # SLAM 이 멈췄으면 옛 map 자세를 쓰지 않음 (11-02 GPT 검토)
                if stamp.nanoseconds and (n.get_clock().now() - stamp).nanoseconds * 1e-9 > 1.0:
                    send(0.0, 0.0); v_f[:] = 0
                    if now - last_print > 1:
                        print('\r  map→odom 1 s 넘게 안 옴 — SLAM · locate 확인 (0 명령)        ', flush=True); last_print = now
                    continue
                x, y, yaw = tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
            if theta is None:                                             # 처음 = 몸 앞쪽에 가장 가까운 벽 방향 (11-01 실물: 몸이 벽과 약 15° 틀어져 계단 모양으로 감)
                g = wall_grid(st['pts'])
                theta, theta_cmd = yaw + g, yaw
                print(f'\r  벽 방향에 맞춤 {math.degrees(g):+.0f}°        ')
            if last_xy is not None:
                dist += math.hypot(x - last_xy[0], y - last_xy[1])
            last_xy = (x, y)
            p = st['pts']
            # 진행 방향 좌표 (몸 기준): u = 앞, w = 벽 쪽
            th_b = theta_cmd - yaw
            u = np.array([math.cos(th_b), math.sin(th_b)]); w = sgn * np.array([u[1], -u[0]])
            along, lat = p @ u, p @ w
            band = np.abs(lat) < a.body + 0.03
            front = np.min(along[band & (along > 0)], initial=9.0)
            side_pts = (np.abs(along) < a.body) & (lat > 0)
            side = np.min(lat[side_pts], initial=9.0)
            state = '따라감'
            if side < a.dist + 0.25:
                if home is None:                                          # 처음 한 번만 (모서리마다 다시 잡으면 통로 왕복을 한 바퀴로 착각 — 10-31 시뮬)
                    home = (x, y); print(f'\r  벽 찾음 — 이 자리로 돌아오면 벽 반대쪽으로 벗어남 ({x:+.2f}, {y:+.2f})        ')
                have_wall, open_from = True, None
            # 섬 돌기 감지 (10-31 시뮬: 미로 안 떨어진 벽을 처음 만나면 그 둘레만 계속 돎): 2 m 넘게 전에 같은 방향으로 지난 자리(0.25 m 안)에 (11-01 실물: 경기장 통로 1 m 폭 · 한 바퀴 2.6 m — 4 m 기준이면 2 바퀴 뒤에야 감지)
            #   다시 오면 벽 반대쪽으로 꺾어 다음 벽이 나올 때까지 직진 (have_wall 끔 → 옆 벽이 생기면 다시 따라감)
            if not trail or dist - trail[-1][3] > 0.2:
                trail.append((x, y, theta, dist))
            if have_wall and not escaping and any(dist - d0 > 2.0 and math.hypot(x - x0, y - y0) < 0.25 and abs(math.remainder(theta - t0_, 2 * math.pi)) < 0.5
                                                 for x0, y0, t0_, d0 in trail[:-10]):
                escapes += 1; escaping = True; have_wall = False; open_from = None; trail.clear()
                theta += sgn * math.pi / 2
                print(f'\r  같은 자리 다시 — 섬 둘레로 보고 벗어남 ({escapes} 번째)        ')
            if escaping and side < a.dist + 0.25 and abs(theta - theta_cmd) < 0.1:
                escaping = False
            if front < a.front:                                           # 앞이 막힘 → 벽 반대쪽으로 90°
                if abs(theta - theta_cmd) < 0.1:
                    theta += sgn * math.pi / 2
                state = '꺾음 (앞 막힘)'
            elif have_wall and not escaping and side > a.dist + 0.25:                      # 옆 벽이 끊김 → 몸 하나만큼 더 가서 벽 쪽으로
                if open_from is None:
                    open_from = dist
                if dist - open_from > a.body + 0.10 and abs(theta - theta_cmd) < 0.1:
                    theta -= sgn * math.pi / 2; open_from = None; have_wall = False
                state = '모서리'
            # 진행 방향 명령: 초당 90° 까지만, 방향 차가 크면 느리게 (조향이 따라올 시간)
            d = math.atan2(math.sin(theta - theta_cmd), math.cos(theta - theta_cmd))
            theta_cmd += max(-math.pi / 2 / RATE, min(math.pi / 2 / RATE, d))
            v = a.speed * max(0.0, math.cos(d)) ** 4 * min(1.0, max(0.0, (front - a.front + 0.10) / 0.15))
            # 가까움 정지 (10-31 시뮬: 몸 둘레 전체로 보면 진행과 상관없는 기둥 · 모서리에 영원히 멈춤 → 진행 경로 앞 + 몸에 닿기 직전 옆만)
            # 진행 경로 앞 = path_follow 와 같은 몸 식 (대각 포함, 11-02 GPT 검토) + 몸 직사각형 3 cm 안
            in_path = obstacle_hit(p, u[0], u[1], 0.0, margin=a.stop) is not None
            touching = (np.abs(p[:, 0]) < BODY_HX + 0.03) & (np.abs(p[:, 1]) < BODY_HY + 0.03)
            near = in_path or bool(np.any(touching))
            near_pt = (p[touching][0].round(3).tolist() if np.any(touching) else 'path') if near else None
            corr = 0.0 if not have_wall or side > a.dist + 0.25 else max(-0.3, min(0.3, 1.5 * (side - a.dist))) * v
            th_b = theta_cmd - yaw
            u = np.array([math.cos(th_b), math.sin(th_b)]); w = sgn * np.array([u[1], -u[0]])
            target = v * u + corr * w
            if near:
                target[:] = 0; v_f[:] = 0; state = '정지 (가까움 — 사람 · 선 · 벽)'
            v_f += (target - v_f) * min(1.0, (1.0 / RATE) / 0.3)          # 저역 통과 0.3 s (명령 흔들림 → 조향 게이트 막힘 방지)
            send(float(v_f[0]), float(v_f[1]))
            if home and dist > 3.0 and math.hypot(x - home[0], y - home[1]) < 0.3 and not escaping:   # 11-01 실물: 5.8 m 만에 끝나 아래쪽을 못 감
                escapes += 1; escaping = True; have_wall = False; open_from = None; trail.clear(); home = None
                theta += sgn * math.pi / 2
                print(f'\r  처음 벽 자리로 돌아옴 — 벽 반대쪽으로 벗어남 ({escapes} 번째)        ')
            log.append({'t': round(now - t0, 2), 'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3), 'dir': round(theta_cmd, 3),
                        'front': round(float(front), 3), 'side': round(float(side), 3), 'cmd': [round(float(c), 3) for c in v_f], 'state': state,
                        'near_pt': near_pt})
            if now - last_print > 1.0:
                last_print = now
                print(f'\r  {now - t0:6.1f} s · {dist:5.2f} m · 앞 {front:4.2f} · 벽 {side:4.2f} m · {state}            ', flush=True)
    finally:
        if old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)          # 터미널 먼저 되돌림
        try:
            for _ in range(5):
                send(0.0, 0.0); time.sleep(0.05)
        except Exception:                                                 # Ctrl+C 로 rclpy 가 먼저 닫힘 — 컨트롤러가 0.3 s 뒤 스스로 정지
            pass
        print(f'\n끝: {why or "Ctrl+C"} · {dist:.2f} m · {time.monotonic() - t0:.0f} s')
        with open(a.out, 'w') as f:
            json.dump({'why': why, 'dist_m': dist, 'frame': frame, 'args': vars(a), 'log': log}, f)
        print('기록', a.out)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
