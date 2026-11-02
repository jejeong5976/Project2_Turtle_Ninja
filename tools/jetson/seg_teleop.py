#!/usr/bin/env python3
"""구간 운전 (2026-10-31, 매핑 · 확인용) — 키 하나 = 정해진 한 구간 움직이고 섬. run_robot.sh drive|slam|locate 가 떠 있을 때 다른 터미널에서.

  python3 tools/jetson/seg_teleop.py [--v 0.08] [--w 0.30] [--step 0.10] [--turn 15]
  키: w/s 앞 · 뒤   a/d 왼 · 오른 (옆으로, 스워브)   r/t 앞왼 · 앞오른   f/g 뒤왼 · 뒤오른 (대각)   q/e 왼 · 오른 회전   + / - 구간 2 배 · 반   c = 연속 모드 켜기/끄기   스페이스 · 다른 키 = 바로 정지   x = 끝
  (움직이는 중 같은 키가 또 오면 = 키를 누르고 있어 생긴 자동 반복 → 무시. 구간이 끝나면 쌓인 키는 버림 — 한 번 누름 = 한 구간)

왜 구간식: teleop_twist_keyboard 는 키를 누를 때만 한 번 냄 → 처음 누르고 자동 반복까지(0.25~0.6 s) 명령이 끊겨 컨트롤러(0.3 s 끊기면 정지)가 멈췄다 감.
  여기서는 움직이는 동안 20 Hz 로 계속 내고, 구간 시간이 끝나면 0 을 내고 섬 (10-27 매핑 콘솔처럼 서서 맞추고 다음 구간).
속도 사다리꼴: 0.3 s 가속 · 0.3 s 감속 (컨트롤러 가속 제한 0.5 m/s² · 2.0 rad/s² 안). 거리 · 각도는 명령 기준 (바닥 밀림은 SLAM 이 봄).
"""
import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped

D = 0.70710678                                                     # 대각 = 앞뒤 · 옆 같은 비율 (한 구간 거리는 대각 방향으로 --step)
KEYS = {'w': (1, 0, 0), 's': (-1, 0, 0), 'a': (0, 1, 0), 'd': (0, -1, 0), 'q': (0, 0, 1), 'e': (0, 0, -1),
        'r': (D, D, 0), 't': (D, -D, 0), 'f': (-D, D, 0), 'g': (-D, -D, 0)}   # 11-01: 대각 (앞왼 · 앞오른 · 뒤왼 · 뒤오른)
RATE = 20.0
RAMP = 0.3


def key(timeout):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--v', type=float, default=0.08, help='직진 · 옆 최고 속도 m/s (최대 0.20)')
    ap.add_argument('--w', type=float, default=0.30, help='회전 최고 속도 rad/s (최대 0.8)')
    ap.add_argument('--step', type=float, default=0.10, help='한 구간 거리 m')
    ap.add_argument('--turn', type=float, default=15.0, help='한 구간 회전 °')
    a = ap.parse_args()
    a.v, a.w = min(a.v, 0.20), min(a.w, 0.8)
    rclpy.init()
    node = rclpy.create_node('seg_teleop')
    pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)

    def send(vx, vy, wz):
        m = TwistStamped()
        m.header.stamp = node.get_clock().now().to_msg(); m.header.frame_id = 'base_footprint'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = vx, vy, wz
        pub.publish(m)

    def segment(k):
        d = KEYS[k]
        lin = d[2] == 0
        dist = a.step if lin else a.turn * 3.14159265 / 180.0
        vmax = a.v if lin else a.w
        T = max(dist / vmax + RAMP, 2 * RAMP)                            # 사다리꼴 넓이 = 거리 (가속 · 감속 각 RAMP)
        peak = dist / (T - RAMP)
        if mode['cont']:                                                 # 연속 모드 (11-01): 한 번 누르면 다른 키까지 계속 (최고 속도, 가속만 사다리꼴)
            T, peak = float('inf'), vmax
        t0 = time.time()
        while True:
            t = time.time() - t0
            if t >= T:
                break
            s = peak * min(1.0, t / RAMP, (T - t) / RAMP if T != float('inf') else 1.0)
            send(d[0] * s if lin else 0.0, d[1] * s if lin else 0.0, d[2] * s)
            got = key(1.0 / RATE)
            if got is not None and got != k:                             # 다른 키 = 바로 정지 (같은 키 = 자동 반복, 무시)
                send(0.0, 0.0, 0.0)
                print('\r  정지 (키)            ')
                return False
        send(0.0, 0.0, 0.0)
        return True

    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        print(__doc__.split('\n')[3].strip())
        n = 0
        mode = {'cont': False}
        while rclpy.ok():
            k = key(0.5)
            if k is None:
                continue
            if k == 'x':
                break
            if k == 'c':
                mode['cont'] = not mode['cont']
                print(f"\r  {'연속 모드 — 한 번 누르면 다른 키(스페이스)까지 계속' if mode['cont'] else '구간 모드 — 한 번 = 한 구간'}        ")
                continue
            if k in '+-':
                a.step = min(0.5, a.step * 2) if k == '+' else max(0.02, a.step / 2)
                a.turn = min(90.0, a.turn * 2) if k == '+' else max(3.0, a.turn / 2)
                print(f'\r  구간 {a.step * 100:.0f} cm · {a.turn:.0f}°        ')
                continue
            if k in KEYS:
                n += 1
                ok = segment(k)
                termios.tcflush(sys.stdin, termios.TCIFLUSH)              # 구간 동안 쌓인 자동 반복 키를 버림 (다음 구간이 저절로 시작되지 않게)
                print(f'\r  {n:3d} {k} {"끝" if ok else "멈춤"} ({a.step * 100:.0f} cm · {a.turn:.0f}°)   ')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)              # 터미널 먼저 되돌림 (아래가 예외여도 cbreak 로 안 남게)
        try:
            send(0.0, 0.0, 0.0)
        except Exception:                                                 # Ctrl+C 로 rclpy 가 먼저 닫힘 — 못 보내도 컨트롤러가 0.3 s 뒤 스스로 정지
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
