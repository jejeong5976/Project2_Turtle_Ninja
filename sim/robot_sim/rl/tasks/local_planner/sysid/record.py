#!/usr/bin/env python3
"""실물 주행 응답 기록: 계단 명령을 /cmd_vel 로 보내고 /odom · /joint_states · IMU · TF (map · odom → base_footprint) 를 같이 저장.
지역 계획기 학습 (rl/tasks/local_planner) 의 도메인 랜덤화 범위 (지연 · 시간 상수 · 미끄럼 · 최소 속도 · 조향 전환) 를 실측으로 맞추려고 씀.

전제: 로봇이 DRIVE (control_master), swerve_controller 가 /cmd_vel (TwistStamped) 받음. Nav2 · teleop 은 끔 (/cmd_vel 발행자가 있으면 멈춤).
      위치 기준이 필요하면 slam.launch.py (localization) 를 같이 띄움 → TF map → base_footprint 로 미끄럼 (odom 대비) 을 봄.
공간: 계단마다 같은 크기로 되돌아오는 짝이라 출발점 둘레 약 0.4 m 안에서 움직임. 앞뒤 · 옆 1 m 씩 비워 둘 것.
안전: 속도는 0.2 m/s · 0.5 rad/s 로 자름, Ctrl+C 면 0 을 1 s 보내고 끝, 계단 사이 2 s 정지.
사용: python3 record.py --out ~/sysid/run1            (--dry_run: 순서만 출력, --scale 0.5: 모든 속도 절반, --only fwd,rot: 이름에 들어간 것만,
                                                      --sim_time: Isaac 브리지 시험)
결과: <out>/data.npz (토픽별 배열) · segments.json (계단 이름 · 시각 · 명령). 분석은 analyze.py
"""
import argparse
import json
import math
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState

from steps import PAUSE_S, VMAX, WMAX, plan  # noqa: E402  계단 순서 · 한계 (ROS 없는 모듈)


class Rec(Node):
    def __init__(self, sim_time=False):
        super().__init__('sysid_record', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=sim_time)])
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.d = {k: [] for k in ('cmd', 'odom', 'js_t', 'js_pos', 'js_vel', 'imu', 'tf_map', 'tf_odom')}
        self.js_names = None
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.create_subscription(Imu, '/imu_sensor_broadcaster/imu', self.on_imu, qos_profile_sensor_data)
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.create_timer(0.02, self.on_tf)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        p, q, t = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.d['odom'].append((rclpy.time.Time.from_msg(m.header.stamp).nanoseconds * 1e-9, p.x, p.y, yaw, t.linear.x, t.linear.y, t.angular.z))

    def on_js(self, m):
        if self.js_names is None:
            self.js_names = list(m.name)
        if list(m.name) != self.js_names:
            return
        self.d['js_t'].append(rclpy.time.Time.from_msg(m.header.stamp).nanoseconds * 1e-9)
        self.d['js_pos'].append(list(m.position))
        self.d['js_vel'].append(list(m.velocity) if m.velocity else [math.nan] * len(m.name))

    def on_imu(self, m):
        self.d['imu'].append((rclpy.time.Time.from_msg(m.header.stamp).nanoseconds * 1e-9, m.angular_velocity.z))

    def on_tf(self):
        for parent, key in (('map', 'tf_map'), ('odom', 'tf_odom')):
            try:
                t = self.tf.lookup_transform(parent, 'base_footprint', rclpy.time.Time())
            except Exception:
                continue
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            st = rclpy.time.Time.from_msg(t.header.stamp).nanoseconds * 1e-9
            if not self.d[key] or self.d[key][-1][0] != st:
                self.d[key].append((st, t.transform.translation.x, t.transform.translation.y, yaw))

    def send(self, vx, vy, wz):
        v = math.hypot(vx, vy)
        if v > VMAX:                                            # 안전: 실물 한계로 자름
            vx, vy = vx * VMAX / v, vy * VMAX / v
        wz = max(-WMAX, min(WMAX, wz))
        m = TwistStamped()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), 'base_footprint'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(m)
        self.d['cmd'].append((self.now(), vx, vy, wz))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', required=True)
    p.add_argument('--scale', type=float, default=1.0)
    p.add_argument('--only', default='')
    p.add_argument('--dry_run', action='store_true')
    p.add_argument('--sim_time', action='store_true', help='시뮬 (/clock) 시각으로 기록 — Isaac 브리지 시험용')
    a = p.parse_args()
    if not 0 < a.scale <= 1:
        raise SystemExit('--scale 은 0 초과 1 이하 (실물 한계를 넘기지 않게)')
    segs = [s for s in plan() if not a.only or any(k in s[0] for k in a.only.split(','))]
    segs = [(n, vx * a.scale, vy * a.scale, wz * a.scale, h, ps) for n, vx, vy, wz, h, ps in segs]
    total = sum(h + ps for *_, h, ps in segs)
    for n, vx, vy, wz, h, ps in segs:
        print(f'  {n:16s} vx {vx:+.2f} vy {vy:+.2f} wz {wz:+.2f}  {h:.1f} s' + ('  (멈추지 않고 다음으로)' if ps == 0 else ''))
    print(f'계단 {len(segs)} 개, 약 {total:.0f} s')
    if a.dry_run:
        return
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Rec(a.sim_time)
    stop = {'v': False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__('v', True))

    def spin_for(sec, cmd):
        t_end = time.time() + sec
        nxt = time.time()
        while time.time() < t_end and not stop['v']:
            n.send(*cmd)
            nxt += 0.02                                         # 50 Hz (컨트롤러 0.3 s 시간 초과보다 충분히 짧게)
            while time.time() < nxt:
                rclpy.spin_once(n, timeout_sec=max(0.0, nxt - time.time()))

    # 시작 확인: /odom 이 오고 다른 /cmd_vel 발행자가 없어야 함
    t0 = time.time()
    while not n.d['odom'] and time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.d['odom']:
        raise SystemExit('/odom 이 안 옴 → swerve_controller · DRIVE 확인')
    others = n.count_publishers('/cmd_vel') - 1
    if others > 0:
        raise SystemExit(f'/cmd_vel 발행자가 {others} 개 더 있음 (Nav2 · teleop?) → 끄고 다시')
    print('map → base_footprint TF', '있음' if n.d['tf_map'] else '없음 (미끄럼은 odom 만으로 못 봄)', flush=True)
    log = []
    spin_for(PAUSE_S, (0, 0, 0))
    for name, vx, vy, wz, hold, pause in segs:
        if stop['v']:
            break
        ts = n.now()
        print(f'{name:16s} {vx:+.2f} {vy:+.2f} {wz:+.2f}', flush=True)
        spin_for(hold, (vx, vy, wz))
        te = n.now()
        spin_for(pause, (0, 0, 0))
        log.append(dict(name=name, cmd=[vx, vy, wz], t_start=ts, t_zero=te, t_end=n.now()))
    for _ in range(50):                                         # 끝 · Ctrl+C: 0 을 1 s
        n.send(0, 0, 0)
        time.sleep(0.02)
    np.savez_compressed(out / 'data.npz', **{k: np.array(v, dtype=float) for k, v in n.d.items()},
                        js_names=np.array(n.js_names or []))
    (out / 'segments.json').write_text(json.dumps(dict(segments=log, vmax=VMAX, wmax=WMAX, scale=a.scale,
                                                       stopped_early=stop['v']), indent=1, ensure_ascii=False))
    print(f'저장 → {out} (계단 {len(log)} 개{", 중간에 멈춤" if stop["v"] else ""})')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
