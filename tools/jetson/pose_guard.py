#!/usr/bin/env python3
"""위치 추정 감시 (2026-11-01) — SLAM · 라이다 · IMU · odom 을 서로 비교해 튀거나 끊기면 알림. run_robot.sh slam/locate 가 떠 있을 때.

  python3 tools/jetson/pose_guard.py [--out ~/pose_guard.json]

/localization_health (diagnostic_msgs/DiagnosticStatus, 5 Hz): level 0 정상 · 1 경고(느리게) · 2 정지. path_follow.py 가 받아서 따름.
제일 믿는 값 = SLAM(map). 단 라이다가 지도 벽과 맞을 때만 — 맞음 비율이 낮으면 SLAM 위치를 못 믿음.
기준 (11-01 실측: 맞음 99 ~ 100 % · map→odom 한 번 고침 ≤ 6 cm · 1.4° · 자이로와 SLAM 방향 차 ≤ 0.15°/s · 라이다 10 Hz 약 260 점):
  정지  라이다(/scan) 0.5 s 끊김 · 지도 맞음 < 60 % 가 1 s · 지도 맞음을 2 s 넘게 못 잼 · map→odom 한 번에 > 25 cm 또는 > 10° 인데 그 뒤 맞음 < 85 %
        · map→odom 방향 고침 5 s 평균 > 3°/s · 가속도 크기 10 ± 3 밖이 1 s (들림 · 부딪힘)
  경고  맞음 < 85 % · map→odom 한 번에 > 10 cm 또는 > 3° (맞음이 높으면 '큰 보정 받아들임') · 방향 고침 > 1°/s
        · 라이다 < 7 Hz 또는 < 150 점 · IMU 0.5 s 끊김(컨트롤러가 바퀴 ω 로 바꿈) · 자이로 > 5 rad/s 튐(컨트롤러가 버림)
  정지 · 경고는 원인이 없어진 뒤 1 s 더 유지 (깜빡임 방지).
자이로 이동 치우침 학습 (11-01 실물: 움직이는 동안 자이로가 0.09°/s 덜 느낌 → 5 분에 29° 를 SLAM 이 고침, 시간 갈수록 지도 맞음 95 → 57 %):
  바퀴로 움직이는 5 s 동안 판정이 정상이면 그 사이 map→odom 방향이 바뀐 속도만큼 조금씩(0.1 /s) 더해 /imu_motion_bias 로 보냄
  → swerve_controller 가 움직일 때만 자이로에 더함 (±0.005 rad/s 에서 자름). 배운 값은 ~/.imu_motion_bias 에 남겨 다음에 이어 씀 (--no-bias = 안 보냄).
"""
import argparse
import json
import math
import os
import sys
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sensor_check import wall_dist, wrap, yaw_of   # noqa: E402

BASE = 'base_footprint'
HOLD_S = 1.0
BIAS_FILE = os.path.expanduser('~/.imu_motion_bias')
BIAS_GAIN, BIAS_MAX, BIAS_WIN = 0.1, 0.005, 5.0


def bias_update(est, hist):
    """hist = [(t, map→odom 방향, 움직임, 판정)] 최근 BIAS_WIN s. 판정이 내내 정상이고 60 % 넘게 움직였으면
    est += 이득 · (방향 고친 양 / 움직인 시간) [rad/s] — path_follow 는 구간마다 0.5 s 멈춤, 치우침은 움직일 때만 생김"""
    if len(hist) < 2 or hist[-1][0] - hist[0][0] < BIAS_WIN - 0.5 or any(h[3] != 0 for h in hist):
        return est
    moving_s = sum(b[0] - a[0] for a, b in zip(hist, hist[1:]) if b[2])
    if moving_s < 0.6 * (hist[-1][0] - hist[0][0]):
        return est
    rate = wrap(hist[-1][1] - hist[0][1]) / moving_s
    if abs(rate) > 0.02:                                               # 1.15°/s 넘음 = 치우침이 아니라 위치 다시 잡기
        return est
    return max(-BIAS_MAX, min(BIAS_MAX, est + BIAS_GAIN * rate))


def evaluate(m):
    """지표 → (level, 이유 목록). 순수 함수 (시험용). 없는 지표(None)는 건너뜀."""
    err, warn = [], []
    if m['scan_age'] is None or m['scan_age'] > 0.5:
        err.append('라이다 끊김')
    r = m['match']
    if r is None:
        (err if m.get('match_none_s', 0.0) > 2.0 else warn).append('지도 맞음 모름 (지도 · TF · 스캔 기다림)')
    else:
        if m['match_low_s'] >= 1.0:
            err.append(f'지도 맞음 {r * 100:.0f}% (1 s 넘게 < 60%)')
        elif r < 0.85:
            warn.append(f'지도 맞음 {r * 100:.0f}%')
    jm, jd = m['jump_m'], m['jump_deg']
    if jm > 0.25 or jd > 10.0:
        (warn if r is not None and r >= 0.85 else err).append(f'SLAM 큰 보정 {jm * 100:.0f} cm · {jd:.1f}°' + (' 받아들임' if r is not None and r >= 0.85 else ''))
    elif jm > 0.10 or jd > 3.0:
        warn.append(f'SLAM 보정 {jm * 100:.0f} cm · {jd:.1f}°')
    yr = m['yaw_fix_rate']
    if yr is not None:
        if yr > 3.0:
            err.append(f'방향 고침 {yr:.1f}°/s')
        elif yr > 1.0:
            warn.append(f'방향 고침 {yr:.1f}°/s')
    if m['scan_hz'] is not None and m['scan_hz'] < 7.0:
        warn.append(f'라이다 {m["scan_hz"]:.0f} Hz')
    if m['scan_pts'] is not None and m['scan_pts'] < 150:
        warn.append(f'라이다 점 {m["scan_pts"]}')
    if m['imu_age'] is None or m['imu_age'] > 0.5:
        warn.append('IMU 끊김 (바퀴 회전값 사용)')
    if m['gyro_spikes'] > 0:
        warn.append(f'자이로 튐 {m["gyro_spikes"]}')
    if m['acc_bad_s'] >= 1.0:
        err.append('가속도 이상 1 s (들림 · 부딪힘?)')
    elif m['acc_bad_s'] > 0:
        warn.append('가속도 튐')
    return (2 if err else 1 if warn else 0), err + warn


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.expanduser('~/pose_guard.json'))
    ap.add_argument('--no-bias', action='store_true', help='자이로 이동 치우침을 보내지 않음')
    a = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('pose_guard')
    tfb = Buffer(); TransformListener(tfb, n)
    pub = n.create_publisher(DiagnosticStatus, '/localization_health', 10)
    pub_bias = n.create_publisher(Vector3Stamped, '/imu_motion_bias', 10)
    try:
        bias = float(open(BIAS_FILE).read())
    except (OSError, ValueError):
        bias = 0.0016                                                   # 11-01 두 번 잰 값 (0.09°/s)
    bias_hist, odom_v = deque(), {'v': 0.0}
    st = {'scan': None, 'scan_t': deque(maxlen=50), 'imu_t': None, 'spikes': deque(), 'acc_bad_since': None,
          'map': None, 'map_D': None, 'map_t': 0.0, 'm2o': None, 'jumps': deque(), 'm2o_hist': deque(),
          'match_hist': deque(maxlen=5), 'match_t': time.monotonic(), 'low_since': None, 'level': 0, 'last_err': -1e9, 'last_warn': -1e9, 'reasons': []}
    log = []

    def on_scan(m):
        st['scan'] = m; st['scan_t'].append(time.monotonic())

    def on_imu(m):
        now = time.monotonic(); st['imu_t'] = now
        if abs(m.angular_velocity.z) > 5.0:
            st['spikes'].append(now)
        g = math.sqrt(m.linear_acceleration.x ** 2 + m.linear_acceleration.y ** 2 + m.linear_acceleration.z ** 2)
        if abs(g - 9.8) > 3.0:
            st['acc_bad_since'] = st['acc_bad_since'] or now
        else:
            st['acc_bad_since'] = None

    def on_map(m):
        st['map'] = m

    n.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    n.create_subscription(Imu, '/imu_sensor_broadcaster/imu', on_imu, qos_profile_sensor_data)
    n.create_subscription(Odometry, '/odom', lambda m: odom_v.update(v=math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)), 10)
    n.create_subscription(OccupancyGrid, '/map', on_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))

    def scan_points(sc):
        if sc is None:
            return None
        r = np.asarray(sc.ranges, float)
        return int(np.sum(np.isfinite(r) & (r > sc.range_min)))

    def match_ratio():
        """지금 스캔 점(4 m 안) 중 지도 벽 5 cm 안 비율 — 지도는 바뀌면 5 s 에 한 번만 다시 계산"""
        mp, sc = st['map'], st['scan']
        if mp is None or sc is None:
            return None
        now = time.monotonic()
        if st['map_D'] is None or (mp is not st['map_D'][0] and now - st['map_t'] > 5.0):
            grid = np.asarray(mp.data, np.int16).reshape(mp.info.height, mp.info.width) > 50
            st['map_D'] = (mp, wall_dist(grid, cap=3) * mp.info.resolution); st['map_t'] = now
        mp, D = st['map_D']
        try:
            tr = tfb.lookup_transform('map', sc.header.frame_id, Time()).transform
        except Exception:
            return None
        r = np.asarray(sc.ranges, float); ang = sc.angle_min + sc.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > 0.25) & (r < 4.0)
        if ok.sum() < 30:
            return None
        yw = yaw_of(tr.rotation); c, s = math.cos(yw), math.sin(yw)
        lx, ly = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
        u = np.floor((tr.translation.x + c * lx - s * ly - mp.info.origin.position.x) / mp.info.resolution).astype(int)
        v = np.floor((tr.translation.y + s * lx + c * ly - mp.info.origin.position.y) / mp.info.resolution).astype(int)
        inb = (u >= 0) & (u < mp.info.width) & (v >= 0) & (v < mp.info.height)
        hit = np.zeros(len(u), bool); hit[inb] = D[v[inb], u[inb]] < 0.05
        return float(np.mean(hit))   # 지도 밖 점도 분모에 (11-02 GPT 검토: 빼면 위치가 크게 틀려도 높게 나옴)

    def tick():
        now = time.monotonic()
        # map→odom: 바뀔 때마다 한 번 고친 양, 5 s 동안 방향 고친 속도
        try:
            t = tfb.lookup_transform('map', 'odom', Time()).transform
            mo = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        except Exception:
            mo = None
        if mo is not None:
            if st['m2o'] is not None and mo != st['m2o']:
                p = st['m2o']
                st['jumps'].append((now, math.hypot(mo[0] - p[0], mo[1] - p[1]), abs(math.degrees(wrap(mo[2] - p[2])))))
            st['m2o'] = mo
            st['m2o_hist'].append((now, mo[2]))
        while st['jumps'] and now - st['jumps'][0][0] > 1.0:
            st['jumps'].popleft()
        while st['m2o_hist'] and now - st['m2o_hist'][0][0] > 5.0:
            st['m2o_hist'].popleft()
        while st['spikes'] and now - st['spikes'][0] > 1.0:
            st['spikes'].popleft()
        h = st['m2o_hist']
        yaw_rate = abs(math.degrees(wrap(h[-1][1] - h[0][1]))) / (h[-1][0] - h[0][0]) if len(h) > 1 and h[-1][0] - h[0][0] > 4.0 else None
        r = match_ratio()
        if r is None:                                                   # 지금 못 재면 예전 값으로 정상이라 하지 않음 (11-02 GPT 검토)
            st['match_hist'].clear()
        else:
            st['match_hist'].append(r); st['match_t'] = now
        rm = float(np.median(st['match_hist'])) if st['match_hist'] else None
        if rm is not None and rm < 0.60:
            st['low_since'] = st['low_since'] or now
        else:
            st['low_since'] = None
        sts = [x for x in st['scan_t'] if now - x < 1.0]
        sc = st['scan']
        m = {'scan_age': now - st['scan_t'][-1] if st['scan_t'] else None,
             'scan_hz': float(len(sts)) if st['scan_t'] and now - st['scan_t'][0] > 1.0 else None,
             'scan_pts': scan_points(sc),
             'match': rm, 'match_low_s': now - st['low_since'] if st['low_since'] else 0.0, 'match_none_s': now - st['match_t'],
             'jump_m': max((j[1] for j in st['jumps']), default=0.0), 'jump_deg': max((j[2] for j in st['jumps']), default=0.0),
             'yaw_fix_rate': yaw_rate, 'imu_age': now - st['imu_t'] if st['imu_t'] else None,
             'gyro_spikes': len(st['spikes']), 'acc_bad_s': now - st['acc_bad_since'] if st['acc_bad_since'] else 0.0}
        lvl, why = evaluate(m)
        if lvl == 2:                                                    # 올라갈 땐 바로, 내려갈 땐 원인 없어진 뒤 1 s 더 유지
            st['last_err'] = now
        if lvl >= 1:
            st['last_warn'] = now; st['reasons'] = why
        st['level'] = 2 if now - st['last_err'] < HOLD_S else 1 if now - st['last_warn'] < HOLD_S else 0
        if st['level'] == 0:
            st['reasons'] = []
        out = DiagnosticStatus(level=bytes([st['level']]), name='localization', hardware_id=BASE,
                               message=' · '.join(st['reasons']) or '정상')
        out.values = [KeyValue(key=k, value='' if v is None else f'{v:.3f}' if isinstance(v, float) else str(v)) for k, v in m.items()]
        pub.publish(out)
        nonlocal bias
        if st['m2o'] is not None:
            bias_hist.append((now, st['m2o'][2], odom_v['v'] > 0.02, st['level']))
        while bias_hist and now - bias_hist[0][0] > BIAS_WIN:
            bias_hist.popleft()
        if len(log) % 5 == 0 and not a.no_bias:                        # 1 s 마다
            bias = bias_update(bias, list(bias_hist))
            b = Vector3Stamped(); b.header.stamp = n.get_clock().now().to_msg(); b.header.frame_id = BASE; b.vector.z = bias
            pub_bias.publish(b)
        log.append({'t': round(now, 2), 'level': st['level'], 'why': st['reasons'], 'bias_deg_s': round(math.degrees(bias), 4), **{k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.items()}})
        if len(log) % 5 == 0:
            f = lambda v, k=1: '-' if v is None else f'{v:.{k}f}'
            print(f"\r  {['정상', '경고', '정지'][st['level']]} · 지도 맞음 {f(rm * 100 if rm is not None else None, 0)}% · SLAM 보정 {m['jump_m'] * 100:.0f} cm {m['jump_deg']:.1f}° · "
                  f"방향 고침 {f(yaw_rate, 2)}°/s · 자이로 보정 {math.degrees(bias):+.3f}°/s · 라이다 {f(m['scan_hz'], 0)} Hz {m['scan_pts']} 점 · {out.message[:40]}          ", end='', flush=True)

    n.create_timer(0.2, tick)
    print('위치 감시 — /localization_health (Ctrl+C 끝)')
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:                                                # Ctrl+C 순간 메시지 꺼내다 실패 (11-01 실물) — 끄는 중이면 무시
        if rclpy.ok():
            raise
    finally:
        with open(a.out, 'w') as fo:
            json.dump(log, fo)
        if not a.no_bias:
            with open(BIAS_FILE, 'w') as fb:
                fb.write(f'{bias:.6f}\n')
            print(f'\n자이로 이동 치우침 {math.degrees(bias):+.3f}°/s → {BIAS_FILE}')
        print(f'\n기록 {a.out} ({len(log)} 줄) · 정지 {sum(1 for x in log if x["level"] == 2)} · 경고 {sum(1 for x in log if x["level"] == 1)}')
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
