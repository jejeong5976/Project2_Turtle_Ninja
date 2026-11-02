#!/usr/bin/env python3
"""위치 추정 센서 점검 — run_robot.sh locate (또는 slam) 이 떠 있을 때 다른 터미널에서.

  python3 tools/diagnostics/sensor_check.py            가만히 20 s 재고 → 제자리 한 바퀴 → 앞(또는 빈 쪽)으로 0.5 m 갔다 옴
  python3 tools/diagnostics/sensor_check.py --still    가만히 재기만 (움직이지 않음)
  python3 tools/diagnostics/sensor_check.py --no-rot --dist 1.0    회전 없이 가만히 → 1 m 갔다 옴 (직진 중 방향 오차가 회전 끝 자락 때문인지)

1. 가만히 (20 s): 토픽 주기 (IMU · odom · 바퀴 joint_states · 앞 · 뒤 라이다 · 합친 /scan),
   IMU 자이로 바이어스 · 잡음 · 중력 방향(장착 뒤집힘), odom 이 저절로 움직이는 양(자이로 드리프트),
   라이다마다 지도 벽까지 거리 (가까운 점 · 먼 점 따로 — 먼 점만 크면 장착 방향(yaw) 틀림)
2. 제자리 회전 360° (odom 기준, 0.3 rad/s): odom · 자이로 적분 · SLAM(map) 회전량 비교 → IMU 배율 · 바퀴 회전 계산
3. 직진 0.5 m 갔다 옴 (0.06 m/s, 막히지 않은 방향 고름): odom 거리 vs SLAM 거리 → 바퀴 지름 · 미끄러짐
안전: 움직이기 전 /scan · /odom · IMU · map · odom TF 가 모두 와야 시작. 움직이는 동안 스캔 · TF 가
  0.5 s 넘게 끊기면 · 시간이 넘으면 · 3 s 동안 5 mm 도 안 나가면 0 명령 · 끝. 앞 물체 = body_footprint (path_follow 와 같은 식).
  아무 키 = 정지 · 끝.
"""
import argparse
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
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "drive"))   # tools/drive/body_footprint
from body_footprint import obstacle_hit, self_mask   # noqa: E402

BASE = 'base_footprint'
IMU_TOPIC = '/imu_sensor_broadcaster/imu'
SCANS = ('/lidar_front/scan', '/lidar_rear/scan', '/scan')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rel(p0, p1):
    """p0 → p1 상대 이동 (p0 몸 기준) [m, m, rad]"""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap(p1[2] - p0[2])


def wall_dist(grid, cap=12):
    """점유 칸까지 거리 [칸] (4 · 8 이웃 번갈아 늘림 ≈ 팔각, cap 칸에서 자름) — scipy 없이"""
    d = np.where(grid, 0, cap).astype(np.int16)
    reach = grid.copy()
    for k in range(1, cap):
        g = reach.copy()
        g[1:] |= reach[:-1]; g[:-1] |= reach[1:]; g[:, 1:] |= reach[:, :-1]; g[:, :-1] |= reach[:, 1:]
        if k % 2 == 0:
            g[1:, 1:] |= reach[:-1, :-1]; g[:-1, :-1] |= reach[1:, 1:]; g[1:, :-1] |= reach[:-1, 1:]; g[:-1, 1:] |= reach[1:, :-1]
        d[g & ~reach] = k
        reach = g
    return d


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--still', action='store_true', help='가만히 재기만')
    ap.add_argument('--no-rot', action='store_true', help='제자리 회전 건너뜀 (가만히 바로 뒤 직진 — 회전 끝 자락이 자이로 바이어스에 섞이는지 가리기)')
    ap.add_argument('--dist', type=float, default=0.5, help='직진 거리 [m] (0.3~1.5)')
    ap.add_argument('--secs', type=float, default=20.0, help='가만히 재는 시간 [s]')
    a = ap.parse_args()
    a.dist = max(0.3, min(1.5, a.dist))

    rclpy.init()
    n = rclpy.create_node('sensor_check')
    tfb = Buffer(); TransformListener(tfb, n)
    pub = n.create_publisher(TwistStamped, '/cmd_vel', 10)
    cnt = {k: 0 for k in (IMU_TOPIC, '/odom', '/joint_states') + SCANS}
    st = {'gz': [], 'acc': [], 'gz_t': None, 'gz_int': 0.0, 'gz_bias': 0.0, 'map': None, 'scan': {}, 'pts': None, 'scan_t': 0.0}

    def on_imu(m):
        cnt[IMU_TOPIC] += 1
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        st['gz'].append(m.angular_velocity.z); st['acc'].append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
        if st['gz_t'] is not None:
            st['gz_int'] += (m.angular_velocity.z - st['gz_bias']) * max(0.0, min(0.1, t - st['gz_t']))
        st['gz_t'] = t

    def on_scan(topic):
        def f(m):
            cnt[topic] += 1
            st['scan'][topic] = m
            if topic == '/scan':
                st['scan_t'] = time.monotonic()
        return f

    def on_map(m):
        st['map'] = m

    n.create_subscription(Imu, IMU_TOPIC, on_imu, qos_profile_sensor_data)
    n.create_subscription(Odometry, '/odom', lambda m: cnt.__setitem__('/odom', cnt['/odom'] + 1), 10)
    n.create_subscription(JointState, '/joint_states', lambda m: cnt.__setitem__('/joint_states', cnt['/joint_states'] + 1), 10)
    for t in SCANS:
        n.create_subscription(LaserScan, t, on_scan(t), qos_profile_sensor_data)
    n.create_subscription(OccupancyGrid, '/map', on_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))

    def pose(frame):
        """frame → 몸 자세. TF 가 0.5 s 넘게 묵었으면 None (정적 TF 시각 0 은 통과)"""
        try:
            t = tfb.lookup_transform(frame, BASE, Time())
        except Exception:
            return None
        stamp = Time.from_msg(t.header.stamp)
        if stamp.nanoseconds and (n.get_clock().now() - stamp).nanoseconds * 1e-9 > 0.5:
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def spin(sec):
        t1 = time.monotonic() + sec
        while time.monotonic() < t1:
            rclpy.spin_once(n, timeout_sec=0.01)

    def send(vx=0.0, vy=0.0, wz=0.0):
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = BASE
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = vx, vy, wz
        pub.publish(m)

    def body_pts():
        """합친 /scan 점 (몸 기준, 자기 몸 직사각형 밖). 못 받았거나 0.5 s 넘게 묵었으면 None"""
        m = st['scan'].get('/scan')
        if m is None or time.monotonic() - st['scan_t'] > 0.5:
            return None
        r = np.asarray(m.ranges, float); ang = m.angle_min + m.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        return self_mask(np.c_[r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])])

    def halt(why):
        for _ in range(5):
            send(); spin(0.02)
        print(f'  ✗ 정지 · 끝: {why}')

    tty_in = sys.stdin.isatty()
    old = termios.tcgetattr(sys.stdin) if tty_in else None

    def key():
        return tty_in and bool(select.select([sys.stdin], [], [], 0)[0]) and sys.stdin.read(1)

    try:
        if tty_in:
            tty.setcbreak(sys.stdin.fileno()); termios.tcflush(sys.stdin, termios.TCIFLUSH)
        # ── 1. 가만히 ──
        print(f'1. 가만히 {a.secs:.0f} s — 로봇 건드리지 말 것')
        spin(1.0)
        for k in cnt:
            cnt[k] = 0
        st['gz'].clear(); st['acc'].clear()
        o0, m0, t0 = pose('odom'), pose('map'), time.monotonic()
        spin(a.secs)
        dt = time.monotonic() - t0
        o1, m1 = pose('odom'), pose('map')
        print('  토픽 주기 [Hz]:  ' + '  '.join(f'{k} {cnt[k] / dt:.1f}' for k in cnt))
        miss = [k for k in cnt if cnt[k] == 0]
        if miss:
            print('  ✗ 안 오는 토픽:', ', '.join(miss))
        if st['gz']:
            gz = np.array(st['gz']); acc = np.array(st['acc']); g = acc.mean(0)
            st['gz_bias'] = float(gz.mean())
            print(f'  IMU 자이로 z: 평균 {math.degrees(gz.mean()):+.3f}°/s (= {math.degrees(gz.mean()) * 60:+.1f}°/분 — 컨트롤러가 켤 때 imu_bias_time_s(5 s) 평균으로 빼는 값) · 잡음 {math.degrees(gz.std()):.3f}°/s')
            print(f'  IMU 가속도 평균 ({g[0]:+.2f}, {g[1]:+.2f}, {g[2]:+.2f}) m/s² · 크기 {np.linalg.norm(g):.2f}'
                  + ('  ✗ z 가 음수 — 뒤집혀 붙음?' if g[2] < 0 else '') + (f'  ⚠ 기울기 {math.degrees(math.atan2(math.hypot(g[0], g[1]), abs(g[2]))):.1f}°' if math.hypot(g[0], g[1]) > 0.3 else ''))
        if o0 and o1:
            r = rel(o0, o1)
            print(f'  odom 저절로 움직임 {dt:.0f} s: {math.hypot(r[0], r[1]) * 1000:.1f} mm · {math.degrees(r[2]):+.2f}° (= {math.degrees(r[2]) / dt * 60:+.2f}°/분)  — 0.2°/분 넘으면 자이로 드리프트')
        if m0 and m1:
            r = rel(m0, m1)
            print(f'  SLAM 위치 흔들림: {math.hypot(r[0], r[1]) * 1000:.1f} mm · {math.degrees(r[2]):+.2f}°')
        # 라이다 ↔ 지도 벽
        mp = st['map']
        if mp is None:
            print('  ✗ /map 못 받음 — 라이다 · 지도 비교 건너뜀')
        else:
            W, H, res = mp.info.width, mp.info.height, mp.info.resolution
            ox, oy = mp.info.origin.position.x, mp.info.origin.position.y
            grid = np.asarray(mp.data, np.int16).reshape(H, W) > 50
            D = wall_dist(grid) * res
            for topic in SCANS[:2]:
                m = st['scan'].get(topic)
                if m is None:
                    continue
                try:
                    tr = tfb.lookup_transform('map', m.header.frame_id, Time()).transform
                except Exception:
                    print(f'  ✗ {topic}: map → {m.header.frame_id} TF 없음'); continue
                r = np.asarray(m.ranges, float); ang = m.angle_min + m.angle_increment * np.arange(len(r))
                ok = np.isfinite(r) & (r > 0.25) & (r < 4.0)
                yw = yaw_of(tr.rotation); c, s = math.cos(yw), math.sin(yw)
                lx, ly = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
                X, Y = tr.translation.x + c * lx - s * ly, tr.translation.y + s * lx + c * ly
                u, v = np.floor((X - ox) / res).astype(int), np.floor((Y - oy) / res).astype(int)
                inb = (u >= 0) & (u < W) & (v >= 0) & (v < H)
                d = np.full(len(u), np.inf); d[inb] = D[v[inb], u[inb]]   # 지도 밖 = 맞지 않음 
                rr = r[ok]
                near, far = d[(rr < 1.5) & inb], d[(rr > 2.0) & inb]
                txt = f'  {topic}: 점 {ok.sum()} · 지도 벽까지 중앙 {np.median(d) * 100:.1f} cm · 5 cm 안 {np.mean(d < 0.05) * 100:.0f}% (지도 밖 {np.mean(~inb) * 100:.0f}%)'
                if len(near) > 20 and len(far) > 20:
                    txt += f' · 가까운(<1.5 m) {np.median(near) * 100:.1f} / 먼(>2 m) {np.median(far) * 100:.1f} cm'
                    if np.median(far) > 2 * np.median(near) + 0.02:
                        txt += '  ⚠ 먼 점만 큼 → 장착 방향(yaw) 의심'
                print(txt + '  (지도에 없는 박스 · 사람 점도 섞임)')
        if a.still:
            return
        need = [k for k in ('/scan', '/odom', IMU_TOPIC) if cnt[k] == 0]
        if need or body_pts() is None or not (pose('odom') and pose('map')):
            print(f"✗ 움직이는 시험 안 함 — 안 오는 것: {', '.join(need) or ''} {'' if body_pts() is not None else '/scan(지금)'} "
                  f"{'' if pose('odom') else 'odom TF'} {'' if pose('map') else 'map TF'}"); return
        # ── 2. 제자리 360° ──
        if a.no_rot:
            print('2. 회전 건너뜀 (--no-rot)')
        else:
            p = body_pts()
            if p is None:
                print('✗ 스캔 끊김 — 회전 안 함'); return
            if obstacle_hit(p, 0, 0, 0.3) is not None or (len(p) and np.min(np.hypot(p[:, 0], p[:, 1])) < 0.30):
                print('✗ 몸 0.30 m 안에 점 — 회전 안 함 (넓은 곳으로 옮길 것)'); return
            print('2. 제자리 왼쪽으로 한 바퀴 (0.3 rad/s, 약 25 s) — 아무 키 = 정지')
            o0, m0 = pose('odom'), pose('map'); st['gz_int'] = 0.0
            acc_yaw, last, t_end = 0.0, o0[2], time.monotonic() + 40
            stale_since = None
            while acc_yaw < 2 * math.pi - 0.02:
                rclpy.spin_once(n, timeout_sec=0.01)
                if key():
                    halt('키'); return
                if time.monotonic() > t_end:
                    halt('회전 시간 초과 (40 s)'); return
                o, p = pose('odom'), body_pts()
                if o is None or p is None:                              # 끊기면 0 명령, 0.5 s 넘으면 끝
                    send(); stale_since = stale_since or time.monotonic()
                    if time.monotonic() - stale_since > 0.5:
                        halt('odom TF · 스캔 끊김'); return
                    continue
                stale_since = None
                acc_yaw += wrap(o[2] - last); last = o[2]
                wz = max(0.08, min(0.3, 0.8 * (2 * math.pi - acc_yaw)))
                if obstacle_hit(p, 0, 0, wz) is not None:
                    halt('가까운 점'); return
                send(wz=wz)
            for _ in range(10):
                send(); spin(0.05)
            spin(2.0)                                                       # SLAM 이 따라잡게
            o1, m1 = pose('odom'), pose('map')
            if o1 is None or m1 is None:
                print('  ✗ 회전 뒤 odom · map TF 없음 — 회전 결과 측정 실패 (SLAM · locate 확인)'); return
            od = acc_yaw + wrap(o1[2] - last)
            md = od + wrap((m1[2] - m0[2]) - od)
            gi = st['gz_int']
            print(f'  회전량: odom {math.degrees(od):.1f}° · 자이로 적분 {math.degrees(gi):.1f}° · SLAM {math.degrees(md):.1f}°')
            print(f'  → odom 오차 {math.degrees(od - md):+.1f}° ({(od / md - 1) * 100 if abs(md) > 1e-6 else float('nan'):+.1f}%) · 자이로 오차 {math.degrees(gi - md):+.1f}° ({(gi / md - 1) * 100 if abs(md) > 1e-6 else float('nan'):+.1f}%) · 제자리 밀림 {math.hypot(m1[0] - m0[0], m1[1] - m0[1]) * 1000:.0f} mm')
        # ── 3. 직진 a.dist 갔다 옴 ──
        p = body_pts()
        if p is None:
            halt('스캔 끊김'); return
        best = None
        for name, d in (('앞', (1, 0)), ('뒤', (-1, 0)), ('왼', (0, 1)), ('오른', (0, -1))):
            al, la = p @ np.array(d), p @ np.array((-d[1], d[0]))
            free = np.min(al[(al > 0) & (np.abs(la) < 0.25)], initial=9.0)
            if free > a.dist + 0.5:
                best = (name, d); break
        if best is None:
            print(f'✗ {a.dist + 0.5:.1f} m 비어 있는 방향 없음 — 직진 안 함'); return
        name, d = best
        print(f'3. {name}으로 {a.dist:.1f} m 갔다 옴 (0.06 m/s) — 아무 키 = 정지')
        res_ = []
        for sgn in (1, -1):
            o0, m0 = pose('odom'), pose('map'); st['gz_int'] = 0.0      # 자이로 적분 = 가만히 20 s 평균 바이어스 고정 (컨트롤러는 멈출 때마다 바이어스를 고침)
            if o0 is None or m0 is None:
                halt('odom · map TF 없음'); return
            t_end, stale_since, best_done, best_t = time.monotonic() + a.dist / 0.02 + 10.0, None, 0.0, time.monotonic()
            while True:
                rclpy.spin_once(n, timeout_sec=0.01)
                now = time.monotonic()
                if key():
                    halt('키'); return
                if now > t_end:
                    halt('직진 시간 초과'); return
                o, p = pose('odom'), body_pts()
                if o is None or p is None:
                    send(); stale_since = stale_since or now
                    if now - stale_since > 0.5:
                        halt('odom TF · 스캔 끊김'); return
                    continue
                stale_since = None
                r = rel(o0, o)
                done = r[0] * d[0] * sgn + r[1] * d[1] * sgn
                if done >= a.dist:
                    break
                if done > best_done + 0.005:
                    best_done, best_t = done, now
                elif now - best_t > 3.0:
                    halt('3 s 동안 5 mm 도 못 나감'); return
                v = max(0.02, min(0.06, 0.5 * (a.dist - done) + 0.02))
                vx, vy = v * d[0] * sgn, v * d[1] * sgn
                if obstacle_hit(p, vx, vy, 0.0) is not None:
                    halt('앞에 점'); return
                send(vx=vx, vy=vy)
            for _ in range(10):
                send(); spin(0.05)
            spin(2.0)
            o1, m1 = pose('odom'), pose('map')
            if o1 is None or m1 is None:
                print('  ✗ 직진 뒤 odom · map TF 없음 — 직진 결과 측정 실패 (SLAM · locate 확인)'); return
            ro, rm = rel(o0, o1), rel(m0, m1)
            res_.append((math.hypot(*ro[:2]), math.hypot(*rm[:2]), ro[2], rm[2]))
            print(f'  {"가기" if sgn > 0 else "오기"}: odom {res_[-1][0] * 1000:.0f} mm · SLAM {res_[-1][1] * 1000:.0f} mm ({(res_[-1][0] / max(res_[-1][1], 1e-3) - 1) * 100:+.1f}%) · '
                  f'방향 odom {math.degrees(ro[2]):+.2f}° / 자이로(고정 바이어스) {math.degrees(st["gz_int"]):+.2f}° / SLAM {math.degrees(rm[2]):+.2f}°')
        print('판정 기준: 회전 오차 ±2% · 직진 거리 ±3% · 가만히 odom 0.2°/분 · 라이다 벽 거리 중앙 3 cm 안 이면 정상')
        print('직진 방향: 자이로(고정)만 SLAM 과 맞고 odom 이 틀리면 → 컨트롤러가 멈출 때 바이어스를 잘못 고침 · 둘 다 틀리면 → 움직일 때 IMU 자체 (진동)')
    finally:
        try:
            for _ in range(5):
                send(); time.sleep(0.05)
        except Exception:
            pass
        if old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
