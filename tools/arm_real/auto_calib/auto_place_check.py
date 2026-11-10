#!/usr/bin/env python3
"""auto_place_check.py — 로봇이 스스로 집기 → 놓기 → 손목 카메라로 재기를 반복해 놓기 오차 · 반복성을 숫자로 내고 보정값을 제안

배경 (11-09 실물): 같은 빨강 선 퍽을 같은 자리에서 같은 방법 (태그 · 손목 −90°) 으로 집어 place 0.25 0 에 두 번 놓았더니
  손목 카메라로 잰 자리가 14.8 mm 다름. 칸 · 퍽마다 사람이 mm 를 보고 param set 하지 않고 로봇이 재고 계산한다.
하는 일
  - 팔 작업은 /arm_task (robot_cli robot_link.cpp 와 같은 방식): HOME 0 · PICK 1 · PLACE 2 · LOOK 5, 목표 frame base_link
  - PICK 목표 = 손목 검출 → base_link (det_pick.cpp · planning grasp 규칙, grasp_offset 0.015). control_master 가 자기 검출로 다시 맞춤
  - PLACE 목표 = (--x, --y, z 0 = 기본 높이), 방향 0,0,0,0 (모름 = control_master 기본)
  - 재기 = /perception/wrist/detections 를 --measure_s 모아 퍽마다 중앙값. control_master on_detections 와 같은 변환:
    카메라 점을 /wrist_cam/drift (frame 이 같을 때) 로 돌린 뒤 TF → base_link (차체는 멈춰 있으니 odom 을 거치지 않고 바로)
모드
  look  : 퍽 그대로. N 번 HOME → LOOK → 재기 → 같은 퍽 위치가 매번 얼마나 다른지 (LOOK 자세 · 카메라 반복성)
  place : N 번 LOOK → 재기 (집기 전) → PICK → PLACE (--x --y) → LOOK → 재기 (놓은 뒤). 오차 = 놓은 뒤 − 목표 (기본)
  lying : 회차마다 "누운 퍽을 박스에 놓고 Enter" 뒤 place 와 같이 (PICK 이 세우기 · 다시 쥐기까지 함)
쓰는 법 (젯슨, nts 뒤)
  python3 tools/arm_real/auto_calib/auto_place_check.py --n 5 --x 0.25 --y 0.0
  python3 tools/arm_real/auto_calib/auto_place_check.py --mode look --n 5
  python3 tools/arm_real/auto_calib/auto_place_check.py --mode lying --n 3 --tag 7 --home_between
  python3 tools/arm_real/auto_calib/auto_place_check.py --n 5 --apply --yes     # 끝에 place_offset_x/y_mm 를 제안값으로 바꿈
결과: ~/nt_test/runs/auto_place_<시각>.csv (회차별 모든 값) + 같은 이름 .txt (요약 · 판정 · 보정 제안)
  보정값은 --apply 없이는 바꾸지 않음 (--apply 도 place_offset_x/y_mm 만)
안전: 시작 때 무엇을 몇 번 할지 보여 주고 y/N (--yes 면 생략). 팔 작업이 실패 · 거절 · 시간 초과면 즉시 멈춤 —
  PICK · PLACE 사이면 퍽을 쥔 채일 수 있어 HOME 을 안 보냄 (사람 복구, ArmTask.action 머리말). Ctrl+C = 진행 중 목표 취소 뒤 종료
순수 계산 (쿼터니언 · drift · 중앙값 · 오차 요약 · 보정 제안) 은 rclpy 없이 import 됨 → test_auto_place_check.py
"""
import argparse
import csv
import math
import os
import re
import statistics as st
import sys
import time

GRASP_OFFSET_M = 0.015   # det_pick.hpp kGraspOffsetM = planning grasp_offset_m
TASK = {'HOME': 0, 'PICK': 1, 'PLACE': 2, 'LOOK': 5}
STD_OK_MM, STD_BAD_MM = 1.5, 3.0
# control_master PICK 로그 (pick_plan · pick_path) 에서 뽑는 것
RE_LOG = {
    'tag_fix': re.compile(r'태그 퍽 보정 \(([-+\d.]+), ([-+\d.]+)\) mm'),
    'shift': re.compile(r'쥐는 점을 \(([-+\d.]+), ([-+\d.]+)\) mm 옮김'),
    'rot_deg': re.compile(r'손목을 기본에서 ([-+\d.]+)° 돌려 쥠'),
    'depth_mm': re.compile(r'쥐는 깊이 ([-+\d.]+) mm'),
}
# 퍽 종류 → control_master 집기 보정 파라미터 (x · y)
PICK_PARAM = {'태그 선': 'pick_offset_tagged_%s_mm', '깊이 선': 'pick_offset_%s_mm',
              '태그 누움': 'pick_offset_lying_tagged_%s_mm', '깊이 누움': 'pick_offset_lying_%s_mm'}


# ---------------------------------------------------------------- 순수 계산 (rclpy 없이)
def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def qrot(q, v):
    return qmul(qmul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))[:3]


def apply_drift(p, q, drift, frame):
    """control_master on_detections 와 같음: drift = (쿼터니언, frame_id) 의 frame 이 검출 frame 과 같을 때만 카메라 점 · 방향을 돌림"""
    if drift is None or drift[1] != frame:
        return p, q
    return qrot(drift[0], p), (qmul(drift[0], q) if q is not None else None)


def to_base(p, q, tf):
    """카메라 점 · 방향 → base_link. tf = ((tx, ty, tz), (qx, qy, qz, qw)) base_link ← 카메라"""
    r = qrot(tf[1], p)
    return (r[0] + tf[0][0], r[1] + tf[0][1], r[2] + tf[0][2]), (qmul(tf[1], q) if q is not None else None)


def quat_cols(q):
    """회전 행렬의 x 열 · z 열 (= 퍽 x 축 · 긴축을 기준 프레임으로 돌린 것, planning grasp.cpp)"""
    x, y, z, w = q
    return ((1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y)),
            (2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)))


def _wrap(a, period):
    r = math.fmod(a + period / 2.0, period)
    return (r + period if r < 0.0 else r) - period / 2.0


def _perp(v, z):
    d = sum(a * b for a, b in zip(v, z))
    p = [a - d * b for a, b in zip(v, z)]
    n = math.sqrt(sum(a * a for a in p))
    return tuple(a / n for a in p) if n > 1e-6 else None


def quat_from_xz(x, z):
    """회전 행렬 열 x · z (서로 수직 단위 벡터) → 쿼터니언 (x, y, z, w), y = z × x (planning quat_from_xz, Shepperd)"""
    y = (z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0])
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
    return tuple(-v for v in q) if q[3] < 0.0 else q


def grasp_target(c, q, yaw_known, shape, off=GRASP_OFFSET_M):
    """det_pick.cpp 와 같은 규칙: 퍽 중심 c · 방향 q (base_link, None = 모름) → (그리퍼 중심이 갈 점, 목표 방향, 선 퍽?)
    방향 모름 = 선 퍽 · 위쪽 끝 · 방향 0,0,0,0. 누운 원기둥 · 모양 모르는 누운 퍽은 ValueError (control_master 는 누운 네모만 집음)"""
    if q is None:
        return (c[0], c[1], c[2] + off), (0.0, 0.0, 0.0, 0.0), True
    cx, a = quat_cols(q)
    n = math.sqrt(sum(v * v for v in a))
    if n < 1e-9:
        return (c[0], c[1], c[2] + off), (0.0, 0.0, 0.0, 0.0), True
    a = tuple(v / n for v in a)
    upright = abs(a[2]) >= math.sqrt(0.5)
    if not upright and shape != 'box':
        raise ValueError('누운 원기둥 — 안 집음 (누운 네모만)' if shape == 'cylinder' else '누운 퍽인데 모양 모름 — 누운 네모만 집음')
    if (a[2] < 0.0) if upright else (a[0] * c[0] + a[1] * c[1] > 0.0):   # 선 퍽 = 위쪽 끝, 누움 = 로봇 쪽 끝
        a = (-a[0], -a[1], -a[2])
    g = tuple(c[i] + a[i] * off for i in range(3))
    if not upright:
        ref = (0.0, 0.0, 1.0)
    elif shape == 'box' and yaw_known and math.hypot(cx[0], cx[1]) > 1e-6:
        th = _wrap(math.atan2(cx[1], cx[0]), math.pi / 2.0)
        ref = (math.cos(th), math.sin(th), 0.0)
    else:
        ref = (c[0], c[1], 0.0) if math.hypot(c[0], c[1]) > 1e-6 else (1.0, 0.0, 0.0)
    x = _perp(ref, a) or _perp((1.0, 0.0, 0.0), a) or _perp((0.0, 1.0, 0.0), a)
    return g, quat_from_xz(x, a), upright


def summarize_samples(rows):
    """재기 한 번의 표본 [{x, y, z [mm], tag, up}] → 중앙값 · 흔들림 (max−min) · 검출 수 · 태그로 잰 수 · 선/누움 (다수)"""
    xs, ys, zs = ([r[k] for r in rows] for k in 'xyz')
    return dict(n=len(rows), x=st.median(xs), y=st.median(ys), z=st.median(zs), swing_x=max(xs) - min(xs),
                swing_y=max(ys) - min(ys), tag=sum(r['tag'] for r in rows), up=sum(r['up'] for r in rows) * 2 >= len(rows))


def puck_type(m):
    return ('태그' if m['tag'] * 2 >= m['n'] else '깊이') + ' ' + ('선' if m['up'] else '누움')


def verdict(std_mm):
    if std_mm <= STD_OK_MM:
        return '반복됨: 보정으로 맞출 수 있음'
    if std_mm > STD_BAD_MM:
        return '매번 다름: 보정값으로 못 고침 (원인 먼저)'
    return '중간 (1.5 ~ 3 mm): 더 돌려 보고 판단'


def summarize_errors(ex, ey):
    """평균 · 표준편차 (표본, n=1 이면 0) · 최대 |오차| · 평균에서 가장 먼 점 [mm] · 판정 (표준편차 큰 축)"""
    mean = (st.fmean(ex), st.fmean(ey))
    std = (st.stdev(ex), st.stdev(ey)) if len(ex) > 1 else (0.0, 0.0)
    return dict(n=len(ex), mean=mean, std=std, max_abs=(max(map(abs, ex)), max(map(abs, ey))),
                max_dev=max(math.hypot(x - mean[0], y - mean[1]) for x, y in zip(ex, ey)), verdict=verdict(max(std)))


def propose_place_offset(cur, mean):
    """place_offset 새 값 = 지금 − 평균 오차 (놓은 퍽이 +3 앞이면 목표를 −3)"""
    return (cur[0] - mean[0], cur[1] - mean[1])


def propose_pick_offset(cur, mean):
    """pick_offset 대안 = 지금 + 평균 오차 (손이 퍽에서 +d 비껴 쥐면 놓인 퍽은 −d → 부호 반대)"""
    return (cur[0] + mean[0], cur[1] + mean[1])


def group_means(rows, key):
    """key(row) 별 오차 평균 — 2 회 이상인 묶음만 [(key, n, mean_x, mean_y)]"""
    G = {}
    for r in rows:
        G.setdefault(key(r), []).append((r['err_x'], r['err_y']))
    return [(k, len(v), st.fmean(e[0] for e in v), st.fmean(e[1] for e in v)) for k, v in G.items() if len(v) >= 2]


def parse_cm_log(lines):
    """control_master PICK 로그 줄들 → 태그 퍽 보정 · 쥐는 점 옮김 · 손목 돌림 · 쥔 깊이 (없으면 빈 칸)"""
    out = {k: '' for k in RE_LOG}
    for ln in lines:
        for k, r in RE_LOG.items():
            m = r.search(ln)
            if m:
                out[k] = ','.join(m.groups())
    return out


def prefix(m, p):
    return {p + k: v for k, v in m.items() if k != 'nom'}


def make_report(a, rows, cur, stop_msg):
    """요약 글 (txt 그대로) + 보정 제안 [(이름, 값)] (place_offset 만 --apply 대상)"""
    L = ['auto_place_check %s · mode %s · %d / %d 회 · 목표 (%.3f, %.3f) · 재기 %.1f s' % (
        time.strftime('%Y-%m-%d %H:%M:%S'), a.mode, len(rows), a.n, a.x, a.y, a.measure_s)]
    if stop_msg:
        L.append('멈춤: ' + stop_msg)
    for r in rows:
        if 'err_x' in r:
            L.append('%2d | 집기 전 (%.1f, %.1f) %s | 놓은 뒤 (%.1f, %.1f) n %d 흔들림 (%.1f, %.1f) | 오차 (%+.1f, %+.1f) | 태그보정 %s · 옮김 %s · 돌림 %s° · 깊이 %s' % (
                r['round'], r['before_x'], r['before_y'], r['type'], r['after_x'], r['after_y'], r['after_n'], r['after_swing_x'],
                r['after_swing_y'], r['err_x'], r['err_y'], r['tag_fix'] or '-', r['shift'] or '-', r['rot_deg'] or '-', r['depth_mm'] or '-'))
        elif 'after_x' in r:
            L.append('%2d | (%.1f, %.1f, %.1f) %s n %d 흔들림 (%.1f, %.1f)' % (
                r['round'], r['after_x'], r['after_y'], r['after_z'], r['type'], r['after_n'], r['after_swing_x'], r['after_swing_y']))
    props = []
    done = [r for r in rows if 'after_x' in r]
    if a.mode == 'look':
        if done:
            s = summarize_errors([r['after_x'] for r in done], [r['after_y'] for r in done])
            L.append('LOOK 반복성 [mm]: 평균 (%.1f, %.1f) · 표준편차 (%.2f, %.2f) · 평균에서 최대 %.1f → %s' % (*s['mean'], *s['std'], s['max_dev'], s['verdict']))
        return '\n'.join(L), props
    errs = [r for r in rows if 'err_x' in r]
    if not errs:
        L.append('오차 자료 없음 (놓기까지 간 회차가 없음)')
        return '\n'.join(L), props
    s = summarize_errors([r['err_x'] for r in errs], [r['err_y'] for r in errs])
    L.append('오차 (놓은 뒤 − 목표) [mm]: 평균 (%+.2f, %+.2f) · 표준편차 (%.2f, %.2f) · 최대 |x| %.1f |y| %.1f · 판정: %s' % (
        *s['mean'], *s['std'], *s['max_abs'], s['verdict']))
    for k, n, mx, my in group_means(errs, lambda r: r['type']) + group_means(errs, lambda r: '돌림 ' + (r['rot_deg'] or '0') + '°'):
        L.append('  %s (%d 회): 평균 (%+.2f, %+.2f)' % (k, n, mx, my))
    px, py = cur.get('place_offset_x_mm'), cur.get('place_offset_y_mm')
    if px is None or py is None:
        L.append('지금 place_offset 을 /control_master 파라미터에서 못 읽음 → 0 으로 보고 계산 (실제 값과 다를 수 있음)')
        px, py = 0.0, 0.0
    nx, ny = propose_place_offset((px, py), s['mean'])
    props = [('place_offset_x_mm', nx), ('place_offset_y_mm', ny)]
    L.append('보정 제안 (놓기 쪽): place_offset_x_mm %.1f → %.1f · place_offset_y_mm %.1f → %.1f  (= 지금 − 평균 오차)' % (px, nx, py, ny))
    types = {r['type'] for r in errs}
    if len(types) == 1:   # 같은 퍽 종류로만 돌렸으면 집기 쪽 대안도
        t = types.pop()
        kx, ky = PICK_PARAM[t] % 'x', PICK_PARAM[t] % 'y'
        cx_, cy_ = cur.get(kx), cur.get(ky)
        if cx_ is None or cy_ is None:
            L.append('지금 %s · %s 를 못 읽음 → 0 으로 보고 계산' % (kx, ky))
            cx_, cy_ = 0.0, 0.0
        ax, ay = propose_pick_offset((cx_, cy_), s['mean'])
        L.append('대안 (집기 쪽, 모두 "%s" 퍽): %s %.1f → %.1f · %s %.1f → %.1f  (= 지금 + 평균 오차: 쥘 때 손이 퍽에서 비낀 만큼 놓인 퍽은 반대로 가므로 부호 반대)' % (
            t, kx, cx_, ax, ky, cy_, ay))
        L.append('  판단 근거 (단정 아님): 오차가 쥐는 방법 (태그/깊이 · 돌림 각) 에 따라 다르면 집기 쪽 (손 안 치우침), 방법과 무관하게 같으면 놓기 쪽 — 한쪽만 바꿔 다시 돌려 확인')
    if s['verdict'].startswith('매번'):
        L.append('⚠ 표준편차가 커서 어느 보정값도 평균만 옮길 뿐 — 흔들림 (pick_settle_s · place_settle_s) · 쥐는 깊이 · 카메라 drift 부터')
    return '\n'.join(L), props


def save(rows, text):
    d = os.path.expanduser('~/nt_test/runs')
    os.makedirs(d, exist_ok=True)
    base = os.path.join(d, 'auto_place_' + time.strftime('%Y%m%d_%H%M%S'))
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(base + '.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, restval='')
        w.writeheader()
        w.writerows(rows)
    with open(base + '.txt', 'w') as f:
        f.write(text + '\n')
    return base


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--mode', choices=('look', 'place', 'lying'), default='place')
    p.add_argument('--n', type=int, default=3, help='반복 횟수 1~20')
    p.add_argument('--x', type=float, default=0.25, help='PLACE 목표 x [m, base_link]')
    p.add_argument('--y', type=float, default=0.0, help='PLACE 목표 y [m]')
    p.add_argument('--tag', default='', help='퍽이 둘 이상이면 이 태그 ID 퍽')
    p.add_argument('--measure_s', type=float, default=2.0, help='재기 모으는 시간')
    p.add_argument('--settle_s', type=float, default=0.5, help='LOOK 뒤 재기 전 기다림')
    p.add_argument('--timeout_s', type=float, default=60.0, help='팔 작업 하나 시간 초과')
    p.add_argument('--home_between', action='store_true', help='회차마다 HOME 거침 (미션과 같게)')
    p.add_argument('--apply', action='store_true', help='끝에 place_offset_x/y_mm 를 제안값으로 param set')
    p.add_argument('--yes', action='store_true', help='시작 확인 생략')
    a = p.parse_args(argv)
    if not 1 <= a.n <= 20:
        p.error('--n 은 1~20')
    return a


def confirm(a):
    plan = {'look': 'HOME → LOOK → 재기',
            'place': 'LOOK → 재기 → PICK → PLACE (%.3f, %.3f) → LOOK → 재기' % (a.x, a.y),
            'lying': 'Enter 기다림 → LOOK → 재기 → PICK (세우기) → PLACE (%.3f, %.3f) → LOOK → 재기' % (a.x, a.y)}[a.mode]
    print('할 일: [%s] × %d 회%s · 재기 %.1f s · 시간 초과 %.0f s · 보정값 %s' % (
        plan, a.n, ' (회차마다 HOME)' if a.home_between else '', a.measure_s, a.timeout_s,
        '끝에 자동 적용 (--apply, place_offset 만)' if a.apply else '출력만'))
    print('조건: 팔이 받침 (HOME) · 박스 위 퍽 하나 (아니면 --tag) · perception · control_master 켜짐')
    if not a.yes and input('진행? y/N ').strip().lower() != 'y':
        sys.exit('취소')


# ---------------------------------------------------------------- ROS
def ros_main(a):
    import rclpy
    import tf2_ros
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import QuaternionStamped
    from interfaces.action import ArmTask
    from interfaces.msg import DetectionArray
    from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    class ArmFail(Exception):
        """팔 작업 거절 · 실패 · 시간 초과 · 재기 실패 → 즉시 멈춤"""

    class Checker(Node):
        def __init__(self):
            super().__init__('auto_place_check')
            self.buf = tf2_ros.Buffer()
            self.tl = tf2_ros.TransformListener(self.buf, self)
            self.samples = None   # 재는 동안 {(label, shape, id): [표본]}, 아니면 None
            self.drift = None     # (쿼터니언, frame_id) — control_master 와 같은 토픽
            self.cm_log = None    # PICK 동안 control_master 로그 줄 모음
            self.gh = None
            self.create_subscription(DetectionArray, '/perception/wrist/detections', self.on_det, 10)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(QuaternionStamped, '/wrist_cam/drift', self.on_drift, latched)
            self.create_subscription(Log, '/rosout', self.on_log, 200)
            self.arm = ActionClient(self, ArmTask, '/arm_task')
            self.get_cli = self.create_client(GetParameters, '/control_master/get_parameters')
            self.set_cli = self.create_client(SetParameters, '/control_master/set_parameters')

        def on_drift(self, m):
            q = (m.quaternion.x, m.quaternion.y, m.quaternion.z, m.quaternion.w)
            if abs(math.sqrt(sum(v * v for v in q)) - 1.0) < 0.01:
                self.drift = (q, m.header.frame_id)

        def on_log(self, m):
            if self.cm_log is not None and m.name == 'control_master':
                self.cm_log.append(m.msg)

        def on_det(self, m):
            if self.samples is None:
                return
            for d in m.detections:
                p = d.pose.position
                if d.kind != 'puck' or not all(map(math.isfinite, (p.x, p.y, p.z))):
                    continue
                frame = d.header.frame_id or m.header.frame_id
                try:
                    t = self.buf.lookup_transform('base_link', frame, rclpy.time.Time())
                except Exception:
                    continue
                tr, r = t.transform.translation, t.transform.rotation
                tf = ((tr.x, tr.y, tr.z), (r.x, r.y, r.z, r.w))
                o = d.pose.orientation
                q = (o.x, o.y, o.z, o.w) if o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w > 0.5 else None
                pc, qc = to_base(*apply_drift((p.x, p.y, p.z), q, self.drift, frame), tf)   # 재기: control_master 와 같이 되돌린 뒤 TF
                pn, qn = to_base((p.x, p.y, p.z), q, tf)   # PICK 목표: 되돌리기 전 (robot_cli pick det 와 같음, control_master 가 25 mm 안에서 맞춤)
                up = qc is None or abs(quat_cols(qc)[1][2]) >= math.sqrt(0.5)
                self.samples.setdefault((d.label, d.shape or '?', d.id or '-'), []).append(dict(
                    x=pc[0] * 1000.0, y=pc[1] * 1000.0, z=pc[2] * 1000.0, tag=bool(d.center_from_tag), up=up, nom=(pn, qn, d.yaw_known, d.shape)))

        def spin_for(self, s):
            end = time.monotonic() + s
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)

        def wait_future(self, fut, timeout, what):
            end = time.monotonic() + timeout
            while not fut.done():
                if time.monotonic() > end:
                    raise ArmFail('%s: %.0f s 안에 답 없음' % (what, timeout))
                rclpy.spin_once(self, timeout_sec=0.05)
            return fut.result()

        def measure(self, tag):
            """LOOK 자세에서 settle_s 기다린 뒤 measure_s 모음 → 고른 퍽 요약. 퍽이 둘 이상이면 --tag 로, 아니면 멈춤"""
            self.spin_for(a.settle_s)
            self.samples = {}
            self.spin_for(a.measure_s)
            S, self.samples = self.samples, None
            seen = ', '.join('/'.join(k) for k in S) or '검출 없음'
            if tag:
                keys = [k for k in S if k[2] == tag]
                if not keys:
                    raise ArmFail('태그 %s 퍽이 손목 검출에 없음 (%s)' % (tag, seen))
            elif len(S) != 1:
                raise ArmFail('손목 검출 퍽이 %d 개 (%s) → 하나만 두거나 --tag ID (LOOK 자세 · 인지 켜짐 확인)' % (len(S), seen))
            else:
                keys = list(S)
            rows = S[keys[0]]
            m = summarize_samples(rows)
            m.update(label=keys[0][0], shape=keys[0][1], id=keys[0][2], nom=rows[-1]['nom'])
            print('  재기 %s: (%.1f, %.1f, %.1f) mm · n %d · 흔들림 (%.1f, %.1f) · %s' % (
                '/'.join(keys[0]), m['x'], m['y'], m['z'], m['n'], m['swing_x'], m['swing_y'], puck_type(m)))
            return m

        def run(self, name, x=0.0, y=0.0, q=(0.0, 0.0, 0.0, 0.0), z=0.0):
            """팔 작업 하나 → 결과 message. 거절 · 실패 · 시간 초과는 ArmFail, Ctrl+C 는 진행 중 목표 취소 뒤 다시 올림"""
            if not self.arm.wait_for_server(5.0):
                raise ArmFail('/arm_task 서버 없음 (control_master 꺼짐?)')
            g = ArmTask.Goal()
            g.task = TASK[name]
            if name in ('PICK', 'PLACE'):
                g.target.header.frame_id = 'base_link'
                g.target.header.stamp = self.get_clock().now().to_msg()
                g.target.pose.position.x, g.target.pose.position.y, g.target.pose.position.z = x, y, z
                g.target.pose.orientation.x, g.target.pose.orientation.y, g.target.pose.orientation.z, g.target.pose.orientation.w = q
            print('  팔 %s%s 보냄' % (name, ' (%.3f, %.3f, %.3f)' % (x, y, z) if name in ('PICK', 'PLACE') else ''))
            t0 = time.monotonic()
            try:
                self.gh = self.wait_future(self.arm.send_goal_async(g), 10.0, name + ' 받음 답')
                if not self.gh.accepted:
                    raise ArmFail(name + ' 거절 (바쁨 · ready 아님 · 시작 자세 · 닿지 않음 — 이유는 control_master 로그)')
                r = self.wait_future(self.gh.get_result_async(), a.timeout_s, name)
            except (ArmFail, KeyboardInterrupt):
                if self.gh is not None:
                    try:
                        self.wait_future(self.gh.cancel_goal_async(), 3.0, '취소')
                    except Exception:
                        pass
                raise
            finally:
                self.gh = None
            ok = r.status == GoalStatus.STATUS_SUCCEEDED and r.result.success
            print('  팔 %s → %s %.1f s %s' % (name, '성공' if ok else '실패', time.monotonic() - t0, r.result.message))
            if not ok:
                raise ArmFail('%s 실패: %s' % (name, r.result.message))
            return r.result.message

        def get_params(self, names):
            """/control_master 파라미터 (double 만) → {이름: 값}. 못 읽으면 None"""
            try:
                if not self.get_cli.wait_for_service(2.0):
                    return None
                res = self.wait_future(self.get_cli.call_async(GetParameters.Request(names=names)), 3.0, 'param get')
            except Exception:
                return None
            return {n: v.double_value for n, v in zip(names, res.values) if v.type == ParameterType.PARAMETER_DOUBLE}

        def set_params(self, kv):
            """ros2 param set 과 같음 (--apply 때만)"""
            try:
                if not self.set_cli.wait_for_service(2.0):
                    return False
                req = SetParameters.Request(parameters=[
                    Parameter(name=k, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))) for k, v in kv])
                res = self.wait_future(self.set_cli.call_async(req), 3.0, 'param set')
            except Exception:
                return False
            return all(r.successful for r in res.results)

    rclpy.init()
    node = Checker()
    rows, held, stop = [], False, ''
    try:
        for i in range(1, a.n + 1):
            print('\n=== 회차 %d / %d ===' % (i, a.n))
            row = {'round': i}
            if a.mode == 'look':
                node.run('HOME')
                node.run('LOOK')
                m = node.measure(a.tag)
                row.update(prefix(m, 'after_'), type=puck_type(m))
            else:
                if a.mode == 'lying':
                    input('  누운 퍽을 박스에 놓고 Enter (Ctrl+C = 끝) ')
                if a.home_between and i > 1:
                    node.run('HOME')
                node.run('LOOK')
                b = node.measure(a.tag)
                row.update(prefix(b, 'before_'), type=puck_type(b))
                c, q, yaw_known, shape = b['nom']
                try:
                    g, gq, _ = grasp_target(c, q, yaw_known, shape)
                except ValueError as e:
                    raise ArmFail('PICK 목표: ' + str(e))
                node.cm_log = []
                held = True   # 여기서 놓기까지 사이에 멈추면 퍽을 쥔 채일 수 있음
                node.run('PICK', g[0], g[1], gq, g[2])
                row.update(parse_cm_log(node.cm_log))
                node.cm_log = None
                node.run('PLACE', a.x, a.y)
                held = False
                node.run('LOOK')
                m = node.measure(a.tag)
                row.update(prefix(m, 'after_'), err_x=m['x'] - a.x * 1000.0, err_y=m['y'] - a.y * 1000.0)
                print('  집기 전 (%.1f, %.1f) %s → 놓은 뒤 (%.1f, %.1f) · 오차 (%+.1f, %+.1f) mm · 돌림 %s°' % (
                    b['x'], b['y'], row['type'], m['x'], m['y'], row['err_x'], row['err_y'], row['rot_deg'] or '-'))
            rows.append(row)
    except ArmFail as e:
        stop = str(e)
    except (KeyboardInterrupt, EOFError):
        stop = 'Ctrl+C — 진행 중 목표 취소하고 종료'
    except Exception as e:   # rclpy 종료 등
        stop = '예외: %r' % (e,)
    if stop:
        print('\n멈춤: ' + stop)
        if held:
            print('⚠ 퍽을 쥔 채일 수 있음 → HOME 을 보내지 않음. 사람이 퍽을 빼고 복구 (ArmTask.action: 쥔 채 HOME 거절)')
    names = ['place_offset_x_mm', 'place_offset_y_mm'] + [PICK_PARAM[t] % ax for t in PICK_PARAM for ax in 'xy']
    cur = node.get_params(names)
    if cur is None:
        print('/control_master 파라미터를 못 읽음 → 지금 값 0 으로 보고 계산')
        cur = {}
    text, props = make_report(a, rows, cur, stop)
    print('\n' + text)
    base = save(rows, text)
    print('저장: %s.csv · .txt' % base)
    if a.apply and props and not stop:
        ok = node.set_params(props)
        print('적용 (%s): %s' % ('성공' if ok else '실패', ' · '.join('%s=%.1f' % kv for kv in props)))
    elif a.apply:
        print('적용 안 함 (멈췄거나 제안 없음) — 값은 위 제안대로 ros2 param set 으로')
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


def main():
    a = parse_args()
    confirm(a)
    ros_main(a)


if __name__ == '__main__':
    main()
