#!/usr/bin/env python3
"""실물 bag 로 지역 계획기 입력 확인 (오프라인, torch 없음). 시스템 Lyrical 파이썬.

A. 몸 점 · 가림 (원본 /lidar_front/scan · /lidar_rear/scan + /joint_states + /tf_static 있을 때):
   - 몸 상자 (scan_merger body_half) 밖인데 0.30 m 안 점 = 걸러지지 않은 자기 몸 점 후보 (넓은 곳에서 녹화했으면 0 이어야 함) → 방향별 개수
   - 가림표 (make_lidar_occlusion.py) 가 가린다고 한 빔 중 실물도 몸에 맞았거나 (몸 상자 안 점) 안 돌아온 비율 · 반대로 실물 몸 점 중 가림표가 가린다고 한 비율
B. 관측 분포 (/scan + /joint_states + /odom, 있으면 /cmd_vel_nav): 정책 관측 중 라이다 36 · 바퀴 상태 22 를 배포 노드와 같은 식으로 만들어
   정책 npz 의 정규화 mean · std 와 비교 → 칸마다 z 평균 · |z| > 4 비율 (학습 때 본 적 없는 값 찾기). 경로 · 목표 관측은 bag 에 없어 안 봄
녹화 예) ros2 bag record /lidar_front/scan /lidar_rear/scan /scan /joint_states /odom /tf /tf_static /cmd_vel_nav
실행 예) python3 rl/tasks/local_planner/deploy/check_bag.py <bag 폴더> --policy <exported/policy_N.npz> --cfg <run>/cfg.json
"""
import argparse
import json
import math
import sys
import warnings
from pathlib import Path

import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

warnings.filterwarnings('ignore', category=DeprecationWarning)      # rosbag2_py read_next
HERE = Path(__file__).resolve().parent
SIM = HERE.parents[3]
REPO = SIM.parents[1]
sys.path.insert(0, str(HERE))

p = argparse.ArgumentParser()
p.add_argument('bag')
p.add_argument('--policy', required=True, help='export.py 의 policy_N.npz (정규화 mean · std)')
p.add_argument('--cfg', required=True, help='학습 실행 cfg.json')
p.add_argument('--occlusion', default=str(SIM / 'rl' / 'data' / 'local_planner_nav2' / 'lidar_occlusion.npz'))
p.add_argument('--swerve_yaml', default='', help='cfg.json 경로가 이 PC 에 없을 때')
p.add_argument('--near', type=float, default=0.30, help='몸 상자 밖 이 반경 안 점 = 자기 몸 점 후보 [m]')
a = p.parse_args()

c = json.loads(Path(a.cfg).read_text())['env']
sw = yaml.safe_load(open(a.swerve_yaml or c['swerve_yaml']))
STEER, DRIVE = sw['joints']['steer'], sw['joints']['drive']
RMIN, RMAX = np.array(sw['steer_range_rad']['min']), np.array(sw['steer_range_rad']['max'])
MAXW = float(sw['hardware_max']['wheel_speed'])
nav = yaml.safe_load((REPO / 'ros2_ws' / 'src' / 'navigation' / 'config' / 'navigation.yaml').read_text())
BX, BY = nav['scan_merger']['ros__parameters']['body_half']
LIM3 = np.array([c['vmax'], c['vmax'], c['wmax']])
NB, RM, CLIP = c['lidar_rays'], c['lidar_max'], c['lidar_obs_clip']

# ---------------------------------------------------------------- bag 읽기
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=a.bag), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
msgs = {k: [] for k in ('/lidar_front/scan', '/lidar_rear/scan', '/scan', '/joint_states', '/odom', '/cmd_vel_nav', '/tf_static')}
while r.has_next():
    topic, data, t = r.read_next()
    if topic in msgs:
        msgs[topic].append((t * 1e-9, deserialize_message(data, get_message(types[topic]))))
print('토픽 개수: ' + ', '.join(f'{k} {len(v)}' for k, v in msgs.items()))
js_t = np.array([t for t, _ in msgs['/joint_states']])


def joints_at(t):
    if not len(js_t):
        return None
    m = msgs['/joint_states'][int(np.clip(np.searchsorted(js_t, t), 0, len(js_t) - 1))][1]
    idx = {n: i for i, n in enumerate(m.name)}
    if not all(n in idx for n in STEER + DRIVE):
        return None
    q = np.array([m.position[idx[n]] for n in STEER])
    w = np.array([m.velocity[idx[n]] for n in DRIVE]) if m.velocity else np.zeros(4)
    return q, w


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# ---------------------------------------------------------------- A. 몸 점 · 가림
mount = {}
for _, m in msgs['/tf_static']:
    for tr in m.transforms:
        mount[(tr.header.frame_id, tr.child_frame_id)] = tr.transform


def base_from(frame):
    """base_footprint ← frame (tf_static 사슬, 2D)"""
    x, y, yw, f = 0.0, 0.0, 0.0, frame
    for _ in range(10):
        if f == 'base_footprint':
            return x, y, yw
        hit = [(pa, tr) for (pa, ch), tr in mount.items() if ch == f]
        if not hit:
            return None
        pa, tr = hit[0]
        ty = yaw_of(tr.rotation)
        x, y = tr.translation.x + math.cos(ty) * x - math.sin(ty) * y, tr.translation.y + math.sin(ty) * x + math.cos(ty) * y
        yw += ty
        f = pa
    return None


occ = np.load(a.occlusion) if Path(a.occlusion).exists() else None
if msgs['/lidar_front/scan'] or msgs['/lidar_rear/scan']:
    print('\n[A] 몸 점 · 가림 (원본 스캔)')
    near_dir = np.zeros(36, int)
    n_scan, n_near = 0, 0
    agree = {'pred_occ': 0, 'pred_occ_real_self': 0, 'real_self': 0, 'real_self_pred_occ': 0}
    for li, side in enumerate(('front', 'rear')):
        for t, m in msgs[f'/lidar_{side}/scan']:
            T = base_from(m.header.frame_id)
            if T is None:
                print(f'  {m.header.frame_id}: tf_static 에 base_footprint 까지 없음 → 건너뜀')
                break
            rr = np.asarray(m.ranges, float)
            ang = m.angle_min + np.arange(len(rr)) * m.angle_increment
            valid = np.isfinite(rr) & (rr >= m.range_min) & (rr <= m.range_max)
            bx = T[0] + np.cos(ang + T[2]) * rr
            by = T[1] + np.sin(ang + T[2]) * rr
            inside = valid & (np.abs(bx) <= BX) & (np.abs(by) <= BY)
            near = valid & ~inside & (np.hypot(bx, by) < a.near)
            n_scan += 1
            n_near += int(near.sum())
            np.add.at(near_dir, (np.round(np.arctan2(by[near], bx[near]) / math.radians(10)).astype(int)) % 36, 1)
            if occ is not None:
                jw = joints_at(t)
                if jw is None:
                    continue
                Q = occ['occ_module'].shape[2]
                qi = np.clip(((np.mod(jw[0] + math.pi, 2 * math.pi)) / (2 * math.pi) * Q).astype(int), 0, Q - 1)
                blk = np.minimum(occ['occ_static'][li], occ['occ_module'][li, np.arange(4), qi].min(0))      # (N,)
                N = blk.shape[0]
                bang = occ['beam_angle'][li]
                j = np.round(np.mod(ang + T[2] - bang[0], 2 * math.pi) / (2 * math.pi / N)).astype(int) % N
                pred = np.isfinite(blk[j]) & (blk[j] < 0.35)
                real_self = inside | ~valid
                agree['pred_occ'] += int(pred.sum())
                agree['pred_occ_real_self'] += int((pred & real_self).sum())
                agree['real_self'] += int(inside.sum())
                agree['real_self_pred_occ'] += int((inside & pred).sum())
    print(f'  스캔 {n_scan} 장, 몸 상자 밖 {a.near} m 안 점 합 {n_near} (장당 {n_near / max(n_scan, 1):.1f}) — 넓은 곳 녹화면 0 이어야 함')
    if n_near:
        top = np.argsort(near_dir)[::-1][:6]
        print('  방향 (몸 기준 °, 개수): ' + ', '.join(f'{k * 10 if k * 10 <= 180 else k * 10 - 360}° {near_dir[k]}' for k in top if near_dir[k]))
        print('  → 몸 상자 (navigation.yaml scan_merger body_half) 를 키우거나 그 방향 부품 확인')
    if occ is not None and agree['pred_occ']:
        print(f'  가림표가 가린다고 한 빔 중 실물도 몸 점 · 안 돌아옴: {agree["pred_occ_real_self"] / agree["pred_occ"] * 100:.0f} % '
              f'(낮으면 가림표가 실물보다 많이 가림)')
        if agree['real_self']:
            print(f'  실물 몸 점 중 가림표도 가린다고 한 것: {agree["real_self_pred_occ"] / agree["real_self"] * 100:.0f} % (낮으면 가림표에 없는 몸 부분)')
else:
    print('\n[A] 원본 /lidar_*/scan 없음 → 건너뜀')

# ---------------------------------------------------------------- B. 관측 분포
pol = np.load(a.policy)
mean, std, eps = pol['mean'], pol['std'], float(pol['eps'])
L0 = 2 * c['path_obs_points'] + 5                         # 경로 32 · 목표 5 다음이 라이다
F0 = L0 + NB                                              # 그 다음 첫 프레임 (22)
od_t = np.array([t for t, _ in msgs['/odom']])
cv_t = np.array([t for t, _ in msgs['/cmd_vel_nav']])
rows = []
for t, m in msgs['/scan']:
    rr = np.asarray(m.ranges, float)
    ang = m.angle_min + np.arange(len(rr)) * m.angle_increment
    ok = np.isfinite(rr) & (rr >= m.range_min) & (rr <= m.range_max)
    x, y = rr[ok] * np.cos(ang[ok]), rr[ok] * np.sin(ang[ok])
    keep = ~((np.abs(x) <= BX) & (np.abs(y) <= BY))
    out = np.full(NB, RM)
    np.minimum.at(out, np.round(np.arctan2(y[keep], x[keep]) / (2 * math.pi / NB)).astype(int) % NB, np.hypot(x[keep], y[keep]))
    jw = joints_at(t)
    if jw is None or not len(od_t):
        continue
    q, w = jw
    o = msgs['/odom'][int(np.clip(np.searchsorted(od_t, t), 0, len(od_t) - 1))][1].twist.twist
    odom = np.array([o.linear.x, o.linear.y, o.angular.z])
    last = np.zeros(3)
    if len(cv_t):
        cm = msgs['/cmd_vel_nav'][int(np.clip(np.searchsorted(cv_t, t), 0, len(cv_t) - 1))][1].twist
        last = np.array([cm.linear.x, cm.linear.y, cm.angular.z])
    rpos = (q - RMIN) / (RMAX - RMIN) * 2 - 1
    frame = np.concatenate([np.sin(q), np.cos(q), rpos, w / MAXW, odom / LIM3, last / LIM3])
    rows.append(np.concatenate([np.clip(out, 0, CLIP) / CLIP, frame]))
if rows:
    X = np.array(rows)
    idx = np.r_[L0:L0 + NB, F0:F0 + 22]
    z = (X - mean[idx]) / (std[idx] + eps)
    names = [f'라이다 {k * 10}°' for k in range(NB)] + [f'sin q{i}' for i in range(4)] + [f'cos q{i}' for i in range(4)] + \
            [f'조향 범위 위치 {i}' for i in range(4)] + [f'바퀴 {i}' for i in range(4)] + ['odom vx', 'odom vy', 'odom wz', '명령 vx', '명령 vy', '명령 wz']
    bad = np.abs(z) > 4
    print(f'\n[B] 관측 {len(X)} 개 (라이다 36 · 바퀴 상태 22) vs 학습 정규화: |z| > 4 비율 전체 {bad.mean() * 100:.2f} %')
    order = np.argsort(-bad.mean(0))
    for k in order[:8]:
        if bad[:, k].mean() > 0:
            print(f'  {names[k]:14s} z 평균 {z[:, k].mean():+6.2f} · |z|>4 {bad[:, k].mean() * 100:5.1f} % (bag 평균 {X[:, k].mean():+.3f}, 학습 평균 {mean[idx][k]:+.3f} ± {std[idx][k]:.3f})')
    if not bad.any():
        print('  학습 범위 밖 값 없음')
else:
    print('\n[B] /scan · /joint_states · /odom 이 다 있어야 함 → 건너뜀')
