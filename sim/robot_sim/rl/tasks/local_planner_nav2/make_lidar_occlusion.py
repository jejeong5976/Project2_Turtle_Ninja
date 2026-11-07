#!/usr/bin/env python3
"""라이다 자기 몸 가림표 → rl/data/local_planner_nav2/lidar_occlusion.npz (학습 env lidar_model='real' 이 읽음) + 사각 통계.

Isaac 브리지 (sim_nav_bridge.py) 와 같은 모델: sim_core/lidar_self.py 로 팀 URDF 메시를 스캔 높이에서 자른 단면 → 빔이 몸에 먼저 맞으면 그 뒤는 안 보임.
  고정 부분 (차체 · 기둥 · 반대쪽 라이다) = 라이다마다 한 번, 조향 모듈 (조향 · 바퀴 링크) = 모듈마다 조향각 Q 칸별로.
  가림 거리 occ[l, m, q, i] = 라이다 l 빔 i 가 모듈 m (조향각 칸 q) 에 처음 맞는 거리 (없으면 inf). 학습 때 min(고정, 모듈 4 개) < 지도 거리 → 점 없음.
저장: origin (2,2) base_footprint 기준 라이다 원점, beam_angle (2,N) base_footprint 기준 빔 방향, occ_static (2,N), occ_module (2,4,Q,N),
      steer_names (4, team_swerve.yaml 순서), q_bins (Q,) 칸 가운데 조향각, body_half (scan_merger 몸 상자).
통계: 합친 스캔 (scan_merger 식: 두 라이다 점 → 몸 상자 밖만 → 몸 중심 10° 칸) 에서 반경 R 원 위 점이 두 라이다 다 가려 안 보이는 칸 수.
예) python3 rl/tasks/local_planner_nav2/make_lidar_occlusion.py [--urdf <팀 robot.urdf>] [--beams 360] [--q_bins 72]
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]                                   # sim/robot_sim
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT))
from sim_core.lidar_self import SelfOcclusion  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('--urdf', default=str(REPO.parent / 'Project2_moveit' / 'sim' / 'robot_sim' / 'robot_description' / 'team' / 'robot.urdf'),
               help='팀 URDF (Isaac 검증 run_isaac.sh 와 같은 것, 루트 base_footprint)')
p.add_argument('--beams', type=int, default=360, help='라이다 한 대 빔 수 (LDS-03 ≈ 1°)')
p.add_argument('--q_bins', type=int, default=72, help='조향각 칸 수 (−π..π)')
p.add_argument('--out', default=str(ROOT / 'rl' / 'data' / 'local_planner_nav2' / 'lidar_occlusion.npz'))
a = p.parse_args()

nav = yaml.safe_load((REPO / 'ros2_ws' / 'src' / 'navigation' / 'config' / 'navigation.yaml').read_text())
BODY = tuple(nav['scan_merger']['ros__parameters']['body_half'])
STEER = yaml.safe_load((ROOT / 'rl' / 'data' / 'team_swerve.yaml').read_text())['joints']['steer']
SELF = SelfOcclusion(Path(a.urdf))
N, Q = a.beams, a.q_bins
q_bins = -math.pi + (np.arange(Q) + 0.5) * 2 * math.pi / Q


def module_segments(steer, q):
    """조향 관절 steer 에 달린 부분만 (조향각 q) base_footprint xy 선분"""
    out = []
    for name, st, seg in SELF.parts:
        if st != steer:
            continue
        P = SELF.steer_origin[st]
        c, s = math.cos(q), math.sin(q)
        R = P[:2, :2] @ np.array([[c, -s], [s, c]])
        out.append(seg @ R.T + P[:2, 3])
    return np.concatenate(out) if out else np.zeros((0, 2, 2))


origin, beam, occ_s, occ_m = np.zeros((2, 2)), np.zeros((2, N)), np.zeros((2, N)), np.full((2, 4, Q, N), np.inf)
for li, side in enumerate(('front', 'rear')):
    T = SELF._fk(f'base_scan_{side}')
    o, yaw = T[:2, 3], math.atan2(T[1, 0], T[0, 0])
    ang = -math.pi + np.arange(N) * 2 * math.pi / N + yaw
    origin[li], beam[li] = o, ang
    occ_s[li] = SELF.ray_hits(o, ang, SELF.segments({}, (f'lidar_{side}_link',), moving=False))
    for mi, st in enumerate(STEER):
        for qi, q in enumerate(q_bins):
            seg = module_segments(st, q)
            if len(seg):
                occ_m[li, mi, qi] = SELF.ray_hits(o, ang, seg)
    print(f'[{side}] 원점 ({o[0]:+.3f}, {o[1]:+.3f}) yaw {math.degrees(yaw):+.0f}°, 고정 부분에 가린 빔 {np.isfinite(occ_s[li]).mean() * 100:.0f} %')

out = Path(a.out)
out.parent.mkdir(parents=True, exist_ok=True)
np.savez_compressed(out, origin=origin.astype(np.float32), beam_angle=beam.astype(np.float32), occ_static=occ_s.astype(np.float32),
                    occ_module=occ_m.astype(np.float32), steer_names=np.array(STEER), q_bins=q_bins.astype(np.float32),
                    body_half=np.array(BODY, np.float32), urdf=np.array(a.urdf))
print(f'→ {out} (빔 {N} × 조향 칸 {Q}), 몸 상자 ±{BODY[0]} × ±{BODY[1]}')


# ---------------------------------------------------------------- 사각 통계
def blind_bins(qs, R, bins=36):
    """조향각 4 개 (rad) 일 때 반경 R 원 위 점 (칸마다 9 점) 중 두 라이다 다 못 보는 점만 있는 10° 칸 수"""
    qi = np.clip(((np.asarray(qs) + math.pi) / (2 * math.pi) * Q).astype(int), 0, Q - 1)
    occ = np.minimum(occ_s, np.min(occ_m[:, np.arange(4), qi], axis=1))          # (2,N)
    blind = 0
    for k in range(bins):
        seen = False
        for th in math.radians(10) * k + np.radians(np.linspace(-4.5, 4.5, 9)):
            P = np.array([R * math.cos(th), R * math.sin(th)])
            if abs(P[0]) <= BODY[0] and abs(P[1]) <= BODY[1]:
                continue
            for li in range(2):
                d = P - origin[li]
                ang = math.atan2(d[1], d[0])
                i = int(round((ang - beam[li, 0]) / (2 * math.pi / N))) % N
                if occ[li, i] > np.hypot(*d):
                    seen = True
                    break
            if seen:
                break
        blind += not seen
    return blind


cases = {'조향 0°': [0, 0, 0, 0], '조향 90°': [math.pi / 2] * 4, '조향 −90°': [-math.pi / 2] * 4, '조향 45°': [math.pi / 4] * 4,
         '제자리 회전 (X 자)': [math.pi / 4, -math.pi / 4, -math.pi / 4, math.pi / 4]}
rng = np.random.default_rng(0)
print('\n합친 스캔 10° 칸 36 개 중 사각 칸 수 (반경 R 원 위 점을 두 라이다 다 못 봄)')
print(f'  {"":18s}' + ''.join(f'R={r:.2f}'.rjust(9) for r in (0.3, 0.5, 1.0, 2.0)))
for k, qs in cases.items():
    print(f'  {k:18s}' + ''.join(f'{blind_bins(qs, r):9d}' for r in (0.3, 0.5, 1.0, 2.0)))
rs = [[blind_bins(rng.uniform(-math.pi, math.pi, 4), r) for r in (0.3, 0.5, 1.0, 2.0)] for _ in range(100)]
print(f'  {"무작위 100 (평균)":18s}' + ''.join(f'{v:9.1f}' for v in np.mean(rs, 0)))
