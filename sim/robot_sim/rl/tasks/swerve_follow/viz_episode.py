#!/usr/bin/env python3
"""에피소드 그림: 평가 지도 (기본 dice) 경로를 지도 방향 그대로 간이 플랜트로 달리고, 미로 비용지도 위에 그림.
배경: 검정 = 벽 (라이다 높이 단면), 회색 = 내접 띠 (로봇 중심 금지, 0.17 m), 옅은 회색 = 팽창 비용.
왼쪽 = 지도 전체 (경로 · 지나온 길), 오른쪽 = 로봇 근접 (차체 ±0.16 m, 바퀴 4 개 조향 방향, 게이트 정지 표시).
※ 학습 · 이 그림 모두 벽 충돌 물리 없음 (간이 플랜트). 미로는 경로를 만든 지도를 배경으로 깐 것.

예) cd sim/robot_sim
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/swerve_follow/viz_episode.py --baseline --map dice --k 0
    ... --checkpoint rl/logs/swerve_follow/<실행>/model_1500.pt
출력: rl/data/swerve_follow/viz/<지도>_<번호>_<이름>.gif · .png
"""
import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
plt.rcParams['font.family'] = ['NanumGothic', 'Noto Sans CJK KR', 'DejaVu Sans']
import numpy as np  # noqa: E402
import torch  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402
from scipy import ndimage  # noqa: E402

from swerve_follow.cfg import Cfg  # noqa: E402
from swerve_follow.env import SwerveFollowEnv  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('--checkpoint', default='')
p.add_argument('--baseline', action='store_true')
p.add_argument('--map', default='dice')
p.add_argument('--k', type=int, default=0, help='그 지도 평가 경로 중 몇 번째 (길이 4 m 넘는 것 중)')
p.add_argument('--min_len', type=float, default=4.0)
p.add_argument('--steer_rate', type=float, default=0.96)
p.add_argument('--profile', default='0.13,0.2')
p.add_argument('--start_yaw_deg', type=float, default=None, help='시작 yaw (기본 = 경로 시작 접선 + 90°)')
p.add_argument('--every', type=int, default=4, help='gif 프레임 간격 (정책 스텝, 20 Hz)')
p.add_argument('--device', default='cuda')
a = p.parse_args()

bank = np.load(ROOT / 'rl' / 'data' / 'path_bank.npz')
names = list(bank['names'])
sel = np.nonzero((bank['split'] == 1) & (bank['src'] == names.index(a.map)) & ((bank['n'] - 1) * 0.05 >= a.min_len))[0]
i = int(sel[a.k % len(sel)])
n = int(bank['n'][i])
pts0 = bank['pts'][i]
start = bank['start'][i]
gyaw = float(bank['goal_yaw'][i])
L = (n - 1) * 0.05

cfg = Cfg()
cfg.env.num_envs, cfg.env.device, cfg.env.curriculum = 1, a.device, False
cfg.env.random_rotation, cfg.env.random_steer_prob = False, 1.0
cfg.dr.steer_rate_levels, cfg.dr.steer_rate_scale = (a.steer_rate,), (1.0, 1.0)
cfg.env.limit_profiles = (tuple(float(x) for x in a.profile.split(',')),)
if a.checkpoint:
    saved = json.loads((Path(a.checkpoint).parent / 'cfg.json').read_text())
    cfg.env.action_mode, cfg.env.residual_lin, cfg.env.residual_ang = (saved['env'][k] for k in ('action_mode', 'residual_lin', 'residual_ang'))
env = SwerveFollowEnv(cfg)
dev = a.device
d0 = pts0[min(4, n - 1)] - pts0[0]
syaw = math.radians(a.start_yaw_deg) if a.start_yaw_deg is not None else math.atan2(d0[1], d0[0]) + math.pi / 2
obs = env.force_path(torch.tensor([0], device=dev), torch.tensor(pts0, device=dev)[None], torch.tensor([n], device=dev),
                     torch.tensor([gyaw], device=dev), torch.tensor([syaw], device=dev))
policy, name = None, 'baseline'
if a.checkpoint:
    from rsl_rl.runners import OnPolicyRunner
    runner = OnPolicyRunner(env, cfg.train.runner_dict(), None, dev)
    runner.load(a.checkpoint, map_location=dev)
    policy, name = runner.get_inference_policy(dev), Path(a.checkpoint).parent.name

rec = []
with torch.inference_mode():
    for t in range(int(env.max_episode_length_t[0]) + 5):
        act = policy(obs) if policy is not None else torch.zeros(1, 3, device=dev)
        rec.append((env.plant.pose[0].tolist(), env.plant.q[0].tolist(), env.last_cmd[0].tolist(), bool(env.ctrl.gate_blocked[0])))
        obs, _, done, _ = env.step(act)
        if bool(done[0]):
            d = env.last_done
            result = 'success' if bool(d['success'][0]) else ('failure' if bool(d['failure'][0]) else 'timeout')
            T = float(d['time_s'][0])
            break
P = np.array([r[0] for r in rec]); Q = np.array([r[1] for r in rec]); G = np.array([r[3] for r in rec])
P[:, :2] += start          # env 좌표 → 지도 좌표

# 지도 비용 (경로 은행과 같은 식)
md = ROOT / 'rl' / 'data' / 'maps'
stem = 'maze_real_map' if a.map == 'real' else f'maze_{a.map}_map'
raw = (md / f'{stem}.pgm').read_bytes().split(b'\n', 4)
Wm, Hm = map(int, raw[2].split())
occ = (np.frombuffer(raw[4], np.uint8)[:Wm * Hm].reshape(Hm, Wm)[::-1] == 0)
meta = {k.strip(): v.strip() for k, v in (l.split(':', 1) for l in (md / f'{stem}.yaml').read_text().splitlines() if ':' in l and not l.startswith('#'))}
res, org = float(meta['resolution']), json.loads(meta['origin'])[:2]
dd = ndimage.distance_transform_edt(~occ) * res
cost = np.where(dd <= 0.17, 253.0, np.where(dd <= 0.25, 252 * np.exp(-10 * (dd - 0.17)), 0.0)); cost[occ] = 254
ext = [org[0], org[0] + Wm * res, org[1], org[1] + Hm * res]
path = pts0[:n] + start
g = env.geom
MX, MY, OFF = g.mx.cpu().numpy(), g.my.cpu().numpy(), g.off.cpu().numpy()


def robot_shapes(pose, q):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    R = np.array([[c, -s], [s, c]])
    body = (np.array([[0.16, 0.16], [0.16, -0.16], [-0.16, -0.16], [-0.16, 0.16]]) @ R.T) + pose[:2]
    wheels = []
    for k in range(4):
        cp = np.array([MX[k] - OFF[k] * math.sin(q[k]), MY[k] + OFF[k] * math.cos(q[k])])   # 접지점 (body)
        dvec = np.array([math.cos(q[k]), math.sin(q[k])]) * 0.033
        seg = np.stack([cp - dvec, cp + dvec]) @ R.T + pose[:2]
        wheels.append(seg)
    head = np.stack([pose[:2], pose[:2] + R @ np.array([0.22, 0.0])])
    return body, wheels, head


fig, (ax, az) = plt.subplots(1, 2, figsize=(12, 7), gridspec_kw={'width_ratios': [1.1, 1]})
for axx in (ax, az):
    axx.imshow(np.where(occ, 1.0, np.where(cost >= 253, 0.35, cost / 253 * 0.25)), origin='lower', extent=ext, cmap='Greys', vmin=0, vmax=1)
    axx.plot(path[:, 0], path[:, 1], '-', color='tab:blue', lw=1.2, label='Nav2 식 경로')
    axx.plot(*path[-1], 'b*', ms=12)
    axx.arrow(*path[-1], 0.25 * math.cos(gyaw), 0.25 * math.sin(gyaw), color='b', width=0.01)
    axx.set_aspect('equal')
trail, = ax.plot([], [], '-', color='tab:red', lw=1.5, label='로봇 (참 자세)')
trail2, = az.plot([], [], '-', color='tab:red', lw=1.5)
body_p = Polygon(np.zeros((4, 2)), closed=True, fc='orange', ec='k', alpha=0.6); az.add_patch(body_p)
body_s = Polygon(np.zeros((4, 2)), closed=True, fc='orange', ec='k'); ax.add_patch(body_s)
wl = [az.plot([], [], '-', color='k', lw=4)[0] for _ in range(4)]
hd, = az.plot([], [], '-', color='tab:green', lw=2)
txt = az.text(0.02, 0.98, '', transform=az.transAxes, va='top', fontsize=10, bbox=dict(fc='w', alpha=0.8))
ax.legend(loc='lower right', fontsize=8)
ax.set_title(f'{a.map} 평가 경로 #{i} ({L:.1f} m) — {name}: {result} {T:.1f} s')
az.set_title(f'근접 (조향 속도 {a.steer_rate} rad/s, 한계 {a.profile})')


def draw(f):
    k = min(f * a.every, len(P) - 1)
    body, wheels, head = robot_shapes(P[k], Q[k])
    trail.set_data(P[:k + 1, 0], P[:k + 1, 1]); trail2.set_data(P[:k + 1, 0], P[:k + 1, 1])
    body_p.set_xy(body); body_s.set_xy(body)
    for w_, seg in zip(wl, wheels):
        w_.set_data(seg[:, 0], seg[:, 1]); w_.set_color('red' if G[k] else 'k')
    hd.set_data(head[:, 0], head[:, 1])
    az.set_xlim(P[k, 0] - 0.6, P[k, 0] + 0.6); az.set_ylim(P[k, 1] - 0.6, P[k, 1] + 0.6)
    txt.set_text(f't {k * env.dt:5.1f} s\n조향 {np.degrees(Q[k]).round(0).astype(int).tolist()}°' + ('\n게이트 정지 (빨간 바퀴)' if G[k] else ''))
    return []


out = ROOT / 'rl' / 'data' / 'swerve_follow' / 'viz'
out.mkdir(parents=True, exist_ok=True)
stem_o = out / f'{a.map}_{i}_{name}'
frames = (len(P) - 1) // a.every + 1
FuncAnimation(fig, draw, frames=frames).save(str(stem_o) + '.gif', writer=PillowWriter(fps=20))
draw(frames - 1); fig.savefig(str(stem_o) + '.png', dpi=90)
print(f'{result} {T:.1f} s, 길이 {L:.2f} m, 게이트 정지 {G.sum() * env.dt:.2f} s (정책 스텝 기준) → {stem_o}.gif')
