#!/usr/bin/env python3
"""학습 플랜트에서 정책 · 기준 추종기를 같은 dice 구간에 나란히 돌려 mp4 (CPU, 학습 GPU 와 안 겹침).

그림: 왼쪽 = 지도 위 두 로봇 (외곽 ±0.16, 몸 방향 화살표, 바퀴 4 개 조향 방향 선, 지나온 길), 오른쪽 위 = |wz| 명령 · 회전 상한,
      오른쪽 아래 = 누적 조향 이동량 [rad]. 둘 다 같은 구간 · 같은 DR 아닌 고정 조건 (잡음 그대로).
예) env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/local_planner_nav2/render_leg.py \\
      rl/logs/local_planner_nav2/<run>/model_3000.pt --leg B2.S->B6.W --out leg.mp4
"""
import argparse
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from local_planner.cfg import Cfg  # noqa: E402
from local_planner.env import LocalPlannerEnv  # noqa: E402
from swerve_follow.follower import follow  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('checkpoint')
p.add_argument('--leg', default='B2.S->B6.W')
p.add_argument('--out', default='leg.mp4')
p.add_argument('--fps', type=int, default=20)
p.add_argument('--label', default='')
a = p.parse_args()

ck = Path(a.checkpoint)
saved = json.loads((ck.parent / 'cfg.json').read_text())
c = Cfg()
for sec in ('env', 'dr', 'reward'):
    for k, v in saved[sec].items():
        if hasattr(getattr(c, sec), k):
            setattr(getattr(c, sec), k, tuple(v) if isinstance(v, list) else v)
if 'std_range' in saved.get('train', {}):
    from local_planner_nav2.cfg import TrainCfg
    c.train = TrainCfg(std_range=tuple(saved['train']['std_range']))
c.env.num_envs, c.env.device, c.env.compile, c.env.curriculum = 2, 'cpu', False, False
c.env.people_prob = (0.0,) * 3
env = LocalPlannerEnv(c)
env.level = 2
runner = OnPolicyRunner(env, c.train.runner_dict(), None, 'cpu')
runner.load(str(ck), map_location='cpu')
pol = runner.get_inference_policy('cpu')

z = np.load(c.env.dice_legs)
alias = dict(x.split('=') for x in map(str, z['aliases'])) if 'aliases' in z else {}
s_, g_ = a.leg.split('->')
leg = f'{alias.get(s_, s_)}->{alias.get(g_, g_)}'
k = env.dice_first + env.dice_legs.index(leg)
env.forced = torch.full((2,), k, dtype=torch.long)
env._reset_idx(torch.arange(2))
env.obs = env._compute_obs(torch.arange(2))
env.forced = torch.full((2,), -1, dtype=torch.long)
path = env.path.pts[0, :int(env.path.n[0])].numpy().copy()
lim = (torch.full((2,), c.env.vmax), torch.full((2,), c.env.amax), torch.full((2,), c.env.wmax))

rec = {i: [] for i in range(2)}
done = [None, None]
T = int(env.max_episode_length_t.max()) + 2
with torch.inference_mode():
    for t in range(T):
        act = pol(env.obs)
        act[1] = (follow(env.path, env._est_pose(), env.s_est, *lim, 0.20, 1.5) / env.lim3)[1]
        pre = (env.plant.pose.clone(), env.plant.q.clone())
        env.step(act)
        cmd = env.last_cmd
        for i in range(2):
            if done[i] is None:
                rec[i].append((pre[0][i].numpy().copy(), pre[1][i].numpy().copy(), cmd[i].numpy().copy(),
                               bool(getattr(env, 'rot_capped', torch.zeros(2, dtype=torch.bool))[i])))
        d = env.last_done
        if d is not None:
            for j, i in enumerate(d['ids'].tolist()):
                if done[i] is None:
                    done[i] = 'success' if d['success'][j] else ('collision' if d['collision'][j] else ('timeout' if d['timeout'][j] else 'lost'))
        if all(x is not None for x in done):
            break
names = [a.label or ('policy ' + ck.parent.name.split('_', 2)[-1]), 'baseline follower']
print(f'{leg}: ' + ', '.join(f'{n} {done[i]} {len(rec[i]) * env.dt:.1f} s' for i, n in enumerate(names)))

# ---------------------------------------------------------------- 그림
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
ko = [f.name for f in font_manager.fontManager.ttflist if 'Noto Sans CJK' in f.name or 'NanumGothic' in f.name]
if ko:
    plt.rcParams['font.family'] = ko[0]
w = env.world
mi = env.map_names.index('dice')
D = w.dfield[mi].numpy()
o = w.origin[mi].numpy()
mx, my = env.geom.mx.numpy().reshape(-1), env.geom.my.numpy().reshape(-1)
cols = ['tab:red', 'tab:blue']
allp = np.concatenate([np.array([r[0][:2] for r in rec[i]]) for i in range(2)] + [path])
lo, hi = allp.min(0) - 0.45, allp.max(0) + 0.45
N = max(len(rec[0]), len(rec[1]))
wz = [np.array([abs(r[2][2]) for r in rec[i]]) for i in range(2)]
steer = [np.concatenate([[0], np.cumsum(np.abs(np.diff(np.array([r[1] for r in rec[i]]), axis=0)).sum(1))]) for i in range(2)]
cap = np.array([r[3] for r in rec[0]])
fig = plt.figure(figsize=(12, 6.4))
ax = fig.add_axes([0.03, 0.06, 0.55, 0.86])
a1 = fig.add_axes([0.64, 0.56, 0.34, 0.36])
a2 = fig.add_axes([0.64, 0.08, 0.34, 0.36])
tmp = Path(tempfile.mkdtemp())
sq = np.array([[0.16, 0.16], [0.16, -0.16], [-0.16, -0.16], [-0.16, 0.16], [0.16, 0.16]])
tt = np.arange(N) * env.dt
for f in range(N):
    ax.clear()
    ax.imshow(D <= 0, origin='lower', cmap='Greys', extent=[o[0], o[0] + D.shape[1] * w.res, o[1], o[1] + D.shape[0] * w.res])
    ax.plot(path[:, 0], path[:, 1], '--', color='gray', lw=1)
    for i in range(2):
        r = rec[i][min(f, len(rec[i]) - 1)]
        x, y, yaw = r[0]
        tr = np.array([q[0][:2] for q in rec[i][:f + 1]])
        ax.plot(tr[:, 0], tr[:, 1], color=cols[i], lw=1, alpha=0.6)
        cy, sy = math.cos(yaw), math.sin(yaw)
        R = np.array([[cy, -sy], [sy, cy]])
        b = sq @ R.T + [x, y]
        ax.plot(b[:, 0], b[:, 1], color=cols[i], lw=1.5)
        ax.arrow(x, y, 0.18 * cy, 0.18 * sy, head_width=0.04, color=cols[i])
        for m in range(4):                                   # 바퀴: 모듈 자리에서 조향 방향 선
            px, py = R @ [mx[m], my[m]] + [x, y]
            ang = yaw + r[1][m]
            ax.plot([px - 0.04 * math.cos(ang), px + 0.04 * math.cos(ang)], [py - 0.04 * math.sin(ang), py + 0.04 * math.sin(ang)], color='k', lw=2.5)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect('equal')
    ax.set_title(f'{leg}   t = {f * env.dt:5.2f} s   (빨강 = {names[0]}, 파랑 = {names[1]}, 검정 선 = 바퀴 조향)', fontsize=9)
    a1.clear()
    for i in range(2):
        n = min(f + 1, len(wz[i]))
        a1.plot(tt[:n], wz[i][:n], color=cols[i], lw=1)
    n0 = min(f + 1, len(cap))
    a1.fill_between(tt[:n0], 0, 0.55, where=cap[:n0], color='orange', alpha=0.2, step='mid', label='정책 회전 상한 걸림')
    a1.set_xlim(0, tt[-1]); a1.set_ylim(0, 0.55); a1.set_ylabel('|wz| 명령 [rad/s]'); a1.legend(fontsize=7, loc='upper right')
    a2.clear()
    for i in range(2):
        n = min(f + 1, len(steer[i]))
        a2.plot(tt[:n], steer[i][:n], color=cols[i], lw=1.2, label=f'{names[i]} ({steer[i][-1]:.0f} rad, {done[i]} {len(rec[i]) * env.dt:.1f} s)')
    a2.set_xlim(0, tt[-1]); a2.set_ylim(0, max(steer[0][-1], steer[1][-1]) * 1.1 + 1); a2.set_ylabel('누적 조향 이동 [rad]'); a2.set_xlabel('s')
    a2.legend(fontsize=7, loc='upper left')
    fig.savefig(tmp / f'{f:05d}.png', dpi=90)
plt.close(fig)
subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(a.fps), '-i', str(tmp / '%05d.png'), '-pix_fmt', 'yuv420p',
                '-vf', 'scale=trunc(iw/2)*2:trunc(ih/2)*2', a.out], check=True)
print('영상 →', a.out)
