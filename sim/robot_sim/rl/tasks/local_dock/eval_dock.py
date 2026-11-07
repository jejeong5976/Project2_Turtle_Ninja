#!/usr/bin/env python3
"""정밀 접근 정책 평가: 접근 구간 (dock_legs.npz) 을 평균 행동으로 → 성공 · 충돌 · 도착 오차 · 시간 · 마지막 0.3 m 안 회전량.
출발 거리별 (MPPI 가 서는 0.03–0.15 m / 0.15–0.8 m) · 출발 yaw 오차별 (≤ 10° / 그 밖) · 출발 모드별 (v2: pre · fix · broad) · 박스 밀림 여부로 나눠 봄. 사람 없음.
예) env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/local_dock/eval_dock.py rl/logs/local_dock/<run>/model_1500.pt
"""
import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from local_dock.cfg import Cfg, TrainCfg  # noqa: E402
from local_planner.env import LocalPlannerEnv  # noqa: E402
from swerve_follow.paths import wrap  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('checkpoint')
p.add_argument('--num_envs', type=int, default=4096)
p.add_argument('--steps', type=int, default=1200)
p.add_argument('--device', default='cuda')
a = p.parse_args()

ck = Path(a.checkpoint)
saved = json.loads((ck.parent / 'cfg.json').read_text())
c = Cfg()
for sec in ('env', 'dr', 'reward'):
    for k, v in saved[sec].items():
        if hasattr(getattr(c, sec), k):
            setattr(getattr(c, sec), k, tuple(v) if isinstance(v, list) else v)
from local_planner.cfg import Cfg as _BaseCfg                 # 저장된 cfg.json 에 없는 항목 = 그 학습엔 없던 기능 → 꺼진 기본값
_base = _BaseCfg()
for sec in ('env', 'reward'):
    for k in vars(getattr(_base, sec)):
        if k not in saved[sec] and k not in ('path_bank', 'dice_legs', 'maps_dir', 'swerve_yaml'):
            setattr(getattr(c, sec), k, getattr(getattr(_base, sec), k))
c.train = TrainCfg(std_range=tuple(saved['train']['std_range']))
c.env.num_envs, c.env.device, c.env.curriculum = a.num_envs, a.device, False
c.env.people_prob = (0.0,) * 3
env = LocalPlannerEnv(c)
env.level = 2
runner = OnPolicyRunner(env, c.train.runner_dict(), None, a.device)
runner.load(str(ck), map_location=a.device)
pol = runner.get_inference_policy(a.device)

B, dev = a.num_envs, a.device
L0 = env.path.length.clone()
E0 = wrap(env.path.goal_yaw - env.plant.pose[:, 2]).abs()
rot_near = torch.zeros(B, device=dev)
K0 = env.cur_k.clone()
SH = (env.world.box_k >= 0) if env.world.boxes else torch.zeros(B, dtype=torch.bool, device=dev)
_z = np.load(c.env.dice_legs)
leg_mode = _z['mode'] if 'mode' in _z else None
mode_names = [str(x) for x in _z['modes']] if 'modes' in _z else []
rows = []
with torch.inference_mode():
    obs = env.get_observations()
    for _ in range(a.steps):
        fresh = env.episode_length_buf == 0                         # 방금 리셋된 env: 출발 거리 · yaw 오차 기록
        L0 = torch.where(fresh, env.path.length, L0)
        E0 = torch.where(fresh, wrap(env.path.goal_yaw - env.plant.pose[:, 2]).abs(), E0)
        rot_near = torch.where(fresh, torch.zeros_like(rot_near), rot_near)
        K0 = torch.where(fresh, env.cur_k, K0)
        if env.world.boxes:
            SH = torch.where(fresh, env.world.box_k >= 0, SH)
        obs, _, _, _ = env.step(pol(obs))
        dist_t, _ = env._goal_err(env.plant.pose)
        rot_near += (dist_t < 0.3).float() * env.plant.odom[:, 2].abs() * env.dt
        d = env.last_done
        if d is None:
            continue
        ids = d['ids']
        for j, i in enumerate(ids.tolist()):
            md = int(leg_mode[int(K0[i]) - env.dice_first]) if leg_mode is not None else -1
            rows.append((float(L0[i]), float(E0[i]), bool(d['success'][j]), bool(d['collision'][j]), float(d['time_s'][j]),
                         float(d['final_dist_true'][j]), float(d['final_dyaw_true'][j]), float(rot_near[i]), md, float(SH[i])))
R = np.array(rows, dtype=float)
cols = dict(L=0, e=1, ok=2, col=3, t=4, err=5, yaw=6, rot=7)


def show(name, m):
    r = R[m]
    if not len(r):
        return
    ok = r[:, 2] > 0
    print(f'{name:28s} {len(r):6d}  성공 {ok.mean() * 100:5.1f} %  충돌 {r[:, 3].mean() * 100:4.1f} %  '
          f'시간 {np.median(r[ok, 4]) if ok.any() else float("nan"):4.1f} s  오차 {np.median(r[ok, 5]) * 100 if ok.any() else float("nan"):3.1f} cm · '
          f'{np.degrees(np.median(r[ok, 6])) if ok.any() else float("nan"):3.1f}°  마지막 0.3 m 회전 {np.degrees(np.median(r[:, 7])):4.1f}° (p90 {np.degrees(np.percentile(r[:, 7], 90)):4.1f}°)')


print(f'{"묶음":28s} {"에피소드":>6s}')
show('전체', np.ones(len(R), bool))
show('출발 0.03–0.15 m (MPPI 자리)', R[:, 0] < 0.15)
show('출발 0.15–0.8 m', R[:, 0] >= 0.15)
show('출발 yaw ≤ 10°', R[:, 1] <= math.radians(10))
show('출발 yaw > 10°', R[:, 1] > math.radians(10))
show('가깝고 yaw > 10° (가장 어려움)', (R[:, 0] < 0.15) & (R[:, 1] > math.radians(10)))
for i, nm in enumerate(mode_names):
    show(f'모드 {nm}', R[:, 8] == i)
if env.world.boxes:
    show('박스 밀림', R[:, 9] > 0)
    show('박스 그대로', R[:, 9] == 0)
