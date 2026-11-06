#!/usr/bin/env python3
"""지역 계획기 환경 확인: 무작위 행동 · 기준 추종기 (swerve_follow.follower, 장애물 안 피함) 를 정책 자리에 넣고 결과 비율.
예) cd sim/robot_sim && env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/local_planner/check_env.py --level 2
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import torch  # noqa: E402

from local_planner.cfg import Cfg  # noqa: E402
from local_planner.env import LocalPlannerEnv  # noqa: E402
from swerve_follow.follower import follow  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('--num_envs', type=int, default=2048)
p.add_argument('--steps', type=int, default=1200)
p.add_argument('--level', type=int, default=0)
p.add_argument('--compile', action='store_true')
a = p.parse_args()

cfg = Cfg()
cfg.env.num_envs, cfg.env.compile = a.num_envs, a.compile
cfg.env.curriculum, cfg.env.curriculum_start = False, a.level
for mode in ('random', 'follower'):
    env = LocalPlannerEnv(cfg)
    env.level = a.level
    obs = env.get_observations()
    print(f'[{mode}] 관측 policy {obs["policy"].shape[1]} · privileged {obs["privileged"].shape[1]}, 사람 켜짐 {float(env.world.people_on.any(1).float().mean()):.2f}')
    acc = {k: [] for k in ('success', 'collision', 'lost', 'timeout', 'dice', 'time_s', 'length', 'final_dist_true', 'final_dyaw_true', 'clear_min')}
    t0 = time.time()
    B = cfg.env.num_envs
    vmax = torch.full((B,), cfg.env.vmax, device=env.device)
    amax = torch.full((B,), cfg.env.amax, device=env.device)
    wmax = torch.full((B,), cfg.env.wmax, device=env.device)
    for _ in range(a.steps):
        if mode == 'random':
            act = torch.rand(B, 3, device=env.device) * 2 - 1
        else:
            pose = env._est_pose()
            cmd = follow(env.path, pose, env.s_est, vmax, amax, wmax, 0.20, 1.5)
            act = cmd / env.lim3
        env.step(act)
        if env.last_done is not None:
            for k in acc:
                acc[k].append(env.last_done[k].float().cpu())
    cat = {k: torch.cat(v) for k, v in acc.items()}
    n = len(cat['success'])
    print(f'  {a.steps} 스텝 {time.time() - t0:.1f} s, 끝난 에피소드 {n}')
    for name, m in (('전체', torch.ones(n, dtype=torch.bool)), ('dice', cat['dice'] > 0), ('지도', cat['dice'] == 0)):
        if m.sum() == 0:
            continue
        s = cat['success'][m] > 0
        line = ' '.join(f"{k} {cat[k][m].mean():.2f}" for k in ('success', 'collision', 'lost', 'timeout'))
        extra = ''
        if s.any():
            extra = (f" | 성공: 시간/길이 {(cat['time_s'][m][s] / cat['length'][m][s]).mean():.2f} s/m, 참 위치 오차 {cat['final_dist_true'][m][s].mean() * 100:.1f} cm,"
                     f" yaw {cat['final_dyaw_true'][m][s].mean() * 57.3:.1f}°")
        print(f'  {name:4s} n {int(m.sum()):5d}: {line}, 최소 여유 {cat["clear_min"][m].mean() * 100:.1f} cm{extra}')
