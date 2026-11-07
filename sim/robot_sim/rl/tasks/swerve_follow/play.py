#!/usr/bin/env python3
"""간이 플랜트 평가: 평가 경로(은행 split 1 = dice · rand_07 지도 + 합성)를 env 하나에 하나씩, 첫 에피소드만 집계.
기준선(잔차 0 = 기준 추종기) 과 정책을 같은 경로 · 같은 DR · 같은 초기 상태로 비교.

예) cd sim/robot_sim
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/swerve_follow/play.py --baseline
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/swerve_follow/play.py --checkpoint rl/logs/swerve_follow/<실행>/model_1500.pt
지표: 성공 · 실패 · 시간 초과 비율, 성공 구간 시간 · 평균 속도, 경로까지 거리 평균 · 최대 (평균, p95), 분기 전환 수, 조향 이동량, 게이트 시간,
     끝 오차 (추정 자세 = 판정 기준, 참 자세 p95 = 위치 오차 포함 실제 도착).
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import torch  # noqa: E402

from swerve_follow.cfg import Cfg  # noqa: E402
from swerve_follow.env import SwerveFollowEnv  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('--checkpoint', default='')
p.add_argument('--baseline', action='store_true', help='잔차 0 (기준 추종기만)')
p.add_argument('--repeats', type=int, default=2, help='평가 경로 하나당 env 수 (DR · 초기 상태 다름)')
p.add_argument('--device', default='cuda')
p.add_argument('--seed', type=int, default=7)
p.add_argument('--split', type=int, default=1)
p.add_argument('--steer_rate', type=float, default=None, help='조향 속도 고정 [rad/s] (기본 = DR 3 조건)')
p.add_argument('--profile', default='', help='속도 한계 고정 "v,a" (예 0.13,0.2)')
p.add_argument('--out', default='')
a = p.parse_args()

cfg = Cfg()
if a.checkpoint:
    saved = json.loads((Path(a.checkpoint).parent / 'cfg.json').read_text())
    for k in ('action_mode', 'residual_lin', 'residual_ang', 'limit_profiles'):
        setattr(cfg.env, k, saved['env'][k] if k != 'limit_profiles' else tuple(map(tuple, saved['env'][k])))
cfg.env.device, cfg.env.seed, cfg.env.split = a.device, a.seed, a.split
cfg.env.curriculum, cfg.env.sequential_paths = False, True
cfg.env.random_steer_prob = 1.0
if a.steer_rate:
    cfg.dr.steer_rate_levels, cfg.dr.steer_rate_scale = (a.steer_rate,), (1.0, 1.0)
if a.profile:
    cfg.env.limit_profiles = (tuple(float(x) for x in a.profile.split(',')),)
import numpy as np  # noqa: E402
n_paths = int((np.load(cfg.env.path_bank)['split'] == a.split).sum())
cfg.env.num_envs = n_paths * a.repeats
env = SwerveFollowEnv(cfg)

policy = None
if a.checkpoint:
    from rsl_rl.runners import OnPolicyRunner
    runner = OnPolicyRunner(env, cfg.train.runner_dict(), None, a.device)
    runner.load(a.checkpoint, map_location=a.device)
    policy = runner.get_inference_policy(a.device)
    name = Path(a.checkpoint).parent.name + '/' + Path(a.checkpoint).stem
else:
    assert a.baseline or cfg.env.action_mode == 'residual', '--checkpoint 또는 --baseline'
    name = 'baseline(잔차 0)'

B = env.num_envs
done = torch.zeros(B, dtype=torch.bool, device=a.device)
rec = {k: torch.zeros(B, device=a.device) for k in ('success', 'failure', 'timeout', 'time_s', 'flips', 'travel', 'lat_max', 'lat_mean',
                                                      'gate_s', 'final_dist', 'final_dist_true', 'final_dyaw', 'length')}
obs = env.get_observations()
t0, steps = time.time(), 0
with torch.inference_mode():
    while not bool(done.all()):
        act = policy(obs) if policy is not None else torch.zeros(B, 3, device=a.device)
        obs, _, _, _ = env.step(act)
        steps += 1
        d = env.last_done
        if d is not None:
            first = ~done[d['ids']]
            ids = d['ids'][first]
            for k in rec:
                rec[k][ids] = d[k][first].float()
            done[ids] = True
dt = time.time() - t0
s = rec['success'].bool()
lat = rec['lat_max']
res = {'name': name, 'episodes': B, 'success': float(s.float().mean()), 'failure': float(rec['failure'].mean()), 'timeout': float(rec['timeout'].mean()),
       'time_s_mean(success)': float(rec['time_s'][s].mean()) if s.any() else None,
       'speed_mps_mean(success)': float((rec['length'][s] / rec['time_s'][s]).mean()) if s.any() else None,
       'lat_mean_m': float(rec['lat_mean'].mean()), 'lat_max_m_mean': float(lat.mean()), 'lat_max_m_p95': float(lat.quantile(0.95)),
       'flips_mean': float(rec['flips'].mean()), 'steer_travel_rad_mean': float(rec['travel'].mean()), 'gate_s_mean': float(rec['gate_s'].mean()),
       'final_dist_m_mean(success)': float(rec['final_dist'][s].mean()) if s.any() else None,
       'final_dist_true_m_p95(success)': float(rec['final_dist_true'][s].quantile(0.95)) if s.any() else None,
       'sim_steps_per_s': B * steps / dt}
for k, v in res.items():
    print(f'{k:>28}: {v:.4f}' if isinstance(v, float) else f'{k:>28}: {v}')
if a.out:
    Path(a.out).write_text(json.dumps(res, indent=1, ensure_ascii=False))
