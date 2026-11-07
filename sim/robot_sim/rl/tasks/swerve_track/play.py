#!/usr/bin/env python3
"""swerve_track 간이 플랜트 평가: 평가 명령 열 (은행 split 1) 을 env 하나에 하나씩, 첫 에피소드만 집계.
기준선 (잔차 0 = nt_swerve 가 /cmd_vel 그대로) 과 정책을 같은 명령 · 같은 DR · 같은 초기 조향으로 비교. 출처 (follower · synth) 별로도 나눔.

예) cd sim/robot_sim
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/swerve_track/play.py --baseline
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/swerve_track/play.py --checkpoint rl/logs/swerve_track/<실행>/model_1000.pt
지표 (참 몸 속도 vs 명령): 선속도 · 각속도 RMSE (열마다 → 평균 · p95), 변화 구간 RMSE (0.5 s 전 명령과 차이 큰 스텝만),
     조향 이동량 · 분기 전환 (초당), 조향 끝 0.5° 안 스텝 비율, 게이트 시간, 잔차 평균 크기.
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import numpy as np  # noqa: E402
import torch  # noqa: E402

from swerve_track.cfg import Cfg  # noqa: E402
from swerve_track.env import SwerveTrackEnv  # noqa: E402

KEYS = ('rmse_lin', 'rmse_ang', 'rmse_lin_chg', 'rmse_ang_chg', 'has_chg', 'time_s', 'travel_per_s', 'flips_per_s', 'end_frac',
        'gate_s', 'res_lin', 'res_ang', 'src')

p = argparse.ArgumentParser()
p.add_argument('--checkpoint', default='')
p.add_argument('--baseline', action='store_true')
p.add_argument('--repeats', type=int, default=2)
p.add_argument('--device', default='cuda')
p.add_argument('--seed', type=int, default=7)
p.add_argument('--split', type=int, default=1)
p.add_argument('--steer_rate', type=float, default=None, help='조향 속도 고정 [rad/s] (기본 = DR 3 조건)')
p.add_argument('--no_compile', action='store_true')
p.add_argument('--out', default='')
a = p.parse_args()

cfg = Cfg()
if a.checkpoint:
    saved = json.loads((Path(a.checkpoint).parent / 'cfg.json').read_text())
    for k in ('residual_lin', 'residual_ang', 'preview', 'preview_every', 'history'):
        setattr(cfg.env, k, saved['env'][k])
cfg.env.device, cfg.env.seed, cfg.env.split = a.device, a.seed, a.split
cfg.env.sequential, cfg.env.random_steer_prob = True, 1.0
cfg.env.compile = not a.no_compile
if a.steer_rate:
    cfg.dr.steer_rate_levels, cfg.dr.steer_rate_scale = (a.steer_rate,), (1.0, 1.0)
d = np.load(cfg.env.cmd_bank)
cfg.env.num_envs = int((d['split'] == a.split).sum()) * a.repeats
env = SwerveTrackEnv(cfg)

policy = None
if a.checkpoint:
    from rsl_rl.runners import OnPolicyRunner
    runner = OnPolicyRunner(env, cfg.train.runner_dict(), None, a.device)
    runner.load(a.checkpoint, map_location=a.device)
    policy = runner.get_inference_policy(a.device)
    name = Path(a.checkpoint).parent.name + '/' + Path(a.checkpoint).stem
else:
    assert a.baseline, '--checkpoint 또는 --baseline'
    name = 'baseline(잔차 0)'

B = env.num_envs
done = torch.zeros(B, dtype=torch.bool, device=a.device)
rec = {k: torch.zeros(B, device=a.device) for k in KEYS}
obs = env.get_observations()
t0, steps = time.time(), 0
with torch.inference_mode():
    while not bool(done.all()):
        act = policy(obs) if policy is not None else torch.zeros(B, 3, device=a.device)
        obs, _, _, _ = env.step(act)
        steps += 1
        ld = env.last_done
        if ld is not None:
            first = ~done[ld['ids']]
            ids = ld['ids'][first]
            for k in KEYS:
                rec[k][ids] = ld[k][first].float()
            done[ids] = True
el = time.time() - t0


def summary(m):
    r = {k: v[m] for k, v in rec.items()}
    c = r['has_chg'].bool()
    q = lambda x: float(x.quantile(0.95)) if len(x) else None  # noqa: E731
    return {'episodes': int(m.sum()),
            'rmse_lin_mean': float(r['rmse_lin'].mean()), 'rmse_lin_p95': q(r['rmse_lin']),
            'rmse_ang_mean': float(r['rmse_ang'].mean()), 'rmse_ang_p95': q(r['rmse_ang']),
            'rmse_lin_chg_mean': float(r['rmse_lin_chg'][c].mean()) if c.any() else None,
            'rmse_ang_chg_mean': float(r['rmse_ang_chg'][c].mean()) if c.any() else None,
            'pass_rate(lin≤0.03·ang≤0.10)': float(((r['rmse_lin'] <= 0.03) & (r['rmse_ang'] <= 0.10)).float().mean()),
            'travel_rad_per_s': float(r['travel_per_s'].mean()), 'flips_per_s': float(r['flips_per_s'].mean()),
            'end_frac': float(r['end_frac'].mean()), 'gate_s_mean': float(r['gate_s'].mean()),
            'res_lin_mean': float(r['res_lin'].mean()), 'res_ang_mean': float(r['res_ang'].mean())}


src = rec['src'].long()
res = {'name': name, 'all': summary(torch.ones(B, dtype=torch.bool, device=a.device)),
       'follower': summary(src == 0), 'synth': summary(src == 1), 'sim_steps_per_s': B * steps / el}
print(f'== {name}')
for grp in ('all', 'follower', 'synth'):
    print(f'-- {grp}')
    for k, v in res[grp].items():
        print(f'{k:>30}: {v:.4f}' if isinstance(v, float) else f'{k:>30}: {v}')
if a.out:
    Path(a.out).write_text(json.dumps(res, indent=1, ensure_ascii=False))
