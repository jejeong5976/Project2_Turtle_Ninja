#!/usr/bin/env python3
"""dice 구간 전부 (네 면 후보끼리) 주행 시간표: 구간마다 env reps 개 (DR 다름) → 성공률 · 충돌 · 성공 시간 중앙값.
정책 없으면 기준 추종기 (swerve_follow.follower). 간이 플랜트 결과라 Isaac 과 숫자는 다름 → 면 고르기 · 정책 비교용.
결과 json = best_faces.py 입력. 사람 없음, 성공 판정 단계 2 (2 cm · 2°).
예) cd sim/robot_sim
    PY="env -u PYTHONPATH $HOME/pa-course/IsaacLab-6.2/.venv/bin/python"
    $PY rl/tasks/local_planner_nav2/eval_legs.py --checkpoint rl/logs/local_planner_nav2/<run>/model_3000.pt --paths navfn
    $PY rl/tasks/local_planner_nav2/eval_legs.py --paths navfn                 # 기준 추종기
결과: rl/data/local_planner_nav2/leg_times_<정책 이름 | follower>_<paths>.json
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import numpy as np  # noqa: E402
import torch  # noqa: E402

from local_planner.env import LocalPlannerEnv  # noqa: E402
from local_planner_nav2.cfg import DATA, Cfg  # noqa: E402
from local_planner_nav2.cfg import TrainCfg as NavTrain  # noqa: E402
from swerve_follow.follower import follow  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('--checkpoint', default='')
p.add_argument('--paths', default='navfn', help='dice 구간 파일 = dice_legs_<paths>.npz')
p.add_argument('--reps', type=int, default=4, help='구간당 env 수')
p.add_argument('--legs', default='', help='이 문자열을 포함한 구간만 (예: START)')
p.add_argument('--device', default='cuda')
p.add_argument('--out', default='')
a = p.parse_args()

cfg = Cfg(paths=a.paths)
if a.checkpoint:
    saved = json.loads((Path(a.checkpoint).parent / 'cfg.json').read_text())
    for sec in ('env', 'dr', 'reward'):
        for k, v in saved[sec].items():
            if hasattr(getattr(cfg, sec), k) and k not in ('path_bank', 'dice_legs', 'maps_dir', 'swerve_yaml'):
                setattr(getattr(cfg, sec), k, tuple(v) if isinstance(v, list) else v)
    from local_planner.cfg import Cfg as _BaseCfg                 # 저장된 cfg.json 에 없는 항목 = 그 학습엔 없던 기능 → 꺼진 기본값
    _base = _BaseCfg()
    for sec in ('env', 'reward'):
        for k in vars(getattr(_base, sec)):
            if k not in saved[sec] and k not in ('path_bank', 'dice_legs', 'maps_dir', 'swerve_yaml'):
                setattr(getattr(cfg, sec), k, getattr(getattr(_base, sec), k))
    if 'std_range' not in saved.get('train', {}):            # local_planner 실행 (scalar std) 체크포인트
        from local_planner.cfg import TrainCfg as LPTrain
        cfg.train = LPTrain()
    else:
        cfg.train = NavTrain(std_range=tuple(saved['train']['std_range']))
legs = [str(x) for x in np.load(cfg.env.dice_legs)['legs']]
pick = [i for i, n in enumerate(legs) if a.legs in n]
B = len(pick) * a.reps
cfg.env.num_envs, cfg.env.device, cfg.env.compile = B, a.device, True
cfg.env.curriculum, cfg.env.dice_prob, cfg.env.timeout_slack_s = False, 1.0, 30.0
cfg.env.people_prob = (0.0, 0.0, 0.0)
env = LocalPlannerEnv(cfg)
env.level = 2
dev = a.device
policy = None
if a.checkpoint:
    from rsl_rl.runners import OnPolicyRunner
    runner = OnPolicyRunner(env, cfg.train.runner_dict(), None, dev)
    runner.load(a.checkpoint, map_location=dev)
    policy = runner.get_inference_policy(dev)                 # 평균 행동 (샘플 아님)
lim = (torch.full((B,), cfg.env.vmax, device=dev), torch.full((B,), cfg.env.amax, device=dev), torch.full((B,), cfg.env.wmax, device=dev))

leg_of = torch.tensor([env.dice_first + i for i in pick for _ in range(a.reps)], device=dev)
env.forced = leg_of.clone()
ids = torch.arange(B, device=dev)
env._reset_idx(ids)
env.obs = env._compute_obs(ids)
env.forced = torch.full((B,), -1, dtype=torch.long, device=dev)
res = {}
with torch.inference_mode():
    for t in range(int(env.max_episode_length_t.max()) + 2):
        act = policy(env.obs) if policy is not None else follow(env.path, env._est_pose(), env.s_est, *lim, 0.20, 1.5) / env.lim3
        env.step(act)
        d = env.last_done
        if d is not None:
            for j, i in enumerate(d['ids'].tolist()):
                if i not in res:
                    res[i] = {k: float(d[k][j]) for k in ('success', 'collision', 'lost', 'timeout', 'time_s', 'travel', 'clear_min', 'length')}
        if len(res) == B:
            break
        if t % 200 == 0:
            print(f'  {t * env.dt:6.1f} s  끝난 env {len(res)}/{B}', flush=True)

table = {}
for n, li in enumerate(pick):
    r = [res[k] for k in range(n * a.reps, (n + 1) * a.reps) if k in res]
    ok = [x for x in r if x['success']]
    table[legs[li]] = dict(success=float(np.mean([x['success'] for x in r])) if r else 0.0,
                           collision=float(np.mean([x['collision'] for x in r])) if r else 0.0,
                           time_s=float(np.median([x['time_s'] for x in ok])) if ok else None,
                           length=float(r[0]['length']) if r else None,
                           steer_rad=float(np.median([x['travel'] for x in r])) if r else None,
                           clear_cm=float(100 * np.median([x['clear_min'] for x in r])) if r else None)
name = Path(a.checkpoint).parent.name if a.checkpoint else 'follower'
out = Path(a.out) if a.out else DATA / f'leg_times_{name}_{a.paths}.json'
out.write_text(json.dumps({'checkpoint': a.checkpoint, 'paths': a.paths, 'reps': a.reps, 'legs': table}, indent=1, ensure_ascii=False))
s = np.array([v['success'] for v in table.values()])
print(f'구간 {len(table)} 개: 성공 평균 {s.mean():.3f}, 전부 성공 {(s == 1).mean():.2f}, 한 번도 성공 못 함 {(s == 0).sum()} 개 → {out}')
