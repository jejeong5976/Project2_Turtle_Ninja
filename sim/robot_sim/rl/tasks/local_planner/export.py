#!/usr/bin/env python3
"""지역 계획기 체크포인트 → TorchScript (관측 정규화 포함) <run>/exported/policy_<반복>.pt + numpy 판 policy_<반복>.npz.
ROS 노드 (deploy/local_planner_node.py) 가 읽음. npz = torch 없이 numpy 로 돌림 (Jetson 용): 정규화 mean · std · eps + 선형 층 W · b (ELU, 마지막 층 선형 = 평균 행동).
예) env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/local_planner/export.py rl/logs/local_planner/<run>/model_400.pt
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import json  # noqa: E402

import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from local_planner.cfg import Cfg  # noqa: E402
from local_planner.env import LocalPlannerEnv  # noqa: E402

ck = Path(sys.argv[1])
cfg = Cfg()
saved = json.loads((ck.parent / 'cfg.json').read_text())
if 'std_range' in saved.get('train', {}):                  # local_planner_nav2 실행 = log std · 상한 (체크포인트 매개변수 이름이 다름)
    from local_planner_nav2.cfg import TrainCfg as _NavTrain
    cfg.train = _NavTrain(std_range=tuple(saved['train']['std_range']))
cfg.env.num_envs, cfg.env.compile = 4, False
env = LocalPlannerEnv(cfg)
runner = OnPolicyRunner(env, cfg.train.runner_dict(), None, cfg.env.device)
runner.load(str(ck), map_location=cfg.env.device)
name = f'policy_{ck.stem.split("_")[-1]}.pt'
runner.export_policy_to_jit(str(ck.parent / 'exported'), name)
pol = torch.jit.load(str(ck.parent / 'exported' / name), map_location='cpu')
o = env.get_observations()['policy'][:1].cpu()
ref = runner.get_inference_policy(cfg.env.device)(env.get_observations())[:1].cpu()
print(f'→ {ck.parent / "exported" / name}, 입력 {o.shape[1]}, 출력 차이 {float((pol(o) - ref).abs().max()):.2e}')

# numpy 판: TorchScript 의 정규화 · 선형 층을 꺼내 저장 → 같은 입력으로 비교
sd = {k: v.cpu().numpy() for k, v in pol.state_dict().items()}
layers = sorted({int(k.split('.')[1]) for k in sd if k.startswith('mlp.') and k.endswith('.weight')})
npz = ck.parent / 'exported' / name.replace('.pt', '.npz')
import numpy as np  # noqa: E402
np.savez(npz, mean=sd['obs_normalizer._mean'][0], std=sd['obs_normalizer._std'][0], eps=np.float32(pol.obs_normalizer.eps),
         n_layers=np.int32(len(layers)), activation=np.array('elu'),
         **{f'W{i}': sd[f'mlp.{l}.weight'] for i, l in enumerate(layers)}, **{f'b{i}': sd[f'mlp.{l}.bias'] for i, l in enumerate(layers)})
sys.path.insert(0, str(Path(__file__).resolve().parent / 'deploy'))
from numpy_policy import NumpyPolicy  # noqa: E402
x = env.get_observations()['policy'][:64].cpu()
d = float(np.abs(NumpyPolicy(npz)(x.numpy()) - pol(x).detach().numpy()).max())
print(f'→ {npz} (층 {len(layers)}), torch 대비 최대 차이 {d:.2e} (64 관측)')
