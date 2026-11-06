#!/usr/bin/env python3
"""학습 플랜트 (swerve_follow.plant + sim_core.swerve_torch, 팀 swerve.yaml) 에 record.py 와 같은 계단을 넣어 같은 형식으로 저장.
→ analyze.py 로 실물 · Isaac · 학습 플랜트를 같은 표로 비교 (DR 범위가 실측을 덮는지 확인).
DR 값은 local_planner cfg 의 범위에서 고름: --dr mid (가운데) · fast · slow (범위 끝)
사용: env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/local_planner/sysid/plant_response.py --out <폴더> [--dr mid]
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]          # sim/robot_sim
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import numpy as np  # noqa: E402
import torch  # noqa: E402

from local_planner.cfg import Cfg  # noqa: E402
from sim_core.swerve_torch import SwerveGeometry, SwerveTorch  # noqa: E402
from swerve_follow.plant import SwervePlant  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from steps import plan  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('--out', required=True)
p.add_argument('--dr', choices=['mid', 'fast', 'slow'], default='mid')
a = p.parse_args()
cfg = Cfg()
c, dr = cfg.env, cfg.dr
pick = {'mid': lambda r: 0.5 * (r[0] + r[1]), 'fast': lambda r: r[0], 'slow': lambda r: r[1]}[a.dr]
dt = c.ctrl_dt
g = SwerveGeometry('cpu', torch.float32, path=c.swerve_yaml)
ctrl = SwerveTorch(1, 'cpu', geom=g)
ctrl.max_linear[:], ctrl.max_angular[:], ctrl.max_linear_accel[:], ctrl.max_angular_accel[:] = c.vmax, c.wmax, c.amax, c.awmax
pl = SwervePlant(1, g, dt)
# 조향 속도: fast = 가장 빠른 단계 × 큰 배율, slow = 가장 느린 단계 × 작은 배율 (다른 값은 시간이 작을수록 빠름)
rate = {'fast': max(dr.steer_rate_levels) * dr.steer_rate_scale[1], 'slow': min(dr.steer_rate_levels) * dr.steer_rate_scale[0],
        'mid': dr.steer_rate_levels[-1] * 0.5 * sum(dr.steer_rate_scale)}[a.dr]
pl.rate_max[:] = rate
pl.tau_s[:], pl.tau_w[:], pl.tau_b[:] = pick(dr.tau_steer), pick(dr.tau_wheel), pick(dr.tau_body)
pl.slip[:] = pick(dr.slip)
delay = int(round({'mid': 0.5 * sum(dr.cmd_delay_ticks), 'fast': dr.cmd_delay_ticks[0], 'slow': dr.cmd_delay_ticks[1]}[a.dr]))
ctrl.reset(torch.arange(1))

t, cmds, odom, tm, to, log = 0.0, [], [], [], [], []
queue = [(0.0, 0.0, 0.0)] * (delay + 1)
podo = np.zeros(3)


def run(sec, cmd):
    global t, podo
    for _ in range(int(round(sec / dt))):
        queue.append(cmd)
        cd = torch.tensor([queue.pop(0)], dtype=torch.float32)
        sg, wg = ctrl.update(cd, pl.q, pl.w, pl.qd, dt)
        pl.step(sg, wg)
        o = pl.odom[0].numpy()
        th = podo[2]
        podo = podo + np.array([np.cos(th) * o[0] - np.sin(th) * o[1], np.sin(th) * o[0] + np.cos(th) * o[1], o[2]]) * dt
        cmds.append((t, *cmd))
        odom.append((t, *podo, *o))
        tm.append((t, *pl.pose[0].numpy()))
        to.append((t, *podo))
        t += dt


run(2.0, (0.0, 0.0, 0.0))
for name, vx, vy, wz, hold, pause in plan():
    ts = t
    run(hold, (vx, vy, wz))
    tz = t
    run(pause, (0.0, 0.0, 0.0))
    log.append(dict(name=name, cmd=[vx, vy, wz], t_start=ts, t_zero=tz, t_end=t))
out = Path(a.out)
out.mkdir(parents=True, exist_ok=True)
z = np.zeros(0)
np.savez_compressed(out / 'data.npz', cmd=np.array(cmds), odom=np.array(odom), tf_map=np.array(tm), tf_odom=np.array(to),
                    js_t=z, js_pos=z, js_vel=z, imu=z, js_names=np.array([]))
(out / 'segments.json').write_text(json.dumps(dict(segments=log, dr=a.dr, delay_ticks=delay), indent=1))
print(f'학습 플랜트 ({a.dr}: 조향 {float(pl.rate_max):.2f} rad/s · τ 조향 {float(pl.tau_s):.3f} · 바퀴 {float(pl.tau_w):.3f} · 몸 {float(pl.tau_b):.3f} s, '
      f'명령 지연 {delay * 10} ms, 미끄럼 {float(pl.slip[0, 0]):.2f}) → {out}')
