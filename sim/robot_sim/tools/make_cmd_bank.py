#!/usr/bin/env python3
"""속도 추종 학습용 명령 은행 rl/data/swerve_track/cmd_bank.npz (swerve_track).

명령 = 20 Hz 몸 속도 (vx, vy, wz) 열. 두 출처:
  follower: swerve_follow 기준 추종기를 경로 은행 (split 0 학습 · 1 평가) 위에서 간이 플랜트로 닫힌 고리 주행 → 보낸 명령 기록.
            실제 MPPI 출력 대신 쓰는 대용 (MPPI 녹화는 sim_nav2 로 따로, 미구현).
  synth:    구간 상수 명령 (방향 ±90° · 180° 전환, 정지 ↔ 출발, 회전 동시) → 가속 한계로 램프 → 잡음 0.005.
한 열 = 최대 SEQ 스텝 (20 s), 긴 주행은 잘라 나눔. 속도 한계 프로필 (v_max, a_max) 을 열마다 같이 저장 (재생 때 같은 한계).

예) cd sim/robot_sim && env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python tools/make_cmd_bank.py
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

SEQ = 400          # 20 s × 20 Hz
MIN_LEN = 60       # 3 s 보다 짧은 조각은 버림
DT = 0.05
W_PER_V = 1.675 / 0.2612


def follower_cmds(split, device, seed):
    """경로 은행 split 의 경로마다 기준 추종기 첫 에피소드 명령 열 → [(cmd (T,3), vmax, amax)]"""
    import torch
    from swerve_follow.cfg import Cfg
    from swerve_follow.env import SwerveFollowEnv
    cfg = Cfg()
    cfg.env.device, cfg.env.seed, cfg.env.split = device, seed, split
    cfg.env.curriculum, cfg.env.sequential_paths = False, True
    n_paths = int((np.load(cfg.env.path_bank)['split'] == split).sum())
    cfg.env.num_envs = n_paths
    env = SwerveFollowEnv(cfg)
    B, TMAX = env.num_envs, 1600
    buf = torch.zeros(B, TMAX, 3, device=device)
    length = torch.zeros(B, dtype=torch.long, device=device)
    done = torch.zeros(B, dtype=torch.bool, device=device)
    vmax, amax = env.vmax.clone(), env.amax.clone()
    t = 0
    with torch.inference_mode():
        while not bool(done.all()) and t < TMAX:
            env.step(torch.zeros(B, 3, device=device))
            live = ~done
            buf[live, t] = env.last_cmd[live]
            length[live] = t + 1
            d = env.last_done
            if d is not None:
                done[d['ids']] = True
            t += 1
    out = []
    buf, length, vmax, amax = buf.cpu().numpy(), length.cpu().numpy(), vmax.cpu().numpy(), amax.cpu().numpy()
    for i in range(B):
        c = buf[i, :length[i]]
        for k in range(0, len(c), SEQ):
            seg = c[k:k + SEQ]
            if len(seg) >= MIN_LEN:
                out.append((seg, float(vmax[i]), float(amax[i])))
    return out


def synth_cmds(n, rng, profiles):
    out = []
    for _ in range(n):
        vmax, amax = profiles[rng.integers(len(profiles))]
        wmax = vmax * W_PER_V
        tgt = []
        d_prev = rng.uniform(-math.pi, math.pi)
        while len(tgt) < SEQ:
            T = int(rng.uniform(0.5, 3.0) / DT)
            if rng.random() < 0.15:
                v = np.zeros(3)
            else:
                r = rng.random()
                d = d_prev + (rng.choice([-1, 1]) * math.pi / 2 if r < 0.4 else math.pi if r < 0.6 else rng.uniform(-math.pi, math.pi))
                d_prev = d
                s = rng.uniform(0.3, 1.0) * vmax
                wz = 0.0 if rng.random() < 0.5 else rng.uniform(-0.5, 0.5) * wmax
                v = np.array([s * math.cos(d), s * math.sin(d), wz])
            tgt += [v] * T
        tgt = np.array(tgt[:SEQ])
        # 가속 한계 램프 (MPPI 도 가속 한계 안에서 냄) — 선속도 벡터 · 각속도 따로
        c = np.zeros_like(tgt)
        cur = np.zeros(3)
        for k in range(SEQ):
            dl = tgt[k, :2] - cur[:2]
            n_ = np.linalg.norm(dl)
            step = amax * DT
            cur[:2] += dl if n_ <= step else dl / n_ * step
            dw = tgt[k, 2] - cur[2]
            cur[2] += np.clip(dw, -6 * amax * DT, 6 * amax * DT)
            c[k] = cur
        c += rng.normal(0, 0.005, c.shape) * (np.abs(c) > 1e-6)
        out.append((c.astype(np.float32), vmax, amax))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--device', default='cuda')
    p.add_argument('--seed', type=int, default=3)
    p.add_argument('--synth_train', type=int, default=4000)
    p.add_argument('--synth_eval', type=int, default=600)
    p.add_argument('--out', default=str(ROOT / 'rl' / 'data' / 'swerve_track' / 'cmd_bank.npz'))
    a = p.parse_args()
    rng = np.random.default_rng(a.seed)
    profiles = ((0.10, 0.2), (0.13, 0.2))
    rows = []          # (cmd, vmax, amax, split, src)
    for split in (0, 1):
        f = follower_cmds(split, a.device, a.seed + split)
        s = synth_cmds(a.synth_train if split == 0 else a.synth_eval, rng, profiles)
        rows += [(c, v, am, split, 0) for c, v, am in f] + [(c, v, am, split, 1) for c, v, am in s]
        print(f'split {split}: follower {len(f)} · synth {len(s)}', flush=True)
    N = len(rows)
    cmd = np.zeros((N, SEQ, 3), np.float32)
    n = np.zeros(N, np.int64)
    for i, (c, *_r) in enumerate(rows):
        cmd[i, :len(c)] = c
        n[i] = len(c)
    np.savez_compressed(a.out, cmd=cmd, n=n, vmax=np.array([r[1] for r in rows], np.float32), amax=np.array([r[2] for r in rows], np.float32),
                        split=np.array([r[3] for r in rows], np.int64), src=np.array([r[4] for r in rows], np.int64), src_names=np.array(['follower', 'synth']))
    print(f'→ {a.out}  열 {N}, 길이 중앙 {np.median(n) * DT:.1f} s')


if __name__ == '__main__':
    main()
