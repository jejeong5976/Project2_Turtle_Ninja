#!/usr/bin/env python3
"""swerve_torch(벡터 판) ↔ nt_swerve(Python 원본) 일치 검사. Isaac 없이 torch 만 씀.

1) 역기구학 무작위 N 건 (명령 · 측정 조향 · 이전 목표 · 바깥쪽 선호 · cos 감속 조합, 범위 끝 근처 포함)
2) 상태 기계 닫힌 루프: env B 개에 무작위 명령 순서(반전 · 정지 · 제자리 회전 · 포화 · 짧은 펄스)를 주고,
   원본 출력으로 단순 조향 · 바퀴 모형을 움직여 같은 '측정값'을 양쪽에 넣음 → 매 주기 조향 목표 · 바퀴 목표 · 내부 상태 비교.
   한계(max_linear 등)는 env 마다 다르게 (학습과 같음).
float64 로 비교. 기준: 조향 1e-6 rad, 바퀴 1e-5 rad/s.
실행: env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python tools/check_swerve_torch.py
"""
import math
import random
import sys
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'robot_description' / 'swerve'))
import nt_swerve as nt  # noqa: E402
from sim_core import swerve_torch as st  # noqa: E402

torch.set_default_dtype(torch.float64)
DEV = 'cpu'
TOL_A, TOL_W = 1e-6, 1e-5
rng = random.Random(20261030)

kin, base_cfg, _ = nt.from_config(str(st.SWERVE_YAML))
geom = st.SwerveGeometry(DEV, torch.float64)
RANGES = [(r.min, r.max) for r in kin.ranges]


def rand_angle(i):
    lo, hi = RANGES[i]
    u = rng.random()
    if u < 0.15:
        return lo + rng.uniform(0, 0.05)       # 범위 끝 근처
    if u < 0.30:
        return hi - rng.uniform(0, 0.05)
    return rng.uniform(lo, hi)


# ---------------------------------------------------------------- 1) 역기구학
def check_inverse(n=20000):
    V, W = 0.2612, 1.675
    cmds, nows, prevs, hps, pos, css = [], [], [], [], [], []
    exp_a, exp_w, exp_ok = [], [], []
    for _ in range(n):
        c = nt.Twist(rng.uniform(-V, V) * rng.choice([0, 1, 1, 1]), rng.uniform(-V, V) * rng.choice([0, 1, 1, 1]),
                     rng.uniform(-W, W) * rng.choice([0, 0, 1]))
        if rng.random() < 0.05:
            c = nt.Twist(rng.choice([0.0, 5e-5]), 0.0, rng.choice([0.0, 0.3]))
        now = [rand_angle(i) for i in range(4)]
        hp = rng.random() < 0.6
        prev = [rand_angle(i) for i in range(4)]
        po, cs = rng.random() < 0.5, rng.random() < 0.5
        out, ok = kin.inverse(c, [nt.ModuleState(a) for a in now], cs, [nt.ModuleState(a) for a in prev] if hp else [], po)
        cmds.append([c.vx, c.vy, c.wz]); nows.append(now); prevs.append(prev); hps.append(hp); pos.append(po); css.append(cs)
        exp_a.append([m.steer_angle for m in out]); exp_w.append([m.wheel_speed for m in out]); exp_ok.append(ok)
    T = lambda x, **k: torch.tensor(x, **k)  # noqa: E731
    drv = st.SwerveTorch(n, DEV, torch.float64, geom=geom)
    bad = 0
    for cs in (False, True):
        m = T(css) == cs
        a, w, ok = drv.inverse(T(cmds)[m], T(nows)[m], cs, T(prevs)[m], T(hps)[m], T(pos)[m])
        ea, ew, eok = T(exp_a)[m], T(exp_w)[m], T(exp_ok)[m]
        da, dw = (a - ea).abs().amax(-1), (w - ew).abs().amax(-1)
        wrong = (da > TOL_A) | (dw > TOL_W) | (ok != eok)
        bad += int(wrong.sum())
        if wrong.any():
            k = int(torch.nonzero(wrong)[0])
            print(f'  [역기구학 cos={cs}] 첫 불일치: cmd {T(cmds)[m][k].tolist()} now {T(nows)[m][k].tolist()} '
                  f'prev {T(prevs)[m][k].tolist()} hp {T(hps)[m][k].item()} po {T(pos)[m][k].item()}\n'
                  f'    torch {a[k].tolist()} {w[k].tolist()}\n    원본  {ea[k].tolist()} {ew[k].tolist()}')
    print(f'[1] 역기구학 {n} 건: 불일치 {bad}')
    return bad == 0


# ---------------------------------------------------------------- 2) 상태 기계 닫힌 루프
def command_sequence(steps, vmax, wmax):
    """0.01 s 주기 명령 순서: 0.2–2 s 구간마다 종류를 바꿈"""
    seq = []
    while len(seq) < steps:
        kind = rng.choice(['move', 'move', 'reverse', 'stop', 'spin', 'sat', 'pulse', 'turn90'])
        dur = rng.randint(20, 200)
        th = rng.uniform(-math.pi, math.pi)
        v = rng.uniform(0.2, 1.0) * vmax
        if kind == 'move':
            c = (v * math.cos(th), v * math.sin(th), rng.uniform(-0.5, 0.5) * wmax)
            seq += [c] * dur
        elif kind == 'reverse':
            c = (v * math.cos(th), v * math.sin(th), 0.0)
            seq += [c] * (dur // 2) + [(-c[0], -c[1], 0.0)] * (dur // 2)
        elif kind == 'stop':
            seq += [(0.0, 0.0, 0.0)] * dur
        elif kind == 'spin':
            seq += [(0.0, 0.0, rng.choice([-1, 1]) * wmax)] * dur
        elif kind == 'sat':
            seq += [(3 * vmax * math.cos(th), 3 * vmax * math.sin(th), 3 * wmax)] * dur
        elif kind == 'pulse':
            for _ in range(dur // 5):
                seq += [(v * math.cos(th), v * math.sin(th), 0.0)] * 2 + [(0.0, 0.0, 0.0)] * 3
        else:
            c = (v * math.cos(th), v * math.sin(th), 0.0)
            seq += [c] * (dur // 2) + [(-c[1], c[0], 0.0)] * (dur // 2)
    return seq[:steps]


def check_state_machine(B=48, steps=3000, dt=0.01):
    profiles = [(0.10, 0.2, 0.64, 1.2), (0.13, 0.2, 0.834, 1.2), (0.2612, 1.0, 1.675, 6.0)]
    drives, seqs, rates = [], [], []
    drv_t = st.SwerveTorch(B, DEV, torch.float64, geom=geom)
    for e in range(B):
        vmax, acc, wmax, wacc = profiles[e % 3]
        k, c, _ = nt.from_config(str(st.SWERVE_YAML))
        c.max_linear, c.max_linear_accel, c.max_angular, c.max_angular_accel = vmax, acc, wmax, wacc
        c.steer_tolerance = math.pi
        drives.append(nt.SwerveDrive(k, c))
        drv_t.max_linear[e], drv_t.max_linear_accel[e], drv_t.max_angular[e], drv_t.max_angular_accel[e] = vmax, acc, wmax, wacc
        seqs.append(command_sequence(steps, vmax, wmax))
        rates.append(rng.choice([0.96, 2.44, 4.8]))
    q = [[rand_angle(i) for i in range(4)] for _ in range(B)]
    w = [[0.0] * 4 for _ in range(B)]
    qd = [[0.0] * 4 for _ in range(B)]
    bad = {'angle': 0, 'wheel': 0, 'out': 0, 'gate': 0, 'blend': 0}
    first = None
    blend_used = gate_used = 0
    for t in range(steps):
        now = (t + 1) * dt
        exp_a, exp_w, exp_out, exp_gate, exp_blend = [], [], [], [], []
        for e, d in enumerate(drives):
            d.set_command(*seqs[e][t], now)
            meas = [nt.ModuleState(q[e][i], w[e][i], qd[e][i]) for i in range(4)]
            mods = d.update(now, dt, meas)
            exp_a.append([m.steer_angle for m in mods]); exp_w.append([m.wheel_speed for m in mods])
            exp_out.append([d.out.vx, d.out.vy, d.out.wz]); exp_gate.append(d.gate_blocked); exp_blend.append(d.blend_ratio)
        a, wh = drv_t.update(torch.tensor([s[t] for s in seqs]), torch.tensor(q), torch.tensor(w), torch.tensor(qd), dt)
        ea, ew = torch.tensor(exp_a), torch.tensor(exp_w)
        checks = {'angle': (a - ea).abs().amax(-1) > TOL_A, 'wheel': (wh - ew).abs().amax(-1) > TOL_W,
                  'out': (drv_t.out - torch.tensor(exp_out)).abs().amax(-1) > 1e-9,
                  'gate': drv_t.gate_blocked != torch.tensor(exp_gate),
                  'blend': (drv_t.blend_ratio - torch.tensor(exp_blend)).abs() > 1e-9}
        for kk, m in checks.items():
            bad[kk] += int(m.sum())
            if m.any() and first is None:
                e = int(torch.nonzero(m)[0])
                first = (t, e, kk, a[e].tolist(), ea[e].tolist(), wh[e].tolist(), ew[e].tolist(), seqs[e][t])
        blend_used += sum(b < 1.0 for b in exp_blend)
        gate_used += sum(exp_gate)
        # 원본 출력으로 단순 모형 진행 (양쪽 같은 측정값을 받게)
        for e in range(B):
            for i in range(4):
                lo, hi = RANGES[i]
                rate = max(-rates[e], min(rates[e], (exp_a[e][i] - q[e][i]) / 0.08))
                nq = min(max(q[e][i] + rate * dt, lo), hi)
                qd[e][i] = (nq - q[e][i]) / dt
                q[e][i] = nq
                w[e][i] += (exp_w[e][i] - w[e][i]) * (1 - math.exp(-dt / 0.04))
    print(f'[2] 상태 기계 {B} env × {steps} 주기: 불일치 {bad} (blend 동작 {blend_used} 주기, 게이트 {gate_used} 주기)')
    if first:
        print(f'    첫 불일치 t={first[0]} env={first[1]} 항목={first[2]} cmd={first[7]}\n'
              f'    torch 조향 {first[3]} 바퀴 {first[5]}\n    원본  조향 {first[4]} 바퀴 {first[6]}')
    return all(v == 0 for v in bad.values())


if __name__ == '__main__':
    ok1 = check_inverse()
    ok2 = check_state_machine()
    print('통과' if ok1 and ok2 else '실패')
    sys.exit(0 if ok1 and ok2 else 1)
