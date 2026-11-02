#!/usr/bin/env python3
"""Python 스워브(swerve/nt_swerve.py) ↔ 팀 C++ 원본(nt_control swerve.cpp) 일치 검사. 포트·실물 없음.
  1) 역기구학 무작위 N건(명령·현재 조향·직전 목표·바깥쪽 선호·cos 감속 조합)
  2) 상태 기계(SwerveDrive.update) 주행 순서: S자·ㄹ자·전방향·회전→전진·정지·명령 끊김. 조향은 4.8 rad/s 로 목표를 따라가는 단순 모형.
     같은 '실측' 순서를 양쪽에 넣고 매 주기 출력 비교.
실행: python3 tools/check_swerve_port.py <cpp_ref 실행 파일>   (cpp_ref 는 tools/port_check/cpp_ref.cpp 를 팀 libnt_control_core.a 와 빌드)"""
import math, random, subprocess, sys, json
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE / "swerve"))
from nt_swerve import ModuleState, Twist, SwerveDrive, from_config  # noqa: E402

kin, cfg, _ = from_config(str(HERE / "config/swerve.yaml"))
rng = random.Random(20261027)
head = " ".join(f"{m.x} {m.y} {m.offset}" for m in kin.modules) + " " + " ".join(f"{r.min} {r.max}" for r in kin.ranges)
head += f" {kin.r} {kin.branch_margin} {kin.end_slack} {cfg.max_wheel_speed} {cfg.max_linear} {cfg.max_angular} {cfg.max_linear_accel}"
head += f" {cfg.max_angular_accel} {cfg.cmd_timeout_s} {int(cfg.cosine_scale)} {int(cfg.steer_then_drive)} {cfg.steer_tolerance}"
head += f" {int(cfg.roll_while_steering)} {int(cfg.blend_steer_drive)} {cfg.blend_margin} {cfg.blend_stall_s}"
lines, expect = [head], []

# 1) 역기구학
V, W = 0.2612, 1.675
for _ in range(6000):
    c = Twist(rng.uniform(-V, V) * rng.choice([0, 1, 1, 1]), rng.uniform(-V, V) * rng.choice([0, 1, 1, 1]), rng.uniform(-W, W) * rng.choice([0, 0, 1]))
    cur = [rng.uniform(r.min, r.max) for r in kin.ranges] if rng.random() < 0.85 else []
    prev = [rng.uniform(r.min, r.max) for r in kin.ranges] if rng.random() < 0.6 else []
    po, cs = rng.random() < 0.5, rng.random() < 0.5
    out, feas = kin.inverse(c, [ModuleState(a) for a in cur], cs, [ModuleState(a) for a in prev], po)
    lines.append(f"I {c.vx!r} {c.vy!r} {c.wz!r} {int(po)} {int(cs)} {len(cur)} " + " ".join(map(repr, cur)) + f" {len(prev)} " + " ".join(map(repr, prev)))
    expect.append(("I", [int(feas)] + [v for m in out for v in (m.steer_angle, m.wheel_speed)]))

# 2) 주행 순서 (C++ 과 같은 '실측'을 넣기 위해 Python 출력으로 단순 모형을 굴리고 그 실측을 기록)
D = 0.7071 * V
SEQS = {
    "S자": [((V, 0, 0), 1.5), ((D, D, 0), 1.5), ((V, 0, 0), 1.5), ((D, -D, 0), 1.5), ((V, 0, 0), 1.5)],
    "ㄹ자": [((V, 0, 0), 2), ((D, D, 0), 2), ((0, V, 0), 2), ((-V, 0, 0), 2)],
    "전방향": [((math.cos(k * math.pi / 4) * V, math.sin(k * math.pi / 4) * V, 0), 1.2) for k in range(9)],
    "회전→전진": [((0, 0, W), 2), ((V, 0, 0), 2), ((0, 0, 0), 1)],
    "곡선": [((V, 0, 1.0), 2), ((V, 0, -1.0), 2)],
    "옆+회전·끊김": [((0, V, 0.8), 2), (None, 0.6), ((V, 0, 0), 1)],
    "정지·재출발": [((0, V, 0), 1), ((0, 0, 0), 1.5), ((0, -V, 0), 1.5)],
}
dt = 0.01
for name, seq in SEQS.items():
    lines.append("R"); expect.append(("R", []))
    drv = SwerveDrive(kin, cfg)
    meas = [ModuleState(0.0, 0.0, 0.0) for _ in range(4)]
    t = 0.0
    for cmd, secs in seq:
        for _ in range(int(secs / dt)):
            t += dt
            hc = cmd is not None
            if hc:
                drv.set_command(*cmd, t)
            line = f"D {t!r} {dt!r} " + (" ".join(map(repr, cmd)) if hc else "0 0 0") + f" {int(hc)} " + " ".join(f"{m.steer_angle!r} {m.steer_rate!r} {m.wheel_speed!r}" for m in meas)
            out = drv.update(t, dt, meas)
            lines.append(line); expect.append(("D", [v for m in out for v in (m.steer_angle, m.wheel_speed)]))
            new = []
            for o, m in zip(out, meas):     # 단순 모형: 조향 4.8 rad/s 로 목표 추종, 바퀴 즉시
                e = o.steer_angle - m.steer_angle
                step = max(-4.8 * dt, min(4.8 * dt, e))
                new.append(ModuleState(m.steer_angle + step, o.wheel_speed, step / dt))
            meas = new

res = subprocess.run([sys.argv[1]], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True).stdout.splitlines()
got = iter(res)
worst = {"I": 0.0, "D": 0.0}; nbad = {"I": 0, "D": 0}; ncase = {"I": 0, "D": 0}
for kind, exp in expect:
    if kind == "R":
        continue
    vals = [float(x) for x in next(got).split()]
    ncase[kind] += 1
    d = max(abs(a - b) for a, b in zip(vals, exp)) if len(vals) == len(exp) else math.inf
    worst[kind] = max(worst[kind], d)
    if d > 1e-9:
        nbad[kind] += 1
report = {"inverse_cases": ncase["I"], "inverse_mismatch": nbad["I"], "inverse_max_abs_diff": worst["I"],
          "drive_steps": ncase["D"], "drive_mismatch": nbad["D"], "drive_max_abs_diff": worst["D"], "sequences": list(SEQS)}
print(json.dumps(report, ensure_ascii=False, indent=1))
(HERE / "config/swerve_port_check.json").write_text(json.dumps(report, ensure_ascii=False, indent=1))
sys.exit(0 if nbad["I"] == 0 and nbad["D"] == 0 else 1)
