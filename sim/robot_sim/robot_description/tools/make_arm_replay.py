"""실물 팔 실행 기록 → 시뮬 관절 궤적 (config/arm_replay_A9F5.json).
원본: arm_staged_trial_claude_20261021/runs/ARM_20261026T022813Z/03_session.jsonl (A9F5 실물 3/3 중 최단 26.36 s; 인자로 다른 실행 선택) (손 시연 재생, 받침→P0→퍽 잡아 적재함→받침).
snapshot(50 ms) 의 실제 Present Position 을 joint_map.json 식으로 관절값으로 바꿈: q = radians(q_deg_at_ref) + sign·(tick − ref)·2π/4096.
그리퍼(ID15) 는 틱→간격 표 보간(2658 이상 = 16 mm), 손가락 q = (간격 − gap0)/2000.
단계 경계는 02_stages.json 의 재생 시간(τ)을 replay_progress(τ ↔ 실제 시간)로 실제 시간에 맞춤.
실행: python3 tools/make_arm_replay.py [ARM_실행이름 [출력.json]]  (성공 실행 전부: tools/make_all_replays.sh)"""
import json, math, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parents[1]
RUN = Path("/home/pa25/Documents/Codex/2026-10-20/task-3/arm_staged_trial_claude_20261021/runs") / (sys.argv[1] if len(sys.argv) > 1 else "ARM_20261026T022813Z")   # A9F5 3/3 중 가장 짧은 실물 실행(26.36 s)
jm = json.loads((HERE / "config/joint_map.json").read_text())
rows = [json.loads(l) for l in (RUN / "03_session.jsonl").open()]
prog = [r for r in rows if r.get("event") == "replay_progress"]
tau = np.array([p["tau_ms"] for p in prog], float); mono = np.array([p["monotonic_ms"] for p in prog], float)
t0 = mono[0] - tau[0]
snap = [r for r in rows if r.get("event") == "snapshot" and t0 - 1000 <= r["monotonic_ms"] <= mono[-1] + 3000]
tab = np.array(jm["gripper"]["gap_table_tick_mm"], float); gap0 = jm["gripper"]["gap0_mm"]


def arm_q(ids):
    return [math.radians(a["q_deg_at_ref"]) + a["sign"] * (ids[str(a["id"])]["pos"] - a["ref_tick"]) * 2 * math.pi / 4096 for a in jm["arm"]]


def gap_mm(tick):
    return float(np.interp(tick, tab[:, 0], tab[:, 1]))   # clamps: ≤1045 → 79, ≥2658 → 16


T, Q, G = [], [], []
for s in snap:
    T.append((s["monotonic_ms"] - t0) / 1000.0); Q.append(arm_q(s["ids"])); G.append(gap_mm(s["ids"]["15"]["pos"]))
T = np.array(T); keep = np.r_[True, np.diff(T) > 1e-6]
T, Q, G = T[keep], np.array(Q)[keep], np.array(G)[keep]
stages = json.loads((RUN / "02_stages.json").read_text())["rows"]


def wall(tau_s):   # plan time → wall time (s, from replay start)
    return float(np.interp(tau_s * 1000.0, tau, mono) - t0) / 1000.0


seg = []
for r in stages:
    a, b = [float(x) for x in r["status"].split("재생 ")[1].replace("초", "").split("~")]
    seg.append({"name": r["name"], "tau_s": [a, b], "wall_s": [round(wall(a), 3), round(wall(b), 3)]})
seg[0]["wall_s"][0] = 0.0                       # 받침→P0: 받침 자세(재생 시작)부터
seg[-1]["wall_s"][1] = round(float(T[-1]), 3)   # P0→받침: 기록 끝(받침 도착)까지
out = {"source": str(RUN), "formula": jm["formula"], "joint_names": [a["joint"] for a in jm["arm"]],
       "t_s": [round(x, 3) for x in T], "arm_q_rad": [[round(v, 6) for v in q] for q in Q],
       "gripper_gap_mm": [round(g, 2) for g in G], "finger_q_m": [round((g - gap0) / 2000.0, 7) for g in G],
       "segments": seg,
       "note": "실물 측정 위치(목표 아님). 그리퍼 약 2297~2301 틱(약 29 mm)에서 멈춘 구간 = 퍽을 잡은 상태. 시뮬에 퍽이 없으면 그 값으로 위치 지령만 줌."}
OUTF = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE / "config/arm_replay_A9F5.json"
OUTF.parent.mkdir(exist_ok=True); OUTF.write_text(json.dumps(out, ensure_ascii=False))
print(len(T), "samples", round(T[0], 2), "→", round(T[-1], 2), "s")
for s in seg:
    print(f'  {s["name"]:<20} τ {s["tau_s"]}  wall {s["wall_s"]}')
