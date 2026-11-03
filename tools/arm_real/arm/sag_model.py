"""팔 처짐 모델 (2026-10-30 손눈 보정에서 구함, handeye_sag.py): 실제 관절 = 엔코더 + c·τ_j (j = J2 · J3 · J4, τ = 중력 토크 N·m).
c = camera_calib.json 의 sag_c_deg_per_Nm (3.07 °/N·m). 질량 어림: 링크2 구간 0.12 · 링크3 0.12 · 링크4 0.10 · 손목+그리퍼+카메라 0.40 kg.
held_kg: 쥔 물체 (손끝에) — 퍽 쥐면 그만큼 처짐 늘어남 (추정)."""
import json
import math
from pathlib import Path

import numpy as np

G = 9.81
CAL = Path(__file__).with_name('camera_calib.json')


def c_rad():
    return math.radians(json.loads(CAL.read_text())['sag_c_deg_per_Nm'])


def torques(M, q, fq, held_kg=0.0):
    Ts = M.fk(q, fq)
    name = {2: 'arm_link2', 3: 'arm_link3', 4: 'arm_link4_wrist'}
    o = {j: Ts[name[j]][:3, 3] for j in (2, 3, 4)}
    p5 = Ts['arm_link5_wrist_roll'][:3, 3]
    tip = Ts['arm_link5_wrist_roll'][:3, :3] @ M.tip5 + p5
    masses = [((o[2] + o[3]) / 2, 0.12, {2}), ((o[3] + o[4]) / 2, 0.12, {2, 3}), ((o[4] + p5) / 2, 0.10, {2, 3, 4}),
              (p5 + (tip - p5) / 3, 0.40, {2, 3, 4}), (tip, held_kg, {2, 3, 4})]
    tau = {}
    for j in (2, 3, 4):
        ax = Ts[name[j]][:3, :3] @ np.asarray(M.u.joints[f'arm_joint{j}']['axis'], float)
        tau[j] = sum(np.cross(p - o[j], np.array([0, 0, -m * G])) @ ax for p, m, js in masses if j in js and m > 0)
    return tau


def encoder_goal_q(M, q_true, fq, held_kg=0.0, c=None):
    """원하는 실제 관절 q_true 가 되려면 엔코더가 가리켜야 할 관절 (= q_true − c·τ(q_true))"""
    c = c_rad() if c is None else c
    tau = torques(M, q_true, fq, held_kg)
    q = np.array(q_true, float).copy()
    for j in (2, 3, 4):
        q[j - 1] -= c * tau[j]
    return q


# 모터 위치 오차 (엔코더에 보임 — 위 처짐과 별개): 팀 ROS 경로는 위치 P 게인만(기본 800, I 0) → 중력을 버티려면 그만큼 목표에서 밀려남.
# 10-30 실물 멈춤 20 장(runner_20261030T165901 · 172009, chatgpt_runs_review/fit_pgap.py) 맞춤: 오차(실측 − 명령) = a_j · τ_j 틱, 잔차 σ 3~7 틱.
#   (ChatGPT 팔 엔진은 J2 · J3 · J4 위치 I 게인 100 → 같은 하중에서 버틸 때 오차 중앙 0~2 틱, chatgpt_runs_review/static_err.py)
PGAP_P_ONLY = {2: 21.6, 3: -23.7, 4: 22.5}
# 10-30 20:35: 팔 보드 펌웨어 fw_ext (./arm_fw.sh ext) 가 J2 · J3 · J4 위치 I 게인 100 → 모터가 이 오차를 스스로 없앰 → 앞먹임 0 (안 끄면 I 가 반대로 밀어 앞먹임만큼 틀림).
#   게인 없는 펌웨어(nt · ext0) 로 되돌리면 환경 변수 NT_ARM_PONLY=1 로 pose_safety 를 돌려 checked.json 을 다시 만들 것.
import os  # noqa: E402
PGAP_TICK_PER_NM = PGAP_P_ONLY if os.environ.get('NT_ARM_PONLY') == '1' else {2: 0.0, 3: 0.0, 4: 0.0}
PGAP_MAX = 60


def pgap_ticks(M, q_true, fq, held_kg=0.0):
    """명령에 더할 틱 [J1..J5] — 엔코더가 목표에 오도록 중력 위치 오차만큼 미리 반대로 (−a_j·τ_j, ±PGAP_MAX 안)"""
    tau = torques(M, q_true, fq, held_kg)
    out = [0, 0, 0, 0, 0]
    for j in (2, 3, 4):
        out[j - 1] = int(round(max(-PGAP_MAX, min(PGAP_MAX, -PGAP_TICK_PER_NM[j] * tau[j]))))
    return out
