"""로봇 몸 모양과 앞 물체 검사 (2026-11-02) — path_follow.py · jetson/sensor_check.py · jetson/wall_follow.py 가 같이 씀.

몸 = 직사각형 BODY_HX × BODY_HY (반길이 · 반폭, base_footprint 기준). 조향축 앞 +0.060 · 뒤 −0.070 · 좌우 ±0.112 에
바퀴 끝과 11-01 실물에서 늘 보이던 왼앞 선 · 바퀴 점 (0.10, 0.17) 까지 들어가게 잡음 → 그 안 점은 자기 몸으로 버림.
11-02 GPT 검토: 예전처럼 ±0.21 정사각형으로 지우면 대각으로 갈 때 앞 장애물 (0.18, 0.18) 이 지워짐
→ 버리는 건 몸 직사각형만, 정지 범위는 가는 방향의 몸 끝(직사각형 지지 거리)부터 잼.
"""
import math

import numpy as np

BODY_HX, BODY_HY = 0.17, 0.19   # [m]


def self_mask(p):
    """몸 직사각형 밖 점만 (자기 바퀴 · 선 반사 버림). p = N×2 몸 기준 [m]"""
    return p[(np.abs(p[:, 0]) > BODY_HX) | (np.abs(p[:, 1]) > BODY_HY)]


def obstacle_hit(p, vx, vy, wz, remain=None, margin=0.09, side=0.0):
    """몸 속도 (vx, vy, wz) 로 가면 닿을 점이 있나 → 가장 가까운 거리 [m] 또는 None. p = self_mask 를 지난 몸 기준 점.
    직진 · 대각: 그 방향 몸 끝부터 margin 안, 몸 폭(그 방향 직각) + side 안. 목표까지 남은 거리(remain)가 짧으면 그만큼만 봄
    (목표 앞 벽은 안 봄 — 11-01 실물: 벽 27 cm 앞 목표에서 계속 멈춤).
    제자리 회전: 몸 모서리 원 + 1 cm 안."""
    sp = math.hypot(vx, vy)
    if sp > 0.005:
        ux, uy = vx / sp, vy / sp
        front = BODY_HX * abs(ux) + BODY_HY * abs(uy)
        half_w = BODY_HX * abs(uy) + BODY_HY * abs(ux)
        reach = front + (margin if remain is None else min(margin, remain + 0.02))
        al, la = p[:, 0] * ux + p[:, 1] * uy, -p[:, 0] * uy + p[:, 1] * ux
        hit = (al > 0) & (al < reach) & (np.abs(la) < half_w + side)
    elif abs(wz) > 0.01:
        hit = np.hypot(p[:, 0], p[:, 1]) < math.hypot(BODY_HX, BODY_HY) + 0.01
    else:
        return None
    return float(np.min(np.hypot(p[hit, 0], p[hit, 1]))) if hit.any() else None
