"""로봇 몸 모양과 앞 물체 검사 — drive/path_follow.py · diagnostics/sensor_check.py 가 같이 씀.

몸 = 직사각형 BODY_HX × BODY_HY (반길이 · 반폭, base_footprint 기준). 조향축 앞 +0.060 · 뒤 −0.070 · 좌우 ±0.112 에
바퀴 끝과 실물에서 늘 보이던 왼앞 선 · 바퀴 점 (0.10, 0.17) 까지 들어가게 잡음 → 그 안 점은 자기 몸으로 버림.
몸 상자는 navigation/config/navigation.yaml 의 scan_merger body_half 와 같은 값 (합친 /scan 에서 이미 빠짐).
정지 범위는 가는 방향의 몸 끝(직사각형 지지 거리)부터 잼 — 정사각형으로 크게 지우면 대각 앞 장애물까지 지워짐.
"""
import math

import numpy as np

BODY_HX, BODY_HY = 0.17, 0.19   # [m]


def self_mask(p):
    """몸 직사각형 밖 점만 (자기 바퀴 · 선 반사 버림). p = N×2 몸 기준 [m]"""
    return p[(np.abs(p[:, 0]) > BODY_HX) | (np.abs(p[:, 1]) > BODY_HY)]


STOP_LATENCY_S = 0.3   # 스캔(10 Hz) + 판단 + 명령 전달까지 [s]
STOP_DECEL = 0.5       # [m/s²] swerve_controller max_linear_accel 과 같게
SWEEP_S = 0.5          # 이동 + 회전 명령일 때 몸이 쓸고 지나가는 범위를 보는 시간 [s]


def stop_distance(speed):
    """이 속도에서 물체를 본 뒤 멈출 때까지 가는 거리 [m]"""
    return speed * STOP_LATENCY_S + speed * speed / (2.0 * STOP_DECEL)


def obstacle_hit(p, vx, vy, wz, remain=None, margin=0.09, side=0.0):
    """몸 속도 (vx, vy, wz) 로 가면 닿을 점이 있나 → 가장 가까운 거리 [m] 또는 None. p = self_mask 를 지난 몸 기준 점.
    직진 · 대각: 그 방향 몸 끝부터 max(margin, 정지 거리) 안, 몸 폭(그 방향 직각) + side 안.
      목표까지 남은 거리(remain)가 짧으면 그만큼만 봄 (목표 앞 벽은 안 봄 — 벽 27 cm 앞 목표에서 계속 멈추지 않게).
    이동 + 회전: 위에 더해 SWEEP_S 동안 회전하며 움직인 몸 직사각형(+ side) 안에 들어오는 점.
    제자리 회전: 몸 모서리 원 + 1 cm 안."""
    sp = math.hypot(vx, vy)
    if sp > 0.005:
        ux, uy = vx / sp, vy / sp
        front = BODY_HX * abs(ux) + BODY_HY * abs(uy)
        half_w = BODY_HX * abs(uy) + BODY_HY * abs(ux)
        ahead = max(margin, stop_distance(sp))
        reach = front + (ahead if remain is None else min(ahead, remain + 0.02))
        al, la = p[:, 0] * ux + p[:, 1] * uy, -p[:, 0] * uy + p[:, 1] * ux
        hit = (al > 0) & (al < reach) & (np.abs(la) < half_w + side)
        if abs(wz) > 0.01:
            for t in np.linspace(0.1, SWEEP_S, 5):
                c, s = math.cos(-wz * t), math.sin(-wz * t)
                qx, qy = p[:, 0] - vx * t, p[:, 1] - vy * t
                rx, ry = c * qx - s * qy, s * qx + c * qy
                hit |= (np.abs(rx) <= BODY_HX + side) & (np.abs(ry) <= BODY_HY + side)
    elif abs(wz) > 0.01:
        hit = np.hypot(p[:, 0], p[:, 1]) < math.hypot(BODY_HX, BODY_HY) + 0.01
    else:
        return None
    return float(np.min(np.hypot(p[hit, 0], p[hit, 1]))) if hit.any() else None
