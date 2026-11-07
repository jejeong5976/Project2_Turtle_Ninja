"""기준 추종기 (잔차 정책의 바탕 = 잔차 0 기준선). scripts/measure_dice_routes.py 추종기와 같은 식을 torch 로:
  전방 주시점 = 경로 위 (투영 s + lookahead), 남은 길이 < lookahead 면 목표점.
  속도 = min(v_max, √(2·0.8·a_max·남은 거리), max(0.02, 2·남은 거리)) 를 주시점 방향으로 (몸 방향 유지한 채 평행 이동).
  회전 = gain × (목표 yaw − yaw), ±w_max. 처음부터 목표 yaw 로 천천히 돎.
  다른 점: 남은 거리 = max(경로 남은 길이, 목표까지 직선거리), 목표 1 cm 안에서는 속도를 거리 비례로 줄임 (0.02 m/s 떨림 방지).
"""
import torch

from .paths import PathBatch, to_body, wrap


def follow(path: PathBatch, pose, s, vmax, amax, wmax, lookahead: float, yaw_gain: float):
    """pose (B,3) 추정 자세, s (B,) 추정 진행 → body 명령 (B,3)"""
    xy = pose[:, :2]
    rem_path = (path.length - s).clamp(min=0.0)
    tgt, _ = path.point_at((s + lookahead).unsqueeze(-1))
    tgt = tgt.squeeze(1)
    goal = path.goal
    tgt = torch.where((rem_path < lookahead).unsqueeze(-1), goal, tgt)
    d = tgt - xy
    dn = d.norm(dim=-1)
    rem = torch.maximum(rem_path, (goal - xy).norm(dim=-1))
    v = torch.minimum(torch.minimum(vmax, torch.sqrt((2 * 0.8 * amax * rem).clamp(min=0))), torch.clamp(2.0 * rem, min=0.02))
    v = v * (dn / 0.01).clamp(max=1.0)
    vw = d / dn.clamp(min=1e-6).unsqueeze(-1) * v.unsqueeze(-1)
    wz = (yaw_gain * wrap(path.goal_yaw - pose[:, 2]))
    wz = torch.maximum(torch.minimum(wz, wmax), -wmax)
    vb = to_body((xy + vw).unsqueeze(1), pose).squeeze(1)
    return torch.cat([vb, wz.unsqueeze(-1)], -1)
