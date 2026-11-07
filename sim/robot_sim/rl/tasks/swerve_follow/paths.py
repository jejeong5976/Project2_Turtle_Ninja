"""경로 묶음 (torch, env 마다 경로 하나): 호 길이 · 창 안 최근접 투영 · 호 길이 위치 보간.

경로 = 점 256 개 (0.05 m 간격, 남는 칸은 마지막 점 반복 → 길이 0 선분). 좌표는 env 마다 자기 world.
"""
import math

import torch

STEP = 0.05


class PathBatch:
    def __init__(self, num_envs: int, max_pts: int, device):
        self.B, self.M, self.device = num_envs, max_pts, device
        self.pts = torch.zeros(num_envs, max_pts, 2, device=device)
        self.n = torch.full((num_envs,), 2, dtype=torch.long, device=device)
        self.s = torch.zeros(num_envs, max_pts, device=device)        # 각 점까지 누적 호 길이
        self.length = torch.zeros(num_envs, device=device)
        self.goal_yaw = torch.zeros(num_envs, device=device)

    def set(self, ids, pts, n, goal_yaw):
        self.pts[ids] = pts
        self.n[ids] = n
        seg = (pts[:, 1:] - pts[:, :-1]).norm(dim=-1)
        self.s[ids] = torch.cat([torch.zeros_like(seg[:, :1]), seg.cumsum(-1)], -1)
        self.length[ids] = self.s[ids, -1]
        self.goal_yaw[ids] = goal_yaw

    @property
    def goal(self):
        return self.pts[torch.arange(self.B, device=self.device), self.n - 1]

    def point_at(self, s):
        """s (B,K) 호 길이 → 위치 (B,K,2), 단위 접선 (B,K,2). 경로 밖은 끝점으로 자름"""
        s = torch.minimum(torch.maximum(s, torch.zeros_like(s)), self.length.unsqueeze(-1))
        i = torch.searchsorted(self.s, s.contiguous(), right=True) - 1
        i = torch.minimum(i.clamp(min=0), (self.n - 2).clamp(min=0).unsqueeze(-1))
        p0 = self.pts.gather(1, i.unsqueeze(-1).expand(-1, -1, 2))
        p1 = self.pts.gather(1, (i + 1).unsqueeze(-1).expand(-1, -1, 2))
        s0 = self.s.gather(1, i)
        L = (p1 - p0).norm(dim=-1)
        t = ((s - s0) / L.clamp(min=1e-9)).clamp(0, 1)
        tan = (p1 - p0) / L.clamp(min=1e-9).unsqueeze(-1)
        return p0 + (p1 - p0) * t.unsqueeze(-1), tan

    def project(self, xy, s_prev, back: float, fwd: float):
        """창 [s_prev − back, s_prev + fwd] 안 선분들에 투영 → (s, 부호 있는 횡오차 (왼쪽 +), 경로까지 거리, 접선 (B,2)).
        횡오차는 접선 수직 성분이라 경로 끝 너머로 벗어나면 0 → 이탈 판정은 거리로 할 것"""
        nb, nf = int(math.ceil(back / STEP)) + 1, int(math.ceil(fwd / STEP)) + 1
        i0 = torch.searchsorted(self.s, s_prev.unsqueeze(-1).contiguous(), right=True).squeeze(-1) - 1
        idx = i0.unsqueeze(-1) + torch.arange(-nb, nf + 1, device=self.device)
        idx = torch.minimum(idx.clamp(min=0), (self.n - 2).clamp(min=0).unsqueeze(-1))       # (B,W) 선분 번호
        p0 = self.pts.gather(1, idx.unsqueeze(-1).expand(-1, -1, 2))
        p1 = self.pts.gather(1, (idx + 1).unsqueeze(-1).expand(-1, -1, 2))
        d = p1 - p0
        L2 = (d * d).sum(-1)
        rel = xy.unsqueeze(1) - p0
        t = ((rel * d).sum(-1) / L2.clamp(min=1e-12)).clamp(0, 1)
        s0 = self.s.gather(1, idx)
        s = s0 + t * L2.sqrt()
        lo, hi = (s_prev - back).unsqueeze(-1), (s_prev + fwd).unsqueeze(-1)
        s = torch.minimum(torch.maximum(s, lo), hi)
        # 창으로 자른 s 의 위치에서 거리 다시 (창 경계 밖 투영이 이기지 않게)
        t = ((s - s0) / L2.sqrt().clamp(min=1e-9)).clamp(0, 1)
        foot = p0 + d * t.unsqueeze(-1)
        dist = (xy.unsqueeze(1) - foot).norm(dim=-1)                   # 선분 번호는 n−2 까지라 패딩(길이 0) 선분은 안 들어옴
        k = dist.argmin(1, keepdim=True)
        s_best = s.gather(1, k).squeeze(1)
        foot_b = foot.gather(1, k.unsqueeze(-1).expand(-1, -1, 2)).squeeze(1)
        db = d.gather(1, k.unsqueeze(-1).expand(-1, -1, 2)).squeeze(1)
        tan = db / db.norm(dim=-1, keepdim=True).clamp(min=1e-9)
        r = xy - foot_b
        lat = tan[:, 0] * r[:, 1] - tan[:, 1] * r[:, 0]
        return s_best.clamp(min=0.0), lat, r.norm(dim=-1), tan


def to_body(xy, pose):
    """world 점 (B,K,2) → pose (B,3) 기준 body 좌표"""
    c, s = torch.cos(pose[:, 2]).unsqueeze(-1), torch.sin(pose[:, 2]).unsqueeze(-1)
    d = xy - pose[:, None, :2]
    return torch.stack([c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1]], -1)


def wrap(a):
    return torch.remainder(a + math.pi, 2 * math.pi) - math.pi
