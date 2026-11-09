"""지역 계획기 학습용 세계 (torch, env B 개): 점유 격자 거리장 + 사람 원기둥 → 외곽 충돌 · 여유 거리 · 라이다.

지도 = rl/data/maps/maze_<이름>_map.pgm/yaml (tools/layout_to_occupancy.py, 스캔 높이 0.06 m, 0.025 m 칸). env 마다 지도 번호 하나.
거리장 d(p) = 장애물 칸 중심까지 거리 − 반 칸 (장애물 칸 안 = −반 칸). 지도 밖 = 장애물.
사람 = 원기둥 (중심 · 반경), d = |p − c| − r. 세계 거리 = 지도 · 사람 중 작은 값.
외곽 = ±half 사각형 둘레 점 (변마다 per_side 개) → 여유 = 둘레 점 거리 최솟값, 충돌 = 여유 < 0.
라이다 = 로봇 중심에서 몸 기준 등간격 방향 구 추적 (sphere tracing, 최소 걸음 1 cm) → 거리 (최대 max_range).
라이다 'real' (lidar_real) = 실물 · 배포와 같은 처리: 앞 · 뒤 라이다 원점에서 빔 → 자기 몸 가림 (가림표, make_lidar_occlusion.py) →
  점 → scan_merger 몸 상자 안 점 버림 → 몸 중심 칸마다 가장 가까운 점 (빈 칸 = max_range).
"""
import math
from pathlib import Path

import numpy as np
import torch
from scipy import ndimage


def load_pgm_map(maps_dir: Path, name: str):
    stem = 'maze_real_map' if name == 'real' else f'maze_{name}_map'
    raw = (maps_dir / f'{stem}.pgm').read_bytes()
    parts = raw.split(b'\n', 4)                     # P5 / 주석 / W H / 255 / 데이터 (tools/layout_to_occupancy.py 형식)
    W, H = map(int, parts[2].split())
    img = np.frombuffer(parts[4], np.uint8)[:W * H].reshape(H, W)[::-1]     # 행 0 = 아래 (y 최소)
    meta = {k.strip(): v.strip() for k, v in (l.split(':', 1) for l in (maps_dir / f'{stem}.yaml').read_text().splitlines()
                                             if ':' in l and not l.startswith('#'))}
    origin = [float(x) for x in meta['origin'].strip('[]').split(',')][:2]
    return img == 0, float(meta['resolution']), np.array(origin)


class LidarOcclusion:
    """make_lidar_occlusion.py 가림표 (torch). stride = 빔 솎기 (2 = 2° 간격)"""

    def __init__(self, path, device, stride: int = 1):
        z = np.load(path)
        f = lambda k: torch.tensor(z[k], dtype=torch.float32, device=device)  # noqa: E731
        self.origin = f('origin')                                      # (L,2) 몸 좌표
        self.beam = f('beam_angle')[:, ::stride].contiguous()          # (L,N)
        self.static = f('occ_static')[:, ::stride].contiguous()        # (L,N)
        self.module = f('occ_module')[..., ::stride].contiguous()      # (L,4,Q,N)
        self.body_half = [float(x) for x in z['body_half']]
        self.steer_names = [str(x) for x in z['steer_names']]


class World:
    def __init__(self, names, maps_dir, num_envs: int, device, max_people: int = 2):
        self.device, self.B = device, num_envs
        occs, origins, res = [], [], None
        for n in names:
            occ, r, o = load_pgm_map(Path(maps_dir), n)
            assert res is None or abs(r - res) < 1e-9, '지도 해상도가 다름'
            res = r
            occs.append(occ)
            origins.append(o)
        self.res = res
        Hm, Wm = max(o.shape[0] for o in occs), max(o.shape[1] for o in occs)
        dist = np.zeros((len(occs), Hm, Wm), np.float32)
        self._pads = []
        for i, occ in enumerate(occs):
            pad = np.ones((Hm, Wm), bool)                 # 지도 밖 = 장애물
            pad[:occ.shape[0], :occ.shape[1]] = occ
            self._pads.append(pad)
            dist[i] = ndimage.distance_transform_edt(~pad) * res - res / 2
        self.dfield = torch.tensor(dist, device=device)   # (M, H, W)
        self.origin = torch.tensor(np.array(origins), dtype=torch.float32, device=device)
        self.H, self.W = Hm, Wm
        self.map_id = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.people_c = torch.zeros(num_envs, max_people, 2, device=device)
        self.people_r = torch.zeros(num_envs, max_people, device=device)
        self.people_on = torch.zeros(num_envs, max_people, dtype=torch.bool, device=device)
        self.boxes = False
        self.mboxes = False

    # ------------------------------------------------------------------ 지도 ↔ 실제 어긋남: 여러 박스 (nf7)
    def enable_multi_boxes(self, map_boxes: dict, half: float, erase_margin: float):
        """지도마다 박스 목록 {지도 번호: [(cx, cy, 회전), …]} 을 래스터에서 지우고 해석식 (회전 사각형 sdf) 으로 바꿈.
        env 마다 mb_pose (B,K,3) · mb_on (B,K) — 충돌 · 여유 · 라이다는 이 박스를 봄. 지도 · 경로 (map_dist · 경로 은행) 는 래스터 그대로
        → 박스를 옮기면 '지도와 실제 배치가 다름' (실물 arena5: 도면 대비 8–19 cm, 시연 중 밀림)
        박스 없는 지도는 래스터 그대로 (mb_on 전부 False)"""
        M = len(self._pads)
        K = max([len(v) for v in map_boxes.values()] + [1])
        nom = np.zeros((M, K, 3), np.float32)
        valid = np.zeros((M, K), bool)
        nobox = self.dfield.cpu().numpy().copy()
        ys, xs = np.mgrid[0:self.H, 0:self.W]
        for m, boxes in map_boxes.items():
            o = self.origin[m].cpu().numpy()
            wx, wy = o[0] + (xs + 0.5) * self.res, o[1] + (ys + 0.5) * self.res
            pad = self._pads[m].copy()
            for j, (cx, cy, rot) in enumerate(boxes):
                nom[m, j], valid[m, j] = (cx, cy, rot), True
                c, s = math.cos(rot), math.sin(rot)
                lx, ly = c * (wx - cx) + s * (wy - cy), -s * (wx - cx) + c * (wy - cy)
                pad &= ~((np.abs(lx) <= half + erase_margin) & (np.abs(ly) <= half + erase_margin))
            nobox[m] = ndimage.distance_transform_edt(~pad) * self.res - self.res / 2
        self.dfield_nobox = torch.tensor(nobox, device=self.device)
        self.mb_nom = torch.tensor(nom, device=self.device)
        self.mb_valid = torch.tensor(valid, device=self.device)
        self.mb_half = float(half)
        self.mb_pose = torch.zeros(self.B, K, 3, device=self.device)
        self.mb_on = torch.zeros(self.B, K, dtype=torch.bool, device=self.device)
        self.mboxes = True

    def ray_boxes(self, o, dirs, max_range: float):
        """광선 o (B,R,2) · 방향 dirs (B,R,2) ↔ 켜진 해석식 박스 (env 전부) 교차 거리 (B,R), 안 맞으면 max_range (슬랩 방법, 한 번 계산)"""
        pz, on = self.mb_pose, self.mb_on
        c, s = pz[..., 2].cos()[:, None], pz[..., 2].sin()[:, None]                         # (B,1,K)
        rel = o.unsqueeze(2) - pz[:, None, :, :2]                                            # (B,R,K,2)
        ox, oy = c * rel[..., 0] + s * rel[..., 1], -s * rel[..., 0] + c * rel[..., 1]
        dx = c * dirs[..., None, 0] + s * dirs[..., None, 1]
        dy = -s * dirs[..., None, 0] + c * dirs[..., None, 1]
        h, eps = self.mb_half, 1e-9
        ix = 1.0 / torch.where(dx.abs() < eps, torch.full_like(dx, eps), dx)
        iy = 1.0 / torch.where(dy.abs() < eps, torch.full_like(dy, eps), dy)
        tx1, tx2 = (-h - ox) * ix, (h - ox) * ix
        ty1, ty2 = (-h - oy) * iy, (h - oy) * iy
        tmin = torch.maximum(torch.minimum(tx1, tx2), torch.minimum(ty1, ty2))
        tmax = torch.minimum(torch.maximum(tx1, tx2), torch.maximum(ty1, ty2))
        hit = (tmax >= tmin.clamp(min=0.0)) & on[:, None, :]
        t = torch.where(hit, tmin.clamp(min=0.0), torch.full_like(tmin, max_range))
        return t.min(-1).values.clamp(max=max_range)

    def multi_box_sdf(self, xy, poses, on):
        """xy (n,P,2) · poses (n,K,3) · on (n,K) → 켜진 박스 중 가장 가까운 회전 사각형 부호 거리 (n,P) (없으면 1e3)"""
        rel = xy.unsqueeze(2) - poses[:, None, :, :2]                     # (n,P,K,2)
        c, s = poses[:, None, :, 2].cos(), poses[:, None, :, 2].sin()
        lx, ly = c * rel[..., 0] + s * rel[..., 1], -s * rel[..., 0] + c * rel[..., 1]
        q = torch.stack([lx.abs(), ly.abs()], -1) - self.mb_half
        d = q.clamp(min=0).norm(dim=-1) + q.max(-1).values.clamp(max=0)
        d = torch.where(on.unsqueeze(1), d, torch.full_like(d, 1e3))
        return d.min(-1).values

    # ------------------------------------------------------------------ 밀리는 박스 (local_dock)
    def enable_boxes(self, map_index: int, centers, half: float):
        """지도 map_index 의 박스 K 개 (중심 (K,2), 반 폭 half) 를 에피소드마다 옮길 수 있게.
        박스마다 그 박스 칸을 지운 거리장 dfield_minus[k] 을 만들어 두고, 옮긴 박스는 해석식 (회전 사각형 sdf) 으로 더함.
        env 마다 box_k (−1 = 안 옮김, 지도 그대로) · box_pose (중심 x, y, 회전) — 충돌 · 여유 · 라이다가 옮긴 박스를 봄 (지도 · 경로는 그대로)"""
        c = np.asarray(centers, float)
        pad = self._pads[map_index]
        o = self.origin[map_index].cpu().numpy()
        ys, xs = np.mgrid[0:self.H, 0:self.W]
        wx, wy = o[0] + (xs + 0.5) * self.res, o[1] + (ys + 0.5) * self.res
        minus = np.zeros((len(c), self.H, self.W), np.float32)
        for k, (cx, cy) in enumerate(c):
            pk = pad & ~((np.abs(wx - cx) <= half + self.res) & (np.abs(wy - cy) <= half + self.res))
            minus[k] = ndimage.distance_transform_edt(~pk) * self.res - self.res / 2
        self.dfield_minus = torch.tensor(minus, device=self.device)
        self.box_c = torch.tensor(c, dtype=torch.float32, device=self.device)
        self.box_half = float(half)
        self.box_map = int(map_index)
        self.box_k = torch.full((self.B,), -1, dtype=torch.long, device=self.device)
        self.box_pose = torch.zeros(self.B, 3, device=self.device)
        self.boxes = True

    def _lookup(self, field, fidx, xy, origin):
        """field (N,H,W) 의 fidx (n,) 번째 장에서 xy (n,K,2) 거리 (지도 밖 = −반 칸)"""
        ij = torch.floor((xy - origin.unsqueeze(1)) / self.res).long()
        c, r = ij[..., 0], ij[..., 1]
        inside = (c >= 0) & (c < self.W) & (r >= 0) & (r < self.H)
        flat = fidx.unsqueeze(1) * (self.H * self.W) + r.clamp(0, self.H - 1) * self.W + c.clamp(0, self.W - 1)
        d = field.reshape(-1)[flat]
        return torch.where(inside, d, torch.full_like(d, -self.res / 2))

    def box_sdf(self, xy, pose):
        """회전 사각형 (반 폭 box_half) 까지 부호 거리. xy (n,K,2), pose (n,3)"""
        rel = xy - pose[:, None, :2]
        c, s = pose[:, 2].cos().unsqueeze(-1), pose[:, 2].sin().unsqueeze(-1)
        lx, ly = c * rel[..., 0] + s * rel[..., 1], -s * rel[..., 0] + c * rel[..., 1]
        q = torch.stack([lx.abs(), ly.abs()], -1) - self.box_half
        return q.clamp(min=0).norm(dim=-1) + q.max(-1).values.clamp(max=0)

    # ------------------------------------------------------------------ 거리
    def map_dist(self, xy, map_id=None):
        """xy (B,K,2) 지도 좌표 → 지도 거리 (B,K). map_id 기본 = env 지도"""
        mid = self.map_id if map_id is None else map_id
        o = self.origin[mid].unsqueeze(1)
        ij = torch.floor((xy - o) / self.res).long()
        c, r = ij[..., 0], ij[..., 1]
        inside = (c >= 0) & (c < self.W) & (r >= 0) & (r < self.H)
        flat = mid.unsqueeze(1) * (self.H * self.W) + r.clamp(0, self.H - 1) * self.W + c.clamp(0, self.W - 1)
        d = self.dfield.view(-1)[flat]
        return torch.where(inside, d, torch.full_like(d, -self.res / 2))

    def dist(self, xy, map_id=None, people: bool = True, ids=None, mboxes: bool = True):
        """xy (n,K,2) → 거리. map_id 주면 그 지도 (정적, 사람 · 옮긴 박스 없음), 아니면 env 상태 (ids = env 일부, 기본 전부).
        mboxes=False = 해석식 박스 빼고 (라이다 구 추적용, 박스는 ray_boxes 로 따로)"""
        if map_id is not None:
            return self.map_dist(xy, map_id)
        mid = self.map_id if ids is None else self.map_id[ids]
        if self.mboxes:                                # 박스 지운 래스터 + 해석식 박스 (옮긴 자리), 박스 없는 지도는 래스터 그대로
            d = self._lookup(self.dfield_nobox, mid, xy, self.origin[mid])
            if mboxes:
                on = self.mb_on if ids is None else self.mb_on[ids]
                pz = self.mb_pose if ids is None else self.mb_pose[ids]
                d = torch.minimum(d, self.multi_box_sdf(xy, pz, on))
        else:
            d = self.map_dist(xy, mid)
        if self.boxes:                                # 옮긴 박스: 그 박스 칸 지운 거리장 · 옮긴 자리 해석식 중 작은 값
            bk = self.box_k if ids is None else self.box_k[ids]
            bp = self.box_pose if ids is None else self.box_pose[ids]
            dm = self._lookup(self.dfield_minus, bk.clamp(min=0), xy, self.origin[mid])
            moved = torch.minimum(dm, self.box_sdf(xy, bp))
            d = torch.where((bk >= 0).unsqueeze(-1), moved, d)
        if people:
            pc = self.people_c if ids is None else self.people_c[ids]
            pr = self.people_r if ids is None else self.people_r[ids]
            pon = self.people_on if ids is None else self.people_on[ids]
            dp = (xy.unsqueeze(2) - pc.unsqueeze(1)).norm(dim=-1) - pr.unsqueeze(1)    # (n,K,P)
            dp = torch.where(pon.unsqueeze(1), dp, torch.full_like(dp, 1e3))
            d = torch.minimum(d, dp.min(-1).values)
        return d

    # ------------------------------------------------------------------ 외곽
    @staticmethod
    def footprint_points(half: float, per_side: int, device):
        t = torch.linspace(-half, half, per_side + 1, device=device)[:-1]
        h = torch.full_like(t, half)
        return torch.cat([torch.stack([t, -h], -1), torch.stack([h, t], -1), torch.stack([-t, h], -1), torch.stack([-h, -t], -1)], 0)

    def clearance(self, pose, pts_body, map_id=None, people: bool = True, ids=None):
        """pose (n,3) · 외곽 둘레 점 (P,2) → 여유 (n,) [m] (음수 = 충돌). ids = env 일부 (map_id 없을 때)"""
        c, s = torch.cos(pose[:, 2]).unsqueeze(-1), torch.sin(pose[:, 2]).unsqueeze(-1)
        x = pose[:, None, 0] + c * pts_body[:, 0] - s * pts_body[:, 1]
        y = pose[:, None, 1] + s * pts_body[:, 0] + c * pts_body[:, 1]
        return self.dist(torch.stack([x, y], -1), map_id, people, ids).min(-1).values

    # ------------------------------------------------------------------ 라이다
    def trace(self, o, ang, max_range: float, iters: int = 40, min_step: float = 0.01):
        """원점 o (B,R,2) · 방향 ang (B,R) world → 맞는 거리 (B,R), 안 맞으면 max_range"""
        dirs = torch.stack([ang.cos(), ang.sin()], -1)
        t = torch.zeros_like(ang)
        for _ in range(iters):
            d = self.dist(o + dirs * t.unsqueeze(-1), mboxes=False)
            t = torch.where(d > 0.005, t + d.clamp(min=min_step), t).clamp(max=max_range)
        if self.mboxes:                                # 해석식 박스는 광선 교차로 (구 추적 40 번마다 sdf 를 부르면 스텝 6 배 느림)
            t = torch.minimum(t, self.ray_boxes(o, dirs, max_range))
        return t

    def lidar_real(self, pose, q, occ, n_bins: int, max_range: float, noise, drop):
        """pose (B,3) 참 자세 · q (B,4) 조향각 · occ = LidarOcclusion → 몸 중심 n_bins 칸 거리 (B, n_bins).
        noise (B,2N) 빔 거리 잡음 [m] · drop (B,2N) bool 빠진 빔"""
        B, L, N = pose.shape[0], occ.origin.shape[0], occ.beam.shape[1]
        c, s = pose[:, 2].cos(), pose[:, 2].sin()
        ox = pose[:, None, 0] + c[:, None] * occ.origin[None, :, 0] - s[:, None] * occ.origin[None, :, 1]      # (B,L)
        oy = pose[:, None, 1] + s[:, None] * occ.origin[None, :, 0] + c[:, None] * occ.origin[None, :, 1]
        o = torch.stack([ox, oy], -1).unsqueeze(2).expand(B, L, N, 2).reshape(B, L * N, 2)
        ang = (pose[:, 2:3] + occ.beam.reshape(1, -1)).expand(B, L * N)
        t = self.trace(o, ang, max_range).view(B, L, N) + noise.view(B, L, N)
        # 자기 몸 가림: 고정 부분 · 모듈 4 개 (조향각 칸) 중 가장 가까운 것보다 멀면 안 보임
        Q = occ.module.shape[2]
        qi = (torch.remainder(q + math.pi, 2 * math.pi) / (2 * math.pi) * Q).long().clamp(0, Q - 1)            # (B,4)
        om = occ.module[:, torch.arange(4, device=self.device)[None, :], qi]                               # (L,B,4,N)
        block = torch.minimum(occ.static[None], om.min(2).values.permute(1, 0, 2))                         # (B,L,N)
        ok = (t < max_range - 1e-3) & (t < block) & ~drop.view(B, L, N)
        px = occ.origin[None, :, 0, None] + t * occ.beam.cos()[None]                                         # 몸 좌표
        py = occ.origin[None, :, 1, None] + t * occ.beam.sin()[None]
        ok &= ~((px.abs() <= occ.body_half[0]) & (py.abs() <= occ.body_half[1]))                          # scan_merger 몸 상자
        r = torch.where(ok, torch.hypot(px, py), torch.full_like(t, max_range)).view(B, -1)
        k = torch.remainder(torch.round(torch.atan2(py, px) / (2 * math.pi / n_bins)).long(), n_bins).view(B, -1)
        out = torch.full((B, n_bins), max_range, device=self.device)
        return out.scatter_reduce(1, k, r, reduce='amin')

    def lidar(self, pose, n_rays: int, max_range: float, iters: int = 40, min_step: float = 0.01):
        """로봇 중심에서 몸 기준 0, 360/n, … 도 방향 거리 (B, n_rays)"""
        B = pose.shape[0]
        ang = pose[:, 2:3] + torch.arange(n_rays, device=self.device) * (2 * math.pi / n_rays)
        dirs = torch.stack([ang.cos(), ang.sin()], -1)                      # (B,R,2)
        t = torch.zeros(B, n_rays, device=self.device)
        o = pose[:, None, :2]
        for _ in range(iters):
            d = self.dist(o + dirs * t.unsqueeze(-1), mboxes=False)
            t = torch.where(d > 0.005, t + d.clamp(min=min_step), t).clamp(max=max_range)
        if self.mboxes:
            t = torch.minimum(t, self.ray_boxes(o.expand(B, n_rays, 2), dirs, max_range))
        return t
