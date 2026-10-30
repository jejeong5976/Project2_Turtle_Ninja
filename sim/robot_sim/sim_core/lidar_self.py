"""라이다 자기 몸 가림: 배포판 URDF 메시를 라이다 스캔 높이에서 자른 단면 (선분 모음) + 레이 교차.

레이캐스트 라이다(turtleninja_maze_env.add_lidars)는 미로 · 소품만 맞히고 로봇 몸은 뚫고 지나감.
실물 LDS-03 은 바퀴 · 조향 모듈 · 반대쪽 라이다 · 차체 기둥에 가려짐 -> 이 모듈로 가려진 빔을 계산해 '돌아오지 않음(inf)' 처리.
  - 단면: 링크마다 visual 메시를 스캔 높이 평면으로 자른 선분 (링크 좌표). 조향 · 바퀴 링크는 조향 각도를 따라 돎
  - 바퀴 회전(구동)은 무시 (바퀴는 원통이라 단면이 거의 같음)
  - 자기 라이다 머리(센서가 그 안에 있음)는 뺌
실물도 같은 가림이 생기므로, 실물 스캔에서는 몸에 맞은 점을 걸러야 함 (laser_filters 상자 필터 등, 현재 미설치).
"""
from __future__ import annotations

import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(e):
    o, T = e.find('origin'), np.eye(4)
    if o is not None:
        T[:3, :3] = _rpy(*[float(v) for v in (o.get('rpy') or '0 0 0').split()])
        T[:3, 3] = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
    return T


def _load_tris(path: Path) -> np.ndarray:
    """메시 삼각형 (n, 3, 3). OBJ (v/f) · STL (바이너리 · 아스키)."""
    if path.suffix.lower() == '.obj':
        V, F = [], []
        for ln in path.read_text(errors='ignore').splitlines():
            t = ln.split()
            if not t:
                continue
            if t[0] == 'v':
                V.append([float(x) for x in t[1:4]])
            elif t[0] == 'f':
                ids = [int(x.split('/')[0]) for x in t[1:]]
                ids = [i - 1 if i > 0 else len(V) + i for i in ids]
                F += [[ids[0], ids[j], ids[j + 1]] for j in range(1, len(ids) - 1)]
        return np.asarray(V, float)[np.asarray(F, int)]
    b = path.read_bytes()
    if b[:5] == b'solid' and b'facet' in b[:300]:
        v = [[float(x) for x in ln.split()[1:4]] for ln in b.decode(errors='ignore').splitlines() if ln.strip().startswith('vertex')]
        return np.asarray(v, float).reshape(-1, 3, 3)
    n = struct.unpack('<I', b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    return np.frombuffer(a[:, 12:48].tobytes(), dtype='<f4').reshape(-1, 3, 3).astype(float)


def _slice(tris: np.ndarray, z: float) -> np.ndarray:
    """삼각형 (n,3,3) 을 평면 z 로 자른 선분 (m, 2, 2) xy."""
    d = tris[:, :, 2] - z
    s = np.sign(d)
    cross = ~((s > 0).all(1) | (s < 0).all(1))
    out = []
    for t, dd in zip(tris[cross], d[cross]):
        pts = []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            if (dd[i] > 0) != (dd[j] > 0) and dd[i] != dd[j]:
                a = dd[i] / (dd[i] - dd[j])
                pts.append(t[i, :2] + a * (t[j, :2] - t[i, :2]))
        if len(pts) == 2:
            out.append(pts)
    return np.asarray(out, float).reshape(-1, 2, 2)


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _hull_edges(seg: np.ndarray) -> np.ndarray:
    """선분 끝점들의 볼록 껍질 테두리 (k, 2, 2). 조향 · 바퀴 단면을 단순화 (계산량 줄임, 오목한 곳은 채워짐)."""
    P = np.unique(seg.reshape(-1, 2).round(5), axis=0)
    P = P[np.lexsort((P[:, 1], P[:, 0]))]
    if len(P) < 3:
        return seg

    def half(pts):
        h = []
        for p in pts:
            while len(h) >= 2 and _cross2(h[-1] - h[-2], p - h[-2]) <= 0:   # numpy 2 는 2차원 np.cross 를 막음
                h.pop()
            h.append(p)
        return h
    H = np.array(half(P)[:-1] + half(P[::-1])[:-1])
    return np.stack([H, np.roll(H, -1, axis=0)], 1)


class SelfOcclusion:
    """base_link 기준 라이다 가림 계산. steer_joint_names 순서의 조향 각도를 받아 매번 몸 단면을 움직임."""

    def __init__(self, urdf: Path, scan_link: str = 'base_scan_front'):
        root = ET.parse(urdf).getroot()
        self.dir = Path(urdf).parent
        self.joints = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.scan_T = self._fk(scan_link)
        self.z = self.scan_T[2, 3]                                   # 스캔 높이 (base_link)
        self.parts = []   # (링크, 움직이는 조향 관절 이름 또는 None, 선분 (m,2,2) 그 관절 좌표 xy)
        for link in root.findall('link'):
            name = link.get('name')
            vis = [v for v in link.findall('visual') if v.find('geometry/mesh') is not None]
            if not vis:
                continue
            steer = self._steer_joint(name)
            for v in vis:
                m = v.find('geometry/mesh')
                scale = np.array([float(x) for x in (m.get('scale') or '1 1 1').split()])
                tris = _load_tris(self.dir / m.get('filename')) * scale
                M = self._fk(name) @ _origin(v)                     # 조향 0 일 때 base_link 기준
                W = tris @ M[:3, :3].T + M[:3, 3]
                if not (W[:, :, 2].min() <= self.z <= W[:, :, 2].max()):
                    continue
                if steer is not None:                                # 조향 관절 좌표로 (그 관절 축 = z)
                    P = self._fk(self.joints_child(steer))
                    Wl = (W - P[:3, 3]) @ P[:3, :3]
                    seg = _slice(Wl, self.z - P[2, 3])
                else:
                    seg = _slice(W, self.z)
                if len(seg):
                    self.parts.append((name, steer, seg))
        self.steer_origin = {s: self._fk(self.joints_child(s)) for _, s, _ in self.parts if s is not None}
        # 움직이는 부분(조향 · 바퀴)은 볼록 껍질로 단순화
        self.parts = [(n, st, _hull_edges(sg) if st is not None else sg) for n, st, sg in self.parts]

    def joints_child(self, joint_name):
        for child, j in self.joints.items():
            if j.get('name') == joint_name:
                return child
        raise KeyError(joint_name)

    def _fk(self, link):
        M = np.eye(4)
        while link in self.joints:
            j = self.joints[link]
            M = _origin(j) @ M
            link = j.find('parent').get('link')
        return M

    def _steer_joint(self, link):
        while link in self.joints:
            j = self.joints[link]
            if j.get('type') in ('revolute', 'continuous') and j.get('name').startswith('steer_'):
                return j.get('name')
            link = j.find('parent').get('link')
        return None

    def segments(self, steer_q: dict, skip_links=(), moving=None) -> np.ndarray:
        """조향 각도 {관절 이름: rad} 일 때 base_link xy 선분 (n, 2, 2). moving: None = 전부, True = 조향 · 바퀴만, False = 고정 부분만."""
        out = []
        for name, steer, seg in self.parts:
            if name in skip_links or (moving is True and steer is None) or (moving is False and steer is not None):
                continue
            if steer is None:
                out.append(seg)
                continue
            P = self.steer_origin[steer]
            q = steer_q.get(steer, 0.0)
            c, s = math.cos(q), math.sin(q)
            R = P[:2, :2] @ np.array([[c, -s], [s, c]])
            out.append(seg @ R.T + P[:2, 3])
        return np.concatenate(out) if out else np.zeros((0, 2, 2))

    @staticmethod
    def ray_hits(origin_xy, angles, segs) -> np.ndarray:
        """origin 에서 angles (base_link 기준 방향) 로 쏜 레이가 선분에 처음 맞는 거리 (없으면 inf)."""
        d = np.stack([np.cos(angles), np.sin(angles)], 1)[:, None, :]      # (R,1,2)
        a, b = segs[None, :, 0, :] - origin_xy, segs[None, :, 1, :] - origin_xy   # (1,S,2)
        e = b - a
        den = d[..., 0] * e[..., 1] - d[..., 1] * e[..., 0]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (a[..., 0] * e[..., 1] - a[..., 1] * e[..., 0]) / den
            u = (a[..., 0] * d[..., 1] - a[..., 1] * d[..., 0]) / den
        ok = (np.abs(den) > 1e-12) & (t > 1e-4) & (u >= 0) & (u <= 1)
        t = np.where(ok, t, np.inf)
        return t.min(1) if t.shape[1] else np.full(len(angles), np.inf)
