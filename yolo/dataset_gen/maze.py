"""미로 맵(Twin Closed Maze B) 로딩과 배치용 기하 계산.

Gazebo에서 쓰던 맵 STL(m 단위, Z-up, 바닥 Z=0)을 그대로 읽는다.
- 벽 메시 → USD Mesh (generate.py에서 생성)
- 높이맵 → 로봇이 갈 수 있는 영역(벽에서 떨어진 거리), 시야 가림 판정
- 격자 위의 벽 구간 목록 → 선반을 벽에 붙여 배치할 때 사용

Isaac Sim 없이 numpy, cv2만으로 동작한다.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

DEFAULT_STL = (Path.home() / "Desktop/physical_ai/projects/waffle-lyrical-slam/src/waffle_slam_course"
               / "models/twin_closed_maze/meshes/maze.stl")

# 기둥 격자 (model.sdf의 기둥 위치)
GRID_X = -2.202 + 0.367 * np.arange(13)
GRID_Y = -3.152 + 0.394 * np.arange(17)
WALL_HEIGHT = 0.39
RES = 0.01   # 높이맵 해상도 (m/px)


def load_stl(path: Path):
    """binary STL → (points[N,3], face_vertex_indices[M*3]). 1mm 단위로 정점을 합쳐 용량을 줄인다."""
    raw = Path(path).read_bytes()
    n = int.from_bytes(raw[80:84], "little")
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    v = np.frombuffer(raw, dtype=dt, count=n, offset=84)["v"].reshape(-1, 3)
    key = np.round(v * 1000).astype(np.int32)
    uniq, inverse = np.unique(key, axis=0, return_inverse=True)
    return uniq.astype(np.float32) / 1000.0, inverse.reshape(-1).astype(np.int32)


@dataclass
class WallSegment:
    a: tuple          # 끝점 (x, y)
    b: tuple
    axis: str         # "x": x 방향으로 뻗은 벽, "y": y 방향

    @property
    def center(self):
        return ((self.a[0] + self.b[0]) / 2, (self.a[1] + self.b[1]) / 2)

    @property
    def length(self):
        return float(np.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1]))


class MazeMap:
    def __init__(self, points: np.ndarray, faces: np.ndarray):
        self.points, self.faces = points, faces
        lo, hi = points.min(0), points.max(0)
        self.origin = lo[:2] - 0.05
        size = ((hi[:2] - lo[:2]) + 0.1) / RES
        self.height = np.zeros((int(size[1]) + 1, int(size[0]) + 1), np.float32)
        tris = points[faces].reshape(-1, 3, 3)
        top = tris[:, :, 2].max(1)
        for z_level in (0.15,):  # 벽(바닥에서 솟은 구조물)만
            sel = tris[top > z_level]
            for tri, h in zip(sel, top[top > z_level]):
                px = self.to_px(tri[:, :2])
                cv2.fillConvexPoly(self.height, px, float(h))
        self.occupied = (self.height > 0.15).astype(np.uint8)
        inside = self._inside_mask()
        free = ((1 - self.occupied) * inside).astype(np.uint8)
        self.clearance = cv2.distanceTransform(free, cv2.DIST_L2, 5) * RES   # 가장 가까운 벽까지 거리 (m)
        self.segments = self._wall_segments()

    # -- 좌표 변환 ---------------------------------------------------------
    def to_px(self, xy):
        xy = np.asarray(xy, dtype=np.float64)
        return np.round((xy - self.origin) / RES).astype(np.int32)

    def _px_value(self, arr, xy, default=0.0):
        c, r = self.to_px(xy)
        if 0 <= r < arr.shape[0] and 0 <= c < arr.shape[1]:
            return arr[r, c]
        return default

    def clearance_at(self, xy) -> float:
        return float(self._px_value(self.clearance, xy, 0.0))

    def is_wall(self, xy) -> bool:
        return bool(self._px_value(self.occupied, xy, 1))

    def _inside_mask(self):
        """바깥 벽 안쪽만 1 (flood fill로 바깥 영역 제거)."""
        h, w = self.occupied.shape
        fill = self.occupied.copy() * 255
        mask = np.zeros((h + 2, w + 2), np.uint8)
        cv2.floodFill(fill, mask, (0, 0), 128)
        return (fill != 128).astype(np.uint8)

    # -- 벽 구간 -----------------------------------------------------------
    def _wall_segments(self):
        segs = []
        for y in GRID_Y:
            for x0, x1 in zip(GRID_X[:-1], GRID_X[1:]):
                if self.is_wall(((x0 + x1) / 2, y)):
                    segs.append(WallSegment((x0, y), (x1, y), "x"))
        for x in GRID_X:
            for y0, y1 in zip(GRID_Y[:-1], GRID_Y[1:]):
                if self.is_wall((x, (y0 + y1) / 2)):
                    segs.append(WallSegment((x, y0), (x, y1), "y"))
        return segs

    # -- 시야 ---------------------------------------------------------------
    def line_of_sight(self, a, b, eye_z: float, target_z: float) -> bool:
        """a(높이 eye_z)에서 b(높이 target_z)까지 벽에 가리지 않는지 (높이맵 기준)."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(2, int(np.hypot(*(b - a)) / RES))
        for t in np.linspace(0.0, 1.0, n)[1:-1]:
            p = a + (b - a) * t
            z = eye_z + (target_z - eye_z) * t
            if self._px_value(self.height, p, 0.0) > z:
                return False
        return True


def split_joints(points: np.ndarray, faces: np.ndarray, half: float = 0.035):
    """벽 연결 부품(기둥 자리 6x6cm, 높이 0~0.39m)과 벽 패널을 분리한다.

    Returns:
        panels: (points, faces) 패널 메시
        joints: [((x, y), points, faces), ...] 기둥 자리마다 하나씩
    """
    tri = faces.reshape(-1, 3)
    c = points[tri].mean(1)
    ix = np.abs(c[:, 0, None] - GRID_X[None]).argmin(1)
    iy = np.abs(c[:, 1, None] - GRID_Y[None]).argmin(1)
    near = (np.abs(c[:, 0] - GRID_X[ix]) < half) & (np.abs(c[:, 1] - GRID_Y[iy]) < half)

    def submesh(sel):
        t = tri[sel]
        used, inv = np.unique(t.reshape(-1), return_inverse=True)
        return points[used], inv.astype(np.int32)

    joints = []
    key = ix * 100 + iy
    for k in np.unique(key[near]):
        sel = near & (key == k)
        jp, jf = submesh(sel)
        joints.append(((float(GRID_X[k // 100]), float(GRID_Y[k % 100])), jp, jf))
    return submesh(~near), joints


def load_maze(path: Path = DEFAULT_STL) -> MazeMap:
    pts, faces = load_stl(path)
    return MazeMap(pts, faces)


if __name__ == "__main__":
    # 확인용: 벽 구간과 이동 가능 영역을 그림으로 저장
    import sys
    m = load_maze(Path(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1] else DEFAULT_STL)
    panels, joints = split_joints(m.points, m.faces)
    print(f"points {len(m.points)}, triangles {len(m.faces) // 3}, wall segments {len(m.segments)}, "
          f"joints {len(joints)}, panel triangles {len(panels[1]) // 3}")
    img = cv2.cvtColor((255 - m.occupied * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    img[(m.clearance > 0.15)] = (200, 255, 200)
    for s in m.segments:
        cv2.line(img, tuple(m.to_px(s.a)), tuple(m.to_px(s.b)), (0, 0, 255) if s.axis == "x" else (255, 0, 0), 2)
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("maze_debug.png")
    cv2.imwrite(str(out), img[::-1])
    print(f"saved {out}")
