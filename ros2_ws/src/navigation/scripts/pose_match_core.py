"""정지 라이다/점유 지도로 초기 위치를 찾는 계산. ROS·모터 명령 없음."""
from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.ndimage import distance_transform_edt


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class MatchConfig:
    search_x: float = 0.6
    search_y: float = 0.5
    search_yaw_deg: float = 25.0
    coarse_xy: float = 0.04
    coarse_yaw_deg: float = 2.0
    fine_xy: float = 0.01
    fine_yaw_deg: float = 0.5
    wall_distance: float = 0.04
    min_score: float = 0.85
    min_improvement: float = 0.08
    ambiguity_margin: float = 0.03
    distinct_xy: float = 0.12
    distinct_yaw_deg: float = 8.0
    max_points: int = 360
    min_points: int = 30
    refine_modes: int = 8

    def validate(self):
        positive = (self.search_x, self.search_y, self.search_yaw_deg,
                    self.coarse_xy, self.coarse_yaw_deg, self.fine_xy,
                    self.fine_yaw_deg, self.wall_distance, self.distinct_xy,
                    self.distinct_yaw_deg)
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError("탐색 거리·각도·간격은 유한한 양수여야 함")
        if not all(math.isfinite(v) and 0 <= v <= 1 for v in
                   (self.min_score, self.min_improvement, self.ambiguity_margin)):
            raise ValueError("점수는 0~1이어야 함")
        if self.min_points < 3 or self.max_points < self.min_points or self.refine_modes < 2:
            raise ValueError("점 개수·재탐색 개수 확인 필요")


@dataclass
class MapModel:
    grid: np.ndarray
    distance: np.ndarray
    resolution: float
    origin: tuple

    @classmethod
    def create(cls, grid, resolution, origin):
        grid = np.asarray(grid, dtype=np.int16)
        if grid.ndim != 2 or not grid.size or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError("잘못된 지도 크기/해상도")
        if len(origin) != 3 or not np.isfinite(origin).all():
            raise ValueError("잘못된 지도 원점")
        occupied = grid > 50
        if not occupied.any():
            raise ValueError("지도에 점유 벽 없음")
        distance = distance_transform_edt(~occupied) * resolution
        return cls(grid, distance.astype(np.float32), float(resolution), tuple(origin))

    def cells(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x - ox, y - oy
        c, s = math.cos(yaw), math.sin(yaw)
        return (np.floor((c * dx + s * dy) / self.resolution).astype(np.int64),
                np.floor((-s * dx + c * dy) / self.resolution).astype(np.int64))

    def distances(self, x, y):
        u, v = self.cells(x, y)
        valid = (u >= 0) & (v >= 0) & (u < self.grid.shape[1]) & (v < self.grid.shape[0])
        out = np.full(np.shape(x), np.inf)
        out[valid] = self.distance[v[valid], u[valid]]
        return out

    def free_centers(self, poses):
        u, v = self.cells(poses[:, 0], poses[:, 1])
        valid = (u >= 0) & (v >= 0) & (u < self.grid.shape[1]) & (v < self.grid.shape[0])
        free = np.zeros(len(poses), dtype=bool)
        free[valid] = (self.grid[v[valid], u[valid]] >= 0) & (self.grid[v[valid], u[valid]] <= 50)
        return free

    def score_batch(self, points, poses, wall_distance):
        poses = np.atleast_2d(poses)
        scores, residual = [], []
        # 한 번에 만드는 배열 크기를 제한함. 탐색 전체가 큰 한 배열을 만들지 않음.
        for start in range(0, len(poses), 256):
            q = poses[start:start + 256]
            c, s = np.cos(q[:, 2, None]), np.sin(q[:, 2, None])
            x = q[:, 0, None] + c * points[:, 0] - s * points[:, 1]
            y = q[:, 1, None] + s * points[:, 0] + c * points[:, 1]
            d = self.distances(x, y)
            good = self.free_centers(q)
            scores.extend(np.where(good, np.mean(d <= wall_distance, axis=1), -1.0))
            residual.extend(np.where(good, np.mean(np.minimum(d, 0.25), axis=1), np.inf))
        return np.asarray(scores), np.asarray(residual)

    def score(self, points, pose, wall_distance):
        return float(self.score_batch(points, np.array(pose)[None, :], wall_distance)[0][0])


def prepare_points(points, cfg):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("스캔 점은 Nx2여야 함")
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) > cfg.max_points:
        points = points[np.linspace(0, len(points) - 1, cfg.max_points).astype(int)]
    if len(points) < cfg.min_points:
        raise ValueError("유효한 스캔 점 부족")
    return points


def scan_points(ranges, angle_min, angle_increment, range_min=0.25, range_max=4.0):
    ranges = np.asarray(ranges, dtype=float)
    if not math.isfinite(angle_min) or not math.isfinite(angle_increment) or angle_increment == 0:
        raise ValueError("잘못된 스캔 각도")
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    valid = np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max)
    return np.column_stack((ranges[valid] * np.cos(angles[valid]),
                            ranges[valid] * np.sin(angles[valid])))


def offsets(radius, step):
    n = int(math.floor(radius / step + 1e-9))
    return np.unique(np.r_[np.arange(-n, n + 1) * step, -radius, radius, 0.0])


def mesh_poses(center, xr, yr, ar, xy_step, yaw_step):
    x, y, a = np.meshgrid(offsets(xr, xy_step), offsets(yr, xy_step),
                          offsets(ar, yaw_step), indexing="ij")
    return np.column_stack((x.ravel(), y.ravel(), a.ravel())) + np.asarray(center)


def distinct(a, b, cfg):
    return (np.linalg.norm(a[:2] - b[:2]) >= cfg.distinct_xy or
            abs(wrap(a[2] - b[2])) >= math.radians(cfg.distinct_yaw_deg))


def match(model, points, center, current_pose, cfg=MatchConfig()):
    """위치 후보만 반환함. 발행 판단은 최신 스캔/TF로 다시 계산함."""
    started = time.perf_counter()
    cfg.validate()
    points = prepare_points(points, cfg)
    center = np.asarray(center, dtype=float)
    if center.shape != (3,) or not np.isfinite(center).all() or not np.isfinite(current_pose).all():
        raise ValueError("잘못된 탐색 중심/현재 자세")
    coarse = mesh_poses(center, cfg.search_x, cfg.search_y,
                        math.radians(cfg.search_yaw_deg), cfg.coarse_xy,
                        math.radians(cfg.coarse_yaw_deg))
    scores, residual = model.score_batch(points, coarse, cfg.wall_distance)
    order = np.lexsort((residual, -scores))
    seeds = []
    for i in order:
        if scores[i] < 0:
            break
        if all(distinct(coarse[i], p, cfg) for p in seeds):
            seeds.append(coarse[i])
            if len(seeds) >= cfg.refine_modes:
                break
    if not seeds:
        return {"reason": "no_free_pose", "pose": None, "score": 0.0,
                "ambiguous": False, "elapsed_s": time.perf_counter() - started}
    refined = []
    limits = np.array([cfg.search_x, cfg.search_y, math.radians(cfg.search_yaw_deg)])
    for seed in seeds:
        p = mesh_poses(seed, cfg.coarse_xy, cfg.coarse_xy,
                       math.radians(cfg.coarse_yaw_deg), cfg.fine_xy,
                       math.radians(cfg.fine_yaw_deg))
        refined.append(p[np.all(np.abs(p - center) <= limits + 1e-9, axis=1)])
    fine = np.vstack(refined)
    fs, fr = model.score_batch(points, fine, cfg.wall_distance)
    order = np.lexsort((fr, -fs))
    best_index = order[0]
    best, score = fine[best_index], float(fs[best_index])
    other = next((i for i in order if distinct(best, fine[i], cfg)), None)
    runner = float(fs[other]) if other is not None else -1.0
    ambiguous = runner >= cfg.min_score and score - runner < cfg.ambiguity_margin
    result = {"reason": "ambiguous" if ambiguous else "matched", "pose": best.tolist(),
              "score": score, "current_score": model.score(points, current_pose, cfg.wall_distance),
              "runner_up_score": runner, "runner_up_pose": fine[other].tolist() if other is not None else None,
              "ambiguous": ambiguous, "point_count": len(points),
              "candidate_count": len(coarse) + len(fine),
              "elapsed_s": time.perf_counter() - started}
    return result


def decide_publication(result, model, points, current_pose, cfg=MatchConfig()):
    """현재 시점의 스캔/TF와 비교. 불확실하면 위치를 쓰지 않고 다음 정지 때 재탐색함."""
    points = prepare_points(points, cfg)
    current = model.score(points, current_pose, cfg.wall_distance)
    candidate = result.get("pose")
    score = model.score(points, candidate, cfg.wall_distance) if candidate is not None else 0.0
    if current >= cfg.min_score and score - current < cfg.min_improvement:
        reason, publish, retry = "already_aligned", False, False
    elif candidate is None or score < cfg.min_score:
        reason, publish, retry = "low_score", False, True
    elif result.get("ambiguous", False):
        reason, publish, retry = "ambiguous", False, True
    elif score - current < cfg.min_improvement:
        reason, publish, retry = "small_improvement", False, True
    else:
        reason, publish, retry = "improved", True, False
    return {"reason": reason, "publish": publish, "retry": retry,
            "score": score, "current_score": current, "improvement": score - current}


def pose_changed(before, after, xy=0.01, yaw=0.02):
    return (math.hypot(after[0] - before[0], after[1] - before[1]) > xy or
            abs(wrap(after[2] - before[2])) > yaw)


def run_match_job(grid, resolution, origin, points, center, current_pose, cfg):
    # ROS 실행기와 다른 프로세스에서 실행함. 자식은 ROS 노드를 만들지 않음.
    model = MapModel.create(grid, resolution, origin)
    return match(model, points, center, current_pose, cfg), model
