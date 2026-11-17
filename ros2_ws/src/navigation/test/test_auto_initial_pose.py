"""지도 raycast 합성 스캔으로 초기 위치 보정 계산을 검증함. ROS/장비 사용 안 함."""
import math
from pathlib import Path
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from pose_match_core import (MapModel, MatchConfig, decide_publication, match,
                             pose_changed, prepare_points, scan_points, wrap)


def fixture(origin_yaw=0.0):
    res = .025
    grid = np.zeros((200, 240), np.int16)
    grid[:3, :] = grid[-3:, :] = grid[:, :3] = grid[:, -3:] = 100
    grid[20:75, 60:64] = 100
    grid[120:124, 35:110] = 100
    grid[60:150, 175:179] = 100
    grid[150:155, 120:175] = 100
    grid[35:40, 130:190] = 100
    return MapModel.create(grid, res, (-3.0, -2.5, origin_yaw))


def raycast(model, pose, beams=240):
    angles = np.linspace(-math.pi, math.pi, beams, endpoint=False)
    distances = np.arange(.255, 3.99, model.resolution / 5)
    a = angles[:, None] + pose[2]
    x = pose[0] + distances * np.cos(a)
    y = pose[1] + distances * np.sin(a)
    u, v = model.cells(x, y)
    valid = (u >= 0) & (v >= 0) & (u < model.grid.shape[1]) & (v < model.grid.shape[0])
    hit = np.zeros_like(valid)
    hit[valid] = model.grid[v[valid], u[valid]] > 50
    use = hit.any(axis=1)
    r = distances[hit.argmax(axis=1)][use]
    return np.column_stack((r * np.cos(angles[use]), r * np.sin(angles[use])))


class MatchTests(unittest.TestCase):
    def test_offsets_and_heading(self):
        model = fixture()
        truth = np.array([.15, -.15, .08])
        points = raycast(model, truth)
        elapsed = []
        cases = [(dx, dy, da) for dx, dy in [(0, 0), (.1, 0), (.2, 0), (.3, 0), (.4, 0),
                                               (-.4, 0), (0, -.4), (.25, -.2), (-.3, .2)]
                 for da in [-20, -10, 0, 10, 20]]
        for dx, dy, da in cases:
            with self.subTest(dx=dx, dy=dy, yaw=da):
                current = truth + [dx, dy, math.radians(da)]
                result = match(model, points, current, current)
                elapsed.append(result['elapsed_s'])
                q = result['pose']
                self.assertGreaterEqual(result['score'], .95)
                self.assertLess(np.linalg.norm(np.array(q[:2]) - truth[:2]), .035)
                self.assertLess(abs(wrap(q[2] - truth[2])), math.radians(1.1))
                decision = decide_publication(result, model, points, current)
                self.assertEqual(decision['publish'], bool(dx or dy or da), result)
        print(f'synthetic_match_seconds: cases={len(elapsed)} min={min(elapsed):.3f} max={max(elapsed):.3f} median={np.median(elapsed):.3f}', flush=True)

    def test_off_grid_range_noise_and_missing_beams(self):
        model = fixture()
        truth = np.array([.173, -.137, .137])
        points = raycast(model, truth)
        rng = np.random.default_rng(71)
        points *= (1 + rng.normal(0, .004, len(points)) / np.linalg.norm(points, axis=1))[:, None]
        points = points[rng.random(len(points)) > .1]
        current = truth + [.273, -.194, math.radians(13.7)]
        result = match(model, points, current, current)
        self.assertLess(np.linalg.norm(np.array(result['pose'][:2]) - truth[:2]), .035)
        self.assertLess(abs(wrap(result['pose'][2] - truth[2])), math.radians(1.1))
        self.assertTrue(decide_publication(result, model, points, current)['publish'])

    def test_map_origin_rotation(self):
        yaw = .63
        model = fixture(yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        local = np.array([3.15, 2.35])
        truth = np.r_[np.array([-3., -2.5]) + np.array([[c, -s], [s, c]]) @ local, yaw + .08]
        points = raycast(model, truth)
        current = truth + [.25, -.30, -.25]
        result = match(model, points, current, current)
        self.assertLess(np.linalg.norm(np.array(result['pose'][:2]) - truth[:2]), .035)
        self.assertLess(abs(wrap(result['pose'][2] - truth[2])), .02)
        self.assertTrue(decide_publication(result, model, points, current)['publish'])

    def test_corridor_ambiguity_retries(self):
        grid = np.zeros((600, 240), np.int16)
        grid[:, 79:82] = grid[:, 159:162] = 100
        model = MapModel.create(grid, .025, (-3., -7.5, 0.))
        points = raycast(model, [0., 0., 0.])
        current = [.2, .2, .1]
        result = match(model, points, current, current)
        decision = decide_publication(result, model, points, current)
        self.assertTrue(result['ambiguous'], result)
        self.assertFalse(decision['publish'])
        self.assertTrue(decision['retry'])

    def test_invalid_and_sparse_inputs(self):
        with self.assertRaises(ValueError):
            MapModel.create(np.zeros((20, 20)), .02, (0, 0, 0))
        with self.assertRaises(ValueError):
            MapModel.create(np.ones((2, 2)) * 100, 0., (0, 0, 0))
        cfg = MatchConfig()
        points = scan_points([np.nan, np.inf, 0., -.2, .3, 5.], -.5, .1)
        self.assertEqual(len(points), 1)
        with self.assertRaises(ValueError):
            prepare_points(points, cfg)
        with self.assertRaises(ValueError):
            MatchConfig(coarse_xy=0).validate()

    def test_outside_map_counts_as_miss(self):
        model = fixture()
        points = np.tile([50., 50.], (40, 1))
        self.assertEqual(model.score(points, [.15, -.15, 0], .04), 0.)

    def test_fresh_current_pose_comparison(self):
        model = fixture()
        truth = [.15, -.15, .08]
        points = raycast(model, truth)
        old_pose = [.45, -.15, .08]
        result = match(model, points, old_pose, old_pose)
        # 계산 중 AMCL이 이미 맞췄다면 같은 initialpose를 덮어쓰지 않음.
        decision = decide_publication(result, model, points, truth)
        self.assertFalse(decision['publish'])
        self.assertEqual(decision['reason'], 'already_aligned')
        # 계산 이후 전혀 다른 스캔이면 이전 후보를 발행하지 않음.
        wrong_scan = points + [3., 3.]
        self.assertFalse(decide_publication(result, model, wrong_scan, old_pose)['publish'])

    def test_motion_epoch_geometry(self):
        self.assertFalse(pose_changed([0, 0, 3.14], [.001, .002, -3.14]))
        self.assertTrue(pose_changed([0, 0, 0], [.02, 0, 0]))
        self.assertTrue(pose_changed([0, 0, 0], [0, 0, .04]))


if __name__ == '__main__':
    unittest.main(verbosity=2)
