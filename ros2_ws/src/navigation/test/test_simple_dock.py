"""simple_dock_core.DockCore 시험 — 점 로봇 운동 시뮬 (명령 0.1 s 1 차 지연, 20 Hz) + 박스 · 벽 · 사람 선분을 라이다 점으로.

  python3 -m pytest ros2_ws/src/navigation/test/test_simple_dock.py
보는 것: 박스 면 틈 · 옆 위치 · 방향, 몸 사각형 (±0.16) 이 벽 · 박스 · 사람에 안 닿음, 막히면 멈추고 실패
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from simple_dock_core import DockCore, wrap  # noqa: E402

BOX_HALF, BOX_DIST, HALF, FRONT = 0.1025, 0.2575, 0.16, 0.145   # 몸: 앞 0.145 (box_dist = 반 변 + 0.145 + 1 cm) · 옆 · 뒤 0.16 (footprint)
DT = 0.05


def box_segs(c, yaw):
    """박스 중심 c · 로봇이 볼 방향 yaw → 네 변 (선분)"""
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    k = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * BOX_HALF
    v = k @ R.T + c
    return [(v[i], v[(i + 1) % 4]) for i in range(4)]


def sample(segs, step=0.005):
    out = []
    for a, b in segs:
        n = max(2, int(np.linalg.norm(b - a) / step))
        out.append(a + np.linspace(0, 1, n)[:, None] * (b - a))
    return np.vstack(out)


def visible(P, pose, r_max=1.5):
    """로봇 중심에서 보이는 점 (가림 없음 · 몸 안쪽 빼고) → 몸 좌표"""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    d = P - [x, y]
    B = np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]]
    return B[np.hypot(B[:, 0], B[:, 1]) < r_max]


def body_gap(P, pose):
    """몸 사각형 (앞 FRONT · 옆 · 뒤 HALF) 과 점들 사이 가장 짧은 거리 (안쪽이면 음수)"""
    B = visible(P, pose, 50.0)
    if len(B) == 0:
        return 9.0
    cx = (FRONT - HALF) / 2
    dx, dy = np.abs(B[:, 0] - cx) - (FRONT + HALF) / 2, np.abs(B[:, 1]) - HALF
    out = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0))
    inside = (dx < 0) & (dy < 0)
    out[inside] = np.maximum(dx, dy)[inside]
    return float(out.min())


def box_visible(c, yaw, pose):
    """로봇 쪽을 보는 박스 변만 (가림 대신 뒷면 버리기)"""
    out = []
    for a, b in box_segs(c, yaw):
        m = (a + b) / 2
        n = m - c
        if n @ (np.asarray(pose[:2]) - m) > 0:
            out.append((a, b))
    return sample(out)


def run(start, box_c, box_yaw, map_box_c=None, map_box_yaw=None, walls=(), person=None, t_max=60.0, self_pts=True, **kw):
    """map_box_* = 지도 (목표) 박스, box_* = 실제 박스. person = (등장 시각, 선분)"""
    mc = np.asarray(box_c if map_box_c is None else map_box_c, float)
    my = box_yaw if map_box_yaw is None else map_box_yaw
    goal = mc - BOX_DIST * np.array([math.cos(my), math.sin(my)])
    core = DockCore(goal, my, **kw)
    box_c = np.asarray(box_c, float)
    walls_p = sample([(np.asarray(a, float), np.asarray(b, float)) for a, b in walls]) if walls else np.zeros((0, 2))
    box_all = sample(box_segs(box_c, box_yaw))
    pose = np.array(start, float)
    v = np.zeros(3)
    rng = np.random.default_rng(0)
    selfp = np.array([[0.01, 0.04], [0.08, 0.1], [0.16, -0.11], [-0.07, -0.1], [0.09, 0.14], [-0.08, 0.1]])
    min_gap, t, st, modes = 9.0, 0.0, 'run', set()
    while t < t_max:
        world = np.vstack([box_visible(box_c, box_yaw, pose), walls_p])
        W = world
        if person is not None and t >= person[0]:
            W = np.vstack([world, sample([tuple(np.asarray(p, float) for p in person[1])])])
        raw = visible(W, pose) + rng.normal(0, 0.003, (len(visible(W, pose)), 2))
        if self_pts:
            raw = np.vstack([raw, selfp])
        merged = raw[~((np.abs(raw[:, 0]) < 0.17) & (np.abs(raw[:, 1]) < 0.19))]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        odom_pts = np.c_[c * merged[:, 0] - s * merged[:, 1] + pose[0], s * merged[:, 0] + c * merged[:, 1] + pose[1]]
        core.update_box(odom_pts, pose[:2])
        cmd, st = core.step(tuple(pose), raw, t)
        modes.add(core.mode)
        if st != 'run':
            break
        v += (np.array(cmd) - v) * min(1.0, DT / 0.1)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pose = pose + DT * np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])
        pose[2] = wrap(pose[2])
        min_gap = min(min_gap, body_gap(np.vstack([W, box_all]), pose))
        t += DT
    # 끝난 뒤 남은 속도로 미끄러짐 (지연 0.1 s)
    for _ in range(4):
        v *= 0.5
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pose = pose + DT * np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])
        min_gap = min(min_gap, body_gap(np.vstack([W, box_all]), pose))
    bc = np.asarray(box_c, float)
    fwd = np.array([math.cos(box_yaw), math.sin(box_yaw)])
    rel = bc - pose[:2]
    face_gap = float(rel @ fwd) - BOX_HALF - FRONT               # 박스 면 틈 (목표 1 cm)
    side = float(rel @ np.array([-fwd[1], fwd[0]]))
    return dict(st=st, why=core.why, t=t, pose=pose, gap=min_gap, face_gap=face_gap, side=side,
                dyaw=math.degrees(wrap(box_yaw - pose[2])), modes=modes)


def test_straight_box_ahead():
    r = run((0.0, 0.0, 0.0), (0.55, 0.0), 0.0)
    assert r['st'] == 'done', r['why']
    assert 0.004 < r['face_gap'] < 0.016 and abs(r['side']) < 0.008 and abs(r['dyaw']) < 2.5, r


def test_turn_first_then_approach_with_shifted_box():
    """출발 30° 틀어짐 · 박스가 지도보다 옆 4 cm · 5° 돌아감 → 실제 박스에 맞춰 섬"""
    r = run((0.0, 0.05, math.radians(30)), (0.58, -0.04), math.radians(5), map_box_c=(0.58, 0.0), map_box_yaw=0.0)
    assert r['st'] == 'done', r['why']
    assert '돌기' in r['modes']
    assert 0.004 < r['face_gap'] < 0.016 and abs(r['side']) < 0.01 and abs(r['dyaw']) < 3.0, r


@pytest.mark.parametrize('wall_y', [0.205, 0.19])
def test_wall_beside_box_never_touched(wall_y):
    """박스 왼쪽 옆에 벽 (도착 자리 몸 옆면에서 4.5 · 3 cm). 8° 틀어져 출발 → 뒤 왼쪽 모서리가 벽에서 2.5 · 1 cm (띠 안)"""
    wall = ((-0.6, wall_y), (0.9, wall_y))
    r = run((0.0, 0.0, math.radians(-8)), (0.56, 0.0), 0.0, walls=[wall])
    assert r['gap'] > 0.005, r
    assert r['st'] == 'done', r['why']
    assert 0.004 < r['face_gap'] < 0.016, r


def test_rotation_blocked_by_wall_escapes_then_docks():
    """벽에 2 cm 붙은 채 40° 틀어져 출발 → 모서리가 벽을 칠 회전이라 먼저 떨어진 뒤 돌기"""
    wall = ((-0.8, 0.40), (0.9, 0.40))
    th = math.radians(40)
    y0 = 0.40 - 0.02 - HALF * (math.cos(th) + math.sin(th))      # 돌아간 몸의 위쪽 모서리가 벽에서 2 cm
    r = run((0.0, y0, th), (0.58, 0.0), 0.0, walls=[wall])
    assert r['gap'] > 0.003, r
    assert r['st'] == 'done', r['why']


def test_person_steps_in_front_stops():
    """다가가는 중 사람 다리가 몸 앞 약 15 cm 에 들어옴 (0.6 s, 최고 0.15 m/s) → 멈추고 실패 (부딪히지 않음)"""
    leg = ((0.25, -0.05), (0.25, 0.0))
    r = run((-0.15, 0.0, 0.0), (0.55, 0.0), 0.0, person=(0.6, leg))
    assert r['st'] == 'fail' and '막힘' in r['why'], r
    assert r['gap'] > 0.0, r


def test_box_points_do_not_block_final_approach():
    """박스 면 (몸 앞 1 cm) 은 막는 점이 아님 · 지도 박스가 7 cm 어긋나도"""
    r = run((0.0, 0.0, 0.0), (0.55, 0.07), 0.0, map_box_c=(0.55, 0.0), map_box_yaw=0.0)
    assert r['st'] == 'done', r['why']
    assert abs(r['side']) < 0.015 and r['gap'] > 0.0, r


def test_place_stop_without_box():
    """장소 정차 (앞에 박스 없음, 박스 맞춤 켠 채) → 지도 정차점 그대로"""
    goal_box_c = np.array([0.6, 0.0])
    core_goal = goal_box_c - [BOX_DIST, 0]
    r = run((0.0, 0.1, math.radians(-20)), (5.0, 5.0), 0.0, map_box_c=goal_box_c, map_box_yaw=0.0, self_pts=False)
    assert r['st'] == 'done', r['why']
    assert np.hypot(*(r['pose'][:2] - core_goal)) < 0.02, r


def test_leg_beside_does_not_stop_forward():
    """옆 3 cm 에 다리 (앞으로 가는 길 밖) → 안 멈춤"""
    leg = ((0.05, -0.19), (0.12, -0.19))
    r = run((-0.15, 0.0, 0.0), (0.55, 0.0), 0.0, person=(1.0, leg))
    assert r['st'] == 'done', r['why']
    assert r['gap'] > 0.005, r
