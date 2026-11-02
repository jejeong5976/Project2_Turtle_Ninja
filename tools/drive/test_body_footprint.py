"""body_footprint 시험 — 검토에서 찾은 경우와 실물에서 본 경우."""
import numpy as np
import pytest

from body_footprint import obstacle_hit, self_mask


def pts(*xy):
    return self_mask(np.array(xy, float).reshape(-1, 2))


@pytest.mark.parametrize('name, p, cmd, remain, stop', [
    ('대각 진행, 대각 앞 (0.18, 0.18) ', pts(0.18, 0.18), (0.05, 0.05, 0), None, True),
    ('대각 진행, 사람 다리 (0.21, 0.21)', pts(0.21, 0.21), (0.05, 0.05, 0), None, True),
    ('전진, 앞 (0.25, 0)', pts(0.25, 0), (0.05, 0, 0), None, True),
    ('전진, 왼앞 자기 선 점 (0.10, 0.17)', pts(0.10, 0.17), (0.05, 0, 0), None, False),
    ('왼쪽, 왼앞 자기 선 점', pts(0.10, 0.17), (0, 0.05, 0), None, False),
    ('대각, 왼앞 자기 선 점', pts(0.10, 0.17), (0.05, 0.05, 0), None, False),
    ('전진, 옆 벽 27 cm', pts(0, 0.27, 0.2, 0.27, -0.2, 0.27), (0.05, 0, 0), None, False),
    ('전진, 목표 1 cm 앞인데 그 너머 벽 27 cm', pts(0.27, 0), (0.02, 0, 0), 0.01, False),
    ('전진, 아직 50 cm 남았는데 벽 25 cm', pts(0.25, 0), (0.05, 0, 0), 0.5, True),
    ('앞 벽 25 cm 에서 왼쪽으로 빠지기 ', pts(0.25, 0), (0, 0.02, 0), 1.0, False),
    ('제자리 회전, 26 cm 점', pts(0.26, 0), (0, 0, 0.3), None, True),
    ('제자리 회전, 벽 30 cm', pts(0.30, 0), (0, 0, 0.3), None, False),
    ('멈춤 명령', pts(0.18, 0), (0, 0, 0), None, False),
    ('이동 + 회전, 뒤 모서리 옆 점이 쓸려 들어옴', pts(-0.172, 0.180), (0.02, 0, 0.2), None, True),
    ('이동 + 작은 방향 보정, 옆 벽 27 cm', pts(0, 0.27, 0.2, 0.27, -0.2, 0.27), (0.05, 0, 0.03), None, False),
    ('빠른 전진 0.26 m/s, 앞 30 cm (정지 거리 14.6 cm 안)', pts(0.30, 0), (0.26, 0, 0), None, True),
    ('느린 전진 0.05 m/s, 앞 30 cm', pts(0.30, 0), (0.05, 0, 0), None, False),
    ('끝 감속 중 옆 보정 + 작은 회전, 옆 벽 27 cm', pts(*[v for x in np.linspace(-0.4, 0.4, 41) for v in (x, 0.27)]), (0.02, 0.006, 0.03), 0.01, False),
    ('옆으로 다가가기, 옆 벽 22 cm', pts(0, 0.22, 0.1, 0.22), (0, 0.05, 0), None, True),
    ('회전 쓸림 경계 점 (−0.1708, 0.1881)', pts(-0.1708, 0.1881), (0.02, 0, 0.2), None, True),
])
def test_obstacle_hit(name, p, cmd, remain, stop):
    assert (obstacle_hit(p, *cmd, remain=remain) is not None) == stop, name


def test_self_mask_keeps_outside_only():
    p = self_mask(np.array([[0.1, 0.17], [0.16, -0.18], [0.18, 0.0], [0.0, 0.2]]))
    assert p.tolist() == [[0.18, 0.0], [0.0, 0.2]]
