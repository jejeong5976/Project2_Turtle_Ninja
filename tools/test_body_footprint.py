"""body_footprint 시험 (2026-11-02) — 11-02 GPT 검토 재현 사례와 11-01 실물에서 본 경우."""
import numpy as np
import pytest

from body_footprint import obstacle_hit, self_mask


def pts(*xy):
    return self_mask(np.array(xy, float).reshape(-1, 2))


@pytest.mark.parametrize('name, p, cmd, remain, stop', [
    ('대각 진행, 대각 앞 (0.18, 0.18) — GPT S1', pts(0.18, 0.18), (0.05, 0.05, 0), None, True),
    ('대각 진행, 사람 다리 (0.21, 0.21)', pts(0.21, 0.21), (0.05, 0.05, 0), None, True),
    ('전진, 앞 (0.25, 0)', pts(0.25, 0), (0.05, 0, 0), None, True),
    ('전진, 왼앞 자기 선 점 (0.10, 0.17)', pts(0.10, 0.17), (0.05, 0, 0), None, False),
    ('왼쪽, 왼앞 자기 선 점', pts(0.10, 0.17), (0, 0.05, 0), None, False),
    ('대각, 왼앞 자기 선 점', pts(0.10, 0.17), (0.05, 0.05, 0), None, False),
    ('전진, 옆 벽 27 cm', pts(0, 0.27, 0.2, 0.27, -0.2, 0.27), (0.05, 0, 0), None, False),
    ('전진, 목표 1 cm 앞인데 그 너머 벽 27 cm', pts(0.27, 0), (0.02, 0, 0), 0.01, False),
    ('전진, 아직 50 cm 남았는데 벽 25 cm', pts(0.25, 0), (0.05, 0, 0), 0.5, True),
    ('앞 벽 25 cm 에서 왼쪽으로 빠지기 — GPT I6', pts(0.25, 0), (0, 0.02, 0), 1.0, False),
    ('제자리 회전, 26 cm 점', pts(0.26, 0), (0, 0, 0.3), None, True),
    ('제자리 회전, 벽 30 cm', pts(0.30, 0), (0, 0, 0.3), None, False),
    ('멈춤 명령', pts(0.18, 0), (0, 0, 0), None, False),
])
def test_obstacle_hit(name, p, cmd, remain, stop):
    assert (obstacle_hit(p, *cmd, remain=remain) is not None) == stop, name


def test_self_mask_keeps_outside_only():
    p = self_mask(np.array([[0.1, 0.17], [0.16, -0.18], [0.18, 0.0], [0.0, 0.2]]))
    assert p.tolist() == [[0.18, 0.0], [0.0, 0.2]]
