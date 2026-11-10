"""test_auto_place_check.py — auto_place_check 순수 계산 시험 (ROS 없이): pytest tools/arm_real/auto_calib"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import auto_place_check as apc  # noqa: E402

QZ90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))   # z 축 90°


def close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_qrot_z90():
    assert close(apc.qrot(QZ90, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))


def test_apply_drift_frame():
    p, q = apc.apply_drift((1.0, 0.0, 0.0), None, (QZ90, 'cam'), 'other')
    assert p == (1.0, 0.0, 0.0) and q is None   # frame 이 다르면 그대로 (control_master 와 같음)
    p, q = apc.apply_drift((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (QZ90, 'cam'), 'cam')
    assert close(p, (0.0, 1.0, 0.0)) and close(q, QZ90)


def test_to_base():
    c, q = apc.to_base((1.0, 0.0, 0.0), None, ((0.1, 0.2, 0.3), QZ90))
    assert close(c, (0.1, 1.2, 0.3)) and q is None


def test_summarize_samples():
    rows = [dict(x=1.0, y=10.0, z=5.0, tag=True, up=True), dict(x=3.0, y=12.0, z=5.0, tag=False, up=True),
            dict(x=2.0, y=11.0, z=5.0, tag=True, up=True)]
    m = apc.summarize_samples(rows)
    assert m['n'] == 3 and m['x'] == 2.0 and m['y'] == 11.0 and m['swing_x'] == 2.0 and m['tag'] == 2 and m['up']
    assert apc.puck_type(m) == '태그 선'


def test_summarize_errors_verdict():
    s = apc.summarize_errors([1.0, 1.0, 1.0], [2.0, 2.0, 2.0])
    assert s['mean'] == (1.0, 2.0) and s['std'] == (0.0, 0.0) and s['verdict'].startswith('반복됨')
    s = apc.summarize_errors([0.0, 10.0], [0.0, 0.0])
    assert s['std'][0] > 3.0 and s['verdict'].startswith('매번 다름') and s['max_abs'] == (10.0, 0.0)
    assert apc.summarize_errors([1.0], [1.0])['std'] == (0.0, 0.0)   # 1 회면 표준편차 0
    assert apc.verdict(2.0).startswith('중간')


def test_propose_offsets():
    assert apc.propose_place_offset((1.0, -2.0), (3.0, 1.0)) == (-2.0, -3.0)   # 지금 − 평균
    assert apc.propose_pick_offset((1.0, -2.0), (3.0, 1.0)) == (4.0, -1.0)     # 지금 + 평균


def test_group_means():
    rows = [dict(type='a', err_x=1.0, err_y=0.0), dict(type='a', err_x=3.0, err_y=2.0), dict(type='b', err_x=9.0, err_y=9.0)]
    assert apc.group_means(rows, lambda r: r['type']) == [('a', 2, 2.0, 1.0)]   # 2 회 이상만


def test_grasp_target_no_q():
    g, q, up = apc.grasp_target((0.25, 0.0, 0.067), None, False, 'cylinder')
    assert close(g, (0.25, 0.0, 0.082)) and q == (0.0, 0.0, 0.0, 0.0) and up   # 위쪽 끝 + 0.015, 방향 모름


def test_grasp_target_upright_box():
    g, q, up = apc.grasp_target((0.25, 0.0, 0.067), QZ90, True, 'box')
    assert up and close(g, (0.25, 0.0, 0.082))
    x, z = apc.quat_cols(q)
    assert close(z, (0.0, 0.0, 1.0)) and abs(abs(x[0]) - 1.0) < 1e-6   # 면 방향 90° → [-45°, 45°) 로 접으면 0°


def test_grasp_target_lying():
    qy = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))   # y 축 90° → 긴축이 +x (로봇 반대쪽)
    with pytest.raises(ValueError):
        apc.grasp_target((0.25, 0.0, 0.054), qy, False, 'cylinder')   # 누운 원기둥은 안 집음
    g, q, up = apc.grasp_target((0.25, 0.0, 0.054), qy, True, 'box')
    assert not up and close(g, (0.235, 0.0, 0.054))   # 로봇 쪽 끝
    x, z = apc.quat_cols(q)
    assert close(z, (-1.0, 0.0, 0.0)) and close(x, (0.0, 0.0, 1.0))   # z = 쥘 끝, x = 위


def test_parse_cm_log():
    out = apc.parse_cm_log(['PICK: 윗면 태그로 잰 퍽 → 태그 퍽 보정 (+10.0, -3.5) mm (pick_offset_tagged_*)',
                            'PICK: 손목을 기본에서 -90° 돌려 쥠 → 돌림 보정 (+0.4, -7.6) mm (pick_offset_rot_mm)',
                            'PICK: 퍽 (0.250, 0.000) 돌림 0.0° · 손 기울임 0° · 쥐는 깊이 50 mm → 쥐기 틱 [1 2 3 4 5]'])
    assert out == dict(tag_fix='+10.0,-3.5', shift='', rot_deg='-90', depth_mm='50')
