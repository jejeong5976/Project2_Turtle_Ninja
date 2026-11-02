"""두 곳에 있는 값이 어긋나지 않았나 — 몸 상자(scan_merger · body_footprint), 바퀴 · 조향 모터(전체 지도 · 주행 보드만 지도)."""
import os

import yaml

from body_footprint import BODY_HX, BODY_HY

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def load(*parts):
    with open(os.path.join(ROOT, *parts)) as f:
        return yaml.safe_load(f)


def test_body_box_matches_scan_merger():
    p = load('ros2_ws', 'src', 'navigation', 'config', 'navigation.yaml')['scan_merger']['ros__parameters']
    assert p['body_half'] == [BODY_HX, BODY_HY]


def test_drive_motors_same_in_both_hardware_maps():
    full = load('config', 'hardware_map.yaml')
    drive = load('config', 'hardware_map_drive_only.yaml')
    wheels = [j for j in drive['motors'] if j.startswith(('drive_', 'steer_'))]
    assert len(wheels) == 8
    for j in wheels:
        assert full['motors'][j] == drive['motors'][j], j
    bus = lambda m: next(b for b in m['buses'] if b['name'] == 'bus_drive')
    assert bus(full) == bus(drive)
    for k in ('watchdog', 'battery_warn_v', 'battery_stop_v', 'require_all_motors'):
        assert full.get(k) == drive.get(k), k
