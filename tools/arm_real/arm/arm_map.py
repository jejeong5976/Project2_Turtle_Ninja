"""팔 전용 하드웨어 지도 (ros/hardware_map_arm_only.yaml) 에서 관절 영점 · 모드 읽기 (2026-10-30 저녁).
팀 ROS 관절값 = (틱 − 영점) · 2π/4096 (그리퍼는 × 0.01149 m/rad). 영점 없으면 2048. mode gripper = 전류 기반 위치(확장, 여러 바퀴)."""
import re
from pathlib import Path

MAP = Path(__file__).resolve().parent / 'ros' / 'hardware_map_arm_only.yaml'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']


def motors():
    out = {}
    for m in re.finditer(r'^\s+(\w+):\s*\{([^}]*)\}', MAP.read_text(), re.M):
        kv = dict(re.findall(r'(\w+):\s*([\w.\-]+)', m.group(2)))
        out[m.group(1)] = dict(zero=int(kv.get('zero_tick', 2048)), mode=kv.get('mode', 'position'))
    return out


def zeros():
    m = motors()
    return [m[j]['zero'] for j in JOINTS]


def extended():
    m = motors()
    return [m[j]['mode'] == 'gripper' for j in JOINTS]
