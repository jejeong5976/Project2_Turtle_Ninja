#!/usr/bin/env python3
"""robot_description/robot.urdf -> usd/robot/robot/robot.usda (로봇 배포판이 바뀌었을 때 한 번만 실행).

질량 · 관성은 URDF 값 그대로 (3.81 kg). fix_base=False (차체가 바닥에 고정되지 않게).

MuJoCo-Warp(Newton) 백엔드는 Isaac Sim 앱 없이 돌아서 URDF 변환기를 쓸 수 없다.
그래서 PhysX(Isaac Sim) 쪽에서 미리 USD 로 바꿔 두고, 두 물리 엔진 모두 이 USD 를 읽게 한다.
변환 설정은 turtleninja_maze_env.TURTLENINJA_URDF_SPAWN 과 같다 (PhysX·MuJoCo 물리 정보가 같이 들어감).

예) OMNI_KIT_ACCEPT_EULA=YES ~/pa-course/IsaacLab-6.2/.venv/bin/python tools/urdf_to_robot_usd.py
"""
import argparse
import sys
from pathlib import Path as _P
sys.path.insert(0, str(_P(__file__).resolve().parents[1]))   # sim/robot_sim -> sim_core 패키지

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
add_launcher_args(parser)
args = parser.parse_args()

from isaaclab_physx.physics import PhysxCfg

from sim_core import turtleninja_maze_env as tme

with launch_simulation(PhysxCfg(), args):
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

    src = tme.TURTLENINJA_URDF_SPAWN
    cfg = UrdfConverterCfg(
        asset_path=src.asset_path, usd_dir=str(tme.ROBOT_USD.parent.parent), usd_file_name=tme.ROBOT_USD.name,
        force_usd_conversion=True, fix_base=src.fix_base, merge_fixed_joints=src.merge_fixed_joints,
        make_instanceable=src.make_instanceable, joint_drive=src.joint_drive)
    conv = UrdfConverter(cfg)
    print('USD:', conv.usd_path, flush=True)
