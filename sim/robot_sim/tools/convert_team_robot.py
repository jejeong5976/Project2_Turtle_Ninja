#!/usr/bin/env python3
"""팀 URDF (ros2_ws/src/description = 실물) -> 시뮬용 URDF · 스워브 설정 · USD. TN_ROBOT=team 이 이것을 읽음 (sim_core/team_robot.py).
URDF 가 바뀌면 다시 실행.

  source /opt/ros/lyrical/setup.bash && source ../../ros2_ws/install/setup.bash   (xacro · description 패키지. 조향 범위는 설치된
    bringup hardware_map 에서 오므로 ros2_ws 를 colcon build 한 뒤)
  OMNI_KIT_ACCEPT_EULA=YES $ISAACLAB_PY tools/convert_team_robot.py

만드는 것 (GitHub 에 안 올림):
  robot_description/team/robot.urdf          xacro 결과에서 ros2_control 을 빼고 메시를 저장소 절대 경로로
  robot_description/team/config/swerve.yaml  robot_description/config/swerve.yaml + 모듈 위치 (config/controllers.yaml) · 조향 범위 (URDF)
  usd/team_robot/robot/robot.usda            변환 설정은 turtleninja_maze_env.TURTLENINJA_URDF_SPAWN 과 같음
"""
import argparse
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]          # sim/robot_sim
REPO = ROOT.parents[1]
DESC = REPO / 'ros2_ws' / 'src' / 'description'
TEAM_DIR = ROOT / 'robot_description' / 'team'
TEAM_URDF = TEAM_DIR / 'robot.urdf'
TEAM_SWERVE = TEAM_DIR / 'config' / 'swerve.yaml'
TEAM_USD = ROOT / 'usd' / 'team_robot' / 'robot' / 'robot.usda'
PASSIVE = ('gripper_crank_joint', 'gripper_left_rod_joint', 'gripper_right_rod_joint')


def make_urdf():
    if not shutil.which('xacro'):
        sys.exit('xacro 없음 → source /opt/ros/lyrical/setup.bash && source ros2_ws/install/setup.bash 뒤 다시')
    xml = subprocess.run(['xacro', str(DESC / 'urdf' / 'robot.urdf.xacro')], check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(xml)
    for rc in root.findall('ros2_control'):
        root.remove(rc)
    for j in root.findall('joint'):   # 그리퍼 연결기구 (크랭크 · 막대) = 실물은 손가락 위치로 따라 도는 수동 관절 → 시뮬에선 고정 (안 그러면 중력에 흔들림)
        if j.get('name') in PASSIVE:
            j.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics', 'mimic'):
                for e in j.findall(tag):
                    j.remove(e)
    for mesh in root.iter('mesh'):
        mesh.set('filename', mesh.get('filename').replace('package://description/', str(DESC) + '/'))
    TEAM_DIR.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(TEAM_URDF, encoding='unicode')
    return root


def make_swerve(root):
    sw = yaml.safe_load((ROOT / 'robot_description' / 'config' / 'swerve.yaml').read_text())
    real = yaml.safe_load((REPO / 'config' / 'controllers.yaml').read_text())['swerve_controller']['ros__parameters']
    for k in ('module_x', 'module_y', 'module_offset', 'wheel_radius'):
        sw['geometry'][k] = real[k]
    lim = {j.get('name'): j.find('limit') for j in root.findall('joint') if j.find('limit') is not None}
    steer = sw['joints']['steer']
    sw['steer_range_rad'] = {'min': [float(lim[n].get('lower')) for n in steer], 'max': [float(lim[n].get('upper')) for n in steer]}
    TEAM_SWERVE.parent.mkdir(parents=True, exist_ok=True)
    TEAM_SWERVE.write_text('# tools/convert_team_robot.py 가 만듦 — 모듈 위치 = config/controllers.yaml, 조향 범위 = 팀 URDF\n'
                           + yaml.safe_dump(sw, sort_keys=False, allow_unicode=True))


def main():
    root = make_urdf()
    make_swerve(root)
    print('URDF:', TEAM_URDF, '· 스워브:', TEAM_SWERVE, flush=True)
    sys.path.insert(0, str(ROOT))
    import os
    os.environ.pop('TN_ROBOT', None)   # 변환 중에는 예전 로봇 설정 그대로 (팀 USD 가 아직 없을 수 있음)
    from isaaclab.app import add_launcher_args, launch_simulation
    ap = argparse.ArgumentParser()
    add_launcher_args(ap)
    args = ap.parse_args()
    from isaaclab_physx.physics import PhysxCfg
    from sim_core import turtleninja_maze_env as tme
    with launch_simulation(PhysxCfg(), args):
        from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
        src = tme.TURTLENINJA_URDF_SPAWN
        shutil.rmtree(TEAM_USD.parent.parent, ignore_errors=True)   # 있으면 변환기가 덮어쓰지 않고 robot_1/ 처럼 새로 만듦
        cfg = UrdfConverterCfg(
            asset_path=str(TEAM_URDF), usd_dir=str(TEAM_USD.parent.parent), usd_file_name=TEAM_USD.name,
            force_usd_conversion=True, fix_base=src.fix_base, merge_fixed_joints=src.merge_fixed_joints,
            make_instanceable=src.make_instanceable, joint_drive=src.joint_drive)
        out = Path(UrdfConverter(cfg).usd_path)
        assert out.resolve() == TEAM_USD.resolve(), f'USD 가 다른 곳에 생김: {out}'
        print('USD:', out, flush=True)


if __name__ == '__main__':
    main()
