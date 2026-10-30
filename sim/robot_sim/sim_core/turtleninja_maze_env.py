"""TurtleNinja x Twin Closed Maze — Isaac Lab 병렬 환경 설정 (ManagerBasedEnv, 보상 없음).

환경마다 미로 1개 + 로봇 1대. reset마다 칸 하나에서 시작하고(위치 흔들림, yaw 균일),
목표 칸은 같은 영역 안에서 시작 칸과 다르게 뽑는다. 기본은 반쪽 미로(24칸, 영역 1개).

전제와 단순화:
- 차체는 스워브로 주행한다: 행동 (vx, vy, wz) -> 배포판 swerve/nt_swerve.py (팀 swerve.cpp 2026-10-27 판 그대로, 100 Hz:
  가속 제한 -> 역기구학 coherent · 출발 때 바퀴 바깥 · 범위 끝 여유 -> 동시 조향 · 구동) -> 조향 위치 목표 4 + 바퀴 속도 목표 4.
  바퀴와 바닥의 마찰로 움직인다.
- 로봇은 팀 최종 배포판 robot_description/robot.urdf 그대로 (스워브 관절 8개, 모듈별 조향 한계, 센서 프레임). 팔은 접은 자세로 고정.
- 작성 기준은 Isaac Lab 3.0 (PhysX 백엔드)이다. 쿼터니언은 (x, y, z, w), 자산 데이터는 `.torch`로 읽는다.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import MISSING
from pathlib import Path
from typing import TYPE_CHECKING

import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import euler_xyz_from_quat, quat_from_euler_xyz
from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg, PhysxRigidBodyCfg
from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg

if TYPE_CHECKING:  # 런타임 import는 Kit보다 먼저 USD를 불러와 충돌한다
    from isaaclab.assets import Articulation
    from isaaclab.envs import ManagerBasedEnv

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent          # sim/robot_sim (usd/, robot_description/, maze_*_layout.json)
# 'half' = 위쪽 영역 6x4칸만 (두 영역의 통행 구조가 같아서 반으로 줄임, 환경 넓이 절반). 'full' = 원래 6x8칸
# 'real' = 팀 실제 미로 (미로_전체부품_R3_R5_M4_2026-10-23 배치 JSON 의 A 경기장, layout_to_maze_usd.py). 벽판 3 mm·3030/R3 기둥·Ø60 마운트
# 'half' = 이전 Gazebo SDF 위쪽 영역 (통로 구조가 실제와 위아래 반대였음, 보관용)
MAZE_VARIANT = 'real'
_SUFFIX = {'half': '_half', 'real': '_real', 'full': ''}[MAZE_VARIANT]
# 환경 변수 TN_MAZE=<이름> 이면 tools/random_maze.py 가 만든 무작위 미로 (mazes/<이름>_layout.json, usd/mazes/<이름>.usd).
# 대회 맵은 시연 1주 전 공개 (바깥 크기만 같음) -> YOLO 데이터는 여러 미로로 나눠 뽑음. sim_core import 전에 정해야 함
MAZE_NAME = os.environ.get('TN_MAZE', '')
LAYOUT = json.loads((ROOT / 'mazes' / f'{MAZE_NAME}_layout.json' if MAZE_NAME else ROOT / f'maze{_SUFFIX}_layout.json').read_text())
# 로봇 = 팀 최종 배포판 nt_urdf_release_20261027_layout (robot_description/, README_KO.md · CHANGES_20261027_layout.md). URDF 는 수정 없이 그대로 쓰고
#   시뮬레이션 값(팔 effort·gain, armature, 마찰, 접은 자세)은 아래 설정에서만 바꿈.
#   robot_description/robot.urdf = 배포판 urdf/robot.urdf 에서 메시 경로 'package://nt_release_description/' 만 지운 것 (Isaac 변환 · 순기구학 · RViz)
#   바퀴 관절 steer_<모듈>_joint / drive_<모듈>_joint, 총질량 3.81 kg, 라이다 스캔면 바닥 위 60 mm (앞 yaw 180° · 뒤 0°)
# 시뮬은 변환된 USD (usd/robot) 로 돌고, URDF 는 다시 변환할 때만 씀 (urdf_to_robot_usd.py)
ROBOT_DIR = ROOT / 'robot_description'
ROBOT_URDF = ROBOT_DIR / 'robot.urdf'
MAZE_USD = ROOT / 'usd' / 'mazes' / f'{MAZE_NAME}.usd' if MAZE_NAME else ROOT / 'usd' / f'maze{_SUFFIX}.usd'
MAZE_COLLISION_USD = MAZE_USD if MAZE_NAME or MAZE_VARIANT == 'real' else ROOT / 'usd' / f'maze{_SUFFIX}_collision_only.usd'  # 카메라 없이 대량 학습용

CELL_X, CELL_Y = LAYOUT['grid']['cell_m']
NUM_CELLS = len(LAYOUT['cells'])
MAZE_INNER_HALF = tuple(LAYOUT.get('inner_half_m', (2.202, 3.152)))  # 바깥벽 중심선 (x, y)

ARM_EFFORT_NM = 3.8  # XM430-W350 11.1 V 정지토크 (URDF 는 12 V 4.1 N·m, 배터리 11.1 V 라 여기서 덮어씀). 연속 운용 한계는 DR 범위로 다룸
# 팔 관절별 한계 = 실물 운용값 (배포판 config/operating_limits.yaml arm_tested_max, A9F5 실물 3회 2026-10-26). URDF 는 하드웨어 한계 (4.1 N·m · 4.817 rad/s)
#   속도 = 실제 잰 최고 속도 [rad/s], 힘 = 재생 중 출력 상한 (Goal PWM / 885) × 정지토크 [N·m] (역기전력 무시 근사)
#   J5 = 모터 설정값 (Profile Velocity 4 -> 0.1 rad/s, PWM 60 % -> 2.28 N·m). 파일에 J5 가 없으면 J4 값으로 대신함
import yaml as _yaml
_OPS = _yaml.safe_load((ROBOT_DIR / 'config' / 'operating_limits.yaml').read_text())['arm_tested_max']
ARM_VEL_LIMIT = dict(_OPS['velocity_rad_s'])
ARM_EFFORT_LIMIT = {j: round(ARM_EFFORT_NM * pct / 100.0, 3) for j, pct in _OPS['pwm_limit_percent'].items()}
ARM_VEL_LIMIT.setdefault('arm_joint5', ARM_VEL_LIMIT['arm_joint4'])
ARM_EFFORT_LIMIT.setdefault('arm_joint5', ARM_EFFORT_LIMIT['arm_joint4'])
SPAWN_Z = 0.075  # 바퀴 바닥 z=-0.072 + 3 mm (배포판 isaac_lab_init_state.parked_init_state 기본 root_z 와 같음)
# 팔 자세 2개 (config/initial_pose.json)
#   TUCK_POSE  = 주행 기본자세 = 실물 받침(parked, 'positions') + 그리퍼 16 mm (손가락 a·b 둘 다). 시작 · 주행 중 고정 자세
#   READY_POSE = 퍽 집기 준비자세 = 실물 P0 (poses 'C1_P0', 받침에서 든 자세). 퍽 집기 직전에 들어감, 그리퍼 값 없음
#   LOOK_PUCK_POSE = 손목 D435 로 선반 위 퍽을 내려다보는 관찰 자세 (poses 'LOOK_PUCK', 2026-10-29 tools/find_look_pose.py). 그리퍼 값 없음
#   _layout 판에서 parked + 16 mm 정착 · 주행 안정 확인 (자기 충돌 끔이라 손가락-받침 겹침 영향 없음, 2026-10-28)
_INIT_POSE = json.loads((ROBOT_DIR / 'config' / 'initial_pose.json').read_text())
_PARKED = _INIT_POSE['positions']
TUCK_POSE = {**{k: v for k, v in _PARKED.items() if k.startswith('arm_joint')},
             'gripper_finger_a_joint': _PARKED['gripper_finger_a_joint'], 'gripper_finger_b_joint': _PARKED['gripper_finger_a_joint']}
READY_POSE = dict(_INIT_POSE['poses']['C1_P0']['q_rad'])
LOOK_PUCK_POSE = dict(_INIT_POSE['poses']['LOOK_PUCK']['q_rad'])
# J1 정면각: 그리퍼 중심 방위가 차체 +x 가 되는 J1. config/joint_map.json 의 J1 받침 자세 4088 틱 = 정면 −94.6486°
J1_FORWARD = math.radians(-94.6485994803)
# 팔이 차체 안이라 차체 꼭짓점(0.133, 0.133)·조향한 바퀴가 가장 멂 -> 반경 0.20 m
FOOTPRINT_RADIUS = 0.20
# 재질 (마찰은 정지/운동, 반발 0). 부품은 TurtleBot3 Waffle Pi 계열 (ROBOTIS 부품 설명 기준), 마찰 수치는 실측 전 추정값
#   rubber_wheel : TB3 Wheel/Tire Set ISW-01 = 엔지니어링 플라스틱 휠 + 고무 타이어(TR-66, "high grip").
#                  바퀴 질량 29.1 g = TB3 공식 URDF 28.5 g 과 같은 부품. 공식 Gazebo 마찰(mu 100000)은 "신뢰할 수 없는 값" 주석이 있어 안 씀
#   tb3_plastic  : TB3 Waffle Plate(엔지니어링 플라스틱), XM430 케이스(금속·플라스틱), FR12 프레임, PLA 출력 부품(손목 판 등),
#                  2020 알루미늄 프로파일 -> 마찰 범위가 비슷해서 한 묶음 (팔·차체·조향 모듈·손가락)
#   pla_wall     : 미로 벽 (PLA 출력물). build_maze_usd.py 가 USD 에 같은 값으로 넣음
#   floor        : 미로 바닥 (재질 미확인 -> 기울기 시험으로 바퀴-바닥 마찰 실측 필요)
MATERIALS = {
    'rubber_wheel': {'static_friction': 0.9, 'dynamic_friction': 0.8, 'restitution': 0.0},
    'tb3_plastic': {'static_friction': 0.35, 'dynamic_friction': 0.3, 'restitution': 0.0},
    'pla_wall': {'static_friction': 0.35, 'dynamic_friction': 0.3, 'restitution': 0.0},
    'floor': {'static_friction': 0.9, 'dynamic_friction': 0.8, 'restitution': 0.0},
}


def _fixed_material(name: str) -> dict:
    m = MATERIALS[name]
    return {'static_friction_range': (m['static_friction'],) * 2, 'dynamic_friction_range': (m['dynamic_friction'],) * 2,
            'restitution_range': (m['restitution'],) * 2}


# 스워브 = 배포판 config/swerve.yaml (팀 drive_only.yaml 과 같은 모듈 위치, 조향 범위 = 엔코더 창 전체, 모터 레지스터 · 데이터시트 값)
#   SWERVE_MODULES: 모듈 이름 -> (조향축 x, y [m], 바퀴 오프셋 [m] = 조향축에서 바퀴 접지점까지 모듈 y 방향, 조향 한계 min, max [rad])
import yaml as _yaml
SWERVE_YAML = _yaml.safe_load((ROBOT_DIR / 'config' / 'swerve.yaml').read_text())
_g, _r = SWERVE_YAML['geometry'], SWERVE_YAML['steer_range_rad']
SWERVE_MODULES = {m: (_g['module_x'][i], _g['module_y'][i], _g['module_offset'][i], _r['min'][i], _r['max'][i])
                  for i, m in enumerate(SWERVE_YAML['joints']['module_names'])}
STEER_JOINT = 'steer_{}_joint'
DRIVE_JOINT = 'drive_{}_joint'
WHEEL_RADIUS = _g['wheel_radius']
_HW = SWERVE_YAML['hardware_max']        # 바퀴 · 조향 최고 속도 [rad/s] · 힘 [N·m]
_TRAIN = SWERVE_YAML['limits_for_training']


# URDF -> USD 변환 설정 (urdf_to_robot_usd.py 가 사용). PhysX·MuJoCo 물리 정보가 함께 들어간 USD 를 만든다
TURTLENINJA_URDF_SPAWN = sim_utils.UrdfFileCfg(
    asset_path=str(ROBOT_URDF),
    fix_base=False,
    merge_fixed_joints=True,
    make_instanceable=True,
    joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
        gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=None, damping=None)),
)
# URDF 를 tools/urdf_to_robot_usd.py 로 변환한 USD (두 물리 엔진 공용, 없으면 그 스크립트 실행). 질량 · 관성은 URDF 그대로 (3.81 kg).
# 배포판에 든 USD (usd/robot) 는 물리 variant 선택이 비어 있어 따로 지정해야 하고 관절 이득도 다름 -> 같은 URDF 로 직접 변환해 씀
ROBOT_USD = ROOT / 'usd' / 'robot' / 'robot' / 'robot.usda'

TURTLENINJA_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(ROBOT_USD),
        rigid_props=PhysxRigidBodyCfg(max_depenetration_velocity=1.0),
        articulation_props=[
            PhysxArticulationCfg(enabled_self_collisions=False, solver_position_iteration_count=32,
                                 solver_velocity_iteration_count=4),
            NewtonArticulationCfg(self_collision_enabled=False),
        ],
        # PhysX 솔버 반복 8/1 이면 팔을 편 채 벽에 부딪힐 때 손목·손가락 관절이 발산 (2026-10-21 시험). 32/4 에서 0
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, SPAWN_Z), joint_pos=dict(TUCK_POSE)),
    # armature = 모터 회전자 관성 × 감속비² (관절에서 느껴지는 관성). XM430 감속비 353.5 -> 약 0.01 kg·m² (추정값, DR 대상)
    # 0 이면 가벼운 손목이 벽에 닿을 때 관절이 발산함 (2026-10-22 시험). 그리퍼는 배포판 값 (armature 없음)
    actuators={
        # 팔 이득 500 / 10 = 배포판 USD 구동값 (nt_train_kit · 키보드 시연 · verify_* 에서 parked 유지 0.18°, 실측 궤적 재생 확인). 실물 맞춤 아님.
        #   힘 · 속도 한계 = 실물 운용값 (위 ARM_EFFORT_LIMIT · ARM_VEL_LIMIT)
        'arm': ImplicitActuatorCfg(joint_names_expr=['arm_joint[1-5]'], stiffness=500.0, damping=10.0,
                                   joint_effort_limit=ARM_EFFORT_LIMIT, joint_velocity_limit=ARM_VEL_LIMIT, armature=0.01),
        # 그리퍼 = 배포판 nt_train_kit 값: 두 손가락 모두 구동 (USD 의 손가락 b 는 mimic 이라 구동 힘 0) 1000 / 2 · 20 N · 0.15 m/s.
        #   16 mm 유지 · 빠른 개폐 (배포판 확인). 실물은 전류 제한 위치 모드 Goal Current 60 (약 0.16 A)
        'gripper': ImplicitActuatorCfg(joint_names_expr=['gripper_finger_.*'], stiffness=1000.0, damping=2.0,
                                       joint_effort_limit=20.0, joint_velocity_limit=0.15),
        # 조향 · 바퀴 이득 = 배포판 검증값 (config/nt_train_kit.py actuators(), tools/sim_drive_check.py 와 같음. 실물 계단 응답 실측 전)
        #   조향 20 N·m/rad ≈ XM430 기본 Position P Gain 800 + 정지 토크 4.1 N·m 대충 환산(약 19)과 비슷함
        # 조향 XM430-W350 위치 모드. 속도 한계(swerve.yaml 최고 4.796 rad/s)는 SwerveAction 이 목표를 그 속도로 옮겨서 지킴
        'steer': ImplicitActuatorCfg(joint_names_expr=['steer_.*_joint'], stiffness=20.0, damping=0.3,
                                     joint_effort_limit=_HW['steer_torque'], joint_velocity_limit=_HW['steer_speed'], armature=0.012),
        # 바퀴 XM430-W210 속도 모드: stiffness 0, damping = 속도 오차 -> 토크 게인
        'drive': ImplicitActuatorCfg(joint_names_expr=['drive_.*_joint'], stiffness=0.0, damping=1.0,
                                     joint_effort_limit=_HW['wheel_torque'], joint_velocity_limit=_HW['wheel_speed'], armature=0.005),
    },
)


# ---------- 미로 상태 (칸 표, 시작/목표) ----------

def maze_state(env: ManagerBasedEnv) -> dict:
    if not hasattr(env, '_maze'):
        cells = torch.tensor([c['xy'] for c in LAYOUT['cells']], device=env.device)
        region = torch.tensor([c['region'] for c in LAYOUT['cells']], device=env.device)
        passable = torch.eye(NUM_CELLS, dtype=torch.bool, device=env.device)
        for a, b in LAYOUT['passable_pairs']:
            passable[a, b] = passable[b, a] = True
        env._maze = {
            'cells': cells, 'region': region, 'passable': passable,
            'start_cell': torch.zeros(env.num_envs, dtype=torch.long, device=env.device),
            'goal_cell': torch.zeros(env.num_envs, dtype=torch.long, device=env.device),
            'goal_xy': torch.zeros(env.num_envs, 2, device=env.device),
        }
    return env._maze


def cell_index(xy: torch.Tensor) -> torch.Tensor:
    """미로 좌표(환경 원점 기준) -> 칸 번호 (j * nx + i, j=0이 북쪽)."""
    nx, ny = LAYOUT['grid']['nx'], LAYOUT['grid']['ny']
    i = torch.floor(xy[:, 0] / CELL_X + nx / 2).long().clamp(0, nx - 1)
    j = torch.floor(ny / 2 - xy[:, 1] / CELL_Y).long().clamp(0, ny - 1)
    return j * nx + i


def base_xy_in_maze(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg('robot')) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.root_pos_w.torch[:, :2] - env.scene.env_origins[:, :2]


def base_yaw(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg('robot')) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return euler_xyz_from_quat(asset.data.root_quat_w.torch)[2]


# ---------- 이벤트 ----------

def reset_robot_in_maze(env: ManagerBasedEnv, env_ids: torch.Tensor, footprint_radius: float, margin: float,
                        asset_cfg: SceneEntityCfg = SceneEntityCfg('robot')):
    """시작 칸·위치·yaw와 같은 영역의 목표 칸을 뽑고 로봇을 놓는다."""
    state = maze_state(env)
    asset: Articulation = env.scene[asset_cfg.name]
    if isinstance(env_ids, slice):  # 전체 reset은 slice(None)으로 온다
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    n = len(env_ids)
    dev = env.device

    if 'blocked' in state:  # 소품 환경: 선반이 있는 칸에서는 시작하지 않음
        start = torch.rand(n, NUM_CELLS, device=dev).masked_fill(state['blocked'][env_ids], -1.0).argmax(dim=1)
    else:
        start = torch.randint(0, NUM_CELLS, (n,), device=dev)
    scores = torch.rand(n, NUM_CELLS, device=dev)
    other = (state['region'][None, :] != state['region'][start][:, None]) | \
            (torch.arange(NUM_CELLS, device=dev)[None, :] == start[:, None])
    goal = scores.masked_fill(other, -1.0).argmax(dim=1)

    post_half = LAYOUT['post_half_m']
    jitter = torch.tensor([max(0.0, CELL_X / 2 - post_half - footprint_radius - margin),
                           max(0.0, CELL_Y / 2 - post_half - footprint_radius - margin)], device=dev)
    xy = state['cells'][start] + (torch.rand(n, 2, device=dev) * 2 - 1) * jitter
    yaw = (torch.rand(n, device=dev) * 2 - 1) * math.pi

    pos = env.scene.env_origins[env_ids].clone()
    pos[:, :2] += xy
    pos[:, 2] += SPAWN_Z
    zeros = torch.zeros(n, device=dev)
    quat = quat_from_euler_xyz(zeros, zeros, yaw)  # (x, y, z, w)
    asset.write_root_pose_to_sim_index(root_pose=torch.cat([pos, quat], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
    joint_pos = asset.data.default_joint_pos.torch[env_ids].clone()
    asset.write_joint_position_to_sim_index(position=joint_pos, env_ids=env_ids)
    asset.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(joint_pos), env_ids=env_ids)

    state['start_cell'][env_ids] = start
    state['goal_cell'][env_ids] = goal
    state['goal_xy'][env_ids] = state['cells'][goal]


# ---------- 랜덤 소품 (바코드·신호등·선반·퍽) ----------
# 소품은 벽면 자리(slot = 칸 하나의 벽 한 면, 기둥 사이)에 붙인다. reset 마다 env 별로 자리·위치를 다시 뽑는다.
# 크기·색 중 실측이 아닌 값은 (가정) 표시. USD 는 make_props_usd.py 가 만듦
PROPS_DIR = ROOT / 'usd' / 'props'
NUM_BARCODES = 4                          # Code128 P2-01..P2-04 (팀 배포 표식)
BARCODE_W, BARCODE_H = 0.070, 0.024       # 흰 여백 포함 실측 (2026-10-28), tools/make_props_usd.py 와 같게
NUM_PLACE_TAGS = 3                        # 장소용 AprilTag L1–L3 (tag36h11 ID 12–14), 라벨 60 mm
PLACE_TAG_W = 0.060
PLACE_TAG_Z_RANGE = (0.15, 0.32)          # 붙이는 높이 미공개 -> 범위에서 무작위 (가정)
BARCODE_Z_RANGE = (0.18, 0.35)            # 판 중심 높이: 이전 0.10–0.30 에서 8 cm 올림 (위는 벽 0.39 - 판 반 0.02 - 여유)
PROFILE_W = 0.020                         # 신호등 기둥 = 알루미늄 프로파일 20×20×450, 벽면에서 1 cm 띄워 세움
TL_POST_DEPTH = PROFILE_W + 0.040         # 벽 -> 기둥 + 신호등(Ø40) 앞끝까지
# 퍽 12개를 짐 선반에 나눠 담음: reset 마다 env 별로 '선반 4 × 3개' 또는 '선반 3 × 4개' 무작위 + 빈 선반 1개 있음/없음 무작위
# (빈 선반 = 퍽을 옮겨 놓는 미션 대비). 선반 수·배분·빈 선반 확률은 가정. 안 쓰는 선반은 미로 밖에 치워 둠
LOADED_SHELF_CHOICES = (4, 3)             # 짐 선반 수 후보 (12 / 선반 수 = 선반당 퍽 수)
EMPTY_SHELF_PROB = 0.5
NUM_SHELVES = max(LOADED_SHELF_CHOICES) + 1  # 선반 자산 수 (짐 최대 4 + 빈 1)
SHELF_SIZE = (0.27, 0.27, 0.07)           # 깊이 × 폭 × 높이 270×270×70 mm (팀 확정 2026-10-23). 벽에 붙여 놓는다고 가정
ROBOT_FRONT = 0.133                       # base_link -> 차체 앞면
SHELF_STAND_GAP = 0.03                    # 정렬 자세: 차체 앞면 ~ 선반 앞면 간격 (가정)
GOAL_DIST_FROM_WALL = SHELF_SIZE[0] + SHELF_STAND_GAP + ROBOT_FRONT   # 정렬 자세 중심 ~ 벽면 = 0.433 m
PUCK_D, PUCK_H = 0.03, 0.06  # 원기둥 Ø3×6 / 사각기둥 3×3×6 cm
PUCK_MASS = {'box': 0.017, 'cyl': 0.013}  # 실측 (2026-10-24). 시뮬 물리는 usd/props/puck_*.usda 값을 씀 (tools/make_props_usd.py)


def _srgb_to_linear(rgb255):
    """사진·색상표의 sRGB 값 -> 렌더러 재질 입력(선형). 그대로 넣으면 렌더 결과가 실제보다 밝게 나옴."""
    c = [v / 255.0 for v in rgb255]
    return tuple(x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c)


SHELF_SRGB = (191, 166, 134)   # 선반 = 골판지 박스 색. 실물 사진 2026-10-27 (카펫 위 박스) 화소 중앙값 (223, 194, 156) 을
#   같은 사진 카펫 (125) 이 카펫 기준값 real_colors.json (107) 이 되게 선형 공간에서 맞춤 (폰 자동 노출 보정). 이전 10-24 값 (203, 172, 139)
PUCK_COLORS = {'red': _srgb_to_linear((157, 34, 53)),    # 팀 지정 (렌더용 보정은 tools/make_props_usd.py)
               'green': _srgb_to_linear((45, 150, 72))}  # 실물 사진 측정 (make_props_usd.py 와 같게)
# 퍽 이름 = (색, 모양). 어떤 퍽이 어느 선반·자리에 갈지는 reset 마다 무작위
# 퍽 = (색, 모양, 태그 ID). 빨강 R1–R4 = ID 0–3, 초록 G1–G4 = ID 4–7 (파랑 B 는 빠짐, 2026-10-23). 형태별 개수는 미공지 -> 색마다 반반 가정
# 퍽 USD(태그 포함)는 make_props_usd.py 의 PUCKS 와 같은 순서
PUCKS = [('red', 'box', 0), ('red', 'box', 1), ('red', 'cyl', 2), ('red', 'cyl', 3),
         ('green', 'box', 4), ('green', 'box', 5), ('green', 'cyl', 6), ('green', 'cyl', 7)]
assert len(PUCKS) <= 4 * min(LOADED_SHELF_CHOICES)   # 선반 윗면 2×2 칸


def wall_slots(state: dict, device) -> dict:
    """칸마다 막힌 면(바깥벽 또는 통로가 아닌 이웃) = 소품 자리. yaw = 벽에서 칸 안쪽을 보는 방향."""
    nx, ny = LAYOUT['grid']['nx'], LAYOUT['grid']['ny']
    post, wall_half = LAYOUT['post_half_m'], LAYOUT['wall_thickness_m'] / 2
    xy, yaw, length, cell = [], [], [], []
    for c, info in enumerate(LAYOUT['cells']):
        i, j = info['i'], info['j']
        for di, dj, dx, dy in ((0, -1, 0, 1), (0, 1, 0, -1), (1, 0, 1, 0), (-1, 0, -1, 0)):  # 북(+y) 남 동(+x) 서
            ni, nj = i + di, j + dj
            if 0 <= ni < nx and 0 <= nj < ny and bool(state['passable'][c, nj * nx + ni]):
                continue
            half = CELL_Y / 2 if dy else CELL_X / 2
            xy.append((info['xy'][0] + dx * (half - wall_half), info['xy'][1] + dy * (half - wall_half)))
            yaw.append(math.atan2(-dy, -dx))
            length.append((CELL_X if dy else CELL_Y) - 2 * post)
            cell.append(c)
    t = lambda v, dt=torch.float32: torch.tensor(v, dtype=dt, device=device)
    return {'xy': t(xy), 'yaw': t(yaw), 'len': t(length), 'cell': t(cell, torch.long)}


def _place_on_slot(env, slot_ids, out_m, along_m, z, yaw_extra=None):
    """벽면 자리 기준 위치 -> 월드 자세 (n, 7). out_m = 벽에서 안쪽 거리, along_m = 벽을 따라 (자리 중심 기준)."""
    slots = env._maze['slots']
    yaw = slots['yaw'][slot_ids]
    nrm = torch.stack([torch.cos(yaw), torch.sin(yaw)], dim=-1)      # 소품 로컬 +X (벽 -> 칸)
    tan = torch.stack([-torch.sin(yaw), torch.cos(yaw)], dim=-1)     # 소품 로컬 +Y (벽을 따라)
    xy = slots['xy'][slot_ids] + nrm * out_m[:, None] + tan * along_m[:, None]
    yaw_q = yaw if yaw_extra is None else yaw + yaw_extra
    zeros = torch.zeros_like(yaw_q)
    return xy, torch.cat([torch.stack([xy[:, 0], xy[:, 1], z], dim=-1),
                          quat_from_euler_xyz(zeros, zeros, yaw_q)], dim=-1)


def hold_arm_at_default(env: ManagerBasedEnv, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg('robot')):
    """팔·그리퍼 위치 목표 = 기본 자세(TUCK_POSE). 정책 행동에는 팔이 없음 (주행 중 팔 HOME 고정 = 실물 MotionMode).
    목표는 다음 reset 까지 유지됨. 로봇 reset 함수와 따로 둔 이벤트라, 시험 스크립트가 reset 함수를 바꿔도 적용됨."""
    asset: Articulation = env.scene[asset_cfg.name]
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    hold_ids = asset.find_joints(['arm_joint[1-5]', 'gripper_finger_.*'])[0]
    value = asset.data.default_joint_pos.torch[env_ids][:, hold_ids]
    asset.actuators.target_command.set_position_index(value=value, joint_ids=hold_ids, env_ids=env_ids)


def reset_props_in_maze(env: ManagerBasedEnv, env_ids: torch.Tensor):
    """선반(짐 3·4개 + 빈 선반 0·1개, 서로 다른 칸), 신호등 기둥(선반 후보 칸 제외), 바코드 3개(남은 자리)를 무작위 벽면에, 퍽 12개는 짐 선반 위 무작위 자리에."""
    state = maze_state(env)
    if 'slots' not in state:
        state['slots'] = wall_slots(state, env.device)
        state['blocked'] = torch.zeros(env.num_envs, NUM_CELLS, dtype=torch.bool, device=env.device)
    slots = state['slots']
    dev, n, S = env.device, len(env_ids), len(slots['cell'])
    origin = env.scene.env_origins[env_ids]
    rand = lambda *shape: torch.rand(*shape, device=dev)
    sym = lambda *shape: rand(*shape) * 2 - 1

    # 선반: 같은 칸에 두 개면 모서리에서 겹칠 수 있어 칸을 다르게
    scores = rand(n, S)
    shelf_slots = []
    for _ in range(NUM_SHELVES):
        s = scores.argmax(dim=1)
        shelf_slots.append(s)
        scores = scores.masked_fill(slots['cell'][None, :] == slots['cell'][s][:, None], -1.0)
    shelf_slots = torch.stack(shelf_slots, dim=1)                    # (n, NUM_SHELVES)
    shelf_cells = slots['cell'][shelf_slots]                          # (n, NUM_SHELVES)
    # 신호등 기둥: 선반이 있는 칸은 피함 (모서리에서 선반과 겹침 방지)
    in_shelf_cell = (slots['cell'][None, :, None] == shelf_cells[:, None, :]).any(-1)
    tl_slot = rand(n, S).masked_fill(in_shelf_cell, -1.0).argmax(dim=1)
    scores = rand(n, S).scatter(1, shelf_slots, -1.0).scatter(1, tl_slot[:, None], -1.0)
    other_slots = torch.cat([scores.topk(NUM_BARCODES + NUM_PLACE_TAGS, dim=1).indices, tl_slot[:, None]], dim=1)  # 바코드 4 + 장소 태그 3 + 신호등 1

    def write(name, pose):
        pose = pose.clone()
        pose[:, :3] += origin
        env.scene[name].write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)

    for k in range(NUM_BARCODES):
        s = other_slots[:, k]
        along = sym(n) * (slots['len'][s] / 2 - BARCODE_W / 2 - 0.01)
        z = BARCODE_Z_RANGE[0] + rand(n) * (BARCODE_Z_RANGE[1] - BARCODE_Z_RANGE[0])
        write(f'barcode_{k}', _place_on_slot(env, s, torch.full((n,), 0.001, device=dev), along, z)[1])

    for k in range(NUM_PLACE_TAGS):
        s = other_slots[:, NUM_BARCODES + k]
        along = sym(n) * (slots['len'][s] / 2 - PLACE_TAG_W / 2 - 0.01)
        z = PLACE_TAG_Z_RANGE[0] + rand(n) * (PLACE_TAG_Z_RANGE[1] - PLACE_TAG_Z_RANGE[0])
        write(f'place_tag_{k}', _place_on_slot(env, s, torch.full((n,), 0.001, device=dev), along, z)[1])

    s = other_slots[:, NUM_BARCODES + NUM_PLACE_TAGS]
    along = sym(n) * (slots['len'][s] / 2 - 0.02 - 0.01)  # 신호등 Ø40 이 기둥 옆 폭을 정함
    write('traffic_light', _place_on_slot(env, s, torch.full((n,), PROFILE_W / 2 + 0.01, device=dev), along,
                                          torch.zeros(n, device=dev))[1])

    # 선반 구성: 짐 선반 수 L (3 또는 4), 빈 선반 e (0 또는 1). 선반 0..L-1 = 짐, L = 빈 선반(e=1 일 때), 나머지는 치움
    choices = torch.tensor(LOADED_SHELF_CHOICES, device=dev)
    n_loaded = choices[torch.randint(0, len(choices), (n,), device=dev)]
    has_empty = (rand(n) < EMPTY_SHELF_PROB).long()
    active = torch.arange(NUM_SHELVES, device=dev)[None, :] < (n_loaded + has_empty)[:, None]  # (n, NUM_SHELVES)

    shelf_xy, shelf_yaw = [], []
    for k in range(NUM_SHELVES):
        s = shelf_slots[:, k]
        along = sym(n) * (slots['len'][s] / 2 - SHELF_SIZE[1] / 2 - 0.01)
        xy, pose = _place_on_slot(env, s, torch.full((n,), SHELF_SIZE[0] / 2 + 0.001, device=dev), along,
                                  torch.full((n,), SHELF_SIZE[2] / 2, device=dev))
        park = torch.tensor([*_prop_init(8 + k).pos[:2], SHELF_SIZE[2] / 2, 0.0, 0.0, 0.0, 1.0], device=dev)
        write(f'shelf_{k}', torch.where(active[:, k:k + 1], pose, park.expand(n, 7)))
        shelf_xy.append(xy)
        shelf_yaw.append(slots['yaw'][s])

    # 퍽: 짐 선반 길이를 선반당 퍽 수(3·4)만큼 칸으로 나눠 칸마다 하나, 칸 안에서 위치·yaw 무작위. 어떤 퍽이 어느 자리인지도 무작위
    # 선반 윗면을 2×2 칸으로 나눠 칸마다 퍽 하나 (선반당 3개면 4칸 중 무작위 3칸), 칸 안에서 위치·yaw 무작위
    cell_out, cell_along = SHELF_SIZE[0] / 2, SHELF_SIZE[1] / 2
    jitter_out = max(0.0, cell_out / 2 - PUCK_D * 0.75)
    jitter_along = max(0.0, cell_along / 2 - PUCK_D * 0.75)
    cell_perm = rand(n, NUM_SHELVES, 4).argsort(dim=-1)             # 선반마다 칸 순서
    # 퍽 8개를 짐 선반 L 개에 고르게: 무작위 순번 r 의 퍽 -> 선반 r % L, 그 선반의 r // L 번째 칸 (L=4 면 2개씩, L=3 이면 3·3·2)
    place = rand(n, len(PUCKS)).argsort(dim=1)
    for p in range(len(PUCKS)):
        spot = place[:, p]
        k, b = spot % n_loaded, spot // n_loaded
        sxy = torch.stack(shelf_xy, dim=1)[torch.arange(n, device=dev), k]
        syaw = torch.stack(shelf_yaw, dim=1)[torch.arange(n, device=dev), k]
        cell = cell_perm[torch.arange(n, device=dev), k, b]
        out = ((cell // 2).float() - 0.5) * cell_out + sym(n) * jitter_out
        along = ((cell % 2).float() - 0.5) * cell_along + sym(n) * jitter_along
        xy = sxy + torch.stack([torch.cos(syaw), torch.sin(syaw)], -1) * out[:, None] \
            + torch.stack([-torch.sin(syaw), torch.cos(syaw)], -1) * along[:, None]
        z = torch.full((n,), SHELF_SIZE[2] + PUCK_H / 2 + 0.002, device=dev)
        zeros = torch.zeros(n, device=dev)
        pose = torch.cat([torch.stack([xy[:, 0], xy[:, 1], z], -1),
                          quat_from_euler_xyz(zeros, zeros, sym(n) * math.pi)], dim=-1)
        write(f'puck_{p}', pose)
        env.scene[f'puck_{p}'].write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev),
                                                                env_ids=env_ids)

    state['blocked'][env_ids] = False
    state['blocked'][env_ids[:, None], shelf_cells] = active  # 치운 선반의 칸은 막지 않음
    state['blocked'][env_ids, slots['cell'][tl_slot]] = True  # 기둥이 벽에서 6 cm 나와 로봇 시작 범위와 겹칠 수 있음
    state['shelf_slots'] = state.get('shelf_slots', torch.zeros(env.num_envs, NUM_SHELVES, dtype=torch.long, device=dev))
    state['shelf_slots'][env_ids] = shelf_slots
    state['other_slots'] = state.get('other_slots', torch.zeros(env.num_envs, NUM_BARCODES + NUM_PLACE_TAGS + 1, dtype=torch.long, device=dev))
    state['other_slots'][env_ids] = other_slots
    for key, val in (('shelf_active', active), ('n_loaded_shelves', n_loaded), ('has_empty_shelf', has_empty.bool()),
                     ('shelf_xy', torch.stack(shelf_xy, dim=1)), ('shelf_yaw', torch.stack(shelf_yaw, dim=1))):  # 미로 좌표
        if key not in state:
            state[key] = torch.zeros((env.num_envs,) + val.shape[1:], dtype=val.dtype, device=dev)
        state[key][env_ids] = val


def reset_props_and_robot(env: ManagerBasedEnv, env_ids: torch.Tensor, footprint_radius: float, margin: float,
                          asset_cfg: SceneEntityCfg = SceneEntityCfg('robot')):
    """소품을 먼저 놓고(선반 칸 기록) 로봇은 선반이 없는 칸에서 시작."""
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    reset_props_in_maze(env, env_ids)
    reset_robot_in_maze(env, env_ids, footprint_radius, margin, asset_cfg)


# ---------- 관측 ----------

def obs_base_pose(env: ManagerBasedEnv) -> torch.Tensor:
    yaw = base_yaw(env)
    return torch.cat([base_xy_in_maze(env), torch.sin(yaw)[:, None], torch.cos(yaw)[:, None]], dim=-1)


def obs_goal_in_base(env: ManagerBasedEnv) -> torch.Tensor:
    delta = maze_state(env)['goal_xy'] - base_xy_in_maze(env)
    yaw = base_yaw(env)
    c, s = torch.cos(yaw), torch.sin(yaw)
    return torch.stack([c * delta[:, 0] + s * delta[:, 1], -s * delta[:, 0] + c * delta[:, 1]], dim=-1)


# ---------- 스워브 차체 행동 (배포판 nt_swerve.py = 팀 swerve.cpp 2026-10-27 판) ----------

def wrap_angle(a: torch.Tensor) -> torch.Tensor:
    return torch.remainder(a + math.pi, 2 * math.pi) - math.pi


# 배포판 스워브 컨트롤러 (팀 swerve.cpp 2026-10-27 판을 줄 단위로 옮긴 것, C++ 일치 검사 tools/check_swerve_port.py)
import sys as _sys
_sys.path.insert(0, str(ROBOT_DIR / 'swerve'))
import nt_swerve  # noqa: E402


class SwerveAction(ActionTerm):
    """행동 [-1, 1]^3 -> 몸 속도 명령 (vx, vy, wz) -> 배포판 nt_swerve.SwerveDrive (실물 swerve_controller 와 같은 규칙)
    -> 모듈 4개 (조향 위치 목표, 바퀴 속도 목표).

    env 마다 SwerveDrive 하나를 두고 실물처럼 ctrl_hz(100 Hz) 로 update 함. 규칙은 모두 nt_swerve.py 그대로:
      명령 크기 제한 -> 가속 제한 -> 역기구학 (coherent 조향 선택, 멈춤 -> 출발 때 바퀴 바깥쪽 선호, 이동 중 분기 유지,
      범위 끝 1° 여유) -> 동시 조향 · 구동 (blend) -> 조향 정렬 게이트 -> 바퀴 최고 속도 정규화.
    설정 = 배포판 config/swerve.yaml. 단 조향 정렬 허용각만 cfg.steer_tolerance (기본 π = 연속 조향, 사용자 결정) 로 덮어씀.
    조향 모터 속도 한계(steer_rate)는 물리 스텝마다 조향 목표를 그 속도 이하로 옮겨서 흉내 냄 (실물 모터 profile).
    계산은 CPU Python (env 수가 적은 데이터 생성 · 보기용). 빠진 것: 팔 차례(motion_mode) 잠금 = 항상 주행 허용.
    """

    cfg: 'SwerveActionCfg'
    _asset: Articulation

    def __init__(self, cfg: 'SwerveActionCfg', env: ManagerBasedEnv):
        super().__init__(cfg, env)
        names = list(SWERVE_MODULES)
        self._steer_ids, _ = self._asset.find_joints([STEER_JOINT.format(m) for m in names], preserve_order=True)
        self._drive_ids, _ = self._asset.find_joints([DRIVE_JOINT.format(m) for m in names], preserve_order=True)
        kin, dcfg, _ = nt_swerve.from_config(str(ROBOT_DIR / 'config' / 'swerve.yaml'))
        dcfg.max_linear, dcfg.max_angular = cfg.max_linear, cfg.max_angular
        dcfg.max_linear_accel, dcfg.max_angular_accel = cfg.max_linear_accel, cfg.max_angular_accel
        dcfg.steer_tolerance = cfg.steer_tolerance
        self._drives = [nt_swerve.SwerveDrive(kin, dcfg) for _ in range(self.num_envs)]
        self._dt = env.physics_dt
        self._ctrl_every = max(1, round(1.0 / (cfg.ctrl_hz * self._dt)))
        self._step = 0
        self._now = 0.0
        self._raw = torch.zeros(self.num_envs, 3, device=self.device)
        self._cmd = torch.zeros_like(self._raw)
        self._out = torch.zeros_like(self._raw)  # 컨트롤러가 가속 제한 · 게이트를 거쳐 낸 몸 속도
        self._steer_goal = torch.zeros(self.num_envs, 4, device=self.device)   # 컨트롤러 조향 목표
        self._wheel_goal = torch.zeros(self.num_envs, 4, device=self.device)   # 컨트롤러 바퀴 각속도 목표 [rad/s]
        self._steer_target = torch.zeros(self.num_envs, 4, device=self.device)  # 모터 속도 한계를 거친 조향 목표
        self.aligned = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)  # False = 조향 정렬 게이트로 구동 멈춤
        # 검증용: 매 물리 스텝의 최대값 (다운샘플 전). reset 때 0
        self.peak = {k: torch.zeros(self.num_envs, device=self.device)
                     for k in ('steer_rate', 'wheel_speed', 'steer_torque', 'wheel_torque')}

    @property
    def action_dim(self) -> int:
        return 3

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._cmd

    def process_actions(self, actions: torch.Tensor):
        self._raw[:] = actions
        a = actions.clamp(-1.0, 1.0)
        lin = a[:, :2] * self.cfg.max_linear
        norm = lin.norm(dim=1, keepdim=True).clamp(min=1e-9)
        self._cmd[:, :2] = lin * (norm.clamp(max=self.cfg.max_linear) / norm)
        self._cmd[:, 2] = a[:, 2] * self.cfg.max_angular

    def _controller_update(self):
        """env 마다 SwerveDrive.set_command -> update(실측 모듈 상태). 결과를 _steer_goal · _wheel_goal 에 넣음."""
        data = self._asset.data
        q = data.joint_pos.torch[:, self._steer_ids].cpu().tolist()
        qd = data.joint_vel.torch[:, self._steer_ids].cpu().tolist()
        wd = data.joint_vel.torch[:, self._drive_ids].cpu().tolist()
        cmd = self._cmd.cpu().tolist()
        dt = self._ctrl_every * self._dt
        steer, wheel, out, ok = [], [], [], []
        for e, drv in enumerate(self._drives):
            drv.set_command(*cmd[e], self._now)
            meas = [nt_swerve.ModuleState(q[e][i], wd[e][i], qd[e][i]) for i in range(4)]
            mods = drv.update(self._now, dt, meas)
            steer.append([m.steer_angle for m in mods])
            wheel.append([m.wheel_speed for m in mods])
            out.append([drv.out.vx, drv.out.vy, drv.out.wz])
            ok.append(not drv.gate_blocked)
        self._steer_goal[:] = torch.tensor(steer, device=self.device)
        self._wheel_goal[:] = torch.tensor(wheel, device=self.device)
        self._out[:] = torch.tensor(out, device=self.device)
        self.aligned[:] = torch.tensor(ok, device=self.device)

    def apply_actions(self):
        c, dt = self.cfg, self._dt
        data = self._asset.data
        self.peak['steer_rate'] = torch.maximum(self.peak['steer_rate'], data.joint_vel.torch[:, self._steer_ids].abs().amax(1))
        self.peak['wheel_speed'] = torch.maximum(self.peak['wheel_speed'], data.joint_vel.torch[:, self._drive_ids].abs().amax(1))
        self.peak['steer_torque'] = torch.maximum(self.peak['steer_torque'], data.applied_torque.torch[:, self._steer_ids].abs().amax(1))
        self.peak['wheel_torque'] = torch.maximum(self.peak['wheel_torque'], data.applied_torque.torch[:, self._drive_ids].abs().amax(1))
        if self._step % self._ctrl_every == 0:
            self._controller_update()
        self._step += 1
        self._now += dt

        cmd = self._asset.actuators.target_command
        if c.steer_servo:
            # 속도 제한 서보 (Dynamixel profile 흉내): 속도 명령 = clamp(Kp × 각도 오차, ±steer_rate), 속도 구동.
            # 'steer' 액추에이터를 속도 구동(stiffness 0)으로 바꿔야 함 -> use_steer_servo(cfg)
            now = data.joint_pos.torch[:, self._steer_ids]
            self._steer_target[:] = self._steer_goal
            steer_vel = (c.steer_kp * (self._steer_goal - now)).clamp(-c.steer_rate, c.steer_rate)
            cmd.set_velocity_index(value=steer_vel, joint_ids=self._steer_ids)
        else:
            # 조향 모터 profile: 위치 목표를 최대 steer_rate·dt 만큼만 옮김 (위치 구동)
            step = c.steer_rate * dt
            self._steer_target += (self._steer_goal - self._steer_target).clamp(-step, step)
            cmd.set_position_index(value=self._steer_target, joint_ids=self._steer_ids)
        cmd.set_velocity_index(value=self._wheel_goal, joint_ids=self._drive_ids)

    def reset(self, env_ids=None):
        if env_ids is None:
            env_ids = slice(None)
        ids = range(self.num_envs)[env_ids] if isinstance(env_ids, slice) else env_ids.tolist()
        for e in ids:
            self._drives[e].reset()
        self._raw[env_ids] = 0.0
        self._cmd[env_ids] = 0.0
        self._out[env_ids] = 0.0
        self._steer_goal[env_ids] = 0.0    # reset 이벤트가 조향을 기본 자세 0 으로 둔다
        self._wheel_goal[env_ids] = 0.0
        self._steer_target[env_ids] = 0.0
        self.aligned[env_ids] = True
        for v in self.peak.values():
            v[env_ids] = 0.0


@configclass
class SwerveActionCfg(ActionTermCfg):
    """속도 · 가속 한계 = 배포판 config/swerve.yaml 의 limits_for_training (하드웨어 끝 + 추정 가속). 실물 확인은 0.13 m/s · 0.2 m/s² 까지.
    나머지 컨트롤러 규칙 · 값은 swerve.yaml controller 절 그대로 (nt_swerve.from_config)."""

    class_type: type[ActionTerm] = SwerveAction
    asset_name: str = MISSING
    max_linear: float = _TRAIN['max_linear']  # m/s (0.2612 = 바퀴 최고 7.914 rad/s × 0.033 m)
    max_angular: float = _TRAIN['max_angular']  # rad/s (1.675 = 제자리 회전 끝)
    max_linear_accel: float = _TRAIN['max_linear_accel']  # m/s² [추정]
    max_angular_accel: float = _TRAIN['max_angular_accel']  # rad/s² [추정]
    # 조향 정렬 허용각: 모듈 하나라도 이만큼 넘게 어긋나면 구동을 멈추고 조향 먼저. π = 연속 조향 (구동하며 조향, 바퀴 속도는 cos 감속)
    # 연속 조향 채택 (사용자 결정 · 시험 완료, 2026-10-23 reports/steer_modes). 배포판 swerve.yaml 은 아직 0.17 (조향 먼저)
    steer_tolerance: float = math.pi
    ctrl_hz: float = 100.0  # 실물 swerve_controller 주기
    steer_rate: float = _HW['steer_speed']  # rad/s = 조향 모터 Velocity Limit 200 × 0.229 rpm (실물 hardware_map profile_vel 40 = 0.96, 확인 필요)
    steer_servo: bool = False  # True = 조향 속도 제한 서보 (use_steer_servo 로 켜기). False = 위치 구동 + 목표 램프
    steer_kp: float = 4.0  # [1/s] 각도 오차 -> 조향 속도 명령. steer_rate/4 안쪽부터 감속


# ---------- 환경 설정 ----------

@configclass
class MazeSceneCfg(InteractiveSceneCfg):
    # 바닥 (미로 바닥 재질 미확인). 접촉 마찰 = 두 재질 평균 (PhysX average 결합)
    ground = AssetBaseCfg(prim_path='/World/ground', spawn=sim_utils.GroundPlaneCfg(
        physics_material=PhysxRigidBodyMaterialCfg(**MATERIALS['floor'])))
    light = AssetBaseCfg(prim_path='/World/light', spawn=sim_utils.DomeLightCfg(intensity=2000.0))
    maze = AssetBaseCfg(prim_path='{ENV_REGEX_NS}/Maze', spawn=sim_utils.UsdFileCfg(usd_path=str(MAZE_USD)))
    robot: ArticulationCfg = TURTLENINJA_CFG.replace(prim_path='{ENV_REGEX_NS}/Robot')


def _prop_init(k: int, row: int = 0):  # 첫 reset 전 자리: 미로 남쪽 바깥 (env 사이 빈 바닥)에 줄지어
    return RigidObjectCfg.InitialStateCfg(pos=(-2.0 + 0.35 * k, -MAZE_INNER_HALF[1] - 0.4 - 0.4 * row, 0.1))


_KINEMATIC = sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True)


def _puck_cfg(k: int, color: str, shape: str, tag_id: int) -> RigidObjectCfg:
    """퍽 USD (몸체 + AprilTag 라벨, 질량·마찰 포함) = make_props_usd.py 가 만듦."""
    return RigidObjectCfg(prim_path=f'{{ENV_REGEX_NS}}/Puck_{k}', init_state=_prop_init(k, row=1),
                          spawn=sim_utils.UsdFileCfg(usd_path=str(PROPS_DIR / f'puck_{k}.usda')))


@configclass
class MazePropsSceneCfg(MazeSceneCfg):
    barcode_0 = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/Barcode_0', init_state=_prop_init(0),
                               spawn=sim_utils.UsdFileCfg(usd_path=str(PROPS_DIR / 'barcode_0.usda')))
    barcode_1 = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/Barcode_1', init_state=_prop_init(1),
                               spawn=sim_utils.UsdFileCfg(usd_path=str(PROPS_DIR / 'barcode_1.usda')))
    barcode_2 = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/Barcode_2', init_state=_prop_init(2),
                               spawn=sim_utils.UsdFileCfg(usd_path=str(PROPS_DIR / 'barcode_2.usda')))
    barcode_3 = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/Barcode_3', init_state=_prop_init(3),
                               spawn=sim_utils.UsdFileCfg(usd_path=str(PROPS_DIR / 'barcode_3.usda')))
    place_tag_0 = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/PlaceTag_0', init_state=_prop_init(4),
                                 spawn=sim_utils.UsdFileCfg(usd_path=str(PROPS_DIR / 'place_tag_0.usda')))
    place_tag_1 = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/PlaceTag_1', init_state=_prop_init(5),
                                 spawn=sim_utils.UsdFileCfg(usd_path=str(PROPS_DIR / 'place_tag_1.usda')))
    place_tag_2 = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/PlaceTag_2', init_state=_prop_init(6),
                                 spawn=sim_utils.UsdFileCfg(usd_path=str(PROPS_DIR / 'place_tag_2.usda')))
    traffic_light = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/TrafficLightPost', init_state=_prop_init(7),
                                   spawn=sim_utils.UsdFileCfg(usd_path=str(PROPS_DIR / 'traffic_light_post.usda')))
    shelf_0 = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/Shelf_0', init_state=_prop_init(8), spawn=sim_utils.CuboidCfg(
        size=SHELF_SIZE, rigid_props=_KINEMATIC, collision_props=sim_utils.UsdPhysicsCollisionCfg(),
        physics_material=PhysxRigidBodyMaterialCfg(**MATERIALS['pla_wall']),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=_srgb_to_linear(SHELF_SRGB), roughness=0.8)))
    shelf_1 = shelf_0.replace(prim_path='{ENV_REGEX_NS}/Shelf_1', init_state=_prop_init(9))
    shelf_2 = shelf_0.replace(prim_path='{ENV_REGEX_NS}/Shelf_2', init_state=_prop_init(10))
    shelf_3 = shelf_0.replace(prim_path='{ENV_REGEX_NS}/Shelf_3', init_state=_prop_init(11))
    shelf_4 = shelf_0.replace(prim_path='{ENV_REGEX_NS}/Shelf_4', init_state=_prop_init(12))
    puck_0 = _puck_cfg(0, *PUCKS[0])
    puck_1 = _puck_cfg(1, *PUCKS[1])
    puck_2 = _puck_cfg(2, *PUCKS[2])
    puck_3 = _puck_cfg(3, *PUCKS[3])
    puck_4 = _puck_cfg(4, *PUCKS[4])
    puck_5 = _puck_cfg(5, *PUCKS[5])
    puck_6 = _puck_cfg(6, *PUCKS[6])
    puck_7 = _puck_cfg(7, *PUCKS[7])


@configclass
class ActionsCfg:
    # 행동 = 차체 (vx, vy, wz) 3차원뿐. 팔은 reset 때 TUCK_POSE 목표를 걸어 고정 (reset_robot_in_maze) -> 정책 출력 3차원
    base = SwerveActionCfg(asset_name='robot')


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_pose = ObsTerm(func=obs_base_pose)
        goal_in_base = ObsTerm(func=obs_goal_in_base)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        arm_joint_pos = ObsTerm(func=mdp.joint_pos_rel,
                                params={'asset_cfg': SceneEntityCfg('robot', joint_names=['arm_joint[1-5]'])})

        def __post_init__(self):
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventsCfg:
    # 재질 분리: 바퀴 = TB3 고무 타이어, 나머지 = TB3 플라스틱·금속 부품 묶음. 범위를 넓히면 그대로 DR 이 됨
    wheel_material = EventTerm(
        func=mdp.randomize_rigid_body_material, mode='startup',
        params={'asset_cfg': SceneEntityCfg('robot', body_names='v5_.*_wheel_link_0'), 'num_buckets': 1,
                **_fixed_material('rubber_wheel')})
    body_material = EventTerm(
        func=mdp.randomize_rigid_body_material, mode='startup',
        params={'asset_cfg': SceneEntityCfg('robot', body_names='^(?!v5_.*_wheel_link_0$).*'), 'num_buckets': 1,
                **_fixed_material('tb3_plastic')})
    reset_in_maze = EventTerm(func=reset_robot_in_maze, mode='reset',
                              params={'footprint_radius': FOOTPRINT_RADIUS, 'margin': 0.02})
    hold_arm = EventTerm(func=hold_arm_at_default, mode='reset')  # 팔 action 항 대신 (정책 출력 3차원)


@configclass
class TurtleNinjaMazeEnvCfg(ManagerBasedEnvCfg):
    scene: MazeSceneCfg = MazeSceneCfg(num_envs=64, env_spacing=LAYOUT['recommended_env_spacing_m'])
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventsCfg = EventsCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.dt = 0.005  # 정책 주기 20 ms (50 Hz)
        self.sim.render_interval = self.decimation


@configclass
class TurtleNinjaMazePropsEnvCfg(TurtleNinjaMazeEnvCfg):
    """미로 + 랜덤 소품 (바코드 3, 신호등 기둥 1, 짐 선반 3·4 + 빈 선반 0·1, 퍽 12). reset 마다 소품을 먼저 놓고 로봇은 선반·기둥 없는 칸에서 시작."""
    scene: MazePropsSceneCfg = MazePropsSceneCfg(num_envs=64, env_spacing=LAYOUT['recommended_env_spacing_m'])

    def __post_init__(self):
        super().__post_init__()
        self.events.reset_in_maze.func = reset_props_and_robot


def use_steer_servo(cfg: 'TurtleNinjaMazeEnvCfg', kp: float = 4.0, damping: float = 1.0):
    """조향을 속도 제한 서보로: 행동 쪽 서보 계산 켜기 + 'steer' 액추에이터를 속도 구동으로.

    속도 구동 = stiffness 0, damping = 속도 오차 -> 토크 게인 [N·m/(rad/s)].
    조향 관성(armature 0.01 + 모듈 약 1e-4 kg·m²) / damping 1.0 -> 속도 추종 시정수 약 0.01 s.
    """
    cfg.actions.base.steer_servo = True
    cfg.actions.base.steer_kp = kp
    steer = cfg.scene.robot.actuators['steer']
    steer.stiffness = 0.0
    steer.damping = damping


# ---------- 실제 촬영 배경·바닥 (photos_to_textures.py + layout_to_maze_usd.py 출력) ----------
REAL_DIR = ROOT / 'usd' / 'textures'   # 실제 사진으로 만든 파노라마·카펫 텍스처


def use_real_look(cfg: 'TurtleNinjaMazeEnvCfg', pano: str = 'pano_191331.jpg', yaw_deg: float = 0.0, intensity: float = 1800.0):
    """돔 조명을 실제 강의실 파노라마(위경도 2:1)로, 바닥에 실제 카펫 텍스처 판(보이기만)을 깐다.

    yaw_deg: 파노라마를 z 축으로 돌림 (사진 어느 방향이 미로 북쪽인지 미확인 -> 눈으로 맞춤).
    파노라마는 무한히 먼 배경이라 가까운 물체는 원근이 안 맞음. 사진 속 실제 미로 벽은 수평선 아래라 시뮬 바닥에 가려짐.
    """
    q = (0.0, 0.0, math.sin(math.radians(yaw_deg) / 2), math.cos(math.radians(yaw_deg) / 2))   # (x, y, z, w)
    cfg.scene.light = AssetBaseCfg(prim_path='/World/light', init_state=AssetBaseCfg.InitialStateCfg(rot=q),
                                   spawn=sim_utils.DomeLightCfg(intensity=intensity, texture_file=str(REAL_DIR / pano),
                                                                texture_format='latlong'))
    cfg.scene.floor = AssetBaseCfg(prim_path='/World/Floor', init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0005)),
                                   spawn=sim_utils.UsdFileCfg(usd_path=str(ROOT / 'usd' / 'floor_carpet.usda')))


# ---------- 천장 조명 (강의실 천장 사각 조명 흉내. 위치 · 크기 · 세기는 실측 아님) ----------
# 파노라마 돔만 쓰면 벽이 실제보다 어둡고 파랗고(채도 67 vs 실제 51), 비친 퍽이 검게 나옴 -> 위에서 비추는 조명 추가
# 세기 10000 + 돔 0.25–1.1 배: 벽/바닥 밝기 비 0.65 (실제 0.67), 벽 채도 56, 벽판 ior 2.6 일 때 반사율 0.13 (실제 0.13). 2026-10-27 미리보기
CEIL_LIGHT_XY = [(x, y) for x in (-1.2, 1.2) for y in (-2.0, 0.0, 2.0)]   # 미로 중심 기준 (m)
CEIL_LIGHT_SIZE = (0.6, 1.2)                                               # 사각 조명 가로 · 세로 (m)
CEIL_LIGHT_Z = 2.6
CEIL_LIGHT_INT = 10000.0
CEIL_LIGHT_COLOR = (1.0, 0.97, 0.92)                                       # 살짝 따뜻한 흰색 (형광등 · LED 가정)


def add_ceiling_lights(stage, env_origins, intensity: float = CEIL_LIGHT_INT) -> list:
    """env 마다 천장 사각 조명 (RectLight 는 로컬 -Z = 아래로 비춤). 반환: 세기 속성 목록 (장면마다 배율 바꿀 때 씀)."""
    from pxr import Gf, UsdGeom, UsdLux
    attrs = []
    for e, o in enumerate(env_origins.cpu().numpy() if hasattr(env_origins, 'cpu') else env_origins):
        for i, (x, y) in enumerate(CEIL_LIGHT_XY):
            lt = UsdLux.RectLight.Define(stage, f'/World/CeilingLights/env_{e}_{i}')
            lt.CreateWidthAttr(CEIL_LIGHT_SIZE[0]); lt.CreateHeightAttr(CEIL_LIGHT_SIZE[1])
            lt.CreateColorAttr(Gf.Vec3f(*CEIL_LIGHT_COLOR))
            attrs.append(lt.CreateIntensityAttr(float(intensity)))
            UsdGeom.XformCommonAPI(lt.GetPrim()).SetTranslate(Gf.Vec3d(float(o[0] + x), float(o[1] + y), CEIL_LIGHT_Z))
    return attrs


# ---------- 로봇 실물 색 (URDF 색은 부품 구분용 임시 색) ----------
# 손목 D435 와 뒤 받침대(PLA). 실물 사진 (2026-10-24, 휴대폰) 화소 중앙값 sRGB. 폰 자동 노출이라 실제보다 약간 밝을 수 있음
# URDF 원본: 받침대 주황 (0.97, 0.56, 0.14), D435 하늘색 (0.22, 0.68, 0.85) -> 상단 카메라 화면 아래에 크게 보임
# 팔 프레임·손가락 URDF 연회색 (0.62, 0.66, 0.70), 모터 (0.15, 0.15, 0.17) -> 실물은 둘 다 검정. 팔은 움직여 광각 화면에 들어옴
_ARM_BLACK, _MOTOR_BLACK = (28, 28, 30), (36, 36, 40)   # 팔 프레임·손가락 (검정 판금·출력물), Dynamixel 몸체 (사진 2026-10-24, 눈대중)
ROBOT_REAL_COLORS = {   # 재질 이름: (sRGB, roughness). 부품(visual) 이름 대신 재질 이름으로 찾음
    'link5_down_PLA_mat': ((220, 208, 195), 0.6),     # 크림색 PLA 받침대
    'link5_down_D435_mat': ((218, 222, 222), 0.4),    # 은색 알루미늄 D435 몸체
    **{m: (_ARM_BLACK, 0.5) for m in (
        'arm_link1__frame_mat', 'arm_link2__frame_mat', 'arm_link3__frame_mat', 'link4_up_frame_mat', 'link5_down_frame_mat',
        'gripper_crank__frame_mat', 'gripper_nut__frame_mat', 'finger_a_mat', 'finger_b_mat')},
    **{m: (_MOTOR_BLACK, 0.45) for m in (
        'arm_base_link__motor_mat', 'arm_link1__motor_mat', 'arm_link2__motor_mat', 'arm_link3__motor_mat',
        'link4_up_motor_mat', 'link5_down_motor_mat')},
    **{f'v5_{m}_steer_link_0_color': (_ARM_BLACK, 0.5) for m in ('fl', 'fr', 'rl', 'rr')},
    **{m: (_ARM_BLACK, 0.5) for m in ('bottom_half_front_mat', 'bottom_half_rear_mat')},   # 라이다 앞 · 뒤 판 (URDF 적갈색 -> 검정, 사용자 2026-10-27)   # 스워브 조향 모듈 (URDF 주황, 실물 검정 2026-10-27 사용자)
}


# ---------- 신호등 겉모습 (실제 영상에 맞춘 값, 2026-10-28) ----------
# 실제 = yolo/wide/datasets/real_frames/wide_traffic_light_0 (같은 자리 꺼짐 · 빨 · 노 · 초 프레임), 시뮬 = PathTracing, 신호등 앞 0.6 m · 높이 0.45 m
#   꺼진 렌즈 색 / 카펫 비율: 빨 (1.37, 0.36, 0.30) · 노 (1.42, 0.96, 0.29) · 초 (0.32, 0.89, 0.52) -> 시뮬 ±0.03 안
#   켜진 칸: 가운데가 거의 흰색 (실제 sRGB 약 242) -> 발광을 세게 (색은 가장자리 · 번짐에만 남음)
#   주변 번짐 (등 둘레 6–12 cm 카펫, 켜짐 / 꺼짐): 빨 R × 1.35 · 노 R × 1.14 · 초 G × 1.16 -> 시뮬 1.29 · 1.13 · 1.15 (등마다 작은 구형 조명)
TL_LAMP = {
    'red': {'off_srgb': (204, 72, 64), 'emit': (14.0, 3.5, 3.0), 'glow_color': (1.0, 0.10, 0.07), 'glow': 45000.0},
    'yellow': {'off_srgb': (214, 145, 62), 'emit': (14.0, 10.0, 3.5), 'glow_color': (1.0, 0.28, 0.05), 'glow': 9000.0},
    'green': {'off_srgb': (68, 137, 95), 'emit': (3.5, 14.0, 6.0), 'glow_color': (0.10, 1.0, 0.15), 'glow': 10500.0},
}


class TrafficLightLook:
    """신호등 한 대의 렌즈 색 · 발광 · 번짐 조명. lamps = {'red': (등 원기둥 prim 경로, 등 재질 Shader 경로), ...}, root = 조명을 붙일 Xform.
    set(색 이름 또는 None, gain) 으로 켜고 끔. 번짐 조명은 등 원기둥보다 4 mm 큰 구 (화면에는 안 보임, 주변만 비춤)."""

    def __init__(self, stage, root: str, lamps: dict):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux
        self.Gf = Gf
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy'])
        root_prim = stage.GetPrimAtPath(root)
        self.emit, self.light = {}, {}
        for c, (geom, shader) in lamps.items():
            sh = stage.GetPrimAtPath(shader)
            sh.CreateAttribute('inputs:diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*_srgb_to_linear(TL_LAMP[c]['off_srgb'])))
            self.emit[c] = sh.CreateAttribute('inputs:emissiveColor', Sdf.ValueTypeNames.Color3f)
            rng = bc.ComputeRelativeBound(stage.GetPrimAtPath(geom), root_prim).ComputeAlignedRange()
            mid, half = (rng.GetMin() + rng.GetMax()) / 2, (rng.GetMax() - rng.GetMin()) / 2
            L = UsdLux.SphereLight.Define(stage, f'{root}/tl_glow_{c}')
            L.CreateRadiusAttr(float(max(half[0], half[1])) + 0.004)
            L.CreateColorAttr(Gf.Vec3f(*TL_LAMP[c]['glow_color']))
            L.CreateIntensityAttr(0.0)
            L.AddTranslateOp().Set(Gf.Vec3d(mid))
            L.GetPrim().CreateAttribute('visibleInPrimaryRay', Sdf.ValueTypeNames.Bool).Set(False)
            self.light[c] = L.GetIntensityAttr()
        self.set(None)

    def set(self, color, gain: float = 1.0):
        for c in self.emit:
            on = c == color
            self.emit[c].Set(self.Gf.Vec3f(*(gain * v for v in TL_LAMP[c]['emit'])) if on else self.Gf.Vec3f(0.0, 0.0, 0.0))
            self.light[c].Set(gain * TL_LAMP[c]['glow'] if on else 0.0)


def add_crank_visuals(stage):
    """그리퍼 크랭크(회색) · 링크 막대(노랑)를 손가락 위치에 맞춰 움직이는 보이는 모양 (물리 무관) = 배포판 config/nt_train_kit.CrankVisuals.
    URDF 막대는 CAD 자세(58.5 mm)로 고정돼 있어 그리퍼를 16 mm 로 닫으면 막대 끝이 옆으로 튀어나와 보임 -> 카메라 쓸 때 붙임.
    env.instantiate 뒤에 한 번 부르고, 손가락을 바꾼 뒤 렌더 전에 update_crank_visuals."""
    import sys as _s
    if str(ROBOT_DIR) not in _s.path:
        _s.path.insert(0, str(ROBOT_DIR))
    from config.nt_train_kit import CrankVisuals
    return CrankVisuals(stage)


def update_crank_visuals(crank, robot):
    """env 별 손가락 a · b 평균값으로 크랭크 · 막대 자세 갱신 (바뀐 env 만 씀)."""
    ids = robot.find_joints(['gripper_finger_a_joint', 'gripper_finger_b_joint'], preserve_order=True)[0]
    crank.update(robot.data.joint_pos.torch[:, ids].mean(1).cpu().numpy())


def apply_robot_real_colors(stage) -> dict:
    """env.instantiate 뒤에 호출. 로봇 부품 재질(Material 입력값)을 실물 색으로 덮어씀. 재질별로 바꾼 개수를 돌려줌.

    로봇 USD 는 instanceable 이라 재질이 instance 안쪽(읽기 전용)에 있음 -> 재질을 가진 부품 Xform 만 instanceable 을 풀고 고침
    (보이기 전용 부품). 재질 경로 = <부품>/VisualMaterials/<재질 이름>"""
    from pxr import Gf, Sdf, Usd
    done = {name: 0 for name in ROBOT_REAL_COLORS}
    found = [q.GetPath() for q in Usd.PrimRange(stage.GetPrimAtPath('/World/envs'), Usd.TraverseInstanceProxies())
             if q.GetName() in ROBOT_REAL_COLORS and q.GetTypeName() == 'Material']
    for path in found:
        part = stage.GetPrimAtPath(path.GetParentPath().GetParentPath())
        while part and part.IsInstanceProxy():          # instance 루트까지 올라가서 풂
            part = part.GetParent()
        if part and part.IsInstance():
            part.SetInstanceable(False)
        mat = stage.GetPrimAtPath(path)
        if not mat or mat.IsInstanceProxy():
            continue
        rgb, rough = ROBOT_REAL_COLORS[mat.GetName()]
        mat.CreateAttribute('inputs:diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*_srgb_to_linear(rgb)))
        mat.CreateAttribute('inputs:roughness', Sdf.ValueTypeNames.Float).Set(rough)
        done[mat.GetName()] += 1
    return done


# ---------- 상단 광각 카메라 ----------
# URDF birdeye_camera_link (프로파일 기둥 위 브래킷, 아래로 40°·왼쪽 15.7°)의 자세를 그대로 씀.
#        링크 원점 = 카메라 기판(PCB) 면이고 렌즈가 광축(+x)으로 20.8 mm 튀어나와 있음 -> 시뮬 카메라는 렌즈 앞면(+21 mm)에 둠
#        (원점에 두면 렌즈 메시 안에서 찍어 화면이 가려짐). 배포판 README: 카메라 외부 파라미터는 미확정 (사진·실측으로 맞출 것)
BIRDEYE_LENS_FRONT_M = 0.021
BIRDEYE_K_FULL = dict(fx=853.374030, fy=852.156716, cx=675.548425, cy=371.440164)  # 팀 실측 config/camera_wide.yaml (1280x720)
BIRDEYE_D = (-0.128704, 0.012667, -0.039807, 0.022852)                               # OpenCV fisheye k1..k4


def urdf_link_pose(urdf: Path, link: str, root: str = 'base_link'):
    """URDF 고정 사슬로 root 기준 link 자세 -> (xyz 3, 회전행렬 3×3 리스트). 움직이는 관절은 0 으로 봄."""
    import xml.etree.ElementTree as ET
    joints = {j.find('child').get('link'): j for j in ET.parse(str(urdf)).getroot().findall('joint')}

    def rpy_mat(r, p, y):
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr]]
    mm = lambda a, b: [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    mv = lambda a, v: [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]
    chain = []
    while link != root:
        j = joints[link]
        chain.append(j)
        link = j.find('parent').get('link')
    R, t = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0]
    for j in reversed(chain):
        o = j.find('origin')
        xyz = [float(v) for v in ((o.get('xyz') if o is not None else None) or '0 0 0').split()]
        rpy = [float(v) for v in ((o.get('rpy') if o is not None else None) or '0 0 0').split()]
        t = [a + b for a, b in zip(t, mv(R, xyz))]
        R = mm(R, rpy_mat(*rpy))
    return t, R


def birdeye_pose_base():
    """상단 카메라 자세 (base_link 기준, world 규약: x 앞·z 위) -> (pos, quat (x, y, z, w))."""
    t, R = urdf_link_pose(ROBOT_URDF, 'birdeye_camera_link')
    pos = tuple(t[i] + R[i][0] * BIRDEYE_LENS_FRONT_M for i in range(3))
    w = math.sqrt(max(0.0, 1.0 + R[0][0] + R[1][1] + R[2][2])) / 2
    quat = ((R[2][1] - R[1][2]) / (4 * w), (R[0][2] - R[2][0]) / (4 * w), (R[1][0] - R[0][1]) / (4 * w), w)
    return pos, quat


# 상단 카메라 기둥(알루미늄 프로파일 20×20) 위치: 팔 자세 후보가 기둥과 겹치지 않게 (sim_core/arm_poses.py)
#   기둥 메시는 profile_column_link 원점이 아니라 그 링크의 (−0.110, −0.096) 에 있음 = 카메라 브래킷 링크 xy
PROFILE_XY = tuple(urdf_link_pose(ROBOT_URDF, 'profile_camera_bracket_TEMP')[0][:2])


def add_birdeye_camera(cfg: 'TurtleNinjaMazeEnvCfg', width: int = 640,
                       height: int = 360, data_types=('rgb',), fisheye: bool = True, name: str = 'birdeye'):
    """로봇 base_link 에 상단 광각 카메라를 붙임 (로봇과 같이 움직임). 실행 스크립트에서 args.enable_cameras = True 필요.

    해상도는 1280x720 의 비율로 줄여 K 를 맞춤. 카메라 1대 × env 수 만큼 렌더링 -> 8 GB 에서 env 수십 개 이내로 쓸 것.
    """
    from isaaclab.sensors import TiledCameraCfg
    from isaaclab.sim.spawners.sensors import OpenCvFisheyeDistortionCfg
    sx, sy = width / 1280.0, height / 720.0
    k = dict(fx=BIRDEYE_K_FULL['fx'] * sx, fy=BIRDEYE_K_FULL['fy'] * sy, cx=BIRDEYE_K_FULL['cx'] * sx, cy=BIRDEYE_K_FULL['cy'] * sy)
    spawn = sim_utils.PinholeCameraCfg.from_intrinsic_matrix([k['fx'], 0.0, k['cx'], 0.0, k['fy'], k['cy'], 0.0, 0.0, 1.0],
                                                             width, height, clipping_range=(0.02, 20.0))
    if fisheye:
        spawn.distortion = OpenCvFisheyeDistortionCfg(**k, k1=BIRDEYE_D[0], k2=BIRDEYE_D[1], k3=BIRDEYE_D[2],
                                                      k4=BIRDEYE_D[3], image_size=(width, height))
    pos, quat = birdeye_pose_base()
    setattr(cfg.scene, name, TiledCameraCfg(
        prim_path='{ENV_REGEX_NS}/Robot/Geometry/base_link/' + name, width=width, height=height,
        data_types=list(data_types), spawn=spawn,
        offset=TiledCameraCfg.OffsetCfg(pos=pos, rot=quat, convention='world')))


# ---------- 손목 D435 (팔 끝 arm_link5_wrist_roll 에 고정, 팔과 같이 움직임) ----------
# 광학 프레임 = URDF d435_color_optical_frame (z 앞 · x 오른쪽 · y 아래) = 컬러 렌즈. d435_cad_optical_proxy_frame(케이스 가운데)에서
# 광학 −x 로 32.5 mm (2026-10-29 실측 + 공장 외부 파라미터로 확인). 내부 파라미터 = 실물 D435 공장값 (2026-10-26, 640×480 컬러,
# serial 261322071838, 왜곡 계수 0). 시뮬 깊이는 컬러와 같은 카메라라 정렬된 깊이와 같은 뜻
D435_SIZE = (640, 480)
D435_K = dict(fx=605.9, fy=605.4, cx=324.0, cy=245.9)
D435_LINK = 'arm_link5_wrist_roll'
D435_PRIM = '{ENV_REGEX_NS}/Robot/Geometry/base_link/arm_link1/arm_link2/arm_link3/arm_link4_wrist/arm_link5_wrist_roll/d435'


def _mat_to_quat_xyzw(R) -> tuple:
    """회전행렬 -> 쿼터니언 (x, y, z, w). 대각합이 작을 때도 안정한 분기."""
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, s / 4)
    i = max(range(3), key=lambda k: R[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i][i] - R[j][j] - R[k][k]) * 2
    q = [0.0, 0.0, 0.0]
    q[i] = s / 4
    q[j] = (R[j][i] + R[i][j]) / s
    q[k] = (R[k][i] + R[i][k]) / s
    return (q[0], q[1], q[2], (R[k][j] - R[j][k]) / s)


D435_COLOR_FRAME = 'd435_color_optical_frame'   # 컬러 렌즈 광학 프레임 (URDF, proxy = 케이스 가운데에서 광학 −x 32.5 mm, 2026-10-29 실측 · 공장값 확인)


def d435_tilt_R(tilt_deg: float) -> list:
    """브래킷 각도: URDF 장착 방향에서 광학 x 축(영상 오른쪽) 둘레로 돌림. − = 영상 아래쪽(그리퍼 쪽)으로 숙임.
    광학 프레임 기준 회전행렬 (3×3 리스트, URDF 광학 R 오른쪽에 곱함). 2026-10-29 계산: −25 ~ −30° 면 쥔 퍽이 화면 아래 가운데에 보임"""
    a = math.radians(tilt_deg)
    return [[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]]


def add_wrist_d435(cfg: 'TurtleNinjaMazeEnvCfg', data_types=('rgb', 'distance_to_image_plane'), name: str = 'wrist_d435',
                   tilt_deg: float = 0.0):
    """손목 D435 컬러 카메라를 붙임 (URDF D435_COLOR_FRAME = 컬러 렌즈, tilt_deg = 브래킷 각도 d435_tilt_R).
    실행 스크립트에서 args.enable_cameras = True 필요."""
    from isaaclab.sensors import TiledCameraCfg
    t, R = urdf_link_pose(ROBOT_URDF, D435_COLOR_FRAME, root=D435_LINK)
    T = d435_tilt_R(tilt_deg)
    R = [[sum(R[i][k] * T[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    w, h = D435_SIZE
    spawn = sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
        [D435_K['fx'], 0.0, D435_K['cx'], 0.0, D435_K['fy'], D435_K['cy'], 0.0, 0.0, 1.0], w, h, clipping_range=(0.03, 10.0))
    setattr(cfg.scene, name, TiledCameraCfg(
        prim_path=D435_PRIM, width=w, height=h, data_types=list(data_types), spawn=spawn,
        update_latest_camera_pose=True,   # 팔 · 로봇이 움직이므로 pos_w · quat_w_* 를 매 프레임 갱신 (기본 False 면 처음 자세가 남음)
        offset=TiledCameraCfg.OffsetCfg(pos=tuple(t), rot=_mat_to_quat_xyzw(R), convention='ros')))


# ---------- 라이다 (LDS-03 앞 · 뒤 2개, URDF base_scan_front / base_scan_rear) ----------
# 레이캐스트 라이다: 스캔 평면 한 줄, 360°. 미로 벽 · 기둥 · 선반 · 퍽 · 신호등 기둥에 맞음. 로봇 자기 몸체에는 안 맞음
#   (실물은 몸체에 가려지는 각도가 있음). 각 분해능 · 최대 거리는 가정값 (LDS-03 사양 확인 안 함)
LIDAR_RES_DEG = 1.0
LIDAR_MAX_M = 8.0


def add_lidars(cfg: 'TurtleNinjaMazeEnvCfg', debug_vis: bool = True):
    """앞 · 뒤 라이다를 base_link 에 붙임 (cfg.scene.lidar_front / lidar_rear). debug_vis 면 맞은 점을 창에 표시."""
    from isaaclab.sensors import MultiMeshRayCasterCfg, patterns
    T = MultiMeshRayCasterCfg.RaycastTargetCfg
    targets = [T(prim_expr='{ENV_REGEX_NS}/Maze', track_mesh_transforms=False)]
    if hasattr(cfg.scene, 'shelf_0'):   # 소품 환경: reset 마다 움직이므로 위치 추적
        targets += [T(prim_expr='{ENV_REGEX_NS}/Shelf_[0-9]+'), T(prim_expr='{ENV_REGEX_NS}/Puck_[0-9]+'),   # '.*' 면 하위 prim 까지 맞음
                    T(prim_expr='{ENV_REGEX_NS}/TrafficLightPost')]
    for side in ('front', 'rear'):
        t, R = urdf_link_pose(ROBOT_URDF, f'base_scan_{side}')
        setattr(cfg.scene, f'lidar_{side}', MultiMeshRayCasterCfg(
            prim_path='{ENV_REGEX_NS}/Robot/Geometry/base_link', mesh_prim_paths=targets,   # 레이캐스터는 있는 링크에 붙고 offset 으로 옮김
            offset=MultiMeshRayCasterCfg.OffsetCfg(pos=tuple(t), rot=_mat_to_quat_xyzw(R)), ray_alignment='base',
            pattern_cfg=patterns.LidarPatternCfg(channels=1, vertical_fov_range=(0.0, 0.0),
                                                 horizontal_fov_range=(-180.0, 180.0), horizontal_res=LIDAR_RES_DEG),
            max_distance=LIDAR_MAX_M, debug_vis=debug_vis))


def set_physics_backend(cfg: 'TurtleNinjaMazeEnvCfg', name: str | None):
    """물리 엔진 선택: None/'physx' = PhysX (기본), 'newton_mjwarp' = Newton + MuJoCo-Warp 솔버."""
    if name in (None, 'physx'):
        return
    if name == 'newton_mjwarp':
        from isaaclab_newton.physics import NewtonCfg  # 이 함수가 불릴 때만 import
        cfg.sim.physics = NewtonCfg()
        return
    raise ValueError(f'모르는 물리 엔진: {name}')
