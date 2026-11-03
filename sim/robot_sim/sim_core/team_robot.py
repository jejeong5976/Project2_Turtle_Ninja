"""팀 URDF 로봇 (ros2_ws/src/description = 실물) 으로 sim_core 환경 쓰기.

켜기: 환경변수 TN_ROBOT=team (sim_core 를 불러오기 전에). turtleninja_maze_env 가 끝에서 install() 을 부름 → 모든 스크립트 · 학습에 같이 적용
먼저 한 번: tools/convert_team_robot.py (팀 URDF → 시뮬용 URDF · 스워브 설정 · USD)

바뀌는 것 (값은 저장소 설정에서 읽음)
  로봇 USD · URDF          팀 URDF 변환본. 루트 = base_footprint (바닥), base_link 는 그 0.033 m 위
  관절 이름                arm_joint1~5 → joint1~5, gripper_finger_a/b → gripper_left/right_joint, 바퀴 링크 wheel_<fl|fr|rl|rr>_link
  팔 접은 자세 (TUCK_POSE)  config/control.yaml poses.home (실물 받침 자세)
  팔 힘 · 속도 한계         팀 URDF 관절 limit (XM430 4.1 N·m · 4.8 rad/s)
  스워브 모듈 위치          config/controllers.yaml (앞 +0.060 · 뒤 −0.070 m), 조향 범위 = 팀 URDF
  카메라 · 라이다           팀 URDF 프레임 (손목 wrist_cam_color_optical_frame, 상단 birdeye_camera_link, 라이다 base_scan_*)
안 바뀌는 것: 행동 (vx, vy, wz) · 학습용 속도 한계 (swerve.yaml limits_for_training) · 미로 · 소품 · 보상
"""
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]           # sim/robot_sim
REPO = ROOT.parents[1]
TEAM_DIR = ROOT / 'robot_description' / 'team'
TEAM_URDF = TEAM_DIR / 'robot.urdf'
TEAM_USD = ROOT / 'usd' / 'team_robot' / 'robot' / 'robot.usda'
BASE_Z = 0.033                                       # base_link 높이 (base_footprint 기준)
SPAWN_Z = 0.003                                      # 루트 = base_footprint (바퀴 바닥) + 3 mm
ARM = [f'joint{i}' for i in range(1, 6)]
GRIP = ['gripper_left_joint', 'gripper_right_joint']
WRIST_PRIM = '{ENV_REGEX_NS}/Robot/Geometry/base_footprint/link2/link3/link4/link5/wrist_link'
BASE_PRIM = '{ENV_REGEX_NS}/Robot/Geometry/base_footprint'


def _home():
    q = yaml.safe_load((REPO / 'config' / 'control.yaml').read_text())['control_master']['ros__parameters']['poses']['home']
    return {**dict(zip(ARM, q)), **{g: 0.0 for g in GRIP}}


def _arm_limits():
    import xml.etree.ElementTree as ET
    lim = {j.get('name'): j.find('limit') for j in ET.parse(TEAM_URDF).getroot().findall('joint') if j.find('limit') is not None}
    return {n: float(lim[n].get('effort')) for n in ARM}, {n: float(lim[n].get('velocity')) for n in ARM}


def install(tme):
    """turtleninja_maze_env 모듈을 팀 로봇으로 바꿈 (한 번만)."""
    if getattr(tme, 'TEAM_ROBOT_INSTALLED', False):
        return
    for p in (TEAM_URDF, TEAM_USD, TEAM_DIR / 'config' / 'swerve.yaml'):
        if not p.exists():
            raise FileNotFoundError(f'{p} 없음 → 먼저 tools/convert_team_robot.py 실행 (TN_ROBOT=team)')
    tme.TEAM_ROBOT_INSTALLED = True
    home = _home()
    effort, vel = _arm_limits()

    # 파일 · 자세 · 스워브 설정
    tme.ROBOT_URDF, tme.ROBOT_USD, tme.ROBOT_DIR = TEAM_URDF, TEAM_USD, TEAM_DIR   # ROBOT_DIR: SwerveAction 이 config/swerve.yaml 을 읽는 곳
    tme.SPAWN_Z = SPAWN_Z
    tme.TUCK_POSE = home
    sw = yaml.safe_load((TEAM_DIR / 'config' / 'swerve.yaml').read_text())
    g, r = sw['geometry'], sw['steer_range_rad']
    tme.SWERVE_MODULES = {m: (g['module_x'][i], g['module_y'][i], g['module_offset'][i], r['min'][i], r['max'][i])
                          for i, m in enumerate(sw['joints']['module_names'])}

    # URDF 자세: 팀 URDF 는 base_link 가 base_footprint 아래 → base_link 기준 요청은 base_footprint 기준으로 구해 0.033 m 내림
    ulp = tme.urdf_link_pose

    def urdf_link_pose(urdf, link, root='base_link'):
        if Path(urdf) == TEAM_URDF and root == 'base_link':
            t, R = ulp(urdf, link, root='base_footprint')
            return [t[0], t[1], t[2] - BASE_Z], R
        return ulp(urdf, link, root)
    tme.urdf_link_pose = urdf_link_pose
    tme.PROFILE_XY = tuple(urdf_link_pose(TEAM_URDF, 'profile_camera_bracket_link')[0][:2])

    # 손목 D435: wrist_link 에 붙음 (팀 USD 경로)
    tme.D435_LINK, tme.D435_PRIM, tme.D435_COLOR_FRAME = 'wrist_link', WRIST_PRIM + '/d435', 'wrist_cam_color_optical_frame'

    # 상단 카메라 · 라이다: 팀 USD 에는 base_link 몸체가 없음 (base_footprint 에 합쳐짐) → 거기에 붙이고 0.033 m 올림
    def _on_base(sensor):
        sensor.prim_path = sensor.prim_path.replace('/Geometry/base_link', '/Geometry/base_footprint')
        p = list(sensor.offset.pos)
        p[2] += BASE_Z
        sensor.offset.pos = tuple(p)

    add_birdeye, add_lidars = tme.add_birdeye_camera, tme.add_lidars

    def add_birdeye_camera(cfg, *a, name='birdeye', **k):
        add_birdeye(cfg, *a, name=name, **k)
        _on_base(getattr(cfg.scene, name))

    def add_lidars_team(cfg, debug_vis=True):
        add_lidars(cfg, debug_vis)
        for side in ('front', 'rear'):
            _on_base(getattr(cfg.scene, f'lidar_{side}'))
    tme.add_birdeye_camera, tme.add_lidars = add_birdeye_camera, add_lidars_team

    # 팔 고정 이벤트 (팀 관절 이름), 예전 로봇 그리퍼 크랭크 보기 끔
    def hold_arm_at_default(env, env_ids, asset_cfg=tme.SceneEntityCfg('robot')):
        import torch
        asset = env.scene[asset_cfg.name]
        if isinstance(env_ids, slice):
            env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
        ids = asset.find_joints(ARM + GRIP, preserve_order=True)[0]
        value = asset.data.default_joint_pos.torch[env_ids][:, ids]
        asset.actuators.target_command.set_position_index(value=value, joint_ids=ids, env_ids=env_ids)
    tme.hold_arm_at_default = hold_arm_at_default
    tme.add_crank_visuals = lambda *a, **k: None
    tme.update_crank_visuals = lambda *a, **k: None

    # 환경 설정: __post_init__ 끝에서 apply. configclass 는 클래스를 만들 때 부모 __post_init__ 을 합쳐 두므로
    #   이미 있는 클래스는 하나씩 바꾸고, 뒤에 만드는 하위 클래스 (dice_mission · 학습 태스크) 는 바뀐 기본을 물려받음
    def wrap(cls):
        post = cls.__post_init__

        def __post_init__(self):
            post(self)
            apply(self, tme, home, effort, vel)
        cls.__post_init__ = __post_init__
    for cls in (tme.TurtleNinjaMazeEnvCfg, tme.TurtleNinjaMazePropsEnvCfg):
        wrap(cls)


def apply(cfg, tme, home, effort, vel):
    """환경 설정 하나를 팀 로봇으로 (install 이 부름)."""
    SceneEntityCfg = tme.SceneEntityCfg
    rob = cfg.scene.robot
    rob.spawn.usd_path = str(TEAM_USD)
    pos = list(rob.init_state.pos)
    pos[2] = SPAWN_Z
    rob.init_state.pos = tuple(pos)
    rob.init_state.joint_pos = {**{k: v for k, v in rob.init_state.joint_pos.items() if k.startswith(('steer', 'drive'))}, **home}
    arm, grip = rob.actuators['arm'], rob.actuators['gripper']
    arm.joint_names_expr, arm.joint_effort_limit, arm.joint_velocity_limit = ['joint[1-5]'], effort, vel
    grip.joint_names_expr = ['gripper_(left|right)_joint']
    ev = cfg.events
    ev.hold_arm.func = tme.hold_arm_at_default
    ev.wheel_material.params['asset_cfg'] = SceneEntityCfg('robot', body_names='wheel_.*_link')
    ev.body_material.params['asset_cfg'] = SceneEntityCfg('robot', body_names='^(?!wheel_.*_link$).*')
    pol = cfg.observations.policy
    if getattr(pol, 'arm_joint_pos', None) is not None:
        pol.arm_joint_pos.params['asset_cfg'] = SceneEntityCfg('robot', joint_names=['joint[1-5]'])


def use_real_speed_limits(cfg):
    """바퀴 속도 · 가속을 실물 설정 (config/controllers.yaml) 으로. 기본 (학습용 하드웨어 끝 0.26 m/s) 대신 실물과 같게 볼 때만."""
    p = yaml.safe_load((REPO / 'config' / 'controllers.yaml').read_text())['swerve_controller']['ros__parameters']
    b = cfg.actions.base
    b.max_linear, b.max_angular = p['max_linear'], p['max_angular']
    b.max_linear_accel, b.max_angular_accel = p['max_linear_accel'], p['max_angular_accel']
    return cfg
