#!/usr/bin/env python3
"""Isaac Sim으로 병렬 미로 환경 만들기

예) OMNI_KIT_ACCEPT_EULA=YES ~/pa-course/IsaacLab-6.2/.venv/bin/python scripts/view_maze.py --viz kit
    --num_envs 4 (기본), --arm_extended (팔을 q=0 으로 편 자세), --still (차체 정지),
    --camera (상단 광각 40° 카메라 birdeye + 손목 D435 wrist_d435, Isaac Sim 창 카메라 메뉴에서 고름), --props (랜덤 소품: 퍽 8·선반 5·장소 태그 3·바코드 4·신호등, reset 마다 새 배치)
    --map (칸 번호·벽 지도만 출력하고 끝), --shelf 'CELL:SIDE[:GAP[:ALONG[:YAW]]]' (여러 번 가능, 선반 고정 배치),
    --robot 'CELL[:DX:DY:YAW]' (로봇 시작 자리 고정). 형식은 fixed_layout.py 설명 참고. 예)
      scripts/view_maze.py --viz kit --props --num_envs 1 --keyboard --shelf 8:N:0.10 --robot 7:0:0:0
    --keyboard (키보드 조종: ↑↓ 앞뒤, ←→ 좌우 횡이동, Z/X 회전, L 정지, N 새 배치. Isaac 창 viewport 를 한 번 클릭해 포커스)
    로봇 부품 색은 항상 실물 색 (tme.apply_robot_real_colors, 데이터 생성기와 같음).
    --real 이면 데이터 생성기와 같은 겉모습: 파노라마 · 카펫 + 천장 사각 조명 6개 + 벽판 유광 반사 (범위 가운데 값으로 고정)
    --render pt 면 PathTracing (데이터 생성기 기본). 움직이는 동안은 노이즈가 많고 멈추면 깨끗해짐
"""
import argparse
import sys
from pathlib import Path as _P
sys.path.insert(0, str(_P(__file__).resolve().parents[1]))   # sim/robot_sim -> sim_core 패키지

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
parser.add_argument('--num_envs', type=int, default=4)
parser.add_argument('--arm_extended', action='store_true', help='팔을 접은 자세 대신 q=0 으로')
parser.add_argument('--still', action='store_true', help='차체 명령 0')
parser.add_argument('--props', action='store_true', help='랜덤 소품 환경 (TurtleNinjaMazePropsEnvCfg)')
parser.add_argument('--camera', action='store_true', help='상단 광각 카메라(birdeye) + 손목 D435(wrist_d435) 붙이기')
parser.add_argument('--map', action='store_true', help='칸 번호·벽 지도 출력 후 종료')
parser.add_argument('--shelf', action='append', default=[], help="선반 고정 'CELL:SIDE[:GAP[:ALONG[:YAW]]]' (--props 필요)")
parser.add_argument('--robot', default=None, help="로봇 시작 고정 'CELL[:DX:DY:YAW]' (--props 필요)")
parser.add_argument('--keyboard', action='store_true', help='키보드로 차체 조종 (자동 reset 끔)')
parser.add_argument('--reset_s', type=float, default=30.0, help='이 시간마다 reset (새 시작 칸)')
parser.add_argument('--real', action='store_true', help='실제 강의실 파노라마 배경 + 카펫 바닥 (tme.use_real_look)')
parser.add_argument('--pano', default='pano_191331.jpg', help='usd/textures/ 안 파노라마 파일')
parser.add_argument('--dome_yaw', type=float, default=0.0, help='파노라마 회전 (도)')
parser.add_argument('--lidar', action='store_true', help='앞 · 뒤 라이다(레이캐스트) 붙이고 맞은 점을 창에 표시')
parser.add_argument('--render', choices=['rt', 'pt'], default='rt',
                    help='rt = Real-Time (기본, 조작용), pt = PathTracing (make_yolo_dataset 기본, spp 16 + OptiX 디노이저)')
parser.add_argument('--panel_ior', type=float, default=2.6, help='--real 벽판 굴절률 (make_yolo_dataset 범위 2.2–3.0, 실제 반사율 0.13 = 2.6)')
parser.add_argument('--panel_rough', type=float, default=0.08, help='--real 벽판 거칠기 (make_yolo_dataset 범위 0.02–0.15)')
add_launcher_args(parser)
args = parser.parse_args()
if args.map:
    from sim_core import fixed_layout as fl
    print(fl.maze_map())
    raise SystemExit
fixed = bool(args.shelf or args.robot)
if fixed and not args.props:
    parser.error('--shelf / --robot 은 --props 와 같이 써야 함')
if args.camera:
    args.enable_cameras = True

import torch

from isaaclab.utils import instantiate
from isaaclab.visualizers import VisualizerCfg

from sim_core import fixed_layout as fl
from sim_core import turtleninja_maze_env as tme

cfg = tme.TurtleNinjaMazePropsEnvCfg() if args.props else tme.TurtleNinjaMazeEnvCfg()
cfg.scene.num_envs = args.num_envs
if args.real:
    tme.use_real_look(cfg, args.pano, args.dome_yaw)
if args.camera:
    tme.add_birdeye_camera(cfg)
    tme.add_wrist_d435(cfg, data_types=('rgb',))   # 보기용이라 깊이는 안 뽑음
if args.lidar:
    tme.add_lidars(cfg)
if args.arm_extended:
    cfg.scene.robot.init_state.joint_pos = {'.*': 0.0, 'arm_joint1': tme.J1_FORWARD}
    cfg.events.reset_in_maze.params['footprint_radius'] = 0.29
cfg.sim.default_visualizer_cfg = VisualizerCfg(eye=(4.0, -9.0, 9.0), lookat=(0.0, 0.0, 0.0))

with launch_simulation(cfg, args):
    env = instantiate(cfg)
    import omni.usd
    from pxr import Sdf
    stage = omni.usd.get_context().get_stage()
    n, dev = env.num_envs, env.device
    if args.render == 'pt':
        # 렌더러가 뜬 뒤에 바꿔야 먹음 (make_yolo_dataset 과 같은 값)
        import carb
        _cs = carb.settings.get_settings()
        for _k, _v in (('/rtx/rendermode', 'PathTracing'), ('/rtx/pathtracing/spp', 16),
                       ('/rtx/pathtracing/totalSpp', 64), ('/rtx/pathtracing/optixDenoiser/enabled', True)):
            _cs.set(_k, _v)
    if args.real:
        tme.add_ceiling_lights(stage, env.scene.env_origins)
        for e in range(n):
            sh = stage.GetPrimAtPath(f'/World/envs/env_{e}/Maze/Looks/panel/Surface')
            if sh:
                sh.CreateAttribute('inputs:ior', Sdf.ValueTypeNames.Float).Set(args.panel_ior)
                sh.CreateAttribute('inputs:roughness', Sdf.ValueTypeNames.Float).Set(args.panel_rough)
    colors = tme.apply_robot_real_colors(stage)
    crank = tme.add_crank_visuals(stage)   # 그리퍼 크랭크 · 막대를 손가락 위치에 맞춤 (배포판 CrankVisuals)
    print(f'[view_maze] 렌더 {args.render}, 로봇 실물 색 재질 {sum(colors.values())}개'
          + (f', 천장 조명 · 벽판 반사 ior {args.panel_ior} 거칠기 {args.panel_rough}' if args.real else ''), flush=True)
    action = torch.zeros(n, env.action_manager.total_action_dim, device=dev)
    reset_steps = int(args.reset_s / (cfg.sim.dt * cfg.decimation))
    kb, want_reset, last_cmd = None, [False], [None]
    if args.keyboard:
        from isaaclab.devices import Se2Keyboard, Se2KeyboardCfg
        # 행동은 정규화 [-1, 1] -> 1.0 = 최대 0.10 m/s · 0.5 rad/s. 대각선은 두 키 동시에
        kb = Se2Keyboard(Se2KeyboardCfg(v_x_sensitivity=1.0, v_y_sensitivity=1.0, omega_z_sensitivity=1.0, sim_device=dev))
        kb.add_callback('N', lambda: want_reset.__setitem__(0, True))
        reset_steps = 10 ** 12
        print(kb, '\n\tNew random layout: N', flush=True)
    count = 0
    shelves = [fl.parse_shelf(sp) for sp in args.shelf]
    robot_fix = fl.parse_robot(args.robot) if args.robot else None
    if fixed:
        print(fl.maze_map())
        print('[view_maze] 고정 배치\n' + fl.describe(shelves), flush=True)

    def do_reset():
        env.reset()
        if fixed:
            fl.apply(env, shelves, robot_fix)

    do_reset()
    print(f'[view_maze] envs {n}, 미로 {tme.MAZE_USD.name}, 로봇 {tme.ROBOT_URDF}. 창을 닫으면 끝', flush=True)
    while env.sim.is_running():
        with torch.inference_mode():
            if (count and count % reset_steps == 0) or want_reset[0]:
                do_reset()
                want_reset[0] = False
            if kb is not None:
                action[:, :3] = kb.advance().clamp(-1, 1)
                cmd = tuple(round(float(v), 2) for v in action[0, :3])
                if cmd != last_cmd[0]:   # 키 입력 확인용: 명령이 바뀔 때만 한 줄 (vx, vy, wz 정규화 -1..1)
                    xy = (env.scene['robot'].data.root_pos_w.torch[0, :2] - env.scene.env_origins[0, :2]).tolist()
                    print(f'[keyboard] 명령 vx {cmd[0]:+.1f} vy {cmd[1]:+.1f} wz {cmd[2]:+.1f}  로봇 위치 ({xy[0]:+.3f}, {xy[1]:+.3f}) m', flush=True)
                    last_cmd[0] = cmd
            elif count % 250 == 0 and not args.still:  # 5 s 마다 새 무작위 명령 (조향에 최대 1.6 s)
                action[:, :3] = torch.rand(n, 3, device=dev) * 2 - 1
            env.step(action)
            tme.update_crank_visuals(crank, env.scene['robot'])
            count += 1
    env.close()
