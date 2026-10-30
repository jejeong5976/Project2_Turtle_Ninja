#!/usr/bin/env python3
"""주사위 기억 미션 경기장 (발제 v5) 띄우기. 맵 · 퍽은 tools/make_dice_map.py 로 먼저 만듦.

예) OMNI_KIT_ACCEPT_EULA=YES env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python scripts/view_dice.py --viz kit --keyboard --real
    --keyboard   키보드 조종 (↑↓ 앞뒤, ←→ 옆, Z/X 회전, L 정지, N 처음 배치로). Isaac 창 viewport 를 한 번 클릭
    --signal_delay S   시작 S 초 뒤 신호등 점멸 시작 (문제 파일 순서, 기본 5 s). 음수면 점멸 안 함. --signal 로 순서 바꿈
    --camera     상단 광각(birdeye) + 손목 D435 붙이기. --real 실제 배경 · 천장 조명 · 벽판 반사, --render pt PathTracing
    --snap DIR   창 없이 확인용: 출발점에서 광각 영상을 신호 켜짐 · 꺼짐 때 저장하고, 퍽이 박스 위에 그대로 있는지 출력 후 끝
                 (--viz 없이 실행 = 창 없음)
"""
import argparse
import json
import sys
from pathlib import Path as _P
sys.path.insert(0, str(_P(__file__).resolve().parents[1]))   # sim/robot_sim -> sim_core

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
parser.add_argument('--num_envs', type=int, default=1)
parser.add_argument('--keyboard', action='store_true')
parser.add_argument('--camera', action='store_true')
parser.add_argument('--real', action='store_true')
parser.add_argument('--pano', default='pano_191331.jpg')
parser.add_argument('--lidar', action='store_true')
parser.add_argument('--render', choices=['rt', 'pt'], default='rt')
parser.add_argument('--signal', default=None, help='점멸 순서 R/Y/G 12개 (기본 = dice_layout.json 의 문제)')
parser.add_argument('--signal_delay', type=float, default=5.0)
parser.add_argument('--snap', default=None, help='확인용 이미지 저장 폴더 (창 없이)')
add_launcher_args(parser)
args = parser.parse_args()
if args.camera or args.snap:
    args.enable_cameras = True

import torch

from isaaclab.utils import instantiate
from isaaclab.visualizers import VisualizerCfg

from sim_core import dice_mission as dm
from sim_core import turtleninja_maze_env as tme

seq = args.signal or dm.LAYOUT['problem']['signal']
assert len(seq) == 12 and set(seq) <= set('RYG'), f'신호 순서 오류: {seq}'
cfg = dm.DiceMissionEnvCfg()
cfg.scene.num_envs = args.num_envs
if args.real:
    tme.use_real_look(cfg, args.pano)
if args.camera or args.snap:
    tme.add_birdeye_camera(cfg, width=1280, height=720)
    tme.add_wrist_d435(cfg, data_types=('rgb',))
if args.lidar:
    tme.add_lidars(cfg)
cfg.sim.default_visualizer_cfg = VisualizerCfg(eye=(0.0, -6.5, 9.0), lookat=(0.0, 0.0, 0.0))

with launch_simulation(cfg, args):
    env = instantiate(cfg)
    import omni.usd
    from pxr import Sdf
    stage = omni.usd.get_context().get_stage()
    n, dev = env.num_envs, env.device
    dt = cfg.sim.dt * cfg.decimation
    if args.render == 'pt':
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
                sh.CreateAttribute('inputs:ior', Sdf.ValueTypeNames.Float).Set(2.6)
                sh.CreateAttribute('inputs:roughness', Sdf.ValueTypeNames.Float).Set(0.08)
    tme.apply_robot_real_colors(stage)
    crank = tme.add_crank_visuals(stage)
    player = dm.SignalPlayer(stage, n, seq)
    action = torch.zeros(n, env.action_manager.total_action_dim, device=dev)
    kb, want_reset, last = None, [False], [None]
    if args.keyboard:
        from isaaclab.devices import Se2Keyboard, Se2KeyboardCfg
        kb = Se2Keyboard(Se2KeyboardCfg(v_x_sensitivity=1.0, v_y_sensitivity=1.0, omega_z_sensitivity=1.0, sim_device=dev))
        kb.add_callback('N', lambda: want_reset.__setitem__(0, True))
        print(kb, '\n\t처음 배치로: N', flush=True)

    def do_reset():
        env.reset()
        if args.signal_delay >= 0:
            player.start(args.signal_delay)

    m = dm.MISSION
    print(f"[dice] 신호 {seq} (시작 {args.signal_delay} s 뒤, {player.duration:.0f} s), 바코드 {m['barcode']['code']}, "
          f"로봇 출발 {m['start_pose']['arena_mm_deg']} (경기장 mm, 도)", flush=True)
    do_reset()
    t, step = 0.0, 0
    snaps = []
    if args.snap:   # 저장 시각: 신호 전, 첫 점멸 켜짐 가운데, 꺼짐 가운데, 두 번째 켜짐
        d = args.signal_delay
        snaps = [(max(1.0, d - 1.0), 'before'), (d + 0.5, f'on1_{seq[0]}'), (d + 1.5, 'off1'), (d + 2.5, f'on2_{seq[1]}')]
        out = _P(args.snap)
        out.mkdir(parents=True, exist_ok=True)
        import cv2
    while env.sim.is_running():
        with torch.inference_mode():
            if want_reset[0]:
                do_reset()
                t, want_reset[0] = 0.0, False
            if kb is not None:
                action[:, :3] = kb.advance().clamp(-1, 1)
                cmd = tuple(round(float(v), 2) for v in action[0, :3])
                if cmd != last[0]:
                    print(f'[keyboard] vx {cmd[0]:+.1f} vy {cmd[1]:+.1f} wz {cmd[2]:+.1f}', flush=True)
                    last[0] = cmd
            c = player.update(t)
            env.step(action)
            tme.update_crank_visuals(crank, env.scene['robot'])
            t += dt
            step += 1
            if snaps and t >= snaps[0][0]:
                _, tag = snaps.pop(0)
                for _ in range(3):   # 재질 바뀐 뒤 렌더 반영
                    env.sim.render()
                for cam in ('birdeye', 'wrist_d435'):
                    rgb = env.scene[cam].data.output['rgb'][0, ..., :3].cpu().numpy()
                    cv2.imwrite(str(out / f'{cam}_{tag}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                print(f'[snap] t {t:.2f} s, 등 {c}, 저장 {tag}', flush=True)
                if not snaps:
                    pucks = dm.puck_states(env)
                    bad = [p for p in pucks if p['tilt_deg'] > 5 or abs(p['arena_mm'][2] - 100) > 5]
                    (out / 'pucks.json').write_text(json.dumps(pucks, indent=1, ensure_ascii=False))
                    rp = (env.scene['robot'].data.root_pos_w.torch[0] - env.scene.env_origins[0]).tolist()
                    o = dm.LAYOUT['arena_origin_in_sim_m']
                    print(f'[snap] 로봇 ({(rp[0] - o[0]) * 1000:.0f}, {(rp[1] - o[1]) * 1000:.0f}) mm, '
                          f'퍽 12개 중 박스 위 똑바로 {12 - len(bad)}개, 이상 {bad}', flush=True)
                    break
    env.close()
