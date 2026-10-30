#!/usr/bin/env python3
"""랜덤 소품 미로를 Isaac Sim(RTX)으로 렌더링해 그림으로 저장 (보고서용). 창 없이 실행.

카메라 3대 × env 2개: 미로 전체(비스듬히 위에서), 신호등 기둥 근접, 선반·퍽 근접. 근접 카메라는 로봇 상단 카메라 높이(0.62 m)에서 봄.
예) OMNI_KIT_ACCEPT_EULA=YES ~/pa-course/IsaacLab-6.2/.venv/bin/python scripts/snapshot_props.py --real --out results/snapshots
"""
import argparse
import sys
from pathlib import Path as _P
sys.path.insert(0, str(_P(__file__).resolve().parents[1]))   # sim/robot_sim -> sim_core 패키지
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
parser.add_argument('--num_envs', type=int, default=2)
parser.add_argument('--out', default='results/snapshots')
parser.add_argument('--seed', type=int, default=3)
parser.add_argument('--real', action='store_true', help='실제 강의실 파노라마 배경 + 카펫 바닥 (tme.use_real_look)')
parser.add_argument('--pano', default='pano_191331.jpg', help='usd/textures/ 안 파노라마 파일')
parser.add_argument('--dome_yaw', type=float, default=0.0, help='파노라마 회전 (도)')
add_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True

import math

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import instantiate
from isaaclab.utils.math import quat_from_matrix

from sim_core import turtleninja_maze_env as tme

W, H = 960, 600  # 1280×800 × 카메라 12대는 8 GB 에서 렌더 메모리 부족
torch.manual_seed(args.seed)
cfg = tme.TurtleNinjaMazePropsEnvCfg()
cfg.scene.num_envs = args.num_envs
if args.real:
    tme.use_real_look(cfg, args.pano, args.dome_yaw)
cam = lambda fov_deg: sim_utils.PinholeCameraCfg(focal_length=10.0, horizontal_aperture=2 * 10.0 * math.tan(math.radians(fov_deg) / 2),
                                                 clipping_range=(0.02, 30.0))
cfg.scene.cam_overview = TiledCameraCfg(prim_path='{ENV_REGEX_NS}/CamOverview', width=W, height=H, data_types=['rgb'], spawn=cam(60))
cfg.scene.cam_tl = TiledCameraCfg(prim_path='{ENV_REGEX_NS}/CamTL', width=W, height=H, data_types=['rgb'], spawn=cam(70))
cfg.scene.cam_shelf = TiledCameraCfg(prim_path='{ENV_REGEX_NS}/CamShelf', width=W, height=H, data_types=['rgb'], spawn=cam(70))


def look_at(eye: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    """광학 좌표계(x 오른쪽, y 아래, z 앞) 회전행렬 (N, 3, 3)."""
    z = torch.nn.functional.normalize(target - eye, dim=-1)
    up = torch.tensor([0.0, 0.0, 1.0], device=eye.device).expand_as(z)
    x = torch.nn.functional.normalize(torch.cross(z, up, dim=-1), dim=-1)
    y = torch.cross(z, x, dim=-1)
    return torch.stack([x, y, z], dim=-1)


def view_from_front(env, name, dist, eye_z, target_z):
    """소품 앞(칸 안쪽 방향) dist 만큼, 높이 eye_z 에서 소품(target_z)을 봄. 약간 옆으로 비껴서."""
    a = env.scene[name]
    p = a.data.root_pos_w.torch
    q = a.data.root_quat_w.torch  # (x, y, z, w)
    yaw = torch.atan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
    d = torch.stack([torch.cos(yaw + 0.35), torch.sin(yaw + 0.35), torch.zeros_like(yaw)], -1)
    target = p.clone(); target[:, 2] = target_z
    eye = p + d * dist; eye[:, 2] = eye_z
    return eye, look_at(eye, target)


out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
with launch_simulation(cfg, args):
    env = instantiate(cfg)
    n, dev = env.num_envs, env.device
    action = torch.zeros(n, env.action_manager.total_action_dim, device=dev)
    with torch.inference_mode():
        env.reset()
        for _ in range(25):  # 퍽 내려앉기 0.5 s
            env.step(action)
        org = env.scene.env_origins
        eye = org + torch.tensor([0.0, -3.6, 3.4], device=dev)
        poses = {'cam_overview': (eye, look_at(eye, org + torch.tensor([0.0, 0.1, 0.0], device=dev))),
                 'cam_tl': view_from_front(env, 'traffic_light', 0.9, 0.62, 0.30),
                 'cam_shelf': view_from_front(env, 'shelf_0', 0.7, 0.62, 0.08)}
        for name, (pos, R) in poses.items():
            env.scene[name].set_world_poses(positions=pos, orientations=quat_from_matrix(R), convention='ros')
        for _ in range(10):  # 렌더링 갱신 (RTX 누적)
            env.step(action)
        from PIL import Image
        for name in poses:
            rgb = env.scene[name].data.output['rgb'][..., :3].cpu().numpy().astype(np.uint8)  # (N, H, W, 3)
            for e in range(n):
                Image.fromarray(rgb[e]).save(out / f'isaac_{name[4:]}_env{e}.png')
            if n >= 2:  # env 0·1 나란히
                Image.fromarray(np.concatenate(rgb[0:2], 1)).save(out / f'isaac_{name[4:]}_pair.png')
        print('[snapshot] saved to', out.resolve(), flush=True)
    env.close()
