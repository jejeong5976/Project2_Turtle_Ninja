#!/usr/bin/env python3
"""팀 로봇 (TN_ROBOT=team) 확인: 주사위 경기장에서 1 s 정지 → 앞으로 3 s 주행 → 옆으로 2 s.
팔이 접은 자세를 유지하는지, 차체가 명령대로 움직이는지, 라이다 · 상단 카메라 · 손목 D435 가 나오는지 봄. 사진은 --out 에.

  OMNI_KIT_ACCEPT_EULA=YES $ISAACLAB_PY tools/check_team_robot.py --out results/team_robot_check
"""
import argparse
import os
import sys
from pathlib import Path

os.environ['TN_ROBOT'] = 'team'
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.app import add_launcher_args, launch_simulation   # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--out', default=str(ROOT / 'results' / 'team_robot_check'))
add_launcher_args(ap)
args = ap.parse_args()
args.enable_cameras = True


def main():
    import math
    import numpy as np
    import torch
    from isaaclab.utils import instantiate
    from sim_core import turtleninja_maze_env as tme
    from sim_core import dice_mission as dm

    cfg = dm.DiceMissionEnvCfg()
    cfg.scene.num_envs = 1
    tme.add_lidars(cfg, debug_vis=False)
    tme.add_birdeye_camera(cfg, width=640, height=360)
    cfg.scene.birdeye.update_latest_camera_pose = True   # 위치 값도 매 프레임 (기본은 처음 자세)
    tme.add_wrist_d435(cfg, data_types=('rgb',))
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import CameraCfg
    eye, at = np.array([-1.15, -2.95, 0.55]), np.array([-1.85, -2.40, 0.15])   # 출발점 근처를 비스듬히 (env 좌표)
    x = (at - eye) / np.linalg.norm(at - eye)
    y = np.cross([0.0, 0.0, 1.0], x); y /= np.linalg.norm(y)
    cfg.scene.ext_cam = CameraCfg(prim_path='{ENV_REGEX_NS}/ext_cam', width=960, height=540, data_types=['rgb'],
                                  spawn=sim_utils.PinholeCameraCfg(focal_length=16.0, clipping_range=(0.05, 20.0)),
                                  offset=CameraCfg.OffsetCfg(pos=tuple(eye), rot=tme._mat_to_quat_xyzw(np.stack([x, y, np.cross(x, y)], 1).tolist()),
                                                             convention='world'))
    with launch_simulation(cfg, args):
        env = instantiate(cfg)
        env.reset()
        robot = env.scene['robot']
        arm = robot.find_joints([f'joint{i}' for i in range(1, 6)], preserve_order=True)[0]
        home = torch.tensor([tme.TUCK_POSE[f'joint{i}'] for i in range(1, 6)], device=env.device)
        org = env.scene.env_origins[0]
        act = torch.zeros(1, env.action_manager.total_action_dim, device=env.device)
        dt = cfg.sim.dt * cfg.decimation
        rows = []
        for name, cmd, secs in (('정지', (0.0, 0.0, 0.0), 1.0), ('앞으로', (0.5, 0.0, 0.0), 3.0), ('옆으로', (0.0, 0.5, 0.0), 2.0)):
            p0 = (robot.data.root_pos_w.torch[0, :2] - org[:2]).clone()
            act[0, :3] = torch.tensor(cmd, device=env.device)
            err = 0.0
            for _ in range(int(secs / dt)):
                env.step(act)
                err = max(err, float((robot.data.joint_pos.torch[0, arm] - home).abs().max()))
            p1 = robot.data.root_pos_w.torch[0, :2] - org[:2]
            d = (p1 - p0).tolist()
            rows.append(f'{name:4s}: 이동 x {d[0]:+.3f} y {d[1]:+.3f} m · 팔 접은 자세와 최대 차 {math.degrees(err):.2f}°')
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        for side in ('front', 'rear'):
            lid = env.scene[f'lidar_{side}']
            hits = lid.data.ray_hits_w.torch[0]
            off = getattr(cfg.scene, f'lidar_{side}').offset.pos
            ok = torch.isfinite(hits).all(dim=-1)
            rows.append(f'라이다 {side}: 붙은 자리 base_footprint ({off[0]:+.4f}, {off[1]:+.4f}, {off[2]:.4f}) m · 맞은 빔 {int(ok.sum())}/{hits.shape[0]} · 맞은 점 높이 {float(hits[ok, 2].mean()):.3f} m')
        import cv2
        for cam in ('birdeye', 'wrist_d435', 'ext_cam'):
            img = env.scene[cam].data.output['rgb'].torch[0, :, :, :3].cpu().numpy()
            if img.dtype != np.uint8:
                img = (np.clip(img, 0, 1) * 255).astype(np.uint8)
            cv2.imwrite(str(out / f'{cam}.png'), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            p = env.scene[cam].data.pos_w.torch[0] - org
            rows.append(f'카메라 {cam}: 위치 ({float(p[0]):+.3f}, {float(p[1]):+.3f}, {float(p[2]):.3f}) m · 평균 밝기 {img.mean():.0f}')
        print('\n'.join(['[팀 로봇 확인]'] + rows), flush=True)
        (out / 'check.txt').write_text('\n'.join(rows) + '\n')


main()
