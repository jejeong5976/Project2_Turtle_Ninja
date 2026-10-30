#!/usr/bin/env python3
"""손목 D435 시뮬 데이터 (집기 직전 관찰 자세): 컬러 · 깊이 · 퍽 마스크 · 퍽 정답 자세(카메라 좌표).

장면마다: 짐 선반 하나 앞(선반 앞면 ~ 차체 앞면 SHELF_STAND_GAP, 좌우 ±--lat_m, 앞뒤 ±--fwd_m, 방향 ±--yaw_deg)에 로봇을 세우고
  팔을 관찰 자세 후보 중 하나(--look bank, 순기구학으로 고른 자세 + ±--arm_jitter rad)로 둔 뒤 손목 D435 (tme.add_wrist_d435, 실물 공장 내부 파라미터)로 찍음.
  시뮬 팔은 무게로 순기구학보다 약 2–8° 더 숙여짐 (관절 위치 제어 처짐, 2026-10-27 확인) -> 실물 서보 처짐과 비슷한 방향
  그 선반 위를 다시 놓음: 퍽 --n_pucks 개(8개 중 무작위, --p_fallen 확률로 눕힘, yaw 무작위) + 파란 퍽 --n_blue 개(라벨 없음)
  --p_touch 장면은 선 퍽 2–3개를 면끼리 붙임. 가림 = 그리퍼 손가락(브래킷 각도 · 그리퍼 벌림에 따라) · 이웃 퍽
  --cam_tilt_deg = RealSense 브래킷 각도 (관찰 자세 후보도 이 각도로 다시 찾음, 숙일수록 그리퍼 손가락이 화면 아래를 가림)
  2026-10-29 추가: 그리퍼 벌림 무작위, 퍽 거칠기, 선반 윗면(대부분) · 앞면에 장소 태그, 선반 앞 바닥 퍽(--p_floor, 바닥을 보는 관찰 자세 후보 따로),
  퍽 태그 붙인 각도 · 위치 오차 (--tag_jitter, 물리 시작 전 = 실행마다). 브래킷 사이 각도는 실행마다 --cam_tilt_deg 를 4° 간격으로
  누운 퍽은 긴축 둘레로도 돌림 (사각기둥 90° 단위, 원기둥 무작위) → 어느 면이 위 · 앞으로 오는지 섞임
  태그 면 무작위 (2026-10-27): 실물 퍽은 태그가 일부 면에만 붙어 있어 단색 면이 보임 (시뮬 USD 는 사각기둥 6면 · 원기둥 3곳 전부)
    고른 퍽마다 --p_no_tag 확률로 태그 전부 숨김, --p_all_tags 확률로 전부 보임, 나머지는 면마다 --p_face_tag 확률로 보임
  장면마다 배경 파노라마 · 밝기 · 천장 조명 · 벽판 반사 ·
  퍽 · 선반 색 흔들기 (make_yolo_dataset.py 와 같은 방식 · 같은 기본 범위)
LOOK = 순기구학 탐색 결과 (2026-10-26): 카메라가 선반 윗면 중심(로봇 앞 0.30 m)을 0.30 m 거리 · 아래 60° 로 봄. 실물 미확인
출력 <out>/<split>/<이름>_rgb.png, _depth.npy (m, float32, 0 = 없음), _mask.png (0 = 배경, k+1 = 퍽 k), _meta.json
  meta: K, 카메라 월드 자세, 퍽마다 {index, color, shape, tag_id, center_cam (m), axis_cam (긴축 = 퍽 +z), xaxis_cam (퍽 +x = 사각기둥 면 법선), px (보이는 화소), bbox,
        lying (긴축이 수직에서 45° 넘게 누움), tag_faces (태그가 켜진 면 이름, [] = 태그 없음)}
  좌표 = 카메라 광학 프레임 (x 오른쪽 · y 아래 · z 앞). 검증: 정답 중심을 투영해 마스크 위인지 (--check)
예) cd sim/robot_sim && OMNI_KIT_ACCEPT_EULA=YES env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python scripts/make_wrist_dataset.py --scenes 20
    -> 기본 출력 yolo/wrist/datasets/sim_wrist (커밋 안 됨)
"""
import argparse
import sys
from pathlib import Path as _P
sys.path.insert(0, str(_P(__file__).resolve().parents[1]))   # sim/robot_sim -> sim_core 패키지
import json
import math
import re
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
parser.add_argument('--scenes', type=int, default=20)
parser.add_argument('--views', type=int, default=2, help='장면 하나에서 로봇 자리 · 팔 자세를 바꿔 찍는 수')
parser.add_argument('--out', default=str(Path(__file__).resolve().parents[3] / 'yolo' / 'wrist' / 'datasets' / 'sim_wrist'))
parser.add_argument('--seed', type=int, default=0)
parser.add_argument('--maze', default='real')
parser.add_argument('--lat_m', type=float, default=0.06)
parser.add_argument('--fwd_m', type=float, default=0.06, help='관찰 거리 흔들기 (선반 앞 서는 자리 앞뒤)')
parser.add_argument('--yaw_deg', type=float, default=10.0)
parser.add_argument('--arm_jitter', type=float, default=0.03, help='고른 관찰 자세에서 관절마다 더 흔들기 (rad)')
parser.add_argument('--look', choices=['bank', 'fixed'], default='bank',
                    help='bank = 순기구학 관찰 자세 후보 (sim_core/arm_poses.make_view_bank: 선반 윗면을 0.20–0.45 m · 아래 35–80° 로 보는 자세, '
                         '절반은 LOOK 근처 · 절반은 관절 전체에서), fixed = LOOK 하나 (+ --arm_jitter)')
parser.add_argument('--n_poses', type=int, default=300, help='--look bank 후보 수')
parser.add_argument('--p_fallen', type=float, default=0.5, help='관찰 선반 위 퍽을 눕힐 확률 (2026-10-27 0.3 → 0.5, 실물 누운 퍽 놓침)')
parser.add_argument('--p_no_tag', type=float, default=0.3, help='퍽 태그를 전부 숨길 확률 (퍽마다 · 장면마다)')
parser.add_argument('--p_all_tags', type=float, default=0.2, help='퍽 태그를 전부 보일 확률 (나머지는 면마다 --p_face_tag)')
parser.add_argument('--p_face_tag', type=float, default=0.5, help='면마다 태그를 보일 확률')
parser.add_argument('--settle_steps', type=int, default=6)
parser.add_argument('--render', choices=['pt', 'rt'], default='pt')
parser.add_argument('--check', action='store_true', help='정답 중심 투영이 마스크 위인지 검사해 출력')
parser.add_argument('--pano', nargs='+', default=['all'], help="배경 파노라마 (usd/textures/), 'all' = pano_*.jpg 전부")
parser.add_argument('--dome_min', type=float, default=0.25, help='배경 밝기 최소 배율 (천장 조명과 같이 씀)')
parser.add_argument('--dome_max', type=float, default=1.1, help='배경 밝기 최대 배율')
parser.add_argument('--ceil_int', type=float, default=10000.0, help='천장 사각 조명 세기 (tme.CEIL_LIGHT_INT 와 같음, 0 = 끔)')
parser.add_argument('--ceil_k', type=float, nargs=2, default=[0.7, 1.3], help='천장 조명 세기 배율 범위 (장면마다)')
parser.add_argument('--panel_rough', type=float, nargs=2, default=[0.02, 0.15], help='벽판 거칠기 범위 (make_yolo_dataset.py 와 같음)')
parser.add_argument('--panel_ior', type=float, nargs=2, default=[2.2, 3.0], help='벽판 굴절률 범위 (반사 세기, make_yolo_dataset.py 와 같음)')
parser.add_argument('--n_pucks', type=int, nargs=2, default=[1, 4], help='관찰 선반 위에 다시 놓을 퍽 수 범위 (8개 중 무작위)')
parser.add_argument('--n_blue', type=int, nargs=2, default=[0, 2], help='관찰 선반 위 파란 퍽 수 범위 (라벨 없음 = 오탐 대비, 실물 파란 퍽과 같은 모양 · 태그)')
parser.add_argument('--blue_srgb', type=int, nargs=3, default=[35, 100, 181],
                    help='파란 퍽 색 (sRGB). 실물 D435 사진 (15, 53, 100) 을 같은 사진 초록 퍽 대비 노출 보정한 값 (2026-10-29)')
parser.add_argument('--look_q', type=float, nargs=5, default=None, help='--look fixed 에서 쓸 관찰 자세 관절 J1..J5 (rad, URDF). 없으면 아래 LOOK (tools/find_look_pose.py 결과 넣기)')
parser.add_argument('--cam_tilt_deg', type=float, default=0.0, help='RealSense 브래킷 각도 URDF 대비 추가분 (°, − = 그리퍼 쪽으로 숙임, tme.d435_tilt_R. URDF 에 실물 경첩 −12° 반영됨 2026-10-29). 실행마다 하나')
parser.add_argument('--p_touch', type=float, default=0.15, help='장면에서 선 퍽 2–3개를 면끼리 붙여 놓을 확률')
parser.add_argument('--p_grip_open', type=float, default=0.3, help='시점마다 그리퍼 다 열림(기본 관찰 자세) 확률. 나머지는 손가락 관절 범위 안 무작위')
parser.add_argument('--puck_rough', type=float, nargs=2, default=[0.3, 0.8], help='퍽 재질 거칠기 범위 (장면마다 퍽마다, PLA 무광 ~ 약한 광택)')
parser.add_argument('--p_place_tag', type=float, default=0.6, help='장면마다 장소 태그(60 mm 라벨)를 관찰 선반에 붙일 확률')
parser.add_argument('--p_tag_top', type=float, default=0.85,
                    help='붙일 때 박스 윗면에 눕혀 붙일 확률 (나머지는 앞면). 장소 태그는 윗면일 가능성이 높음 (2026-10-29 사용자). 바코드는 벽에만')
parser.add_argument('--pose_tries', type=int, default=3, help='팔이 관찰 자세에 못 가면(관절 오차 > 0.05 rad, 충돌로 밀림) 다른 자세로 다시 시도할 횟수. 다 실패하면 그 시점 건너뜀')
parser.add_argument('--p_floor', type=float, default=0.15, help='장면에서 퍽 1–2 개를 선반 앞 바닥에 놓을 확률 (그 장면은 로봇이 --floor_back 만큼 물러나 바닥을 봄)')
parser.add_argument('--floor_back', type=float, default=0.20, help='바닥 퍽 장면에서 로봇이 더 물러서는 거리 (m)')
parser.add_argument('--tag_jitter', type=float, nargs=2, default=[10.0, 0.002], help='퍽 태그 붙인 각도 · 위치 오차 (°, m). 물리 시작 전 한 번 = 실행마다 퍽마다 다름')
parser.add_argument('--no_jitter', action='store_true', help='퍽 · 선반 색 흔들기 끄기')
add_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
if args.maze != 'real':
    import os
    os.environ['TN_MAZE'] = args.maze

import cv2
import numpy as np
import torch

from isaaclab.utils import instantiate

from sim_core import turtleninja_maze_env as tme

# 관찰 자세 = robot_description/config/initial_pose.json poses 'LOOK_PUCK' (tme.LOOK_PUCK_POSE, 2026-10-29 tools/find_look_pose.py, URDF 경첩 −12° 반영 기준).
# 로봇 앞 0.26 m 선반 면을 0.29 m 에서 광축 9° 로 봄. 옛 값 (2026-10-26, 아래 60°): J1 −1.738, J2 −0.706, J3 −0.624, J4 0.312, J5 −0.005. 관절 이름 = URDF
LOOK = {k: tme.LOOK_PUCK_POSE[k] for k in ('arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5')}
ARM_JOINTS = list(LOOK)
if args.look_q is not None:
    LOOK = dict(zip(ARM_JOINTS, args.look_q))
STAND = tme.SHELF_SIZE[0] / 2 + tme.SHELF_STAND_GAP + tme.ROBOT_FRONT     # 선반 중심 -> 로봇 중심
PAT = re.compile(r'/World/envs/env_0/Puck_(\d+)(/.*)?$')

rng = np.random.default_rng(args.seed)

# 관찰 자세 후보 (Isaac 없이 URDF 순기구학). 선반 윗면 중심 = 로봇 앞 STAND, 높이 = 바닥 + 선반 높이 (base_link 기준)
from sim_core import arm_poses
if args.look == 'bank':
    _ztop = arm_poses.FLOOR_Z + tme.SHELF_SIZE[2]
    _K = [[tme.D435_K['fx'], 0, tme.D435_K['cx']], [0, tme.D435_K['fy'], tme.D435_K['cy']], [0, 0, 1]]
    _kw = dict(urdf=tme.ROBOT_URDF, tuck={**tme.READY_POSE, **{f: 0.0 for f in arm_poses.FINGERS}}, rng=rng, pole_xy=tme.PROFILE_XY,
               cam_link=tme.D435_COLOR_FRAME, cam_off=(np.zeros(3), np.array(tme.d435_tilt_R(args.cam_tilt_deg))), K=_K, size=tme.D435_SIZE,
               target=(STAND, 0.0, _ztop), shelf_box=(STAND - tme.SHELF_SIZE[0] / 2, STAND + tme.SHELF_SIZE[0] / 2,
                                                       -tme.SHELF_SIZE[1] / 2, tme.SHELF_SIZE[1] / 2, _ztop))
    _near, _ = arm_poses.make_view_bank(n=args.n_poses // 2, seed_pose=LOOK, **_kw)
    _far, _ = arm_poses.make_view_bank(n=args.n_poses - len(_near), seed_pose=None, **_kw)
    BANK = [b[0] for b in _near + _far]
    # 바닥 퍽 장면: 로봇이 floor_back 만큼 물러서고 선반 앞면에서 8.5 cm 앞 바닥을 봄 (선반 상자 · 벽도 그만큼 뒤)
    FLOOR_TARGET_X = STAND + args.floor_back - (tme.SHELF_SIZE[0] / 2 + 0.085)
    _kwf = {**_kw, 'target': (FLOOR_TARGET_X, 0.0, arm_poses.FLOOR_Z),
            'shelf_box': (STAND + args.floor_back - tme.SHELF_SIZE[0] / 2, STAND + args.floor_back + tme.SHELF_SIZE[0] / 2,
                          -tme.SHELF_SIZE[1] / 2, tme.SHELF_SIZE[1] / 2, _ztop)}
    _fn, _ = arm_poses.make_view_bank(n=args.n_poses // 4, seed_pose=LOOK, **_kwf)
    _ff, _ = arm_poses.make_view_bank(n=args.n_poses // 4, seed_pose=None, **_kwf)
    FLOOR_BANK = [b[0] for b in _fn + _ff] or BANK
    print(f'[wrist] 바닥 관찰 자세 후보 {len(FLOOR_BANK)}개 (목표 = 로봇 앞 {FLOOR_TARGET_X:.2f} m 바닥)', flush=True)
    _v = np.array([[b[1]['dist'], b[1]['pitch'], b[1]['roll']] for b in _near + _far])
    print(f'[wrist] 관찰 자세 후보 {len(BANK)}개 (LOOK 근처 {len(_near)} + 전역 {len(_far)}): 거리 {_v[:, 0].min():.2f}–{_v[:, 0].max():.2f} m, '
          f'숙임 {_v[:, 1].min():.0f}–{_v[:, 1].max():.0f}°, 기울기 ±{np.abs(_v[:, 2]).max():.0f}°', flush=True)
else:
    BANK = [LOOK]
    FLOOR_BANK = BANK
torch.manual_seed(args.seed)
cfg = tme.TurtleNinjaMazePropsEnvCfg()
cfg.scene.num_envs = 1
cfg.scene.env_spacing = 15.0
PANOS = sorted(p.name for p in tme.REAL_DIR.glob('pano_*.jpg')) if args.pano == ['all'] else args.pano
assert PANOS and all((tme.REAL_DIR / p).exists() for p in PANOS), f'파노라마 없음: {PANOS}'


def pano_gain(name):
    """파노라마 평균 밝기(선형)를 첫 장 기준으로 맞추는 돔 세기 배율 (make_yolo_dataset.py 와 같음)."""
    im = cv2.resize(cv2.imread(str(tme.REAL_DIR / name)), (1024, 512)).astype(np.float32) / 255.0
    return float(np.where(im <= 0.04045, im / 12.92, ((im + 0.055) / 1.055) ** 2.4).mean())


PANO_GAIN = {p: pano_gain(PANOS[0]) / pano_gain(p) for p in PANOS}
tme.use_real_look(cfg, PANOS[0])

# 파란 퍽 (라벨 없음 = 오탐 대비, 2026-10-29). 빨간 퍽 USD 를 다시 불러 색만 파랑으로 (모양 · 태그 · 질량 그대로)
#   prim 이름이 BluePuck_k 라 마스크 패턴(Puck_k) 에 안 걸림. 태그 ID 는 빨간 퍽(0–3)과 같음 (ID 평가에는 안 씀)
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
BLUE_SRC = [(0, 'box'), (2, 'cyl'), (1, 'box'), (3, 'cyl')]          # (빨간 퍽 USD 번호, 모양)
PARK = lambda k: (-2.0 + 0.35 * k, -tme.MAZE_INNER_HALF[1] - 1.6)       # 미로 남쪽 바깥 (안 보이는 자리)
for k, (src, _) in enumerate(BLUE_SRC):
    setattr(cfg.scene, f'blue_puck_{k}', RigidObjectCfg(
        prim_path=f'{{ENV_REGEX_NS}}/BluePuck_{k}', spawn=sim_utils.UsdFileCfg(usd_path=str(tme.PROPS_DIR / f'puck_{src}.usda')),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(*PARK(10 + k), 0.1))))
tme.add_wrist_d435(cfg, data_types=('rgb', 'distance_to_image_plane', 'instance_id_segmentation_fast'), tilt_deg=args.cam_tilt_deg)


def jitter_tags(env_, env_ids):
    """퍽 태그 메시 꼭짓점을 면 안에서 돌리고(±tag_jitter[0]°) 옮김(±tag_jitter[1] m) = 손으로 붙인 오차. 물리 시작 전에만 (prestartup)"""
    import omni.usd
    from pxr import UsdGeom, Gf
    st_ = omni.usd.get_context().get_stage(); r_ = np.random.default_rng(args.seed + 1000); n_ = 0
    roots = [f'/World/envs/env_0/Puck_{k}' for k in range(len(tme.PUCKS))] + [f'/World/envs/env_0/BluePuck_{k}' for k in range(len(BLUE_SRC))]
    for root in roots:
        for c_ in st_.GetPrimAtPath(root).GetChildren():
            if not c_.GetName().startswith('tag_') or not c_.IsA(UsdGeom.Mesh):
                continue
            a_ = UsdGeom.Mesh(c_).GetPointsAttr(); P_ = np.array(a_.Get(), float)
            if len(P_) < 3:
                continue
            cen = P_.mean(0); nrm_ = np.cross(P_[1] - P_[0], P_[2] - P_[0]); nrm_ /= np.linalg.norm(nrm_)
            t_ = math.radians(r_.uniform(-args.tag_jitter[0], args.tag_jitter[0]))
            K_ = np.array([[0, -nrm_[2], nrm_[1]], [nrm_[2], 0, -nrm_[0]], [-nrm_[1], nrm_[0], 0]])
            Rr = np.eye(3) + math.sin(t_) * K_ + (1 - math.cos(t_)) * K_ @ K_          # 법선 둘레 회전
            e1 = P_[1] - P_[0]; e1 /= np.linalg.norm(e1); e2 = np.cross(nrm_, e1)
            sh = e1 * r_.uniform(-args.tag_jitter[1], args.tag_jitter[1]) + e2 * r_.uniform(-args.tag_jitter[1], args.tag_jitter[1])
            Q_ = (P_ - cen) @ Rr.T + cen + sh
            a_.Set([Gf.Vec3f(*q) for q in Q_.tolist()]); n_ += 1
    print(f'[wrist] 퍽 태그 붙인 오차 적용 {n_}장', flush=True)


if args.tag_jitter[0] > 0 or args.tag_jitter[1] > 0:
    from isaaclab.managers import EventTermCfg
    cfg.scene.replicate_physics = False          # prestartup 이벤트 조건 (env 1 개라 영향 없음)
    cfg.events.jitter_tags = EventTermCfg(func=jitter_tags, mode='prestartup')
from isaaclab_tasks.utils.presets import set_isaac_rtx_global_settings
set_isaac_rtx_global_settings(cfg.scene.wrist_d435.renderer_cfg, antialiasing_mode='TAA')
K = np.array([[tme.D435_K['fx'], 0, tme.D435_K['cx']], [0, tme.D435_K['fy'], tme.D435_K['cy']], [0, 0, 1.0]])


def quat_to_mat(q):            # (x, y, z, w)
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def srgb2lin(rgb):
    c = np.asarray(rgb, float) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def hsv_jitter(lin):
    """선형 RGB -> sRGB HSV 에서 조금 흔듦 -> 선형 (make_yolo_dataset.py 와 같음)."""
    srgb = np.where(lin <= 0.0031308, lin * 12.92, 1.055 * np.power(np.clip(lin, 1e-6, 1), 1 / 2.4) - 0.055)
    hsv = cv2.cvtColor(np.uint8(np.clip(srgb * 255, 0, 255))[None, None], cv2.COLOR_RGB2HSV_FULL).astype(np.float32)[0, 0]
    hsv[0] = (hsv[0] + rng.uniform(-6, 6)) % 256
    hsv[1] = np.clip(hsv[1] * rng.uniform(0.85, 1.1), 0, 255)
    hsv[2] = np.clip(hsv[2] * rng.uniform(0.8, 1.15), 0, 255)
    s_ = cv2.cvtColor(np.uint8(hsv)[None, None], cv2.COLOR_HSV2RGB_FULL)[0, 0] / 255.0
    return np.where(s_ <= 0.04045, s_ / 12.92, ((s_ + 0.055) / 1.055) ** 2.4)


def shelf_spots(n, rng, half=tme.SHELF_SIZE[0] / 2 - 0.035, sep=0.065, tries=300, avoid=()):
    """선반 윗면 좌표(가운데 0, 앞 = +nrm) 안에서 서로 · avoid(이미 놓은 자리)와 sep 이상 떨어진 점 n 개. 못 채우면 채운 만큼."""
    pts = []
    for _ in range(tries):
        if len(pts) == n:
            break
        q = rng.uniform(-half, half, 2)
        if all(np.linalg.norm(q - r) >= sep for r in list(pts) + list(avoid)):
            pts.append(q)
    return pts


out = Path(args.out)
for sp in ('train', 'val', 'test'):
    (out / sp).mkdir(parents=True, exist_ok=True)
(out / f'look_bank_s{args.seed:02d}.json').write_text(json.dumps(BANK, indent=0))

with launch_simulation(cfg, args):
    env = instantiate(cfg)
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    if args.render == 'pt':          # make_yolo_dataset.py 와 같음: 렌더러가 뜬 뒤 carb 로 PathTracing
        import carb
        cs = carb.settings.get_settings()
        for k_, v_ in (('/rtx/rendermode', 'PathTracing'), ('/rtx/pathtracing/spp', 16), ('/rtx/pathtracing/totalSpp', 64),
                       ('/rtx/pathtracing/optixDenoiser/enabled', True)):
            cs.set(k_, v_)
    print('[wrist] 로봇 실물 색:', sum(tme.apply_robot_real_colors(stage).values()), '개 재질', flush=True)
    crank = tme.add_crank_visuals(stage)   # 그리퍼 크랭크 · 막대를 손가락 위치에 맞춤 (배포판 CrankVisuals)
    robot, cam = env.scene['robot'], env.scene['wrist_d435']
    from pxr import Gf, Sdf, Usd, UsdGeom
    from isaaclab.utils.math import quat_mul

    def find_attr(root, name):
        for q_ in Usd.PrimRange(stage.GetPrimAtPath(root)):
            a_ = q_.GetAttribute(name)
            if a_ and a_.IsValid() and a_.HasAuthoredValue():
                return a_
        return None
    ceil_attrs = tme.add_ceiling_lights(stage, env.scene.env_origins, args.ceil_int) if args.ceil_int > 0 else []
    light = stage.GetPrimAtPath('/World/light')
    base_int = light.GetAttribute('inputs:intensity').Get()
    orient = light.GetAttribute('xformOp:orient')
    dome_tex = light.GetAttribute('inputs:texture:file')
    puck_sh = [stage.GetPrimAtPath(f'/World/envs/env_0/Puck_{k}/Looks/body/Surface').GetAttribute('inputs:diffuseColor')
               for k in range(len(tme.PUCKS))]
    puck_base = [np.array(a_.Get(), float) for a_ in puck_sh]
    panel = stage.GetPrimAtPath('/World/envs/env_0/Maze/Looks/panel/Surface')
    panel_rough, panel_ior = panel.GetAttribute('inputs:roughness'), panel.CreateAttribute('inputs:ior', Sdf.ValueTypeNames.Float)
    shelf_col = [find_attr(f'/World/envs/env_0/Shelf_{k}', 'inputs:diffuseColor') for k in range(tme.NUM_SHELVES)]
    shelf_base = np.array(tme._srgb_to_linear(tme.SHELF_SRGB), float)
    blue_sh = [stage.GetPrimAtPath(f'/World/envs/env_0/BluePuck_{k}/Looks/body/Surface').GetAttribute('inputs:diffuseColor')
               for k in range(len(BLUE_SRC))]
    blue_base = srgb2lin(args.blue_srgb)
    blue_tags = [[UsdGeom.Imageable(c) for c in stage.GetPrimAtPath(f'/World/envs/env_0/BluePuck_{k}').GetChildren()
                  if c.GetName().startswith('tag_')] for k in range(len(BLUE_SRC))]
    print(f'[wrist] 파노라마 {len(PANOS)}장, 퍽 색 {sum(a_ is not None for a_ in puck_sh)}, 선반 색 {sum(a_ is not None for a_ in shelf_col)}, '
          f'파란 퍽 색 {sum(a_.IsValid() for a_ in blue_sh)}, 벽판 {panel.IsValid()}, '
          f'브래킷 {args.cam_tilt_deg:+.0f}°', flush=True)
    dev = env.device
    org = env.scene.env_origins[0]
    act = torch.zeros(1, env.action_manager.total_action_dim, device=dev)
    arm_ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
    # 관찰(LOOK) 때 그리퍼 = 열림 58.5 mm (손가락 0). 기본 자세가 받침 16 mm 로 바뀌어 명시 (2026-10-28 전 데이터와 같은 모습)
    finger_ids = robot.find_joints(['gripper_finger_a_joint', 'gripper_finger_b_joint'], preserve_order=True)[0]
    LOOK_FINGER = 0.0
    FINGER_LIM = (-0.021254, 0.010247)          # URDF gripper_finger_*_joint 범위
    def rough_attr(path):
        sh_ = stage.GetPrimAtPath(path)
        a_ = sh_.GetAttribute('inputs:roughness')
        return a_ if a_ and a_.IsValid() else sh_.CreateAttribute('inputs:roughness', Sdf.ValueTypeNames.Float)
    puck_rough = [rough_attr(f'/World/envs/env_0/Puck_{k}/Looks/body/Surface') for k in range(len(tme.PUCKS))] + \
                 [rough_attr(f'/World/envs/env_0/BluePuck_{k}/Looks/body/Surface') for k in range(len(BLUE_SRC))]
    zero = torch.zeros(1, device=dev)
    # 퍽마다 태그 면 prim (tag_top · tag_bottom · tag_px … / 원기둥 tag_side) → 장면마다 보임 · 숨김
    tag_prims = [{c.GetName()[4:]: UsdGeom.Imageable(c) for c in stage.GetPrimAtPath(f'/World/envs/env_0/Puck_{k}').GetChildren()
                  if c.GetName().startswith('tag_')} for k in range(len(tme.PUCKS))]
    tag_on = [sorted(t) for t in tag_prims]                 # 지금 켜진 면 (meta 용)
    print(f'[wrist] 퍽 태그 면: {[len(t) for t in tag_prims]}', flush=True)

    def set_tags(k, on):
        for f_, im in tag_prims[k].items():
            (im.MakeVisible if f_ in on else im.MakeInvisible)()
        tag_on[k] = sorted(on)
    def grab():
        """지금 렌더 -> (rgb, 깊이 m, 퍽 마스크 0 = 배경 · k+1 = 퍽 k)"""
        rgb_ = cam.data.output['rgb'][0, ..., :3].cpu().numpy().astype(np.uint8)
        dep = cam.data.output['distance_to_image_plane'][0, ..., 0].cpu().numpy().astype(np.float32)
        dep[~np.isfinite(dep) | (dep > 10)] = 0
        sg = cam.data.output['instance_id_segmentation_fast'][0].cpu().numpy().astype(np.uint32)
        seg = sg[..., 0] | (sg[..., 1] << 8) | (sg[..., 2] << 16) | (sg[..., 3] << 24) if sg.shape[-1] == 4 else sg[..., 0]
        id2path = {}
        for key, path in cam.data.info['instance_id_segmentation_fast']['idToLabels'].items():
            if isinstance(key, tuple):
                key = int(key[0]) | (int(key[1]) << 8) | (int(key[2]) << 16) | (int(key[3]) << 24)
            id2path[int(key)] = path
        mk_ = np.zeros(seg.shape, np.uint16)
        for iid in np.unique(seg):
            m_ = PAT.match(str(id2path.get(int(iid), '')))
            if m_:
                mk_[seg == iid] = int(m_.group(1)) + 1
        return rgb_, dep, mk_
    n_img, check_ok, check_all, n_skip_pose = 0, 0, 0, 0
    with torch.inference_mode():
        for s in range(args.scenes):
            env.reset()
            st = tme.maze_state(env)
            loaded = [k for k in range(int(st['n_loaded_shelves'][0]))]
            k_sh = int(rng.choice(loaded))
            sxy = st['shelf_xy'][0, k_sh].cpu().numpy(); syaw = float(st['shelf_yaw'][0, k_sh])
            nrm = np.array([math.cos(syaw), math.sin(syaw)]); tan = np.array([-nrm[1], nrm[0]])
            # ---- 장면 단위: 배경 · 밝기 · 벽판 반사 · 색 ----
            pano = PANOS[int(rng.integers(len(PANOS)))]
            dome_tex.Set(Sdf.AssetPath(str(tme.REAL_DIR / pano)))
            dome_k = float(rng.uniform(args.dome_min, args.dome_max))
            light.GetAttribute('inputs:intensity').Set(base_int * dome_k * PANO_GAIN[pano])
            dy = math.radians(float(rng.uniform(0, 360)))
            qv = (math.cos(dy / 2), 0.0, 0.0, math.sin(dy / 2))
            orient.Set(Gf.Quatd(*qv) if 'quatd' in str(orient.GetTypeName()) else Gf.Quatf(*qv))
            p_rough, p_ior = float(rng.uniform(*args.panel_rough)), float(rng.uniform(*args.panel_ior))
            panel_rough.Set(p_rough); panel_ior.Set(p_ior)
            ceil_k = float(rng.uniform(*args.ceil_k))
            for a_ in ceil_attrs:
                a_.Set(args.ceil_int * ceil_k)
            for a_ in puck_rough:
                a_.Set(float(rng.uniform(*args.puck_rough)))
            if not args.no_jitter:
                for k in range(len(tme.PUCKS)):
                    puck_sh[k].Set(Gf.Vec3f(*hsv_jitter(puck_base[k]).tolist()))
                for k in range(len(BLUE_SRC)):
                    blue_sh[k].Set(Gf.Vec3f(*hsv_jitter(blue_base).tolist()))
                sc = hsv_jitter(shelf_base)
                for a_ in shelf_col:
                    if a_ is not None:
                        a_.Set(Gf.Vec3f(*sc.tolist()))
            # ---- 관찰 선반 위 다시 놓기: 퍽 n 개(8개 중 무작위, 일부 눕힘) + 가짜 물체 m 개, 나머지 퍽은 원래 자리 / 이 선반에 있던 건 치움 ----
            top = org[2].item() + tme.SHELF_SIZE[2]
            eid = torch.tensor([0], device=dev)

            def put(name, xy, z, roll, yaw_, spin=0.0, pitch=0.0):
                """자세 = yaw(월드 z) · pitch(y) · roll(x) · spin(물체 z = 긴축) 순서로 합성"""
                q_ = tme.quat_from_euler_xyz(torch.tensor([roll], device=dev), torch.tensor([pitch], device=dev, dtype=torch.float32),
                                             torch.tensor([yaw_], device=dev, dtype=torch.float32))
                if spin:
                    q_ = quat_mul(q_, tme.quat_from_euler_xyz(zero, zero, torch.tensor([spin], device=dev, dtype=torch.float32)))
                pos_ = torch.tensor([[xy[0] + org[0].item(), xy[1] + org[1].item(), z]], device=dev, dtype=torch.float32)
                env.scene[name].write_root_pose_to_sim_index(root_pose=torch.cat([pos_, q_], -1), env_ids=eid)
                env.scene[name].write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=dev), env_ids=eid)
            n_p = int(rng.integers(args.n_pucks[0], args.n_pucks[1] + 1))
            n_b = int(rng.integers(args.n_blue[0], args.n_blue[1] + 1))
            touch = rng.random() < args.p_touch
            if touch:
                n_p = max(n_p, 2)
            chosen = [int(k) for k in rng.choice(len(tme.PUCKS), n_p, replace=False)]
            # 붙은 퍽: 선 퍽 2–3 개를 한 방향(각 a)으로 31 mm 간격 = 면끼리 맞닿음 (사각기둥은 yaw = a 라 면이 마주봄)
            cluster, touch_yaw = [], 0.0
            if touch:
                n_c = min(len(chosen), int(rng.integers(2, 4)))
                touch_yaw = float(rng.uniform(-math.pi, math.pi)); dvec = np.array([math.cos(touch_yaw), math.sin(touch_yaw)])
                half = tme.SHELF_SIZE[0] / 2 - 0.035 - 0.031 * (n_c - 1)
                q0 = rng.uniform(-half, half, 2) - dvec * 0.031 * (n_c - 1) / 2
                cluster = [q0 + dvec * 0.031 * i for i in range(n_c)]
            # 장소 태그 · 바코드 스티커 (선반에 붙임). 윗면 태그 자리는 퍽 자리에서 뺌
            place_mode = (('top' if rng.random() < args.p_tag_top else 'front') if rng.random() < args.p_place_tag else None)
            tag_top = shelf_spots(1, rng, half=tme.SHELF_SIZE[0] / 2 - 0.04, avoid=cluster) if place_mode == 'top' else []
            others = shelf_spots(n_p - len(cluster) + n_b, rng, avoid=cluster + tag_top)
            spots = cluster + others
            n_b = min(n_b, max(0, len(spots) - n_p))
            for p in range(len(tme.PUCKS)):                  # 안 고른 퍽은 태그 원래대로 (전부)
                if p not in chosen:
                    set_tags(p, list(tag_prims[p]))
            for p in range(len(tme.PUCKS)):                  # 이 선반에 원래 있던 퍽 중 안 고른 것은 치움
                pxy = env.scene[f'puck_{p}'].data.root_pos_w.torch[0, :2].cpu().numpy() - org[:2].cpu().numpy()
                if p not in chosen and np.linalg.norm(pxy - sxy) < tme.SHELF_SIZE[0] * 0.75:
                    put(f'puck_{p}', PARK(p), tme.PUCK_H / 2 + 0.001, 0.0, 0.0)
            for k in range(len(BLUE_SRC)):                   # 파란 퍽은 전부 치운 뒤 고른 것만 놓음
                put(f'blue_puck_{k}', PARK(10 + k), tme.PUCK_H / 2 + 0.001, 0.0, 0.0)
            b_ids = [int(k) for k in rng.choice(len(BLUE_SRC), n_b, replace=False)] if n_b else []
            in_cluster = set()
            for j, q in enumerate(spots):
                xy_ = sxy + nrm * q[0] + tan * q[1]
                if j < len(chosen):
                    p = chosen[j]
                    lying = (j >= len(cluster)) and rng.random() < args.p_fallen
                    if j < len(cluster):
                        in_cluster.add(p)
                    spin = 0.0
                    if lying:   # 긴축 둘레: 사각기둥은 바닥에 면이 닿게 90° 단위, 원기둥은 아무 각
                        spin = float(rng.integers(4)) * math.pi / 2 if tme.PUCKS[p][1] == 'box' else float(rng.uniform(-math.pi, math.pi))
                    yaw_p = (touch_yaw + math.atan2(nrm[1], nrm[0])) if j < len(cluster) else float(rng.uniform(-math.pi, math.pi))
                    put(f'puck_{p}', xy_, top + (tme.PUCK_D / 2 if lying else tme.PUCK_H / 2) + 0.002,
                        math.pi / 2 if lying else 0.0, yaw_p, spin)
                    r_ = rng.random()
                    faces = list(tag_prims[p])
                    set_tags(p, [] if r_ < args.p_no_tag else faces if r_ < args.p_no_tag + args.p_all_tags
                             else [f_ for f_ in faces if rng.random() < args.p_face_tag])
                elif j - len(chosen) < len(b_ids):              # 파란 퍽: 빨강 · 초록과 같은 규칙으로 눕힘 · 태그 면
                    k = b_ids[j - len(chosen)]
                    lying = rng.random() < args.p_fallen
                    spin = (float(rng.integers(4)) * math.pi / 2 if BLUE_SRC[k][1] == 'box' else float(rng.uniform(-math.pi, math.pi))) if lying else 0.0
                    put(f'blue_puck_{k}', xy_, top + (tme.PUCK_D / 2 if lying else tme.PUCK_H / 2) + 0.002,
                        math.pi / 2 if lying else 0.0, float(rng.uniform(-math.pi, math.pi)), spin)
                    r_ = rng.random()
                    for im in blue_tags[k]:
                        show = r_ >= args.p_no_tag and (r_ < args.p_no_tag + args.p_all_tags or rng.random() < args.p_face_tag)
                        (im.MakeVisible if show else im.MakeInvisible)()
            # ---- 선반 앞 바닥 퍽 (--p_floor): 고른 퍽 중 붙은 무리 아닌 것 1–2 개를 바닥 관찰 목표점 둘레 바닥으로 ----
            floor_scene, on_floor = rng.random() < args.p_floor, set()
            if floor_scene:
                cand = [p for p in chosen if p not in in_cluster]
                fl_pts = []
                for p in cand[:int(rng.integers(1, 3))]:
                    for _ in range(30):
                        # 바닥 관찰 목표점(선반 앞면 8.5 cm 앞) 둘레 ±4 cm (앞뒤) · ±7 cm (옆) = 화면 안 (2026-10-29: ±12 cm 면 화면 밖이 많았음)
                        fq = np.array([tme.SHELF_SIZE[0] / 2 + 0.085 + rng.uniform(-0.04, 0.04), rng.uniform(-0.07, 0.07)])
                        if all(np.linalg.norm(fq - o_) > 0.07 for o_ in fl_pts):
                            break
                    fl_pts.append(fq)
                    lying = rng.random() < 0.5
                    spin = (float(rng.integers(4)) * math.pi / 2 if tme.PUCKS[p][1] == 'box' else float(rng.uniform(-math.pi, math.pi))) if lying else 0.0
                    put(f'puck_{p}', sxy + nrm * fq[0] + tan * fq[1], org[2].item() + (tme.PUCK_D / 2 if lying else tme.PUCK_H / 2) + 0.002,
                        math.pi / 2 if lying else 0.0, float(rng.uniform(-math.pi, math.pi)), spin)
                    on_floor.add(p)
                floor_scene = bool(on_floor)
            # ---- 장소 태그 (60 mm, +x = 앞): 선반 윗면(눕혀) 또는 앞면에 붙임 (벽에 있던 것을 옮김). 바코드는 벽에 그대로 ----
            def on_face(name, face, w):
                yw = math.atan2(nrm[1], nrm[0]) + {'front': 0.0, 'left': math.pi / 2, 'right': -math.pi / 2}[face]
                out_ = np.array([math.cos(yw), math.sin(yw)]); side = np.array([-out_[1], out_[0]])
                along = rng.uniform(-1, 1) * (tme.SHELF_SIZE[1] / 2 - w / 2 - 0.01)
                put(name, sxy + out_ * (tme.SHELF_SIZE[0] / 2 + 0.001) + side * along, org[2].item() + tme.SHELF_SIZE[2] / 2, 0.0, yw)
            if place_mode == 'front':
                on_face('place_tag_0', 'front', tme.PLACE_TAG_W)
            elif place_mode == 'top':
                q = tag_top[0] if tag_top else np.zeros(2)
                put('place_tag_0', sxy + nrm * q[0] + tan * q[1], top + 0.0005, 0.0, float(rng.uniform(-math.pi, math.pi)), pitch=-math.pi / 2)
            for _ in range(3):                               # 옮긴 퍽 내려앉기
                env.step(act)
            split = str(rng.choice(['train', 'val', 'test'], p=[0.8, 0.1, 0.1]))
            for v in range(args.views):
                stand = STAND + (args.floor_back if floor_scene else 0.0)
                bank = FLOOR_BANK if floor_scene else BANK
                ok_pose, tries_used = False, 0
                for tries_used in range(1, args.pose_tries + 1):        # 팔이 충돌로 밀려 관찰 자세에 못 가면 다른 자리 · 자세로 다시
                    xy = sxy + nrm * (stand + rng.uniform(-args.fwd_m, args.fwd_m)) + tan * rng.uniform(-args.lat_m, args.lat_m)
                    yaw = math.atan2(-nrm[1], -nrm[0]) + math.radians(rng.uniform(-args.yaw_deg, args.yaw_deg))
                    pos = torch.tensor([[xy[0], xy[1], tme.SPAWN_Z]], device=dev, dtype=torch.float32) + org
                    robot.write_root_pose_to_sim_index(root_pose=torch.cat([pos, tme.quat_from_euler_xyz(zero, zero, torch.tensor([yaw], device=dev, dtype=torch.float32))], -1),
                                                       env_ids=torch.tensor([0], device=dev))
                    robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=dev), env_ids=torch.tensor([0], device=dev))
                    look_k = int(rng.integers(len(bank)))
                    qa = torch.tensor([[bank[look_k][j] + rng.uniform(-args.arm_jitter, args.arm_jitter) for j in ARM_JOINTS]], device=dev, dtype=torch.float32)
                    robot.write_joint_position_to_sim_index(position=qa, joint_ids=arm_ids, env_ids=torch.tensor([0], device=dev))
                    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(qa), joint_ids=arm_ids, env_ids=torch.tensor([0], device=dev))
                    robot.actuators.target_command.set_position_index(value=qa, joint_ids=arm_ids, env_ids=torch.tensor([0], device=dev))
                    grip = LOOK_FINGER if rng.random() < args.p_grip_open else float(rng.uniform(*FINGER_LIM))
                    qf = torch.full((1, 2), grip, device=dev)
                    robot.write_joint_position_to_sim_index(position=qf, joint_ids=finger_ids, env_ids=torch.tensor([0], device=dev))
                    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(qf), joint_ids=finger_ids, env_ids=torch.tensor([0], device=dev))
                    robot.actuators.target_command.set_position_index(value=qf, joint_ids=finger_ids, env_ids=torch.tensor([0], device=dev))
                    crank.update([grip])
                    for _ in range(args.settle_steps):
                        env.step(act)
                    jerr = float((robot.data.joint_pos.torch[0, arm_ids] - qa[0]).abs().max())
                    if jerr < 0.05:
                        ok_pose = True
                        break
                if not ok_pose:
                    n_skip_pose += 1
                    print(f'[wrist] 장면 {s + 1} 시점 {v}: 팔이 관찰 자세에 못 감 (관절 오차 {jerr:.2f} rad, {args.pose_tries}번) -> 건너뜀', flush=True)
                    continue
                rgb, depth, mask = grab()
                # 카메라 자세 (월드, ROS 광학) -> 퍽 정답을 카메라 좌표로
                cp = cam.data.pos_w.torch[0].cpu().numpy()
                Rc = quat_to_mat(cam.data.quat_w_ros.torch[0].cpu().numpy())
                pucks = []
                for p in range(len(tme.PUCKS)):
                    px = int((mask == p + 1).sum())
                    if px == 0:
                        continue
                    obj = env.scene[f'puck_{p}']
                    pw = obj.data.root_pos_w.torch[0].cpu().numpy(); Rp = quat_to_mat(obj.data.root_quat_w.torch[0].cpu().numpy())
                    c_cam = Rc.T @ (pw - cp); a_cam = Rc.T @ Rp[:, 2]; x_cam = Rc.T @ Rp[:, 0]   # x_cam: 면 방향 정답 (선 사각기둥 θ 검증용)
                    ys, xs = np.nonzero(mask == p + 1)
                    color, shape, tag = tme.PUCKS[p]
                    pucks.append({'index': p, 'color': color, 'shape': shape, 'tag_id': tag, 'center_cam': c_cam.round(5).tolist(),
                                  'axis_cam': a_cam.round(5).tolist(), 'xaxis_cam': x_cam.round(5).tolist(), 'px': px,
                                  'bbox': [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
                                  'lying': bool(abs(Rp[2, 2]) < math.sqrt(0.5)), 'tag_faces': tag_on[p],
                                  'touching': p in in_cluster, 'on_floor': p in on_floor})
                    if args.check and c_cam[2] > 0:
                        u = int(round(K[0, 0] * c_cam[0] / c_cam[2] + K[0, 2])); w_ = int(round(K[1, 1] * c_cam[1] / c_cam[2] + K[1, 2]))
                        check_all += 1
                        x0, y0, x1, y1 = pucks[-1]['bbox']
                        check_ok += int(x0 - 3 <= u <= x1 + 3 and y0 - 3 <= w_ <= y1 + 3)
                stem = f'w{args.seed:02d}_{s:04d}_v{v}'
                cv2.imwrite(str(out / split / f'{stem}_rgb.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                np.save(out / split / f'{stem}_depth.npy', depth)
                cv2.imwrite(str(out / split / f'{stem}_mask.png'), mask)
                (out / split / f'{stem}_meta.json').write_text(json.dumps({
                    'K': K.tolist(), 'size': list(tme.D435_SIZE), 'cam_pos_w': cp.round(5).tolist(),
                    'cam_quat_w_ros_xyzw': cam.data.quat_w_ros.torch[0].cpu().numpy().round(6).tolist(),
                    'robot_xy_yaw': [float(xy[0]), float(xy[1]), yaw], 'arm_q': qa[0].cpu().numpy().round(4).tolist(),
                    'look_k': look_k, 'shelf': k_sh,
                    'scene': {'pano': pano, 'dome_k': round(dome_k, 3), 'dome_yaw_deg': round(math.degrees(dy), 1), 'ceil_k': round(ceil_k, 3),
                              'panel_rough': round(p_rough, 3), 'panel_ior': round(p_ior, 3), 'cam_tilt_deg': args.cam_tilt_deg,
                              'touch': bool(touch), 'n_blue': len(b_ids), 'floor': bool(floor_scene),
                              'place_tag': place_mode, 'grip': round(grip, 4), 'pose_try': tries_used,
                              'puck_rough': [round(float(a_.Get()), 3) for a_ in puck_rough],
                    },
                    'pucks': pucks}, indent=1, ensure_ascii=False))
                n_img += 1
            print(f'[wrist] 장면 {s + 1}/{args.scenes}: 누적 {n_img} 장, 이번 장 퍽 {len(pucks)}개 보임', flush=True)
    if args.check:
        print(f'[wrist] 검사: 정답 중심 투영이 퍽 상자 안 {check_ok}/{check_all}', flush=True)
    print(f'[wrist] 팔 자세 실패로 건너뛴 시점 {n_skip_pose}', flush=True)
    print('[wrist] 끝', flush=True)
    env.close()
