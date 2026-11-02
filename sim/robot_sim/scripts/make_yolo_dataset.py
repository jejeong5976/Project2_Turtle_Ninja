#!/usr/bin/env python3
"""YOLO 학습용 시뮬레이션 데이터셋 생성 (로봇 상단 광각 카메라 시점, 어안 왜곡 포함).

클래스 (perception 팀 정의, 이 순서가 YOLO 클래스 번호):
  0 red_cyl  1 green_cyl  2 red_box  3 green_box  4 barcode  5 traffic_light (신호등 몸통만, 기둥 프로파일 제외)
카메라: 1280×720, 팀 실측 K·어안 D (config/camera_wide.yaml = tme.BIRDEYE_K_FULL / BIRDEYE_D), 장착 위치·40° 숙임은 tme 값 (가정)
  기본(--distort cv): 2048×1216 핀홀(f 700)로 렌더 -> cv2.fisheye 모델로 1280×720 어안 영상을 직접 만듦 (Isaac 어안은 모서리가 비어서)
  렌더: 기본 PathTracing (--render pt). Real-Time 2.0 은 TAA 면 노이즈, DLSS 면 방사형 줄무늬 (2026-10-24 비교)
  로봇: 손목 D435·받침대 색을 실물 사진 색으로 덮어씀 (tme.apply_robot_real_colors, URDF·USD 파일은 그대로)
장면: 미로 (--maze: 실제 A 경기장 또는 tools/random_maze.py 무작위 미로, 실행 한 번에 하나) + 랜덤 소품 (reset 마다 env 별 새 배치) + 실제 강의실 파노라마 배경·카펫 (tme.use_real_look)
라벨: instance_id_segmentation 으로 보이는 화소만의 상자 (가려진 부분 제외). 화소 MIN_PX 미만은 버림
도메인 랜덤화 (장면마다): 파노라마 종류(같은 강의실 4곳, --pano)·방향(어느 쪽이 북쪽인지 모름 -> 무작위)·밝기(0.25–1.1배), 천장 사각 조명 6개 (tme.add_ceiling_lights, 세기 0.7–1.3배),
              벽판 광택·반사 (거칠기 0.02–0.15, 굴절률 2.2–3.0: 실제 유광 벽판처럼 퍽·박스·로봇이 비침), 퍽 색 (HSV 조금),
              선반 자리 (50 % 벽에서 0.05–0.35 m 뗌 -> 박스·퍽이 뒤 벽에 비침, 50 % 벽면 끝 = 모서리 -> 옆 벽에 비침. 실제 영상에 둘 다 있음)
              신호등 켜진 불 세기 (실제 영상에 맞춘 값의 0.7–1.5배, --tl_glow_min/max), 선반(골판지 박스) 색 (HSV 조금)
              (사진마다): 팔 자세 (접음 60 %, 나머지 URDF 순기구학으로 거른 무작위 자세, sim_core/arm_poses.py)
              (사진마다): 로봇 위치·방향 (25 % 는 벽면과 20–45° 로 비스듬히 = 반사가 센 각도), 저장 때 노이즈·흐림·감마·JPEG 품질
클래스 균형: 사진마다 '지금까지 상자가 적은 클래스' 의 물체를 우선 겨냥 (가중치 1/(개수+20)). 10 % 는 아무 데나, 10 % 는 가짜 물체를 겨냥
미탐(못 찾음) 대비 가정: 넘어진 퍽 (선반 위에 눕힘, --p_fallen), 바닥에 떨어진 퍽 (선반 앞, --p_floor), 불 켜진 신호등 (빨/노/초/꺼짐 무작위),
  먼 거리·가까운 거리 (0.25–1.8 m), 아주 어둡거나 밝은 조명, 화면 가장자리에 잘린 물체, 서로 가린 퍽
오탐(잘못 찾음) 대비 가정 (라벨 없음 = 배경으로 학습): 벽에 붙은 가짜 종이 3장 (QR·글자·체크무늬. 줄무늬는 진짜 바코드와 구분 안 돼 뺌, 2026-10-28),
  다른 모양의 색 물체 6개 (정육면체·공·원판, 빨강·초록 포함 무작위 색, 바닥 또는 선반 위), 장소용 AprilTag (원래 있음)
나누기: 배치(장면 × env) 단위로 train/val/test 80/10/10 -> 같은 배치의 다른 시점이 train·val 에 섞이지 않음
출력: <out>/images/{train,val,test}/*.jpg, labels/.../*.txt, data.yaml, meta.jsonl, preview/ (상자 그린 앞쪽 몇 장)
예) OMNI_KIT_ACCEPT_EULA=YES ~/pa-course/IsaacLab-6.2/.venv/bin/python scripts/make_yolo_dataset.py --scenes 400 --views 2 --num_envs 1 --maze rand_00
    -> 기본 출력 = 레포의 yolo/wide/datasets/sim (yolo/wide/scripts/prepare_data.py 의 기본 --sim 과 같음, .gitignore 의 datasets/ 로 커밋 안 됨)
    -> 1 × 400 × 2 = 800 장 (미로마다 --maze·--seed 를 바꿔 같은 폴더에 쌓음)
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
parser.add_argument('--maze', default='real', help="미로: real = 팀 실제 A 경기장, 그 밖 = tools/random_maze.py 가 만든 이름 (예 rand_03). 실행 한 번에 미로 하나")
parser.add_argument('--num_envs', type=int, default=2, help='8 GB GPU 에서 4 는 메모리 부족 (분할 이미지 깨짐, 2026-10-24 확인)')
parser.add_argument('--scenes', type=int, default=10, help='배치 다시 뽑는 횟수 (env 마다 다른 배치)')
parser.add_argument('--views', type=int, default=8, help='배치 하나에서 찍는 시점 수')
parser.add_argument('--out', default=str(Path(__file__).resolve().parents[3] / 'yolo' / 'wide' / 'datasets' / 'sim'))
parser.add_argument('--seed', type=int, default=0)
parser.add_argument('--min_px', type=int, default=60, help='보이는 화소가 이보다 적으면 라벨 안 함')
parser.add_argument('--pano', nargs='+', default=['all'],
                    help="배경 파노라마 (usd/textures/ 안 파일). 장면마다 무작위로 하나. 'all' = pano_*.jpg 전부 (같은 강의실 4곳에서 찍음)")
parser.add_argument('--preview', type=int, default=24, help='상자 그린 확인용 그림 장수')
parser.add_argument('--distort', choices=['cv', 'rtx'], default='cv',
                    help='cv = 넓은 핀홀로 렌더 후 OpenCV 어안(팀 K, D)으로 직접 왜곡 (화면 전체), rtx = Isaac 어안 (모서리 회색 원 밖 비어 있음)')
parser.add_argument('--render', choices=['pt', 'rt'], default='pt',
                    help='pt = PathTracing (spp 16 × 4 프레임 + OptiX 디노이저, 깨끗함, 장당 약 1.8 s), '
                         'rt = Real-Time 2.0 (빠름, TAA 면 벽·바닥이 오돌토돌, DLSS 면 방사형 줄무늬. 2026-10-24 비교)')
parser.add_argument('--pt_spp', type=int, default=16, help='PathTracing 프레임당 샘플 수')
parser.add_argument('--pt_total_spp', type=int, default=64, help='PathTracing 누적 샘플 수 (settle_steps × pt_spp 이하면 덜 수렴)')
parser.add_argument('--aa', default='TAA', choices=['TAA', 'FXAA', 'Off', 'DLSS', 'DLAA'],
                    help='--render rt 일 때 RTX 안티에일리어싱. DLSS 는 넓은 화각·카펫 무늬에서 방사형 줄무늬·벽 빛기둥이 생김, DLAA 는 이 GPU 에서 NGX 생성 실패 (2026-10-24 확인)')
parser.add_argument('--no_balance', action='store_true', help='클래스 균형 겨냥 끄기 (대상 무작위)')
parser.add_argument('--no_distractors', action='store_true', help='가짜 물체(오탐 대비) 끄기')
parser.add_argument('--p_fallen', type=float, default=0.2, help='퍽이 선반 위에 넘어져 있을 확률 (미탐 대비)')
parser.add_argument('--p_floor', type=float, default=0.1, help='퍽이 선반 앞 바닥에 떨어져 있을 확률 (미탐 대비)')
parser.add_argument('--dist_min', type=float, default=0.25, help='대상까지 최소 거리 (m)')
parser.add_argument('--dist_max', type=float, default=1.8, help='대상까지 최대 거리 (m)')
parser.add_argument('--dome_min', type=float, default=0.25, help='배경(파노라마 돔) 밝기 최소 배율. 천장 조명과 같이 씀')
parser.add_argument('--dome_max', type=float, default=1.1, help='배경 밝기 최대 배율')
parser.add_argument('--ceil_int', type=float, default=10000.0, help='천장 사각 조명 세기 (tme.CEIL_LIGHT_INT 와 같음, 0 = 끔). 끄면 --dome_min/max 0.4 1.8 이 예전 설정')
parser.add_argument('--ceil_k', type=float, nargs=2, default=[0.7, 1.3], help='천장 조명 세기 배율 범위 (장면마다)')
parser.add_argument('--panel_rough', type=float, nargs=2, default=[0.02, 0.15],
                    help='벽판 거칠기 범위 (장면마다). 작을수록 비친 상이 선명. 실제 반사 경계가 또렷해 0.15 까지 (0.3 은 상이 뭉개짐)')
parser.add_argument('--panel_ior', type=float, nargs=2, default=[2.2, 3.0],
                    help='벽판 굴절률 범위 (장면마다). 실제 벽판은 유광이라 퍽·박스·로봇이 비침 -> 반사율을 굴절률로 올림 (clearcoat 는 RTX 에서 효과 없었음). '
                         '실제 프레임 비친 박스 반사율 0.13 (G) = 렌더 ior 2.6 (2.0 -> 0.07~0.09, 2.3 -> 0.09, 2.6 -> 0.11~0.14). 2026-10-27 측정, 프레임 1장 기준')
parser.add_argument('--p_shelf_off', type=float, default=0.5, help='선반을 벽에서 떼는 확률 (뗀 박스·퍽이 뒤 벽에 비침)')
parser.add_argument('--shelf_off', type=float, nargs=2, default=[0.05, 0.35], help='선반을 벽에서 뗄 거리 범위 (m). 칸 안에 들어가지 않으면 반 · 0 으로 줄임')
parser.add_argument('--p_shelf_corner', type=float, default=0.5, help='선반을 벽면 끝(모서리 쪽)에 붙이는 확률 (옆 벽에 비침)')
parser.add_argument('--p_graze', type=float, default=0.25, help='대상을 벽면과 20–45° 로 비스듬히 보는 시점 확률 (반사가 센 각도)')
parser.add_argument('--tl_glow_min', type=float, default=0.7, help='신호등 불 세기 최소 배율 (tme.TL_LAMP = 실제 영상에 맞춘 값이 1)')
parser.add_argument('--tl_glow_max', type=float, default=1.5, help='신호등 불 세기 최대 배율 (카메라 노출 · 거리 차이 대비)')
parser.add_argument('--p_arm_tuck', type=float, default=0.6,
                    help='팔이 주행용 접은 자세일 확률 (주행·탐색 중엔 HOME 이 대부분). 나머지는 무작위 자세 (팔이 광각 화면에 보이기도·안 보이기도 함)')
parser.add_argument('--no_shelf_jitter', action='store_true', help='선반(골판지 박스) 색 흔들기 끄기 (기본은 장면마다 HSV 조금)')
parser.add_argument('--settle_steps', type=int, default=6, help='로봇 옮긴 뒤 렌더 갱신 스텝')
add_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
if args.maze != 'real':
    import os
    os.environ['TN_MAZE'] = args.maze           # sim_core import 전에 (turtleninja_maze_env.MAZE_NAME)
MAZE_TAG = '' if args.maze == 'real' else f'{args.maze}_'   # 파일 이름 앞에 붙여 미로끼리 안 겹치게

import cv2
import numpy as np
import torch

from isaaclab.utils import instantiate

from sim_core import turtleninja_maze_env as tme

CLASSES = ['red_cyl', 'green_cyl', 'red_box', 'green_box', 'barcode', 'traffic_light']
PUCK_CLASS = {(c, s): CLASSES.index(f'{c}_{s}') for c, s, _ in tme.PUCKS}
TL_PARTS = ('cap', 'div0', 'red', 'div1', 'yellow', 'div2', 'green')     # traffic_light_post.usda 의 몸통 부분 (profile 제외)
W, H = 1280, 720
ENV_SPACING = 15.0          # 이웃 env 미로가 배경에 크게 보이지 않게 (기본 5.5 m)

rng = np.random.default_rng(args.seed)
torch.manual_seed(args.seed)

cfg = tme.TurtleNinjaMazePropsEnvCfg()
cfg.scene.num_envs = args.num_envs
cfg.scene.env_spacing = ENV_SPACING
PANOS = sorted(p.name for p in tme.REAL_DIR.glob('pano_*.jpg')) if args.pano == ['all'] else args.pano
assert PANOS and all((tme.REAL_DIR / p).exists() for p in PANOS), f'파노라마 없음: {PANOS}'


def pano_gain(name):
    """파노라마 평균 밝기(선형)를 첫 장 기준으로 맞추는 돔 세기 배율 (사진마다 폰 자동 노출이 달라서)."""
    im = cv2.resize(cv2.imread(str(tme.REAL_DIR / name)), (1024, 512)).astype(np.float32) / 255.0
    return float(np.where(im <= 0.04045, im / 12.92, ((im + 0.055) / 1.055) ** 2.4).mean())


PANO_GAIN = {p: pano_gain(PANOS[0]) / pano_gain(p) for p in PANOS}
tme.use_real_look(cfg, PANOS[0])
# 핀홀 렌더 크기·초점: 어안 1280×720 모서리(약 57°)까지 덮게. 가운데 화소 밀도는 실제(fx 853)와 비슷하게
WP, HP, FP = 2048, 1216, 700.0
K_REAL = np.array([[tme.BIRDEYE_K_FULL['fx'], 0, tme.BIRDEYE_K_FULL['cx']], [0, tme.BIRDEYE_K_FULL['fy'], tme.BIRDEYE_K_FULL['cy']], [0, 0, 1.0]])
D_REAL = np.array(tme.BIRDEYE_D, float).reshape(4, 1)
if args.distort == 'cv':
    import isaaclab.sim as sim_utils
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    und = cv2.fisheye.undistortPoints(np.stack([uu, vv], -1).reshape(-1, 1, 2), K_REAL, D_REAL).reshape(H, W, 2)
    # cx 675 로 가운데가 아니라 필요한 범위가 비대칭 (왼쪽 x −1.57, 오른쪽 +1.17) -> 핀홀 주점을 옮겨 맞춤
    xl, xr, yt, yb = -und[..., 0].min(), und[..., 0].max(), -und[..., 1].min(), und[..., 1].max()
    CXP = FP * xl + (WP - FP * (xl + xr)) / 2
    CYP = FP * yt + (HP - FP * (yt + yb)) / 2
    tme.add_birdeye_camera(cfg, width=WP, height=HP, data_types=('rgb', 'instance_id_segmentation_fast'), fisheye=False)
    cfg.scene.birdeye.spawn = sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
        [FP, 0.0, CXP, 0.0, FP, CYP, 0.0, 0.0, 1.0], WP, HP, clipping_range=(0.02, 20.0))
    MAP_X = (FP * und[..., 0] + CXP).astype(np.float32)
    MAP_Y = (FP * und[..., 1] + CYP).astype(np.float32)
    assert MAP_X.min() >= 0 and MAP_X.max() < WP and MAP_Y.min() >= 0 and MAP_Y.max() < HP, '핀홀 렌더가 어안 화면을 다 못 덮음'
    MAP_XI = np.clip(np.rint(MAP_X).astype(np.int64), 0, WP - 1)
    MAP_YI = np.clip(np.rint(MAP_Y).astype(np.int64), 0, HP - 1)
else:
    tme.add_birdeye_camera(cfg, width=W, height=H, data_types=('rgb', 'instance_id_segmentation_fast'))
from isaaclab_tasks.utils.presets import set_isaac_rtx_global_settings
set_isaac_rtx_global_settings(cfg.scene.birdeye.renderer_cfg, antialiasing_mode=args.aa)
# 분할 출력은 색칠된 RGBA (colorize 끄기가 이 버전에서 안 먹음) -> 색을 32비트 정수로 묶어 ID 처럼 씀, 매핑 키도 (r, g, b, a)

# ---- 오탐 대비 가짜 물체 (라벨 없음) ----
PAPER_IDS = (1, 2, 3)   # distract_paper_0..3 = 줄무늬 · QR · 글자 · 체크무늬. 0(줄무늬)은 멀리서 진짜 바코드와 구분 안 돼 뺌 (2026-10-28)
SOLIDS = [('cube', 0.05), ('cube', 0.04), ('sphere', 0.02), ('sphere', 0.025), ('disc', (0.03, 0.012)), ('disc', (0.035, 0.010))]
PALETTE = {'red': (157, 34, 53), 'green': (45, 150, 72), 'orange': (220, 110, 30), 'yellow': (220, 200, 40), 'blue': (30, 70, 170),
           'beige': (200, 180, 140), 'white': (225, 225, 225), 'black': (30, 30, 30), 'pink': (220, 120, 150), 'lime': (140, 200, 60)}
if not args.no_distractors:
    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObjectCfg
    _kin = sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True)
    _park = lambda k: RigidObjectCfg.InitialStateCfg(pos=(-2.0 + 0.35 * k, -tme.MAZE_INNER_HALF[1] - 1.6, 0.1))
    for i, k in enumerate(PAPER_IDS):
        setattr(cfg.scene, f'distract_paper_{k}', RigidObjectCfg(
            prim_path=f'{{ENV_REGEX_NS}}/DistractPaper_{k}', init_state=_park(i),
            spawn=sim_utils.UsdFileCfg(usd_path=str(tme.PROPS_DIR / f'distract_paper_{k}.usda'))))
    for k, (kind, size) in enumerate(SOLIDS):
        vis = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.5, 0.5), roughness=0.5)
        if kind == 'cube':
            sp_ = sim_utils.CuboidCfg(size=(size,) * 3, rigid_props=_kin, visual_material=vis)
        elif kind == 'sphere':
            sp_ = sim_utils.SphereCfg(radius=size, rigid_props=_kin, visual_material=vis)
        else:
            sp_ = sim_utils.CylinderCfg(radius=size[0], height=size[1], rigid_props=_kin, visual_material=vis)
        setattr(cfg.scene, f'distract_solid_{k}', RigidObjectCfg(prim_path=f'{{ENV_REGEX_NS}}/DistractSolid_{k}',
                                                                 init_state=_park(len(PAPER_IDS) + k), spawn=sp_))
SOLID_HALF_H = [s_ / 2 if k_ == 'cube' else (s_ if k_ == 'sphere' else s_[1] / 2) for k_, s_ in SOLIDS]   # 바닥에서 중심까지

# 로봇 자리 검사용 2D 장애물 (미로 좌표)
WALL_C = np.array([b['center'][:2] for b in tme.LAYOUT['collision_boxes']])
WALL_H = np.array([[b['size'][0] / 2, b['size'][1] / 2] for b in tme.LAYOUT['collision_boxes']])
MOUNTS = np.array([m['xy'] for m in tme.LAYOUT['mounts']])
HX, HY = tme.MAZE_INNER_HALF
R_ROBOT = 0.24              # 접은 팔 포함 대략 반지름 (실물 미측정)

# 팔 무작위 자세 후보 (sim_core/arm_poses.py, URDF 순기구학으로 바닥·차체·카메라 기둥을 안 뚫는 자세만). 첫 항목 = 접은 자세
from sim_core import arm_poses
# 첫 항목 = 주행 기본자세(받침 + 그리퍼 16 mm). 무작위 자세의 차체 높이 기준은 P0 (받침 자세는 차체 안으로 내려가 있음)
ARM_BANK, _ = arm_poses.make_bank(tme.ROBOT_URDF, tme.TUCK_POSE, 400, rng, tme.PROFILE_XY,
                                  body_ref={**tme.READY_POSE, **{f: 0.0 for f in arm_poses.FINGERS}})
print(f'[yolo] 팔 자세 후보 {len(ARM_BANK) - 1}개 (+ 접은 자세), 접은 확률 {args.p_arm_tuck}', flush=True)


def box_dist(p, c, h):
    q = np.abs(p[None] - c) - h
    return float((np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(1), 0)).min())


def free(p, shelves, tl, floor_obs=(), r_robot=R_ROBOT):
    """로봇 자리 p 가 벽·기둥·선반·바닥 물체·신호등과 r_robot 이상 떨어졌는지 (팔을 뻗으면 r_robot 을 키움)."""
    if abs(p[0]) > HX - r_robot or abs(p[1]) > HY - r_robot:
        return False
    if box_dist(p, WALL_C, WALL_H) < r_robot or np.linalg.norm(MOUNTS - p, axis=1).min() < r_robot + 0.03:
        return False
    for s in shelves:
        if np.linalg.norm(p - s) < r_robot + tme.SHELF_SIZE[0] * 0.72:
            return False
    for q, r in floor_obs:                       # 바닥에 놓인 퍽·가짜 물체
        if np.linalg.norm(p - q) < r_robot + r:
            return False
    return np.linalg.norm(p - tl) > r_robot + 0.05


def srgb2lin(rgb):
    c = np.asarray(rgb, float) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def yaw_of(q):                       # (x, y, z, w)
    return math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))


def augment(img):
    """저장 전 실제 카메라 비슷하게: 감마, 노이즈, 흐림, JPEG 품질 (값은 가정)."""
    p = {'gamma': float(rng.uniform(0.8, 1.25)), 'noise': float(rng.uniform(0, 6)),
         'blur': int(rng.choice([0, 0, 0, 3, 5])), 'jpeg': int(rng.integers(70, 96))}
    x = 255.0 * (img.astype(np.float32) / 255.0) ** p['gamma']
    x += rng.normal(0, p['noise'], x.shape)
    x = np.clip(x, 0, 255).astype(np.uint8)
    if p['blur']:
        x = cv2.GaussianBlur(x, (p['blur'], p['blur']), 0)
    return x, p


def hsv_jitter(lin):
    """선형 RGB -> sRGB HSV 에서 조금 흔듦 -> 선형."""
    srgb = np.where(lin <= 0.0031308, lin * 12.92, 1.055 * np.power(np.clip(lin, 1e-6, 1), 1 / 2.4) - 0.055)
    hsv = cv2.cvtColor(np.uint8(np.clip(srgb * 255, 0, 255))[None, None], cv2.COLOR_RGB2HSV_FULL).astype(np.float32)[0, 0]
    hsv[0] = (hsv[0] + rng.uniform(-6, 6)) % 256
    hsv[1] = np.clip(hsv[1] * rng.uniform(0.85, 1.1), 0, 255)
    hsv[2] = np.clip(hsv[2] * rng.uniform(0.8, 1.15), 0, 255)
    s = cv2.cvtColor(np.uint8(hsv)[None, None], cv2.COLOR_HSV2RGB_FULL)[0, 0] / 255.0
    return np.where(s <= 0.04045, s / 12.92, ((s + 0.055) / 1.055) ** 2.4)


out = Path(args.out)
for sp in ('train', 'val', 'test'):
    (out / 'images' / sp).mkdir(parents=True, exist_ok=True)
    (out / 'labels' / sp).mkdir(parents=True, exist_ok=True)
(out / 'preview').mkdir(parents=True, exist_ok=True)
(out / 'data.yaml').write_text(f'path: {out.resolve()}\ntrain: images/train\nval: images/val\ntest: images/test\n'
                               f'nc: {len(CLASSES)}\nnames: {CLASSES}\n')
PAT = re.compile(r'/World/envs/env_(\d+)/(Puck_(\d+)|Barcode_\d+|TrafficLightPost)(/[^/]+)?')

with launch_simulation(cfg, args):
    env = instantiate(cfg)
    import omni.usd
    from pxr import Gf, Sdf
    stage = omni.usd.get_context().get_stage()
    if args.render == 'pt':
        # 렌더러가 뜬 뒤에 바꿔야 먹음 (renderer_cfg.global_settings.carb_settings 로 주면 RealTimePathTracing 으로 덮어써짐)
        import carb
        _cs = carb.settings.get_settings()
        for _k, _v in (('/rtx/rendermode', 'PathTracing'), ('/rtx/pathtracing/spp', args.pt_spp),
                       ('/rtx/pathtracing/totalSpp', args.pt_total_spp), ('/rtx/pathtracing/optixDenoiser/enabled', True)):
            _cs.set(_k, _v)
        print(f'[yolo] 렌더 PathTracing spp {args.pt_spp} / 누적 {args.pt_total_spp}, OptiX 디노이저', flush=True)
    ceil_attrs = tme.add_ceiling_lights(stage, env.scene.env_origins, args.ceil_int) if args.ceil_int > 0 else []
    print(f'[yolo] 천장 조명 {len(ceil_attrs)}개, 세기 {args.ceil_int} × {args.ceil_k}', flush=True)
    print('[yolo] 로봇 실물 색:', tme.apply_robot_real_colors(stage), flush=True)
    crank = tme.add_crank_visuals(stage)   # 그리퍼 크랭크 · 막대를 손가락 위치에 맞춤 (배포판 CrankVisuals)
    n, dev = env.num_envs, env.device
    ids = torch.arange(n, device=dev)
    org = env.scene.env_origins
    robot = env.scene['robot']
    cam = env.scene['birdeye']
    act = torch.zeros(n, env.action_manager.total_action_dim, device=dev)
    light = stage.GetPrimAtPath('/World/light')
    base_int = light.GetAttribute('inputs:intensity').Get()
    orient = light.GetAttribute('xformOp:orient')
    dome_tex = light.GetAttribute('inputs:texture:file')
    print(f'[yolo] 배경 파노라마 {len(PANOS)}장, 밝기 보정 {({p: round(g, 3) for p, g in PANO_GAIN.items()})}', flush=True)
    puck_sh = {(e, k): stage.GetPrimAtPath(f'/World/envs/env_{e}/Puck_{k}/Looks/body/Surface').GetAttribute('inputs:diffuseColor')
               for e in range(n) for k in range(len(tme.PUCKS))}
    puck_base = {key: np.array(a.Get(), float) for key, a in puck_sh.items()}
    panel_sh = [stage.GetPrimAtPath(f'/World/envs/env_{e}/Maze/Looks/panel/Surface') for e in range(n)]
    panel_rough = [sh.GetAttribute('inputs:roughness') for sh in panel_sh]
    panel_ior = [sh.CreateAttribute('inputs:ior', Sdf.ValueTypeNames.Float) for sh in panel_sh]
    from pxr import Sdf, Usd

    def find_attr(root, name):
        for q in Usd.PrimRange(stage.GetPrimAtPath(root)):
            a = q.GetAttribute(name)
            if a and a.IsValid() and a.HasAuthoredValue():
                return a
        return None
    # 신호등 겉모습 = 실제 영상에 맞춘 값 (tme.TL_LAMP: 렌즈 색 · 켜진 칸 흰색 포화 · 주변 번짐 조명)
    tl_look = [tme.TrafficLightLook(stage, f'/World/envs/env_{e}/TrafficLightPost',
                                    {c: (f'/World/envs/env_{e}/TrafficLightPost/{c}', f'/World/envs/env_{e}/TrafficLightPost/Looks/{c}/Surface')
                                     for c in ('red', 'yellow', 'green')}) for e in range(n)]
    solid_col = {}
    if not args.no_distractors:
        solid_col = {(e, k): find_attr(f'/World/envs/env_{e}/DistractSolid_{k}', 'inputs:diffuseColor')
                     for e in range(n) for k in range(len(SOLIDS))}
    shelf_top = tme.SHELF_SIZE[2]
    shelf_col = {(e, k): find_attr(f'/World/envs/env_{e}/Shelf_{k}', 'inputs:diffuseColor')
                 for e in range(n) for k in range(tme.NUM_SHELVES)}
    shelf_base = np.array(tme._srgb_to_linear(tme.SHELF_SRGB), float)
    arm_ids = robot.find_joints(arm_poses.ARM_JOINTS + arm_poses.FINGERS, preserve_order=True)[0]
    print(f'[yolo] 미로 {args.maze} ({tme.MAZE_USD.name}, 칸 {tme.LAYOUT["grid"]["nx"]}×{tme.LAYOUT["grid"]["ny"]}), envs {n}, 장면 {args.scenes} × 시점 {args.views} -> 최대 {n * args.scenes * args.views} 장, 출력 {out.resolve()}', flush=True)

    meta_f = open(out / 'meta.jsonl', 'a')
    counts = {sp: 0 for sp in ('train', 'val', 'test')}
    cls_count = np.zeros(len(CLASSES), int)
    n_prev = 0
    with torch.inference_mode():
        for s in range(args.scenes):
            env.reset()
            # ---- 장면 단위 랜덤화 ----
            dome_yaw = float(rng.uniform(0, 360)); dome_k = float(rng.uniform(args.dome_min, args.dome_max))
            pano = PANOS[int(rng.integers(len(PANOS)))]
            dome_tex.Set(Sdf.AssetPath(str(tme.REAL_DIR / pano)))
            light.GetAttribute('inputs:intensity').Set(base_int * dome_k * PANO_GAIN[pano])
            qv = (math.cos(math.radians(dome_yaw) / 2), 0.0, 0.0, math.sin(math.radians(dome_yaw) / 2))
            orient.Set(Gf.Quatd(*qv) if 'quatd' in str(orient.GetTypeName()) else Gf.Quatf(*qv))
            ceil_k = float(rng.uniform(*args.ceil_k))
            for a_ in ceil_attrs:
                a_.Set(args.ceil_int * ceil_k)
            rough = [float(rng.uniform(*args.panel_rough)) for _ in range(n)]
            ior = [float(rng.uniform(*args.panel_ior)) for _ in range(n)]
            for e in range(n):
                panel_rough[e].Set(rough[e])
                panel_ior[e].Set(ior[e])
                for k in range(len(tme.PUCKS)):
                    puck_sh[(e, k)].Set(Gf.Vec3f(*hsv_jitter(puck_base[(e, k)]).tolist()))
                if not args.no_shelf_jitter:             # 골판지 박스 색 (선반 전부 같은 박스로 가정)
                    sc = hsv_jitter(shelf_base)
                    for k in range(tme.NUM_SHELVES):
                        if shelf_col[(e, k)] is not None:
                            shelf_col[(e, k)].Set(Gf.Vec3f(*sc.tolist()))
            split = [str(rng.choice(['train', 'val', 'test'], p=[0.8, 0.1, 0.1])) for _ in range(n)]
            st = tme.maze_state(env)
            slots = {k_: v_.cpu().numpy() for k_, v_ in st['slots'].items()}
            scene_info, floor_obs, distract_xy = [], [], []
            # 선반 자리: 벽에서 떼기 · 벽면 끝(모서리). tme.maze_state 는 벽에 붙여 놓으므로 여기서 옮기고 위의 퍽도 같이 옮김
            shelf_off = [[0.0] * tme.NUM_SHELVES for _ in range(n)]
            for e in range(n):
                o = org[e, :2].cpu().numpy()
                eid = torch.tensor([e], device=dev)
                pk = [env.scene[f'puck_{k}'].data.root_pos_w.torch[e].cpu().numpy() for k in range(len(tme.PUCKS))]
                for k in range(tme.NUM_SHELVES):
                    if not bool(st['shelf_active'][e, k]):
                        continue
                    si = int(st['shelf_slots'][e, k]); yw = float(slots['yaw'][si])
                    nrm = np.array([math.cos(yw), math.sin(yw)]); tan = np.array([-nrm[1], nrm[0]])
                    old = st['shelf_xy'][e, k].cpu().numpy()
                    along = float((old - slots['xy'][si]) @ tan)
                    if rng.random() < args.p_shelf_corner:
                        along = float(np.sign(along) or 1.0) * (slots['len'][si] / 2 - tme.SHELF_SIZE[1] / 2 - 0.01)
                    d = float(rng.uniform(*args.shelf_off)) if rng.random() < args.p_shelf_off else 0.0
                    hx, hy = tme.SHELF_SIZE[0] / 2, tme.SHELF_SIZE[1] / 2
                    for d_ in (d, d * 0.5, 0.0):             # 벽 · 기둥 마운트와 겹치면 줄임 (0 = 원래처럼 벽에 붙임)
                        new = slots['xy'][si] + nrm * (hx + 0.001 + d_) + tan * along
                        edge = [new + nrm * a_ * hx + tan * b_ * hy for a_ in (-1, 0, 1) for b_ in (-1, 0, 1) if a_ or b_]
                        if d_ == 0.0 or all(box_dist(c_, WALL_C, WALL_H) > 0.005 and np.linalg.norm(MOUNTS - c_, axis=1).min() > 0.035 for c_ in edge):
                            break
                    shelf_off[e][k] = d_
                    delta = new - old
                    sh_ = env.scene[f'shelf_{k}']
                    pose = sh_.data.root_pose_w.torch[e:e + 1].clone(); pose[0, :2] += torch.tensor(delta, device=dev, dtype=pose.dtype)
                    sh_.write_root_pose_to_sim_index(root_pose=pose, env_ids=eid)
                    for j in range(len(tme.PUCKS)):           # 이 선반 위 퍽
                        if np.linalg.norm(pk[j][:2] - o - old) < 0.2 and pk[j][2] > shelf_top:
                            pp = env.scene[f'puck_{j}'].data.root_pose_w.torch[e:e + 1].clone(); pp[0, :2] += torch.tensor(delta, device=dev, dtype=pp.dtype)
                            env.scene[f'puck_{j}'].write_root_pose_to_sim_index(root_pose=pp, env_ids=eid)
                            env.scene[f'puck_{j}'].write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=dev), env_ids=eid)
                    st['shelf_xy'][e, k] = torch.tensor(new, device=dev, dtype=st['shelf_xy'].dtype)
            for _ in range(2):                                   # 옮긴 선반 · 퍽 위치를 data 에 반영 (아래 퍽 넘어뜨리기가 읽음)
                env.step(act)
            for e in range(n):
                o = org[e, :2].cpu().numpy()
                eid = torch.tensor([e], device=dev)
                act_sh = [k for k in range(tme.NUM_SHELVES) if bool(st['shelf_active'][e, k])]
                sxy = {k: st['shelf_xy'][e, k].cpu().numpy() for k in act_sh}
                syaw = {k: float(st['shelf_yaw'][e, k]) for k in act_sh}
                fobs, modes = [], []

                def put(name, xy, z, roll, yaw):
                    q = tme.quat_from_euler_xyz(torch.tensor([roll], device=dev), torch.zeros(1, device=dev), torch.tensor([yaw], device=dev))
                    pose = torch.cat([torch.tensor([[xy[0] + o[0], xy[1] + o[1], z]], device=dev, dtype=torch.float32), q], -1)
                    env.scene[name].write_root_pose_to_sim_index(root_pose=pose, env_ids=eid)
                    if name.startswith('puck'):
                        env.scene[name].write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=dev), env_ids=eid)
                # 퍽: 넘어짐 / 바닥 / 그대로 (미탐 대비)
                for k in range(len(tme.PUCKS)):
                    pxy = env.scene[f'puck_{k}'].data.root_pos_w.torch[e, :2].cpu().numpy() - o
                    ks = min(act_sh, key=lambda j: np.linalg.norm(sxy[j] - pxy)) if act_sh else None
                    r = rng.random()
                    if ks is not None and r < args.p_floor:
                        nrm = np.array([math.cos(syaw[ks]), math.sin(syaw[ks])]); tan = np.array([-nrm[1], nrm[0]])
                        fxy = sxy[ks] + nrm * (tme.SHELF_SIZE[0] / 2 + rng.uniform(0.06, 0.18)) + tan * rng.uniform(-0.12, 0.12)
                        lying = rng.random() < 0.5
                        put(f'puck_{k}', fxy, tme.PUCK_D / 2 + 0.001 if lying else tme.PUCK_H / 2 + 0.001,
                            math.pi / 2 if lying else 0.0, rng.uniform(-math.pi, math.pi))
                        fobs.append((fxy, 0.04)); modes.append('floor_lying' if lying else 'floor')
                    elif r < args.p_floor + args.p_fallen:
                        put(f'puck_{k}', pxy, shelf_top + tme.PUCK_D / 2 + 0.002, math.pi / 2, rng.uniform(-math.pi, math.pi))
                        modes.append('fallen')
                    else:
                        modes.append('upright')
                # 신호등 불 (꺼짐 / 빨 / 노 / 초), 켜진 불 세기는 배율 무작위 (실물 LED 포화·번짐, 카메라 노출 차이 대비)
                tl_state = str(rng.choice(['off', 'red', 'yellow', 'green']))
                tl_glow_k = float(rng.uniform(args.tl_glow_min, args.tl_glow_max))
                tl_look[e].set(None if tl_state == 'off' else tl_state, tl_glow_k)
                # 가짜 물체 (오탐 대비)
                dxy = []
                if not args.no_distractors:
                    for k in PAPER_IDS:                          # 벽에 붙인 가짜 종이
                        si = int(rng.integers(len(slots['yaw'])))
                        yw = float(slots['yaw'][si]); nrm = np.array([math.cos(yw), math.sin(yw)]); tan = np.array([-nrm[1], nrm[0]])
                        along = rng.uniform(-1, 1) * max(0.0, slots['len'][si] / 2 - 0.06)
                        pxy = slots['xy'][si] + nrm * 0.0015 + tan * along
                        put(f'distract_paper_{k}', pxy, float(rng.uniform(0.10, 0.34)), 0.0, yw)
                        dxy.append((pxy, yw))
                    empty = [j for j in act_sh if j >= int(st['n_loaded_shelves'][e])]   # 빈 선반
                    for k in range(len(SOLIDS)):                 # 색 물체: 바닥 또는 선반 위
                        col = PALETTE[str(rng.choice(list(PALETTE)))]
                        if solid_col.get((e, k)) is not None:
                            solid_col[(e, k)].Set(Gf.Vec3f(*(np.minimum(1.0, 2.4 * srgb2lin(col))).tolist()))
                        if act_sh and rng.random() < 0.4:
                            j = int(rng.choice(empty)) if empty else int(rng.choice(act_sh))
                            nrm = np.array([math.cos(syaw[j]), math.sin(syaw[j])]); tan = np.array([-nrm[1], nrm[0]])
                            pxy = sxy[j] + nrm * rng.uniform(-0.09, 0.09) + tan * rng.uniform(-0.09, 0.09)
                            put(f'distract_solid_{k}', pxy, shelf_top + SOLID_HALF_H[k] + 0.001, 0.0, rng.uniform(-math.pi, math.pi))
                            dxy.append((pxy, syaw[j]))
                        else:
                            for _ in range(30):
                                pxy = rng.uniform([-HX + 0.1, -HY + 0.1], [HX - 0.1, HY - 0.1])
                                if box_dist(pxy, WALL_C, WALL_H) > 0.06 and all(np.linalg.norm(pxy - sxy[j]) > 0.25 for j in act_sh):
                                    break
                            put(f'distract_solid_{k}', pxy, SOLID_HALF_H[k] + 0.001, 0.0, rng.uniform(-math.pi, math.pi))
                            fobs.append((pxy, 0.05)); dxy.append((pxy, float(rng.uniform(-math.pi, math.pi))))
                floor_obs.append(fobs); distract_xy.append(dxy)
                scene_info.append({'tl_state': tl_state, 'tl_glow_k': tl_glow_k, 'puck_modes': modes})
            for _ in range(3):                                   # 옮긴 퍽 내려앉기
                env.step(act)
            for v in range(args.views):
                # ---- 로봇 자리: 대상 물체를 보게 ----
                poses, arm_pick = [], []
                for e in range(n):
                    o = org[e, :2].cpu().numpy()
                    shelves = [st['shelf_xy'][e, k].cpu().numpy() for k in range(tme.NUM_SHELVES) if bool(st['shelf_active'][e, k])]
                    tl_pose = env.scene['traffic_light'].data.root_pose_w.torch[e].cpu().numpy()
                    tl = tl_pose[:2] - o
                    # 대상 후보: (클래스, 위치, 다가갈 방향 yaw). 퍽은 가까운 선반의 정면 방향
                    act_sh = [k for k in range(tme.NUM_SHELVES) if bool(st['shelf_active'][e, k])]
                    by_cls = {c: [] for c in range(len(CLASSES))}
                    for k in range(len(tme.PUCKS)):
                        pxy = env.scene[f'puck_{k}'].data.root_pos_w.torch[e, :2].cpu().numpy() - o
                        ks = min(act_sh, key=lambda j: np.linalg.norm(st['shelf_xy'][e, j].cpu().numpy() - pxy)) if act_sh else None
                        c_, sh_, _ = tme.PUCKS[k]
                        by_cls[PUCK_CLASS[(c_, sh_)]].append((pxy, float(st['shelf_yaw'][e, ks]) if ks is not None else rng.uniform(-math.pi, math.pi)))
                    for b in range(tme.NUM_BARCODES):
                        bp = env.scene[f'barcode_{b}'].data.root_pose_w.torch[e].cpu().numpy()
                        by_cls[4].append((bp[:2] - o, yaw_of(bp[3:])))
                    by_cls[5].append((tl, yaw_of(tl_pose[3:])))
                    present = [c for c in by_cls if by_cls[c]]
                    w_ = np.array([1.0 / (cls_count[c] + 20.0) for c in present]); w_ /= w_.sum()
                    pose = None
                    arm_k = 0 if rng.random() < args.p_arm_tuck else int(rng.integers(1, len(ARM_BANK)))
                    r_rob = max(R_ROBOT, ARM_BANK[arm_k][1] + 0.05)   # 팔 뻗은 만큼 벽·선반에서 더 떨어짐
                    for tri in range(90):
                        if tri == 60 and arm_k:          # 팔 뻗은 채로 자리를 못 찾으면 접고 다시 (30번 더)
                            arm_k, r_rob = 0, R_ROBOT
                        u = rng.random()
                        if u < 0.10:                            # 아무 데나 (대상 없는 사진)
                            p = rng.uniform([-HX, -HY], [HX, HY]); yaw = rng.uniform(-math.pi, math.pi); tgt = 'none'
                        else:
                            if u < 0.20 and distract_xy[e]:     # 가짜 물체 겨냥 (오탐 대비)
                                txy, tyaw = distract_xy[e][rng.integers(len(distract_xy[e]))]; tgt = 'distractor'
                            else:
                                c = present[rng.choice(len(present), p=w_)] if not args.no_balance else present[rng.integers(len(present))]
                                txy, tyaw = by_cls[c][rng.integers(len(by_cls[c]))]; tgt = CLASSES[c]
                            # 카메라가 아래로 40° 숙어져 있어 신호등(0.33–0.45 m)은 약 0.8 m 넘으면 화면 위로 벗어남 -> 가까이서
                            d_hi = min(args.dist_max, 0.8) if tgt == 'traffic_light' else args.dist_max
                            d = rng.uniform(max(args.dist_min, 0.4) if tgt == 'barcode' else args.dist_min, d_hi)
                            if rng.random() < args.p_graze:     # 벽면과 20–45° (tyaw = 벽 법선이라 법선에서 45–72°)
                                a = tyaw + float(rng.choice([-1, 1])) * rng.uniform(0.8, 1.25)
                            else:
                                a = tyaw + rng.uniform(-0.9, 0.9)
                            p = txy + d * np.array([math.cos(a), math.sin(a)])
                            yaw = math.atan2(*(txy - p)[::-1]) + rng.uniform(-0.45, 0.45)
                        if free(p, shelves, tl, floor_obs[e], r_rob):
                            pose = (float(p[0]), float(p[1]), float(yaw), tgt)
                            break
                    if pose is None:                     # 못 찾으면 reset 때 자리 그대로, 팔은 접음
                        xy = (robot.data.root_pos_w.torch[e, :2] - org[e, :2]).cpu().numpy()
                        pose = (float(xy[0]), float(xy[1]), float(tme.base_yaw(env)[e]), 'keep')
                        arm_k = 0
                    poses.append(pose)
                    arm_pick.append(arm_k)
                P = torch.tensor([[p[0], p[1], tme.SPAWN_Z] for p in poses], device=dev) + org
                yaw_t = torch.tensor([p[2] for p in poses], device=dev)
                z0 = torch.zeros_like(yaw_t)
                robot.write_root_pose_to_sim_index(root_pose=torch.cat([P, tme.quat_from_euler_xyz(z0, z0, yaw_t)], -1), env_ids=ids)
                robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
                # 팔 자세: 관절값을 바로 쓰고 위치 목표도 같게 (settle 동안 유지)
                qa = torch.tensor([[ARM_BANK[k][0][j] for j in arm_poses.ARM_JOINTS + arm_poses.FINGERS] for k in arm_pick],
                                  device=dev, dtype=torch.float32)
                robot.write_joint_position_to_sim_index(position=qa, joint_ids=arm_ids, env_ids=ids)
                robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(qa), joint_ids=arm_ids, env_ids=ids)
                robot.actuators.target_command.set_position_index(value=qa, joint_ids=arm_ids, env_ids=ids)
                crank.update(qa[:, -2:].mean(1).cpu().numpy())
                for _ in range(args.settle_steps):
                    env.step(act)
                rgb = cam.data.output['rgb'][..., :3].cpu().numpy().astype(np.uint8)
                sg = cam.data.output['instance_id_segmentation_fast'].cpu().numpy().astype(np.uint32)
                seg = sg[..., 0] | (sg[..., 1] << 8) | (sg[..., 2] << 16) | (sg[..., 3] << 24) if sg.shape[-1] == 4 else sg[..., 0]
                if args.distort == 'cv':                 # 핀홀 -> 어안 (색: 선형 보간, 분할: 가장 가까운 화소)
                    rgb = np.stack([cv2.remap(im, MAP_X, MAP_Y, cv2.INTER_LINEAR) for im in rgb])
                    seg = seg[:, MAP_YI, MAP_XI]
                id2path = {}
                for key, path in cam.data.info['instance_id_segmentation_fast']['idToLabels'].items():
                    if isinstance(key, tuple):
                        key = int(key[0]) | (int(key[1]) << 8) | (int(key[2]) << 16) | (int(key[3]) << 24)
                    id2path[int(key)] = path
                # ---- 라벨 ----
                for e in range(n):
                    objs = {}
                    for iid in np.unique(seg[e]):
                        path = id2path.get(int(iid), '')
                        m = PAT.match(str(path))
                        if not m or int(m.group(1)) != e:
                            continue
                        name, child = m.group(2), (m.group(4) or '').strip('/')
                        if name.startswith('Puck_'):
                            c, sh, _ = tme.PUCKS[int(m.group(3))]; cls = PUCK_CLASS[(c, sh)]
                        elif name.startswith('Barcode'):
                            cls = 4
                        else:
                            if child not in TL_PARTS:
                                continue
                            cls = 5
                        ys, xs = np.nonzero(seg[e] == iid)
                        o_ = objs.setdefault(name, [cls, 0, W, H, 0, 0])
                        o_[1] += len(xs); o_[2] = min(o_[2], xs.min()); o_[3] = min(o_[3], ys.min())
                        o_[4] = max(o_[4], xs.max()); o_[5] = max(o_[5], ys.max())
                    lines = []
                    for name, (cls, npx, x0, y0, x1, y1) in objs.items():
                        if npx < args.min_px or x1 - x0 < 3 or y1 - y0 < 3:
                            continue
                        lines.append((cls, (x0 + x1 + 1) / 2 / W, (y0 + y1 + 1) / 2 / H, (x1 - x0 + 1) / W, (y1 - y0 + 1) / H, name, npx))
                    img, aug = augment(rgb[e])
                    stem = f'{MAZE_TAG}s{args.seed:02d}_{s:04d}_e{e}_v{v}'
                    sp = split[e]
                    cv2.imwrite(str(out / 'images' / sp / f'{stem}.jpg'), cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, aug['jpeg']])
                    (out / 'labels' / sp / f'{stem}.txt').write_text(''.join(f'{l[0]} {l[1]:.6f} {l[2]:.6f} {l[3]:.6f} {l[4]:.6f}\n' for l in lines))
                    counts[sp] += 1
                    for l in lines:
                        cls_count[l[0]] += 1
                    meta_f.write(json.dumps({'image': f'{sp}/{stem}.jpg', 'scene': s, 'env': e, 'view': v, 'robot': poses[e][:3],
                                             'target': poses[e][3], 'maze': args.maze, 'arm_pose': arm_pick[e], 'pano': pano, 'dome_yaw': dome_yaw, 'dome_k': dome_k, 'panel_rough': rough[e], 'panel_ior': ior[e],
                                             'ceil_int': args.ceil_int * ceil_k if ceil_attrs else 0.0, 'shelf_off_m': shelf_off[e],
                                             'aug': aug, **scene_info[e], 'objects': [{'cls': CLASSES[l[0]], 'prim': l[5], 'px': int(l[6])} for l in lines]},
                                            ensure_ascii=False) + '\n')
                    if n_prev < args.preview:
                        pv = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        for cls, cx, cy, bw, bh, name, npx in lines:
                            x0, y0 = int((cx - bw / 2) * W), int((cy - bh / 2) * H)
                            x1, y1 = int((cx + bw / 2) * W), int((cy + bh / 2) * H)
                            cv2.rectangle(pv, (x0, y0), (x1, y1), (0, 255, 255), 2)
                            cv2.putText(pv, CLASSES[cls], (x0, max(15, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.imwrite(str(out / 'preview' / f'{stem}.jpg'), pv)
                        n_prev += 1
            print(f'[yolo] 장면 {s + 1}/{args.scenes}: 누적 {counts}, 클래스별 상자 {dict(zip(CLASSES, cls_count.tolist()))}', flush=True)
    meta_f.close()
    (out / 'summary.json').write_text(json.dumps({'images': counts, 'boxes_per_class': dict(zip(CLASSES, cls_count.tolist())),
                                                  'assumptions': {'p_fallen': args.p_fallen, 'p_floor': args.p_floor,
                                                                  'distractors': not args.no_distractors, 'balance': not args.no_balance,
                                                                  'dist_m': [args.dist_min, args.dist_max], 'dome': [args.dome_min, args.dome_max],
                                                                  'ceil': [args.ceil_int, args.ceil_k], 'panel_ior': args.panel_ior, 'panel_rough': args.panel_rough,
                                                                  'shelf_off': [args.p_shelf_off, args.shelf_off], 'p_shelf_corner': args.p_shelf_corner, 'p_graze': args.p_graze,
                                                                  'tl_glow': [args.tl_glow_min, args.tl_glow_max]},
                                                  'args': vars(args)}, indent=1, ensure_ascii=False, default=str))
    print('[yolo] 끝', flush=True)
    env.close()
