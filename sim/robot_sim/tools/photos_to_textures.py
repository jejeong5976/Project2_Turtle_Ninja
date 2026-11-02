#!/usr/bin/env python3
"""실제 촬영 사진(갤럭시 S23 울트라, 2026-10-23) -> 시뮬레이션용 배경·재질. 시스템 python3 (OpenCV, numpy).

입력: real_photos/ (실제 촬영 원본 중 쓰는 것만 복사, 수정 안 함)
  20261023_191331.jpg 등 파노라마 4장 (초광각 0.6×, 세로로 들고 한 바퀴 넘게 촬영 -> 360° 보다 김)
  20261023_185114/185116.jpg 벽판 정면, 20261023_185440/185453.jpg 바닥 카펫 + A4 용지
출력: usd/textures/
  pano_<이름>.jpg   2:1 위경도(latlong) 파노라마 8192×4096. 돔 조명 텍스처용
  carpet_tile.jpg  이음매 없는 카펫 텍스처 (실제 크기 CARPET_TILE_M 정사각)
  real_colors.json 벽판·파란 출력물·알루미늄 기둥 색 (sRGB 0–255 + 선형 0–1)
가정·한계:
  - 파노라마 360° 폭: 양 끝에 같은 사물함이 두 번 찍힌 위치로 추정 (191331: 10511 px, 세로 시야 약 115°). 몸을 축으로 돌려 찍어 시차가 있어 ±2° 정도 오차 가능
  - 수평선 = 사진 세로 가운데로 가정. 위·아래 빈 곳은 천장·바닥 평균색으로 채움 (바닥 쪽은 시뮬 바닥이 가려서 안 보임)
  - 방향(어느 열이 미로 북쪽인지)은 모름 -> 시뮬에서 dome_yaw 로 맞춤
  - A4 용지(210×297 mm)로 바닥 축척. 카펫 타일 이음·결 방향 무늬는 빠짐 (한 장 안쪽 질감만)
실행: python3 tools/photos_to_textures.py
"""
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parents[1]   # sim/robot_sim
SRC = HERE / 'real_photos'
OUT = HERE / 'usd' / 'textures'
OUT.mkdir(parents=True, exist_ok=True)
# 파일 끝 6자리: 360° 폭 (px). 191331 은 사물함 두 번 찍힌 위치로 측정 (세로 시야 115.1°).
# 나머지는 같은 폰·렌즈·세로 촬영이라 세로 시야가 같다고 보고 360 × 높이 / 115.1 로 계산 (손으로 돌려 찍은 시차로 자동 대조가 안 됨, 2026-10-24)
PANOS = {'191331': 10511, '185756': 10509, '191023': 10559, '191156': 10409}
EQ_W = 8192
CARPET_TILE_M = 0.40               # 타일 한 장(약 0.47 m) 안쪽만 잘라 이음 무늬 없이


def srgb2lin(c):
    c = np.asarray(c, float) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def make_pano(tag, P):
    im = cv2.imread(str(SRC / f'20261023_{tag}.jpg'))
    h, w = im.shape[:2]
    B = min(600, w - P)                                   # 이음매 섞는 폭
    body = im[:, :P].astype(np.float32)
    a = np.linspace(0, 1, B, dtype=np.float32)[None, :, None]
    body[:, :B] = im[:, :B] * a + im[:, P:P + B] * (1 - a)   # 처음 부분을 360° 뒤(겹친 부분)와 섞음
    vfov = 360.0 * h / P
    eh = EQ_W // 2
    ph = int(round(h * EQ_W / P))
    strip = cv2.resize(body, (EQ_W, ph), interpolation=cv2.INTER_AREA)
    top = (eh - ph) // 2
    eq = np.zeros((eh, EQ_W, 3), np.float32)
    eq[top:top + ph] = strip
    ceil = strip[: max(4, ph // 40)].mean((0, 1))
    floor = strip[-max(4, ph // 40):].mean((0, 1))
    F = 60                                                # 사진 끝 -> 평균색 섞는 줄 수
    eq[:top] = ceil
    eq[top + ph:] = floor
    for k in range(F):
        t = (k + 1) / (F + 1)
        eq[top - 1 - k] = strip[0] * (1 - t) + ceil * t
        eq[top + ph + k] = strip[-1] * (1 - t) + floor * t
    out = OUT / f'pano_{tag}.jpg'
    cv2.imwrite(str(out), np.clip(eq, 0, 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(f'{out.name}: 원본 {w}×{h}, 360° 폭 {P} px, 세로 시야 {vfov:.1f}° -> {EQ_W}×{eh} (사진 띠 {ph} px, 위·아래 채움 {top} px)')
    return {'file': out.name, 'period_px': P, 'vfov_deg': round(vfov, 1)}


def make_carpet():
    im = cv2.imread(str(SRC / '20261023_185453.jpg'))     # 4000×3000, 용지 왼쪽
    g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    _, th = cv2.threshold(g, 190, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x, y, cw, ch = cv2.boundingRect(max(cnts, key=cv2.contourArea))
    long_px, short_px = max(cw, ch), min(cw, ch)
    px_per_mm = (long_px / 297.0 + short_px / 210.0) / 2
    side = int(CARPET_TILE_M * 1000 * px_per_mm)
    # 카펫 타일 경계 (사진에서 눈으로 읽음): 세로 x≈2330, 가로 y≈1140·2490 -> 그 안쪽 한 장의 가운데
    x0, y0 = 2330 + (1358 - side) // 2, 1140 + (1350 - side) // 2
    crop = im[y0:y0 + side, x0:x0 + side].astype(np.float32)
    illum = cv2.GaussianBlur(crop, (0, 0), side / 6)       # 조명 기울기 제거
    crop = crop / np.maximum(illum, 1) * illum.mean((0, 1))
    # 이음매 없게: 반 칸 민 것과 섞기 (가장자리는 민 것 = 원래 가운데라 이어짐)
    rolled = np.roll(crop, (side // 2, side // 2), (0, 1))
    u = np.abs(np.linspace(-1, 1, side))
    wgt = (1 - np.maximum(u[:, None], u[None, :])) ** 0.7
    tile = crop * wgt[..., None] + rolled * (1 - wgt[..., None])
    tile = cv2.resize(tile, (1024, 1024), interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(OUT / 'carpet_tile.jpg'), np.clip(tile, 0, 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 92])
    mean_bgr = tile.mean((0, 1))
    print(f'carpet_tile.jpg: A4 {cw}×{ch} px -> {px_per_mm:.2f} px/mm, {CARPET_TILE_M} m = {side} px 자름, 평균 sRGB {mean_bgr[::-1].round()}')
    return {'file': 'carpet_tile.jpg', 'tile_m': CARPET_TILE_M, 'px_per_mm': round(px_per_mm, 3),
            'mean_srgb': [int(v) for v in mean_bgr[::-1]]}


def sample_colors():
    im = cv2.imread(str(SRC / '20261023_185116.jpg'))     # 벽판 정면 4000×3000
    h, w = im.shape[:2]
    panel = im[int(0.42 * h):int(0.58 * h), int(0.28 * w):int(0.48 * w)].reshape(-1, 3)   # 낙서 자국 피해서
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    blue = im[(hsv[..., 0] > 105) & (hsv[..., 0] < 130) & (hsv[..., 1] > 150) & (hsv[..., 2] > 90)]
    alu = im[int(0.45 * h):int(0.62 * h), int(0.205 * w):int(0.222 * w)].reshape(-1, 3)    # 왼쪽 3030 기둥 가운데
    res = {}
    for name, px in (('panel', panel), ('blue_print', blue), ('aluminum', alu)):
        srgb = np.median(px, 0)[::-1]
        res[name] = {'srgb': [int(v) for v in srgb], 'linear': [round(float(v), 4) for v in srgb2lin(srgb)], 'n_px': int(len(px))}
        print(f'{name:10s} sRGB {res[name]["srgb"]}  선형 {res[name]["linear"]}  (화소 {len(px)})')
    return res


if __name__ == '__main__':
    info = {'source': '갤럭시 S23 울트라 2026-10-23 촬영 (real_photos/)', 'panos': {}, 'carpet': make_carpet(), 'colors': sample_colors()}
    for tag, P in PANOS.items():
        info['panos'][tag] = make_pano(tag, P)
    (OUT / 'real_colors.json').write_text(json.dumps(info, indent=1, ensure_ascii=False))
