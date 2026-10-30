#!/usr/bin/env python3
"""표식 텍스처 PNG 생성 (시스템 python3 + OpenCV). 출력: usd/props/tex/

AprilTag: tag36h11 (팀 배포 표식 이미지를 OpenCV 로 검출해 계열·ID 확인: ID 0–14 모두 36h11).
  검은 사각형 = 8×8 칸 (6×6 데이터 + 검은 테두리 1칸), 흰 여백 1칸 -> 라벨 10×10 칸.
  퍽용: 검은 16 mm / 라벨 20 mm (칸 2 mm), 장소용: 검은 48 mm / 라벨 60 mm (칸 6 mm).
  R1–R4 = ID 0–3, G1–G4 = ID 4–7, (B1–B4 = ID 8–11 은 파랑 퍽이 빠져서 안 씀), L1–L3 = ID 12–14
바코드: Code128 'P2-01'…'P2-04'. 팀 배포 이미지에서 막대 폭을 읽어 복원 (모듈 90개 = 시작 B + 5글자 + 체크 + 정지,
  각 글자 폭 합 11 검사). 라이브러리로 새로 인코딩한 것이 아니라 배포 이미지의 막대를 그대로 옮긴 것
실행: python3 tools/make_tag_barcode_images.py
"""
from pathlib import Path

import cv2
import numpy as np

OUT = Path(__file__).resolve().parents[1] / 'usd' / 'props' / 'tex'
OUT.mkdir(parents=True, exist_ok=True)
PX = 24   # 칸당 픽셀

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
gen = cv2.aruco.generateImageMarker if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker
for tid in range(15):
    m = gen(d, tid, 8 * PX, borderBits=1) if gen is cv2.aruco.drawMarker else gen(d, tid, 8 * PX, borderBits=1)
    lab = np.full((10 * PX, 10 * PX), 255, np.uint8)
    lab[PX:9 * PX, PX:9 * PX] = m
    cv2.imwrite(str(OUT / f'tag36h11_{tid:02d}.png'), lab)
# 검출 확인
ok = []
for tid in range(15):
    img = cv2.imread(str(OUT / f'tag36h11_{tid:02d}.png'))
    img = cv2.copyMakeBorder(img, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=(255, 255, 255))
    det = cv2.aruco.ArucoDetector(d) if hasattr(cv2.aruco, 'ArucoDetector') else None
    ids = (det.detectMarkers(img)[1] if det else cv2.aruco.detectMarkers(img, d)[1])
    ok.append(ids is not None and int(ids.flatten()[0]) == tid)
print('AprilTag 15개 생성, 검출 일치', sum(ok), '/ 15')

# Code128 막대 폭 (검정부터 시작, 검정·흰색 번갈아). 팀 이미지 2.png 의 막대 가운데 줄에서 읽음
BARS = {
    'P2-01': '2112143131212232111221321231221232211411222331112',
    'P2-02': '2112143131212232111221321231222232111224112331112',
    'P2-03': '2112143131212232111221321231222211324131112331112',
    'P2-04': '2112143131212232111221321231222212311212412331112',
}
QUIET, MOD_PX, H_PX = 10, 8, 360
for k, (code, w) in enumerate(BARS.items()):
    widths = list(map(int, w))
    assert sum(widths) == 90 and w.startswith('211214') and w.endswith('2331112'), code
    body = widths[6:-7]
    assert all(sum(body[i:i + 6]) == 11 for i in range(0, len(body), 6)), code     # 글자마다 11 모듈
    n = sum(widths) + 2 * QUIET
    img = np.full((H_PX, n * MOD_PX), 255, np.uint8)
    x, black = QUIET * MOD_PX, True
    for wv in widths:
        if black:
            img[int(H_PX * 0.08):int(H_PX * 0.92), x:x + wv * MOD_PX] = 0
        x += wv * MOD_PX
        black = not black
    cv2.imwrite(str(OUT / f'barcode_{k}.png'), img)
print('바코드 4개 (Code128, 모듈 90 + 앞뒤 여백 10):', list(BARS))

# ---------- 오탐(헷갈림) 대비용 가짜 종이 (라벨 안 붙음, 데이터셋에서 벽에 무작위로 붙임) ----------
# 바코드와 비슷하지만 바코드가 아닌 것: 무작위 줄무늬, QR 코드, 인쇄 글자 같은 블록, 체크무늬
rng = np.random.default_rng(7)
W_PX, H_PX2 = 880, 600
fakes = {}
x, img = 40, np.full((H_PX2, W_PX), 255, np.uint8)            # 줄무늬: 폭·간격 무작위 (Code128 규칙 아님)
while x < W_PX - 40:
    w = int(rng.integers(4, 30)); img[60:H_PX2 - 60, x:x + w] = 0; x += w + int(rng.integers(4, 30))
fakes['stripes'] = img
qr = None
if hasattr(cv2, 'QRCodeEncoder'):
    try:
        qr = cv2.QRCodeEncoder.create().encode('TURTLE NINJA DISTRACTOR')
    except cv2.error:
        qr = None
if qr is None:                                                  # QR 인코더가 없으면 무작위 격자
    qr = (rng.random((25, 25)) > 0.5).astype(np.uint8) * 255
img = np.full((H_PX2, W_PX), 255, np.uint8)
q = cv2.resize(qr, (440, 440), interpolation=cv2.INTER_NEAREST); img[80:520, 220:660] = q
fakes['qr'] = img
img = np.full((H_PX2, W_PX), 255, np.uint8)                     # 글자 같은 블록 줄
for r in range(70, H_PX2 - 70, 55):
    x = 50
    while x < W_PX - 80:
        w = int(rng.integers(10, 60)); img[r:r + 26, x:x + w] = int(rng.integers(0, 90)); x += w + int(rng.integers(8, 20))
fakes['text'] = img
yy, xx = np.mgrid[0:H_PX2, 0:W_PX]
fakes['checker'] = (((xx // 60) + (yy // 60)) % 2 * 255).astype(np.uint8)
for k, (name, im) in enumerate(fakes.items()):
    cv2.imwrite(str(OUT / f'distract_{k}.png'), im)
print('가짜 종이 4장 (오탐 대비):', list(fakes))
