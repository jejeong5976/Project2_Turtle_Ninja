#!/usr/bin/env python3
"""손목 D435 퍽 중심 정답 측정용 종이 (A4 가로) 만들기 -> gt_sheet.pdf + gt_sheet.json.

종이: 테두리 = ArUco 마커 30개 (DICT_4X4_100, 20 mm, 퍽 AprilTag 36h11 과 다른 계열이라 안 헷갈림)
      안쪽 = 퍽 발자국 윤곽 5칸 (선 사각 · 선 원기둥 · 누운 퍽 가로 / 세로 / 45°) + 칸 중심 십자
      아래 여백 = 100 mm 자 눈금 (인쇄 축척 확인용)
쓰는 법: 실제 크기(100 %)로 인쇄 -> 자로 100 mm 눈금 · 마커 한 변 20 mm 확인 -> 평평한 곳에 붙임
      -> 퍽을 윤곽에 맞춰 놓고 d435_puck_viewer 로 저장 -> 마커로 종이 자세(PnP) -> 칸 정답 중심과 추정 중심 비교
종이 좌표 (gt_sheet.json 의 단위 m): 원점 = 테두리 바깥 왼쪽 위 모서리, x 오른쪽, y 아래(종이 위에서 봤을 때),
      z = x × y = 종이 속으로 (책상 쪽). 종이 위쪽 = -z
퍽 정답 중심 = 칸 중심 (x, y) + z = -높이/2 (선 퍽 -0.030, 누운 퍽 -0.015)

예) cd tools/wrist_calib && python3 make_gt_sheet.py
"""
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
PPM = 24                                   # 화소 / mm (609.6 dpi). mm 가 정수 화소라 마커 위치 · 크기에 반올림 오차 없음
PAGE_MM = (297, 210)                       # A4 가로
DICT = cv2.aruco.DICT_4X4_100
MARKER_MM, PITCH_MM = 20, 26               # 마커 한 변, 마커 간격 (사이 6 mm)
N_X, N_Y = 10, 7                           # 테두리 가로 · 세로 마커 수 -> 바깥 254 × 176 mm
BOARD_MM = (N_X * PITCH_MM - (PITCH_MM - MARKER_MM), N_Y * PITCH_MM - (PITCH_MM - MARKER_MM))
ORIGIN_MM = ((PAGE_MM[0] - BOARD_MM[0]) / 2, (PAGE_MM[1] - BOARD_MM[1]) / 2)   # 종이 좌표 원점의 쪽 위치
FONT = '/usr/share/fonts/truetype/nanum/NanumGothic.ttf'
FONT_B = '/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf'
PUCK_D, PUCK_H = 30, 60                    # mm

# 발자국 칸: 이름, 모양, 중심 (종이 좌표 mm), 긴축 수평 각 (도, x 축에서 y 쪽으로), 선/누움
SLOTS = [
    dict(name='S', desc='선 사각기둥', shape='box', upright=True, c=(58, 58), yaw=0),
    dict(name='C', desc='선 원기둥', shape='cyl', upright=True, c=(58, 118), yaw=0),
    dict(name='H', desc='누운 퍽 (가로)', shape='any', upright=False, c=(132, 56), yaw=0, label='above'),
    dict(name='D', desc='누운 퍽 (45°)', shape='any', upright=False, c=(130, 112), yaw=45),
    dict(name='V', desc='누운 퍽 (세로)', shape='any', upright=False, c=(203, 88), yaw=90, label='above'),
]


def ring_cells():
    """테두리 마커 칸 (i, j) 순서 = 마커 ID 0, 1, ... (위 왼→오, 오른쪽 위→아래, 아래 오→왼, 왼쪽 아래→위)"""
    cells = [(i, 0) for i in range(N_X)]
    cells += [(N_X - 1, j) for j in range(1, N_Y)]
    cells += [(i, N_Y - 1) for i in range(N_X - 2, -1, -1)]
    cells += [(0, j) for j in range(N_Y - 2, 0, -1)]
    return cells


def marker_corners_mm(i, j):
    """마커 네 꼭짓점 (종이 좌표 mm, OpenCV 순서 = 왼위 · 오위 · 오아래 · 왼아래)"""
    x, y = i * PITCH_MM, j * PITCH_MM
    s = MARKER_MM
    return [(x, y), (x + s, y), (x + s, y + s), (x, y + s)]


def px(p_mm):
    """종이 좌표 mm -> 쪽 화소"""
    return (round((ORIGIN_MM[0] + p_mm[0]) * PPM), round((ORIGIN_MM[1] + p_mm[1]) * PPM))


def slot_outline_mm(s):
    """발자국 윤곽 꼭짓점 (원기둥이면 None)"""
    if s['shape'] == 'cyl':
        return None
    L, W = (PUCK_D, PUCK_D) if s['upright'] else (PUCK_H, PUCK_D)
    a = np.radians(s['yaw'])
    u, v = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
    c = np.array(s['c'], float)
    return [tuple(c + su * L / 2 * u + sv * W / 2 * v) for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def main():
    W, H = PAGE_MM[0] * PPM, PAGE_MM[1] * PPM
    img = Image.new('L', (W, H), 255)
    d = ImageDraw.Draw(img)
    dic = cv2.aruco.getPredefinedDictionary(DICT)
    cells = ring_cells()
    markers = []
    for mid, (i, j) in enumerate(cells):
        cs = marker_corners_mm(i, j)
        m = cv2.aruco.generateImageMarker(dic, mid, MARKER_MM * PPM, borderBits=1)
        img.paste(Image.fromarray(m), px(cs[0]))
        markers.append(dict(id=mid, corners_m=[[x / 1000, y / 1000, 0.0] for x, y in cs]))

    f_s, f_m, f_b = ImageFont.truetype(FONT, 3 * PPM), ImageFont.truetype(FONT, 4 * PPM), ImageFont.truetype(FONT_B, 5 * PPM)
    lw = max(1, round(0.4 * PPM))
    slots = []
    for s in SLOTS:
        c = s['c']
        ol = slot_outline_mm(s)
        if ol is None:
            x0, y0 = px((c[0] - PUCK_D / 2, c[1] - PUCK_D / 2))
            x1, y1 = px((c[0] + PUCK_D / 2, c[1] + PUCK_D / 2))
            d.ellipse([x0, y0, x1, y1], outline=0, width=lw)
        else:
            d.polygon([px(p) for p in ol], outline=0, width=lw)
        if not s['upright']:                               # 누운 퍽 긴축 방향 점선
            a = np.radians(s['yaw'])
            u = np.array([np.cos(a), np.sin(a)])
            for t in np.arange(-27, 27, 6):
                d.line([px(np.array(c) + t * u), px(np.array(c) + (t + 3) * u)], fill=140, width=lw)
        cx, cy = px(c)                                     # 중심 십자 (5 mm)
        r = 5 * PPM // 2
        d.line([cx - r, cy, cx + r, cy], fill=0, width=lw)
        d.line([cx, cy - r, cx, cy + r], fill=0, width=lw)
        ext_y = PUCK_D / 2 if s['upright'] else (abs(np.sin(np.radians(s['yaw']))) * PUCK_H + abs(np.cos(np.radians(s['yaw']))) * PUCK_D) / 2
        above = s.get('label') == 'above'                  # 이름 글자 = 윤곽 아래 (기본) 또는 위
        lx, ly = px((c[0] - 14, c[1] - ext_y - 5 if above else c[1] + ext_y + 1.5))
        d.text((lx, ly), f"{s['name']}  {s['desc']}", font=f_s, fill=0)
        h = (PUCK_H if s['upright'] else PUCK_D) / 2
        a = np.radians(s['yaw'])
        axis = [0.0, 0.0, -1.0] if s['upright'] else [round(float(np.cos(a)), 6), round(float(np.sin(a)), 6), 0.0]
        slots.append(dict(name=s['name'], desc=s['desc'], shape=s['shape'], upright=s['upright'],
                          center_m=[c[0] / 1000, c[1] / 1000, -h / 1000], axis_m=axis, yaw_deg=s['yaw']))

    # 위 여백: 제목 · 주의
    d.text((round(ORIGIN_MM[0] * PPM), round(2.5 * PPM)), '손목 D435 퍽 중심 정답 종이 (tools/wrist_calib/gt_sheet.json)', font=f_b, fill=0)
    d.text((round(ORIGIN_MM[0] * PPM), round(9 * PPM)),
           '실제 크기(100 %)로 인쇄 · 마커 한 변 20.0 mm, 아래 자 100 mm 확인 · 퍽을 윤곽에 맞춰 놓기 · 원점 = 왼쪽 위 마커 바깥 모서리 (x →, y ↓)',
           font=f_s, fill=0)
    # 아래 여백: 100 mm 자
    y0 = round((ORIGIN_MM[1] + BOARD_MM[1] + 4) * PPM)
    x0 = round(ORIGIN_MM[0] * PPM)
    d.line([x0, y0, x0 + 100 * PPM, y0], fill=0, width=lw)
    for k in range(0, 101):
        tl = 4 if k % 10 == 0 else (2.5 if k % 5 == 0 else 1.5)
        d.line([x0 + k * PPM, y0, x0 + k * PPM, y0 + round(tl * PPM)], fill=0, width=lw)
        if k % 10 == 0:
            d.text((x0 + k * PPM - PPM, y0 + round(4.5 * PPM)), str(k), font=f_s, fill=0)
    d.text((x0 + 104 * PPM, y0), 'mm (인쇄 축척 확인: 0–100 이 정확히 100 mm)', font=f_m, fill=0)

    out_pdf, out_png, out_json = HERE / 'gt_sheet.pdf', HERE / 'gt_sheet_preview.png', HERE / 'gt_sheet.json'
    img.convert('1').save(out_pdf, resolution=PPM * 25.4)
    img.resize((W // 8, H // 8), Image.LANCZOS).save(out_png)
    out_json.write_text(json.dumps(dict(
        note='종이 좌표: 원점 = 테두리 바깥 왼쪽 위, x 오른쪽, y 아래, z = 종이 속(책상 쪽). 퍽 정답 중심 = center_m',
        dictionary='DICT_4X4_100', marker_m=MARKER_MM / 1000, page_mm=PAGE_MM, origin_on_page_mm=ORIGIN_MM,
        puck_m=dict(d=PUCK_D / 1000, h=PUCK_H / 1000), markers=markers, slots=slots), ensure_ascii=False, indent=1))
    print(f'{out_pdf}\n{out_json}\n{out_png}')
    print(f'테두리 {BOARD_MM[0]} × {BOARD_MM[1]} mm, 마커 {len(markers)} 개, 칸 {len(slots)} 개, 쪽 여백 좌우 {ORIGIN_MM[0]} · 위아래 {ORIGIN_MM[1]} mm')


if __name__ == '__main__':
    main()
