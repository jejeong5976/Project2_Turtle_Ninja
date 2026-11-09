#!/usr/bin/env python3
"""손목 D435 경첩 각도별 기준 그림 · 실물 화면 겹쳐 보기 (계산만).

  $ARM_PY camera_angle_ref.py                         # 기준 그림 + 표 → camera_ref/
  $ARM_PY camera_angle_ref.py --overlay 화면.png --hinge -30   # 실물 화면 위에 예상 윤곽

경첩 각도 = 팀원 표기 (배포 URDF = 0°, 팀원 PR URDF = −12°, 음수 = 손 쪽으로 숙임).
경첩 회전축은 배포 URDF(0°) 와 PR URDF(−12°) 의 카메라 자세 차이에서 역산 (축 방향 · 축이 지나는 점) → 다른 각도는 그 축으로 돌림.
카메라와 손가락은 같은 손목(link5)에 붙어 있어 화면 속 손가락 · 쥔 퍽 위치는 팔 자세와 무관 = 경첩 각도로만 정해짐.
그림: 손가락(열림 49.8 mm / 쥠 28.1 mm), 쥔 퍽(윗부분 3 cm 쥠: 손끝 = 퍽 바닥 위 30 mm) 윤곽 · 윗면 태그 16 mm. 640 × 480, 실측 K, 컬러 렌즈 위치 (케이스 가운데에서 32.5 mm 옆).
"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.environ.get('REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
sys.path.insert(0, os.environ.get('PLACE_SPOTS', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'place_spots')))
import place_ik as pk  # noqa: E402
from reach_lib import gap_to_q  # noqa: E402
from reach_model import HOLD_GAP_MM, OPEN_GAP_MM, Model  # noqa: E402

REL_URDF = os.path.join(os.environ.get('URDF_RELEASE', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nt_urdf_release_20261027_layout')), 'urdf', 'robot.urdf')
PR_URDF = os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'), 'sim', 'robot_sim', 'robot_description', 'robot.urdf')
PR_HINGE = -12.0
K = np.array([[606.43, 0, 325.19], [0, 606.16, 256.77], [0, 0, 1.0]])   # 실측 (S/N …635, 640 × 480)
COLOR_DX = -0.0325   # 컬러 렌즈 = 케이스 가운데(proxy)에서 광학 −x 32.5 mm (PR URDF d435_color_optical_frame, 실물 손가락 위치로 확인)
W, H = 640, 480
OUT = Path(__file__).with_name('camera_ref')
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def cam_in_link5(urdf):
    c5, cc = pk.Chain(urdf, 'arm_link5_wrist_roll'), pk.Chain(urdf, 'd435_cad_optical_proxy_frame')
    return np.linalg.inv(c5.T({})) @ cc.T({})


def rodrigues(axis, ang):
    a = axis / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k


class Hinge:
    """배포(0°) · PR(−12°) 카메라 자세로 경첩 축을 역산"""

    def __init__(self):
        T0, T1 = cam_in_link5(REL_URDF), cam_in_link5(PR_URDF)
        self.T0 = T0
        Rr = T1[:3, :3] @ T0[:3, :3].T
        ang = math.acos(max(-1.0, min(1.0, (np.trace(Rr) - 1) / 2)))
        axis = np.array([Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0], Rr[1, 0] - Rr[0, 1]]) / (2 * math.sin(ang))
        self.axis, self.per_deg = axis, ang / abs(PR_HINGE) * (1 if PR_HINGE < 0 else -1)   # 음수 경첩 = 이 방향
        # 축이 지나는 점 p: R(c0 − p) + p = c1 → (I − R) p = c1 − R c0 (축 방향은 정해지지 않음 → 최소 노름)
        c0, c1 = T0[:3, 3], T1[:3, 3]
        self.pivot = np.linalg.lstsq(np.eye(3) - Rr, c1 - Rr @ c0, rcond=None)[0]
        self.check = float(np.linalg.norm(Rr @ (c0 - self.pivot) + self.pivot - c1))

    def T(self, hinge_deg):
        R = rodrigues(self.axis, -hinge_deg * self.per_deg)
        T = np.eye(4)
        T[:3, :3] = R @ self.T0[:3, :3]
        T[:3, 3] = R @ (self.T0[:3, 3] - self.pivot) + self.pivot
        return T


def project(Tc_world, P):
    Pc = (P - Tc_world[:3, 3]) @ Tc_world[:3, :3]
    z = Pc[:, 2]
    ok = z > 0.01
    u = K[0, 0] * Pc[:, 0] / np.where(ok, z, 1) + K[0, 2]
    v = K[1, 1] * Pc[:, 1] / np.where(ok, z, 1) + K[1, 2]
    return u, v, z, ok


def scene(M, hinge, hinge_deg, gap_mm, held):
    """그릴 점들: {'손가락': (u, v), '퍽': …, '태그': 모서리 4 개} — link5 기준이라 팔 자세 아무거나 (템플릿)"""
    q = M.template['q']
    Ts = M.fk(q, gap_to_q(gap_mm))
    T5 = Ts['arm_link5_wrist_roll']
    Tc = T5 @ hinge.T(hinge_deg)
    Tc[:3, 3] += Tc[:3, 0] * COLOR_DX
    out = {}
    pts = []
    for L, b in (('gripper_finger_a', 'finger_a'), ('gripper_finger_b', 'finger_b')):
        pts.append(M.bodies[b].pts @ Ts[L][:3, :3].T + Ts[L][:3, 3])
    u, v, z, ok = project(Tc, np.vstack(pts))
    out['손가락'] = (u[ok], v[ok], z[ok])
    app = T5[:3, :3] @ M.app5
    cl = T5[:3, :3] @ M.close5
    w = np.cross(app, cl)
    tip = T5[:3, :3] @ M.tip5 + T5[:3, 3]
    a5 = tip - 0.030 * app                              # 퍽 윗면 가운데 (손끝 = 바닥 위 30 mm → 윗면은 손목 쪽 30 mm)
    if held:
        th = np.linspace(0, 2 * math.pi, 48)
        ring = lambda c: np.array([c + 0.015 * (math.cos(t) * cl + math.sin(t) * w) for t in th])
        side = np.vstack([ring(a5 + s * app) for s in np.linspace(0, 0.06, 13)])
        u, v, z, ok = project(Tc, side)
        out['퍽'] = (u[ok], v[ok], z[ok])
        tag = np.array([a5 + 0.008 * (a * cl + b * w) for a, b in ((1, 1), (-1, 1), (-1, -1), (1, -1))])
        u, v, z, ok = project(Tc, tag)
        out['태그'] = (u, v, z, ok)
    tips = []
    for L, b in (('gripper_finger_a', 'finger_a'), ('gripper_finger_b', 'finger_b')):
        P = M.bodies[b].pts @ Ts[L][:3, :3].T + Ts[L][:3, 3]
        d = (P - T5[:3, 3]) @ app
        u, v, z, ok = project(Tc, P[d > d.max() - 0.008])
        tips.append((float(u[ok].mean()), float(v[ok].mean())) if ok.any() else None)
    out['손가락 끝'] = tips
    return out


def draw(img, sc, label, color_scale=1.0):
    d = ImageDraw.Draw(img, 'RGBA')
    f = ImageFont.truetype(FONT, 16)
    for name, col in (('퍽', (230, 140, 40, 70)), ('손가락', (60, 160, 255, 90))):
        if name in sc:
            u, v, z = sc[name]
            for a, b in zip(u, v):
                if 0 <= a < W and 0 <= b < H:
                    d.ellipse([a - 1.5, b - 1.5, a + 1.5, b + 1.5], fill=col)
    if '태그' in sc:
        u, v, z, ok = sc['태그']
        if ok.all():
            d.polygon(list(zip(u, v)), outline=(255, 40, 40, 255), width=2)
    for t in sc['손가락 끝']:
        if t and 0 <= t[0] < W and 0 <= t[1] < H:
            d.line([t[0] - 8, t[1], t[0] + 8, t[1]], fill=(255, 255, 0, 255), width=2)
            d.line([t[0], t[1] - 8, t[0], t[1] + 8], fill=(255, 255, 0, 255), width=2)
    for y in (120, 240, 360):
        d.line([0, y, W, y], fill=(200, 200, 200, 60))
        d.text((2, y - 18), f'v {y}', fill=(200, 200, 200, 200), font=f)
    d.text((8, 6), label, fill=(255, 255, 255, 255), font=f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hinges', type=float, nargs='+', default=[-12, -20, -25, -30, -35])
    ap.add_argument('--overlay', help='실물 D435 컬러 화면 (640 × 480 권장, 다르면 맞춰 줄임)')
    ap.add_argument('--hinge', type=float, default=-30.0, help='--overlay 때 겹칠 경첩 각도')
    a = ap.parse_args()
    M = Model(spacing=0.003)
    hg = Hinge()
    print(f'경첩 축 (link5 기준) {np.round(hg.axis, 3)} · −12° 재현 오차 {hg.check * 1000:.2f} mm')
    OUT.mkdir(exist_ok=True)
    if a.overlay:
        img = Image.open(a.overlay).convert('RGB')
        if img.size != (W, H):
            print(f'화면 크기 {img.size} → 640 × 480 로 맞춤 (비율이 다르면 위치가 어긋날 수 있음)')
            img = img.resize((W, H))
        for gap, held, tag in ((OPEN_GAP_MM, False, 'open'), (HOLD_GAP_MM, True, 'hold')):
            im = img.copy()
            draw(im, scene(M, hg, a.hinge, gap, held), f'겹쳐 보기 · 경첩 {a.hinge:+.0f}° · ' + ('열림' if not held else '퍽 쥠'))
            p = OUT / f'overlay_{Path(a.overlay).stem}_{a.hinge:+.0f}_{tag}.png'
            im.save(p)
            print('저장', p)
        return
    table = {}
    for h in a.hinges:
        row = {}
        for gap, held, tag in ((OPEN_GAP_MM, False, '열림'), (HOLD_GAP_MM, True, '쥠')):
            sc = scene(M, hg, h, gap, held)
            img = Image.new('RGB', (W, H), (30, 30, 34))
            draw(img, sc, f'경첩 {h:+.0f}° · 그리퍼 {tag} ({gap:.1f} mm)' + (' · 퍽 윗부분 3 cm 쥠' if held else ''))
            img.save(OUT / f'hinge_{h:+.0f}_{"open" if not held else "hold"}.png')
            u, v, z = sc['손가락']
            inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
            row[tag] = dict(fingertips_px=[None if t is None else [round(t[0]), round(t[1])] for t in sc['손가락 끝']],
                            finger_visible_pct=round(float(inside.mean() * 100), 1))
            if held:
                tu, tv, tz, tok = sc['태그']
                vis = bool(tok.all() and ((tu >= 0) & (tu < W) & (tv >= 0) & (tv < H)).all())
                row[tag]['top_tag_px'] = [round(float(tu.mean())), round(float(tv.mean()))] if vis else None
        table[f'{h:+.0f}'] = row
        o, hh = row['열림'], row['쥠']
        print(f"경첩 {h:+4.0f}° · 열림 손가락 끝 {o['fingertips_px']} · 쥠 손가락 끝 {hh['fingertips_px']} · 퍽 윗면 태그 {hh.get('top_tag_px') or '화면 밖'}")
    (OUT / 'table.json').write_text(json.dumps(table, ensure_ascii=False, indent=1))
    print('저장', OUT)


if __name__ == '__main__':
    main()
