#!/usr/bin/env python3
"""시뮬 손목 데이터로 wrist_detector 파이프라인(seg → 자세 → 거르기 → 여러 프레임 확인 → Detection) 정확도 평가 (확정률 · 오탐 · 오차).

시뮬은 정지 영상이라 같은 장을 --repeat 번 넣어 여러 프레임 확인을 흉내 냄 (깊이에 --noise_mm 잡음을 장마다 새로 더함).
정답 짝짓기: 확정 퍽 중심과 정답 퍽 중심이 3 cm 안 (가까운 것부터 일대일)
출력
  정답 퍽(≥ min_px, 잘림 포함 전체 / 잘림 없음) 중 확정 비율, 색 · 모양 · 태그 ID 일치, 중심 · 긴축(쿼터니언 z) 오차
  거른 이유별 개수 (truncated / size / points), 정답과 안 맞는 확정 퍽 수 (= planning 에 들어갈 오탐)
예) cd yolo/wrist/scripts && ~/Desktop/physical_ai/.venvs/yolo/bin/python eval_detector.py --weights ../runs/wrist/seg8_sim/weights/best.pt
"""
import argparse
import collections
import glob
import json
from pathlib import Path

import cv2
import numpy as np

import wrist_detector as wd


def ang(a, b):
    return float(np.degrees(np.arccos(np.clip(abs(a @ b) / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))))


def quat_z(q):
    x, y, z, w = q
    return np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])


def cam_R(m):
    """카메라 광학 프레임 → 월드 회전 (시뮬 정답 카메라 자세)"""
    x, y, z, w = m['cam_quat_w_ros_xyzw']
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def cam_up(m):
    return cam_R(m).T @ np.array([0, 0, 1.0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default='../datasets/sim_wrist')  # test split = 학습에 안 쓴 장면
    ap.add_argument('--split', default='test')
    ap.add_argument('--weights', required=True)
    ap.add_argument('--conf', type=float, default=wd.SEG_CONF)
    ap.add_argument('--repeat', type=int, default=3, help='같은 장을 몇 번 넣을지 (min_hits 와 같게 하면 매 장 확정 가능)')
    ap.add_argument('--min_hits', type=int, default=3)
    ap.add_argument('--noise_mm', type=float, default=2.0, help='장마다 깊이에 더할 가우시안 잡음 (실물 D435 근거리 약 1–3 mm)')
    ap.add_argument('--min_px', type=int, default=300)
    ap.add_argument('--no_gravity', action='store_true')
    a = ap.parse_args()

    det = wd.WristPuckDetector(a.weights, conf=a.conf, window=max(a.repeat, a.min_hits), min_hits=a.min_hits)
    rng = np.random.default_rng(0)
    metas = sorted(glob.glob(str(Path(a.data) / a.split / '*_meta.json')))
    if not metas:
        raise SystemExit(f'데이터 없음: {Path(a.data) / a.split}')
    rows, rejects, n_false, n_det = [], collections.Counter(), 0, 0
    for f in metas:
        m = json.load(open(f))
        base = f[:-10]
        K = np.array(m['K'])
        bgr = cv2.imread(base + '_rgb.png')
        depth = np.load(base + '_depth.npy').astype(np.float32)
        up = None if a.no_gravity else cam_up(m)
        det.reset()
        dets, raws = [], []
        for _ in range(a.repeat):
            d = depth + (rng.normal(0, a.noise_mm / 1000, depth.shape).astype(np.float32) * (depth > 0) if a.noise_mm else 0)
            dets, raws, _ = det.process(bgr, d, K, up)
        rejects.update(r.reject for r in raws if r.reject)
        n_det += len(dets)
        gts = [p for p in m['pucks'] if p['px'] >= a.min_px]
        pairs = sorted(((np.linalg.norm(dd.position - np.array(g['center_cam'])), i, j)
                        for i, dd in enumerate(dets) for j, g in enumerate(gts)), key=lambda x: x[0])
        used_d, used_g = set(), set()
        for dist_m, i, j in pairs:
            if dist_m > 0.03 or i in used_d or j in used_g:
                continue
            used_d.add(i)
            used_g.add(j)
            dd, g = dets[i], gts[j]
            cut = any(r.reject == 'truncated' and np.linalg.norm(r.pose.center - np.array(g['center_cam'])) < 0.03 for r in raws)
            # 로봇 프레임 대신 시뮬 월드 프레임(z 위쪽)으로 planar_pose 검증
            R_wc, t_wc = cam_R(m), np.array(m['cam_pos_w'])
            pp = wd.planar_pose(dd, R_wc, t_wc)
            c_w = R_wc @ np.array(g['center_cam']) + t_wc
            a_w = R_wc @ np.array(g['axis_cam'])
            gt_up = abs(a_w[2]) > wd.UPRIGHT_COS
            th_err = face_err = None
            if not gt_up and not pp.upright:          # 누운 퍽 θ (180° 주기)
                gt_th = np.arctan2(a_w[1], a_w[0])
                th_err = abs(np.degrees(wd._wrap(pp.theta - gt_th, np.pi)))
            box_up = gt_up and g['shape'] == 'box' and 'xaxis_cam' in g   # 선 사각기둥 면 방향 θ (90° 주기, 정답 있는 데이터만)
            if box_up and pp.upright and pp.theta_known:
                x_w = R_wc @ np.array(g['xaxis_cam'])
                face_err = abs(np.degrees(wd._wrap(pp.theta - np.arctan2(x_w[1], x_w[0]), np.pi / 2)))
            rows.append(dict(found=True, cut=cut, label_ok=dd.label == g['color'], shape_ok=dd.shape == wd.SHAPE_NAME[g['shape']],
                             id_ok=dd.id == ('' if g['tag_id'] is None or g.get('tag_faces') == [] else str(g['tag_id'])), id_read=dd.id != '',   # tag_faces [] = 태그 숨긴 퍽
                             pos_mm=dist_m * 1000, axis_deg=ang(quat_z(dd.orientation), np.array(g['axis_cam'])), yaw_known=dd.yaw_known,
                             up_ok=pp.upright == gt_up, gt_up=gt_up, xy_mm=float(np.hypot(*(np.array([pp.x, pp.y]) - c_w[:2])) * 1000),
                             z_mm=float(abs(pp.z - c_w[2]) * 1000), th_err=th_err, box_up=box_up, face_err=face_err))
        for j, g in enumerate(gts):
            if j not in used_g:
                x0, y0, x1, y1 = g['bbox']
                H, W = bgr.shape[:2]
                rows.append(dict(found=False, cut=x0 <= 2 or y0 <= 2 or x1 >= W - 3 or y1 >= H - 3))
        n_false += len(dets) - len(used_d)

    def report(name, rs):
        f = [r for r in rs if r['found']]
        print(f'\n[{name}] 정답 퍽 {len(rs)}, 확정 {len(f)} ({len(f) / max(1, len(rs)):.0%})')
        if not f:
            return
        p, q = np.array([r['pos_mm'] for r in f]), np.array([r['axis_deg'] for r in f])
        print(f'  색 {sum(r["label_ok"] for r in f)}/{len(f)}  모양 {sum(r["shape_ok"] for r in f)}/{len(f)}  '
              f'태그 ID 읽음 {sum(r["id_read"] for r in f)}/{len(f)} · 일치 {sum(r["id_ok"] for r in f)}/{len(f)}  '
              f'면 방향 앎(yaw_known) {sum(r["yaw_known"] for r in f)}/{len(f)}')
        print(f'  중심 오차 mm 중앙 {np.median(p):.1f} · 90% {np.percentile(p, 90):.1f}   '
              f'긴축(쿼터니언 z) 오차 ° 중앙 {np.median(q):.1f} · 90% {np.percentile(q, 90):.1f}')
        xy, zz = np.array([r['xy_mm'] for r in f]), np.array([r['z_mm'] for r in f])
        th = np.array([r['th_err'] for r in f if r['th_err'] is not None])
        n_up = sum(r['gt_up'] for r in f)
        print(f'  [월드 x, y, z, θ] 선/누움 판정 {sum(r["up_ok"] for r in f)}/{len(f)} (정답 선 {n_up} · 누움 {len(f) - n_up})  '
              f'xy 오차 mm 중앙 {np.median(xy):.1f} · 90% {np.percentile(xy, 90):.1f}  z 오차 mm 중앙 {np.median(zz):.1f} · 90% {np.percentile(zz, 90):.1f}')
        if len(th):
            print(f'  누운 퍽 θ 오차 ° ({len(th)}개) 중앙 {np.median(th):.1f} · 90% {np.percentile(th, 90):.1f} · 최대 {th.max():.1f}')
        bu = [r for r in f if r.get('box_up')]
        fe = np.array([r['face_err'] for r in bu if r['face_err'] is not None])
        if bu:
            msg = f'  선 사각기둥 면 θ: {len(bu)}개 중 구함 {len(fe)}'
            if len(fe):
                msg += f' · 오차 ° 중앙 {np.median(fe):.1f} · 90% {np.percentile(fe, 90):.1f} · 최대 {fe.max():.1f}'
            print(msg)

    print(f'{a.split}: {len(metas)}장 × {a.repeat}회, conf {a.conf}, min_hits {a.min_hits}, 깊이 잡음 {a.noise_mm} mm, '
          f'중력 {"없음" if a.no_gravity else "카메라 자세"}')
    report('전체 (잘림 포함)', rows)
    report('잘림 없음', [r for r in rows if not r['cut']])
    print(f'\n거른 원시 퍽 (마지막 회차): {dict(rejects)}')
    print(f'확정 {n_det}개 중 정답과 안 맞음(= planning 에 갈 오탐) {n_false}')


if __name__ == '__main__':
    main()
