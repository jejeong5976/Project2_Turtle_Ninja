#!/usr/bin/env python3
"""시뮬 손목 D435 데이터(sim/robot_sim/scripts/make_wrist_dataset.py)로 wrist_pose 자세 추정 오차 측정.

분할 세 가지:
  gt  = 정답 마스크 (분할이 완벽할 때의 기하 계산 오차: 점군 -> 중심 · 긴축, 태그 보정)
  hsv = wrist_pose.color_masks (색 분할) -> 정답 퍽과 마스크 겹침(IoU)으로 짝지음
  seg = 손목 YOLO-seg (--weights) -> hsv 와 같이 IoU 로 짝지음. 클래스(색 · 모양) 일치도 셈
오차: 중심 거리 (mm), 긴축 각도 (도, 부호 무시), 태그 보정 여부별. 퍽 자세 '누움'(긴축이 카메라 기준이 아니라 월드 수평)은 meta 에 없어서
      긴축과 카메라 광축 사이 각으로 대신 나눔
출력: 표 + <out>/eval_<mode>.jpg (정답 초록 · 추정 노랑)
예) cd yolo/wrist/scripts && ~/Desktop/physical_ai/.venvs/yolo/bin/python eval_sim.py --data ../datasets/sim_wrist --mode gt
    ... eval_sim.py --data ../datasets/sim_wrist --mode seg --weights ../runs/wrist/seg8_sim/weights/best.pt --split test --gravity
"""
import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np

import wrist_pose as wp


def ang(a, b):
    return float(np.degrees(np.arccos(np.clip(abs(a @ b) / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--mode', choices=['gt', 'hsv', 'seg'], default='gt')
    ap.add_argument('--weights', default=None, help='--mode seg 가중치')
    ap.add_argument('--split', default='*', help="평가할 split 폴더 (seg 는 학습에 안 쓴 'test' 권장)")
    ap.add_argument('--min_px', type=int, default=300, help='이보다 적게 보이는 퍽은 평가에서 뺌 (화면 끝에 잘린 퍽)')
    ap.add_argument('--show', type=int, default=6, help='그림으로 저장할 장수')
    ap.add_argument('--gravity', action='store_true', help='카메라 자세로 월드 위쪽을 줘서 중력 보정 (로봇에선 TF)')
    a = ap.parse_args()
    rows, tiles = [], []
    metas = sorted(glob.glob(str(Path(a.data) / a.split / '*_meta.json')))
    seg = wp.Segmenter(a.weights, conf=0.5) if a.mode == 'seg' else None
    n_gt = n_found = 0
    for f in metas:
        m = json.load(open(f)); base = f[:-10]
        K = np.array(m['K']); bgr = cv2.imread(base + '_rgb.png'); depth = np.load(base + '_depth.npy'); mk = cv2.imread(base + '_mask.png', -1)
        tags = wp.detect_tags(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), K)
        up_cam = None
        if a.gravity:                                          # 카메라 자세(월드, ROS 광학) -> 카메라 좌표의 월드 위쪽
            x, y, z, w = m['cam_quat_w_ros_xyzw']
            Rc = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                           [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                           [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            up_cam = Rc.T @ np.array([0, 0, 1.0])
        if a.mode == 'gt':
            cand = [(p['color'], mk == p['index'] + 1, p) for p in m['pucks']]
        else:
            cand = []
            for lab, mask, kind, _ in wp.find_masks(bgr, seg):
                best = max(m['pucks'], key=lambda p: ((mask & (mk == p['index'] + 1)).sum() / max(1, (mask | (mk == p['index'] + 1)).sum())), default=None)
                if best is not None:
                    iou = (mask & (mk == best['index'] + 1)).sum() / max(1, (mask | (mk == best['index'] + 1)).sum())
                    cand.append((kind or lab, mask, best if iou > 0.3 else None))
        gts = [p for p in m['pucks'] if p['px'] >= a.min_px]
        n_gt += len(gts)
        n_found += len({id(g) for _, _, g in cand if g is not None and g['px'] >= a.min_px})
        vis = bgr.copy()
        for lab, mask, gt in cand:
            est = wp.puck_pose(lab.split('_')[0], mask, depth, K, tags, up_cam)
            if gt is None or est is None or gt['px'] < a.min_px:
                continue
            c, ax = np.array(gt['center_cam']), np.array(gt['axis_cam'])
            view = ang(ax, c)                    # 긴축 ↔ 시선 (0 = 카메라를 정면으로 향함)
            rows.append({'shape': gt['shape'], 'color_ok': lab.split('_')[0] == gt['color'],
                         'cls_ok': '_'.join(lab.split('_')[:2]) == f"{gt['color']}_{gt['shape']}",   # 8클래스 이름(_up · _lying)도 색 · 모양만 비교
                         'pos_mm': float(np.linalg.norm(est.center - c) * 1000),
                         'axis_deg': ang(est.axis, ax), 'src': est.axis_source, 'view_deg': view, 'px': gt['px'],
                         'cut': est.truncated})
            P = lambda X: (int(K[0, 0] * X[0] / X[2] + K[0, 2]), int(K[1, 1] * X[1] / X[2] + K[1, 2]))
            cv2.arrowedLine(vis, P(c - ax * 0.03), P(c + ax * 0.03), (0, 255, 0), 2, tipLength=0.3)
            cv2.arrowedLine(vis, P(est.center - est.axis * 0.03), P(est.center + est.axis * 0.03), (0, 255, 255), 1, tipLength=0.3)
            cv2.circle(vis, P(c), 3, (0, 255, 0), -1); cv2.circle(vis, P(est.center), 3, (0, 255, 255), -1)
            cv2.putText(vis, f"{rows[-1]['pos_mm']:.0f}mm {rows[-1]['axis_deg']:.0f}deg {est.axis_source}", (gt['bbox'][0], max(12, gt['bbox'][1] - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
        if len(tiles) < a.show:
            tiles.append(vis)
    if not rows:
        raise SystemExit('평가할 퍽 없음')
    import collections
    print(f'[{a.mode}{" +중력" if a.gravity else ""}] 장 {len(metas)}, 평가 퍽 {len(rows)}')
    print(f'{"묶음":<22}{"개수":>5}{"중심 mm 중앙":>12}{"90%":>7}{"긴축 ° 중앙":>12}{"90%":>7}')
    groups = collections.OrderedDict([('전체', rows)])
    for key in ('shape', 'src'):
        for v in sorted({r[key] for r in rows}):
            groups[f'{key}={v}'] = [r for r in rows if r[key] == v]
    groups['잘림 없음'] = [r for r in rows if not r['cut']]
    groups['잘림 (화면 끝)'] = [r for r in rows if r['cut']]
    for v in sorted({r['src'] for r in rows}):
        groups[f'잘림 없음 · {v}'] = [r for r in rows if not r['cut'] and r['src'] == v]
    groups['긴축-시선 < 45° (세운 퍽 쪽)'] = [r for r in rows if r['view_deg'] < 45]
    groups['긴축-시선 ≥ 45° (누운 퍽 쪽)'] = [r for r in rows if r['view_deg'] >= 45]
    for name, rs in groups.items():
        if not rs:
            continue
        p = np.array([r['pos_mm'] for r in rs]); q = np.array([r['axis_deg'] for r in rs])
        print(f'{name:<22}{len(rs):>5}{np.median(p):>12.1f}{np.percentile(p, 90):>7.1f}{np.median(q):>12.1f}{np.percentile(q, 90):>7.1f}')
    if a.mode != 'gt':
        print('색 일치', sum(r['color_ok'] for r in rows), '/', len(rows))
    if a.mode == 'seg':
        print('클래스(색 · 모양) 일치', sum(r['cls_ok'] for r in rows), '/', len(rows))
    if a.mode != 'gt':
        print(f'정답 퍽(≥ min_px) {n_gt} 중 짝지은 것 {n_found} (못 찾음 = 분할 놓침)')
    while len(tiles) % 2:
        tiles.append(np.zeros_like(tiles[0]))
    out = Path(a.data) / f'eval_{a.mode}{"_g" if a.gravity else ""}.jpg'
    cv2.imwrite(str(out), np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)]))
    print('그림:', out)


if __name__ == '__main__':
    main()
