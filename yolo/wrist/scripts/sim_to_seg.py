#!/usr/bin/env python3
"""시뮬 손목 데이터(make_wrist_dataset.py 출력) -> YOLO-seg 데이터셋 (방식 ①: seg + 기하 계산).

입력 <sim>/<split>/<이름>_rgb.png · _mask.png (0 = 배경, k+1 = 퍽 k) · _meta.json (퍽 k 의 color · shape)
출력 <out>/images/<split>/<이름>.png, <out>/labels/<split>/<이름>.txt, <out>/data.yaml
  라벨 한 줄 = 퍽 하나: <클래스> x1 y1 x2 y2 ... (0–1 정규화 다각형)
  가려져서 여러 조각으로 나뉜 퍽은 조각을 가는 선으로 이어 다각형 하나로 만듦 (ultralytics JSON 변환과 같은 방식)
  --min_px 보다 작게 보이는 퍽은 라벨에서 뺌 (화면 끝 · 선반 뒤로 거의 가려진 퍽)
클래스 (--classes)
  8 (기본): 색 × 모양 × 선/누움. 선/누움 = meta 'lying' (없으면 긴축 · 카메라 자세로 계산, 수직에서 45° 기준)
     0 red_cyl_up 1 green_cyl_up 2 red_box_up 3 green_box_up 4 red_cyl_lying 5 green_cyl_lying 6 red_box_lying 7 green_box_lying
  4 : 색 × 모양 (광각 yolo/wide/ 와 같은 순서)
  출력 <out>/data.yaml 의 names 가 기준 → train.py · val.py 가 이 data.yaml 을 그대로 씀
파란 퍽은 생성기에서 마스크 · meta 에 안 들어감 (배경 = 오탐 방지)
예) cd yolo/wrist/scripts && ~/Desktop/physical_ai/.venvs/yolo/bin/python sim_to_seg.py
    --check 3 : 라벨 다각형을 그려 <out>/check.jpg 로 저장 (3장)
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

CLASSES4 = ['red_cyl', 'green_cyl', 'red_box', 'green_box']
CLASSES8 = [f'{c}_up' for c in CLASSES4] + [f'{c}_lying' for c in CLASSES4]
CLASSES = CLASSES8                     # 기본 (다른 스크립트가 data.yaml 을 못 찾을 때만 씀)
WRIST_DIR = Path(__file__).resolve().parent.parent


def cam_up(meta: dict) -> np.ndarray:
    """카메라 광학 좌표의 월드 위쪽 (meta 카메라 자세 cam_quat_w_ros_xyzw)"""
    x, y, z, w = meta['cam_quat_w_ros_xyzw']
    Rwc = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return Rwc.T @ np.array([0.0, 0.0, 1.0])


def is_lying(p: dict, meta: dict) -> bool:
    """퍽이 누웠는지: meta 'lying' (생성기) 또는 긴축과 위쪽 사이 45° 기준"""
    if 'lying' in p:
        return bool(p['lying'])
    return abs(float(np.dot(p['axis_cam'], cam_up(meta)))) < np.sqrt(0.5)


def class_name(p: dict, meta: dict, n_classes: int) -> str:
    base = f"{p['color']}_{p['shape']}"
    return base if n_classes == 4 else f"{base}_{'lying' if is_lying(p, meta) else 'up'}"


def mask_to_polygon(m: np.ndarray, min_piece: int = 20, eps_px: float = 1.0) -> np.ndarray | None:
    """bool 마스크 -> 다각형 (N, 2) 픽셀. 조각이 여럿이면 가장 가까운 점끼리 이어 붙임."""
    cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cs = [cv2.approxPolyDP(c, eps_px, True)[:, 0, :] for c in cs if cv2.contourArea(c) >= min_piece]
    cs = [c for c in cs if len(c) >= 3]
    if not cs:
        return None
    cs.sort(key=lambda c: -cv2.contourArea(c))
    poly = cs[0]
    for c in cs[1:]:                      # poly 의 점 i -> c 의 점 j 로 건너가 c 를 한 바퀴 돌고 다시 i 로
        d = np.linalg.norm(poly[:, None, :].astype(float) - c[None, :, :], axis=-1)
        i, j = np.unravel_index(np.argmin(d), d.shape)
        c = np.roll(c, -j, axis=0)
        poly = np.concatenate([poly[:i + 1], c, c[:1], poly[i:]])
    return poly


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sim', default=str(WRIST_DIR / 'datasets' / 'sim_wrist'), help='make_wrist_dataset.py 출력 (여러 폴더는 같은 --out 으로 한 번씩)')
    ap.add_argument('--out', default=str(WRIST_DIR / 'datasets' / 'sim_seg'))
    ap.add_argument('--classes', type=int, choices=[4, 8], default=8, help='8 = 색 × 모양 × 선/누움 (기본), 4 = 색 × 모양')
    ap.add_argument('--min_px', type=int, default=150, help='이보다 적게 보이는 퍽은 라벨 안 함')
    ap.add_argument('--check', type=int, default=0, help='라벨 그림 장수 (0 = 안 그림)')
    ap.add_argument('--prefix', default='', help="출력 파일 이름 앞에 붙임. 여러 폴더를 같은 --out 에 합칠 때 필수 (seed 가 같으면 이름이 겹쳐 덮어씀, 예: 'look1_')")
    ap.add_argument('--max_mean', type=float, default=0, help='평균 밝기(회색 0–255)가 이보다 큰 장은 뺌 = 과노출(빨강이 분홍으로 뜸). 0 = 안 뺌. 어두운 쪽은 학습 증강으로 만듦')
    a = ap.parse_args()
    sim, out = Path(a.sim).expanduser().resolve(), Path(a.out).expanduser().resolve()
    names = CLASSES8 if a.classes == 8 else CLASSES4
    old = out / 'data.yaml'
    if old.exists() and f'names: {names}' not in old.read_text():
        raise SystemExit(f'{out} 에 다른 클래스 구성의 라벨이 있음 -> 다른 --out 을 쓰거나 폴더를 지울 것')
    n_img, n_obj, n_skip, n_bright, tiles = 0, 0, 0, 0, []
    per_cls = {n: 0 for n in names}
    for split in ('train', 'val', 'test'):
        (out / 'images' / split).mkdir(parents=True, exist_ok=True)
        (out / 'labels' / split).mkdir(parents=True, exist_ok=True)
        for f in sorted((sim / split).glob('*_meta.json')):
            stem = f.name[:-len('_meta.json')]
            if a.max_mean > 0 and cv2.imread(str(sim / split / f'{stem}_rgb.png'), cv2.IMREAD_GRAYSCALE).mean() > a.max_mean:
                n_bright += 1
                continue
            meta = json.loads(f.read_text())
            mk = cv2.imread(str(sim / split / f'{stem}_mask.png'), cv2.IMREAD_UNCHANGED)
            h, w = mk.shape
            lines, polys = [], []
            for p in meta['pucks']:
                if p['px'] < a.min_px:
                    n_skip += 1
                    continue
                poly = mask_to_polygon(mk == p['index'] + 1)
                if poly is None:
                    continue
                name = class_name(p, meta, a.classes)
                cls = names.index(name)
                per_cls[name] += 1
                xy = (poly / [w, h]).clip(0, 1).reshape(-1)
                lines.append(f'{cls} ' + ' '.join(f'{v:.5f}' for v in xy))
                polys.append((cls, poly))
            dst = out / 'images' / split / f'{a.prefix}{stem}.png'
            if dst.exists() or dst.is_symlink():
                dst.unlink()
            try:                           # 같은 디스크면 하드 링크 (복사 안 함)
                dst.hardlink_to(sim / split / f'{stem}_rgb.png')
            except OSError:
                dst.write_bytes((sim / split / f'{stem}_rgb.png').read_bytes())
            (out / 'labels' / split / f'{a.prefix}{stem}.txt').write_text('\n'.join(lines) + ('\n' if lines else ''))
            n_img += 1; n_obj += len(lines)
            if len(tiles) < a.check:
                vis = cv2.imread(str(dst))
                for cls, poly in polys:
                    col = (0, 0, 255) if names[cls].startswith('red') else (0, 255, 0)
                    cv2.polylines(vis, [poly.astype(np.int32)], True, col, 1)
                    cv2.putText(vis, names[cls], tuple(int(v) for v in poly[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
                tiles.append(vis)
    (out / 'data.yaml').write_text(f'# sim_to_seg.py 가 만듦\npath: {out}\ntrain: images/train\nval: images/val\ntest: images/test\n'
                                   f'nc: {len(names)}\nnames: {names}\n')
    print(f'이미지 {n_img}, 퍽 라벨 {n_obj}, 작아서 뺀 퍽 {n_skip} -> {out}')
    print('클래스별 (이번 실행):', per_cls)
    if a.max_mean > 0:
        print(f'과노출로 뺀 장 (평균 밝기 > {a.max_mean:.0f}): {n_bright}')
    if tiles:
        cv2.imwrite(str(out / 'check.jpg'), np.vstack(tiles))
        print('그림:', out / 'check.jpg')


if __name__ == '__main__':
    main()
