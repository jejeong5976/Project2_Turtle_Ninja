#!/usr/bin/env python3
"""
손목 seg 모델 평가: 클래스별 상자 · 마스크 mAP (test 기준) + confusion matrix (runs/eval/<이름>)
"""
import argparse
from pathlib import Path

from ultralytics import YOLO

WRIST_DIR = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', required=True)
    ap.add_argument('--data', default=str(WRIST_DIR / 'datasets' / 'sim_seg' / 'data.yaml'))
    ap.add_argument('--split', default='test', choices=['val', 'test'])
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--conf', type=float, default=0.001)
    a = ap.parse_args()
    m = YOLO(a.weights).val(data=a.data, split=a.split, imgsz=a.imgsz, batch=8, conf=a.conf, plots=True,
                            project=str(WRIST_DIR / 'runs' / 'eval'), name=f'{Path(a.weights).parent.parent.name}_{a.split}',
                            exist_ok=True, verbose=False)
    print(f'\n상자  mAP50 {m.box.map50:.3f}  mAP50-95 {m.box.map:.3f}')
    print(f'마스크 mAP50 {m.seg.map50:.3f}  mAP50-95 {m.seg.map:.3f}')
    print(f'  {"클래스":<12}{"상자 P":>8}{"R":>7}{"mAP50":>8}{"마스크 P":>10}{"R":>7}{"mAP50":>8}{"50-95":>8}')
    for i, c in enumerate(m.seg.ap_class_index):
        bp, br, b50, _ = m.box.class_result(i)
        sp, sr, s50, s = m.seg.class_result(i)
        print(f'  {m.names[int(c)]:<12}{bp:8.3f}{br:7.3f}{b50:8.3f}{sp:10.3f}{sr:7.3f}{s50:8.3f}{s:8.3f}')
    print('그림:', m.save_dir)


if __name__ == '__main__':
    main()
