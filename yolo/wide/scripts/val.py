#!/usr/bin/env python3
"""학습한 모델 평가: 클래스별 mAP·정밀도·재현율 + 혼동 행렬 (실제 test 셋 기준이 원칙).

시뮬 셋과 실제 셋을 같이 주면 두 점수를 나란히 출력 -> sim-to-real 차이가 숫자로 보임.
  예) 시뮬 mAP50 0.99 / 실제 0.10 -> 시뮬에서만 잘 함 (겉모습 차이·버그)
혼동 행렬 그림: <runs>/eval/<name>/confusion_matrix.png (빨강↔초록, 원기둥↔직육면체 헷갈림 확인)

예) python val.py --weights ../runs/wide/sim_pre/weights/last.pt \
      --data ../configs/data_sim.yaml --split test
    --sim_root <시뮬 데이터셋> 을 주면 시뮬 test 도 같이 평가
"""
import argparse
import tempfile
from pathlib import Path

from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
YOLO_DIR = HERE.parent
CLASSES = ['red_cyl', 'green_cyl', 'red_box', 'green_box', 'barcode', 'traffic_light']


def run(model, data, split, imgsz, name, conf):
    m = model.val(data=str(data), split=split, imgsz=imgsz, batch=8, conf=conf, plots=True,
                  project=str(YOLO_DIR / 'runs' / 'eval'), name=name, exist_ok=True, verbose=False)
    names = m.names if hasattr(m, 'names') else dict(enumerate(CLASSES))
    rows = []
    for i, c in enumerate(m.box.ap_class_index):
        p, r, ap50, ap = m.box.class_result(i)
        rows.append((names[int(c)], p, r, ap50, ap))
    return m, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', required=True)
    ap.add_argument('--data', default=str(YOLO_DIR / 'configs' / 'data_sim.yaml'))
    ap.add_argument('--split', default='test', choices=['val', 'test'])
    ap.add_argument('--sim_root', default=None, help='시뮬 데이터셋 루트 (주면 시뮬 test 도 평가해 비교)')
    ap.add_argument('--imgsz', type=int, default=960)
    ap.add_argument('--conf', type=float, default=0.001, help='mAP 계산용 (낮게 두는 게 표준)')
    a = ap.parse_args()
    model = YOLO(a.weights)
    results = {}
    m, rows = run(model, a.data, a.split, a.imgsz, f'{Path(a.weights).parent.parent.name}_{a.split}', a.conf)
    results['주 데이터'] = (m, rows)
    if a.sim_root:
        root = Path(a.sim_root).expanduser().resolve()
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
            f.write(f'train: {root}/images/train\nval: {root}/images/val\ntest: {root}/images/test\n'
                    f'nc: {len(CLASSES)}\nnames: {CLASSES}\n')
        results['시뮬'] = run(model, f.name, 'test', a.imgsz, f'{Path(a.weights).parent.parent.name}_sim', a.conf)

    for key, (m, rows) in results.items():
        print(f'\n[{key}] 전체 mAP50 {m.box.map50:.3f}  mAP50-95 {m.box.map:.3f}  P {m.box.mp:.3f}  R {m.box.mr:.3f}')
        print(f'  {"클래스":<14}{"P":>7}{"R":>7}{"mAP50":>8}{"mAP50-95":>10}')
        for name, p, r, ap50, ap in rows:
            print(f'  {name:<14}{p:7.3f}{r:7.3f}{ap50:8.3f}{ap:10.3f}')
        print(f'  그림 (혼동 행렬, PR 곡선): {m.save_dir}')


if __name__ == '__main__':
    main()
