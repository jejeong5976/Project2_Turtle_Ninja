#!/usr/bin/env python3
"""예측 결과를 그림으로 저장

conf 를 낮게(기본 0.05) 두고 그려서 '박스는 있는데 클래스가 틀림 / 박스가 아예 없음 / 엉뚱한 곳에 박스' 를 구분.
  클래스가 빨강↔초록으로 틀림 -> 색 문제 (RGB/BGR 뒤바뀜, 퍽 색 차이)
  퍽에 박스가 거의 없음      -> 겉모습 차이 (시뮬과 실제가 너무 다름)
  벽·기둥에 박스             -> 배경 지름길 학습
입력: 이미지 폴더, 동영상 파일, 카메라 번호 (예: 0). 실제 카메라 원본(왜곡 펴지 않은 1280×720)을 그대로 넣을 것
  (시뮬 데이터가 어안 원본 영상 기준이라, 펴서 넣으면 다시 차이가 생김)
출력: <runs>/predict/<name>/ (그린 이미지) + summary.txt (클래스별 검출 수, 신뢰도 분포)

예) python predict.py --weights ../runs/wide/sim_pre/weights/last.pt --source ~/real_frames/
    시험 1 (추론 코드 확인): --source 에 시뮬 test 이미지 폴더를 넣어 잘 맞히는지 먼저 확인
"""
import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np
from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
YOLO_DIR = HERE.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', required=True)
    ap.add_argument('--source', required=True, help='이미지 폴더 / 동영상 / 카메라 번호')
    ap.add_argument('--conf', type=float, default=0.05)
    ap.add_argument('--imgsz', type=int, default=960)
    ap.add_argument('--name', default=None)
    ap.add_argument('--max_frames', type=int, default=500, help='동영상·카메라일 때 최대 장수')
    a = ap.parse_args()
    model = YOLO(a.weights)
    src = int(a.source) if a.source.isdigit() else str(Path(a.source).expanduser())
    name = a.name or (Path(str(src)).stem if not isinstance(src, int) else f'cam{src}')
    confs = defaultdict(list)
    n_img = n_empty = 0
    for i, r in enumerate(model.predict(source=src, conf=a.conf, imgsz=a.imgsz, stream=True, save=True,
                                        project=str(YOLO_DIR / 'runs' / 'predict'), name=name, exist_ok=True,
                                        line_width=2, verbose=False)):
        n_img += 1
        if len(r.boxes) == 0:
            n_empty += 1
        for c, s in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist()):
            confs[r.names[int(c)]].append(s)
        if i + 1 >= a.max_frames:
            break
    out = YOLO_DIR / 'runs' / 'predict' / name
    lines = [f'이미지 {n_img}장, 검출 없음 {n_empty}장 (conf ≥ {a.conf})', f'{"클래스":<14}{"개수":>6}{"중앙 conf":>10}{"conf≥0.5":>10}']
    for k, v in sorted(confs.items()):
        v = np.array(v)
        lines.append(f'{k:<14}{len(v):6d}{np.median(v):10.2f}{(v >= 0.5).sum():10d}')
    (out / 'summary.txt').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    print('그림:', out)


if __name__ == '__main__':
    main()
