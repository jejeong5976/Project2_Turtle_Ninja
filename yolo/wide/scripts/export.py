#!/usr/bin/env python3
"""학습한 .pt -> ONNX (PC 에서). TensorRT 엔진은 반드시 Jetson 에서 직접 빌드 (가이드 6절: 다른 GPU 엔진은 Jetson 에서 안 돎).

광각 엔진 입력 736×1280 (h×w, 32 배수, 1280×720 원본 그대로 + 위아래 8 px 채움). 연산이 부족하면 544×960.
예) PC   : python export.py --weights ../runs/wide/real_ft/weights/best.pt
    PC → Jetson: scp ../models/best.pt ../models/wide_736x1280.onnx <user>@<jetson 주소>:~/models/
    Jetson: python3 -c "from ultralytics import YOLO; YOLO('best.pt').export(format='engine', imgsz=(736, 1280), half=True)"
            -> 이름을 wide_736x1280.engine 으로 바꿈 (가중치·엔진은 커밋하지 않고 공유 드라이브에도 올리지 않음)
"""
import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
YOLO_DIR = HERE.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', required=True)
    ap.add_argument('--imgsz', type=int, nargs=2, default=[736, 1280], help='h w')
    a = ap.parse_args()
    (YOLO_DIR / 'models').mkdir(exist_ok=True)
    path = YOLO(a.weights).export(format='onnx', imgsz=tuple(a.imgsz), dynamic=False, simplify=True)
    dst = YOLO_DIR / 'models' / f'wide_{a.imgsz[0]}x{a.imgsz[1]}.onnx'
    shutil.copy(path, dst)
    shutil.copy(a.weights, YOLO_DIR / 'models' / 'best.pt')
    print(f'ONNX: {dst}\n.pt : {YOLO_DIR / "models" / "best.pt"}\nJetson 에서 엔진 빌드는 이 파일 맨 위 설명 참고')


if __name__ == '__main__':
    main()
