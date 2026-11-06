#!/usr/bin/env python3
"""광각 YOLO 상자 검출 → C++ perception 노드(ros2_ws/src/perception, 광각 파이프라인) 용 ONNX. 손목 export_onnx.py 와 같은 방식.

head = 체크포인트 설정 그대로 (C++ wide_detector 가 두 모양 다 받음)
  end2end  : output0 [1, 300, 6]       (x1, y1, x2, y2, score, class, NMS 없음)
  그 밖    : output0 [1, 4 + nc, 후보]  (cx, cy, w, h + 클래스 점수 → C++ 이 NMS)
입력 크기 기본 544×960 (h×w, 32 배수, 학습 imgsz 960 과 같은 축척, 1280×720 원본을 줄여 위아래 여백)
  736×1280 (원본 축척 그대로) 은 --height 736 --width 1280. C++ 노드는 입력 크기를 엔진에서 읽어 둘 다 받음 → Jetson 속도 재고 고름
  <이름>.onnx 옆에 <이름>.names.txt (클래스 이름 한 줄씩, C++ 노드가 읽음 → 엔진 옆에 같이 둘 것)
Jetson 엔진: trtexec --onnx=<이름>.onnx --saveEngine=<이름>.engine --fp16   (Jetson 에서 직접 만들어야 함, 장비마다 다름)
  ultralytics export(format='engine') 는 쓰지 말 것 (엔진 앞에 메타데이터를 붙여 C++ 노드가 못 읽음)
예) cd yolo/wide && ~/Desktop/physical_ai/.venvs/yolo/bin/python scripts/export_onnx.py --weights runs/wide/real_ft/weights/best.pt
"""
import argparse
import shutil
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', required=True)
    ap.add_argument('--height', type=int, default=544)
    ap.add_argument('--width', type=int, default=960)
    ap.add_argument('--opset', type=int, default=17)
    ap.add_argument('--out', default=None, help='저장 경로 (.onnx). 기본 = 가중치 옆 <이름>_<H>x<W>.onnx')
    a = ap.parse_args()

    from ultralytics import YOLO
    import onnx

    w = Path(a.weights).resolve()
    model = YOLO(str(w))
    names = [model.names[i] for i in range(len(model.names))]
    f = model.export(format='onnx', imgsz=[a.height, a.width], opset=a.opset, simplify=True, dynamic=False)
    out = Path(a.out) if a.out else w.with_name(f'{w.stem}_{a.height}x{a.width}.onnx')
    shutil.move(f, out)
    out.with_suffix('.names.txt').write_text('\n'.join(names) + '\n')
    g = onnx.load(str(out))
    shapes = [(o.name, [d.dim_value for d in o.type.tensor_type.shape.dim]) for o in g.graph.output]
    print(f'저장: {out}\n출력: {shapes}\n클래스: {names}')
    if len(shapes) != 1:
        raise SystemExit(f'출력 {len(shapes)} 개 (상자 모델은 1 개)')
    s = shapes[0][1]
    if not (len(s) == 3 and (s[2] == 6 or s[1] == 4 + len(names))):
        raise SystemExit(f'output0 모양 {s} ≠ [1, N, 6] · [1, 4 + {len(names)}, N]')


if __name__ == '__main__':
    main()
