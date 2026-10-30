#!/usr/bin/env python3
"""손목 YOLO26-seg → C++ 노드(ros2_ws/src/perception wrist_puck_node) 용 ONNX.

head (--head)
  auto (기본) : 체크포인트 설정을 따름 = Python predict (wrist_pose.Segmenter) 와 같은 head
               지금 학습 결과(seg8_sim)는 end2end=False → one2many head + NMS (C++ 이 NMS 함)
  o2m        : one2many head 원시 출력  output0 [1, 4 + nc + 32, A] (상자 xywh)
  o2o        : one2one head 원시 출력   output0 [1, A, 4 + nc + 32] (상자 xyxy, NMS 없음).
               ultralytics end2end 출력 끝의 TopK · Gather 를 빼고 냄 (TopK 는 C++ 이 점수 거르기 · 정렬로 대신)
  ※ 두 head 는 결과가 다름 (시뮬 150 장: 신뢰도 차이 중앙 0.016). Python 평가 수치와 맞추려면 auto
입력 크기 = D435 컬러 크기 (기본 480×640) → 여백(letterbox) 없음, 640×640 보다 계산 25 % 적음
output1 [1, 32, H/4, W/4] = 마스크 원형. 클래스 점수는 둘 다 sigmoid 끝
  <이름>.onnx 옆에 <이름>.names.txt (클래스 이름 한 줄씩, C++ 노드가 읽음)
Jetson 엔진: trtexec --onnx=<이름>.onnx --saveEngine=<이름>.engine --fp16   (Jetson 에서 직접 만들어야 함, 장비마다 다름)
예) cd yolo/wrist && ~/Desktop/physical_ai/.venvs/yolo/bin/python scripts/export_onnx.py --weights runs/wrist/seg8_sim/weights/best.pt
"""
import argparse
import shutil
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', required=True)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--opset', type=int, default=17)
    ap.add_argument('--head', choices=['auto', 'o2m', 'o2o'], default='auto')
    ap.add_argument('--out', default=None, help='저장 경로 (.onnx). 기본 = 가중치 옆 <이름>_<head>_<H>x<W>.onnx')
    a = ap.parse_args()

    from ultralytics import YOLO
    from ultralytics.nn.modules.head import Detect
    import onnx

    w = Path(a.weights).resolve()
    model = YOLO(str(w))
    names = [model.names[i] for i in range(len(model.names))]
    head = a.head
    if head == 'auto':
        head = 'o2o' if getattr(model.model.model[-1], 'end2end', False) else 'o2m'
    if head == 'o2o':
        Detect.postprocess = lambda self, p: p              # TopK 빼고 one2one 원시 출력 [1, A, 4+nc+nm]
    f = model.export(format='onnx', imgsz=[a.height, a.width], opset=a.opset, simplify=True, end2end=head == 'o2o',
                     dynamic=False)
    out = Path(a.out) if a.out else w.with_name(f'{w.stem}_{head}_{a.height}x{a.width}.onnx')
    shutil.move(f, out)
    out.with_suffix('.names.txt').write_text('\n'.join(names) + '\n')
    g = onnx.load(str(out))
    shapes = [(o.name, [d.dim_value for d in o.type.tensor_type.shape.dim]) for o in g.graph.output]
    ops = sorted({n.op_type for n in g.graph.node})
    print(f'저장: {out} (head {head})\n출력: {shapes}\n클래스: {names}\n연산: {ops}')
    if 'TopK' in ops:
        raise SystemExit('TopK 가 남음 → ultralytics 판이 바뀌었는지 확인 (head.postprocess)')
    nc = len(names)
    ch = shapes[0][1][2] if head == 'o2o' else shapes[0][1][1]
    if ch != 4 + nc + 32:
        raise SystemExit(f'output0 채널 {ch} ≠ 4 + {nc} + 32')


if __name__ == '__main__':
    main()
