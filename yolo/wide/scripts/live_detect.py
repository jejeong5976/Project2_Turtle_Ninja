#!/usr/bin/env python3
"""카메라 실시간 YOLO 검출 보기 (학습한 best.pt 로)

- 카메라를 학습 사진과 같은 조건(MJPG 1280x720)으로 열고, 프레임마다 검출해서 박스를 그려 보여 줌
- conf 0.5 = planning 기준. 화면 위에 FPS, 추론 시간, 클래스별 개수 표시
- 키: q / Esc 끝내기, s 지금 화면 저장(runs/live/), 스페이스 일시정지, +/- conf 0.05씩 조정

예)
  cd yolo/wide/scripts
  python live_detect.py                                # 기본: runs/wide/real_ft, /dev/video0
  python live_detect.py --model ../models/best.pt --dev 2
  python live_detect.py --device cpu                   # GPU 로 학습 중일 때
카메라 번호 찾기: v4l2-ctl --list-devices
"""
import argparse
import time
from collections import Counter
from pathlib import Path

import cv2
from ultralytics import YOLO

YOLO_DIR = Path(__file__).resolve().parent.parent
COLORS = {'red_cyl': (60, 60, 255), 'green_cyl': (60, 220, 60), 'red_box': (0, 0, 200),
          'green_box': (0, 150, 0), 'barcode': (255, 200, 0), 'traffic_light': (0, 200, 255)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='real_ft', help='runs/wide/<이름>/weights/best.pt 의 이름, 또는 .pt 경로')
    ap.add_argument('--dev', default='0', help='카메라 번호(/dev/videoN 의 N) 또는 장치 경로')
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--imgsz', type=int, default=960, help='학습과 같은 값')
    ap.add_argument('--device', default='0', help='0 = GPU, cpu')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    a = ap.parse_args()

    w = Path(a.model) if a.model.endswith('.pt') else YOLO_DIR / 'runs' / 'wide' / a.model / 'weights' / 'best.pt'
    if not w.exists():
        raise SystemExit(f'가중치 없음: {w}')
    model = YOLO(str(w))
    names = model.names

    dev = int(a.dev) if a.dev.isdigit() else a.dev
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    if not cap.isOpened():
        raise SystemExit(f'카메라 {a.dev} 를 열 수 없음 -> v4l2-ctl --list-devices 로 번호 확인')
    print(f'모델 {w}\n카메라 {a.dev}: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}, '
          f'conf {a.conf}, imgsz {a.imgsz}, device {a.device}\n키: q 끝, s 저장, 스페이스 일시정지, +/- conf')

    out_dir = YOLO_DIR / 'runs' / 'live'
    conf, paused, fps, frame, shown = a.conf, False, 0.0, None, None
    win = f'live: {w.parent.parent.name}'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    t_prev = time.time()
    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                print('프레임 없음 (카메라 연결 확인)')
                break
            t0 = time.time()
            r = model.predict(frame, conf=conf, imgsz=a.imgsz, device=a.device, verbose=False)[0]
            infer_ms = (time.time() - t0) * 1000
            shown = frame.copy()
            cnt = Counter()
            for (x0, y0, x1, y1), c, s in zip(r.boxes.xyxy.int().tolist(), r.boxes.cls.int().tolist(), r.boxes.conf.tolist()):
                n = names[c]
                cnt[n] += 1
                col = COLORS.get(n, (255, 255, 255))
                cv2.rectangle(shown, (x0, y0), (x1, y1), col, 2)
                cv2.putText(shown, f'{n} {s:.2f}', (x0, max(18, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6) if fps else 1 / max(now - t_prev, 1e-6)
            t_prev = now
            info = f'FPS {fps:4.1f} | infer {infer_ms:4.0f} ms | conf {conf:.2f} | ' + (' '.join(f'{k}:{v}' for k, v in sorted(cnt.items())) or '검출 없음')
            cv2.rectangle(shown, (0, 0), (shown.shape[1], 30), (0, 0, 0), -1)
            cv2.putText(shown, info.replace('검출 없음', 'none'), (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.imshow(win, shown)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
        if k == ord(' '):
            paused = not paused
        if k in (ord('+'), ord('=')):
            conf = min(0.95, conf + 0.05)
        if k in (ord('-'), ord('_')):
            conf = max(0.05, conf - 0.05)
        if k == ord('s') and shown is not None:
            out_dir.mkdir(parents=True, exist_ok=True)
            p = out_dir / f'{w.parent.parent.name}_{time.strftime("%H%M%S")}.jpg'
            cv2.imwrite(str(p), shown)
            cv2.imwrite(str(p.with_name(p.stem + '_raw.jpg')), frame)   # 박스 없는 원본 (나중에 라벨링용)
            print('저장:', p)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
