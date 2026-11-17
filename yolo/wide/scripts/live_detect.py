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
창: OpenCV 가 화면 기능 없는 판(yolo venv, albumentations 가 headless 판을 깔아 cv2.imshow 안 됨)이면 Tk 창으로 대신 보여 줌
"""
import argparse
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

YOLO_DIR = Path(__file__).resolve().parent.parent
COLORS = {'red_cyl': (60, 60, 255), 'green_cyl': (60, 220, 60), 'red_box': (0, 0, 200),
          'green_box': (0, 150, 0), 'barcode': (255, 200, 0), 'traffic_light': (0, 200, 255)}


class Viewer:
    """OpenCV 창, 안 되면 Tk 창(PIL ImageTk). tools/wrist_d435/d435_puck_viewer.py 와 같은 방식"""

    def __init__(self, title: str):
        self.title, self.key, self.tk = title, None, None
        try:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        except cv2.error:
            import tkinter as tk
            from PIL import Image, ImageTk
            root = tk.Tk()
            root.title(title)
            label = tk.Label(root)
            label.pack()
            root.bind('<Key>', lambda e: setattr(self, 'key', e.char or None))
            root.protocol('WM_DELETE_WINDOW', lambda: setattr(self, 'key', 'q'))
            self.tk = (root, label, Image, ImageTk)
            print('OpenCV 창 기능 없음 -> Tk 창으로 표시 (창을 클릭한 뒤 키 입력)', flush=True)

    def show(self, img) -> str | None:
        """그림 갱신 -> 눌린 키 (문자 하나 또는 None)"""
        if self.tk is None:
            cv2.imshow(self.title, img)
            k = cv2.waitKey(1) & 0xFF
            return chr(k) if k != 255 else None
        root, label, Image, ImageTk = self.tk
        photo = ImageTk.PhotoImage(Image.fromarray(np.ascontiguousarray(img[..., ::-1])))
        label.configure(image=photo)
        label.image = photo                                      # 참조 안 잡아 두면 그림이 지워짐
        root.update()
        k, self.key = self.key, None
        return k

    def close(self):
        if self.tk is None:
            cv2.destroyAllWindows()
        else:
            self.tk[0].destroy()


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
    view = Viewer(f'live: {w.parent.parent.name}')
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
        k = view.show(shown)
        if k in ('q', '\x1b'):
            break
        if k == ' ':
            paused = not paused
        if k in ('+', '='):
            conf = min(0.95, conf + 0.05)
        if k in ('-', '_'):
            conf = max(0.05, conf - 0.05)
        if k == 's' and shown is not None:
            out_dir.mkdir(parents=True, exist_ok=True)
            p = out_dir / f'{w.parent.parent.name}_{time.strftime("%H%M%S")}.jpg'
            cv2.imwrite(str(p), shown)
            cv2.imwrite(str(p.with_name(p.stem + '_raw.jpg')), frame)   # 박스 없는 원본 (나중에 라벨링용)
            print('저장:', p)
    cap.release()
    view.close()


if __name__ == '__main__':
    main()
