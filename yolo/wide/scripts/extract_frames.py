#!/usr/bin/env python3
"""실제 광각 카메라 영상 -> 라벨링할 프레임 (재압축 없이 MJPEG 원본 그대로, 끝의 군더더기 바이트만 잘라냄).

  1) ffmpeg 로 모든 프레임을 원본 JPEG 로 꺼냄 (-c:v copy, mjpeg2jpeg) -> 임시 폴더
  2) 구간(--every_s, 기본 0.5 s)마다 가장 선명한 프레임 1장 (Laplacian 분산)
  3) 흐림(선명도 < 중앙값 × --blur_ratio) · 직전에 고른 프레임과 거의 같음(축소 영상 평균 차 < --dup_diff) 은 버림
출력: datasets/real_frames/<영상 이름>/<영상 이름>_<프레임 번호>.jpg, frames.csv (프레임·시각·선명도), contact.jpg (훑어보기)
라벨 달기 전에는 datasets/real/images/ 에 넣지 말 것 (라벨 없는 이미지 = '물체 없음' 으로 학습됨)

예) cd yolo/wide/scripts && python3 extract_frames.py ~/wide_puck_shelf_02.mkv
    (OpenCV 있는 파이썬, ffmpeg 필요)
"""
import argparse
import csv
import subprocess
import tempfile
from pathlib import Path

import cv2
import numpy as np

YOLO_DIR = Path(__file__).resolve().parent.parent


def trim_jpeg(data: bytes) -> bytes:
    """마지막 종료 표시(FFD9) 뒤 군더더기를 잘라냄. 카메라 MJPEG 프레임은 FFD9 뒤에 1바이트(00)가 붙어 있어
    ultralytics 가 '손상된 JPEG' 으로 보고 화질 100 으로 다시 저장함 (파일 약 4배, 2026-10-26 확인). 그림 데이터는 그대로"""
    end = data.rfind(b'\xff\xd9')
    return data[:end + 2] if end >= 0 else data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('videos', nargs='+')
    ap.add_argument('--out', default=str(YOLO_DIR / 'datasets' / 'real_frames'))
    ap.add_argument('--every_s', type=float, default=0.5, help='이 시간마다 1장')
    ap.add_argument('--blur_ratio', type=float, default=0.5)
    ap.add_argument('--dup_diff', type=float, default=4.0, help='0–255 기준, 320×180 축소 영상 평균 절대 차')
    a = ap.parse_args()
    for v in a.videos:
        v = Path(v).expanduser().resolve()
        out = Path(a.out).expanduser() / v.stem
        if out.exists() and any(out.glob('*.jpg')):
            raise SystemExit(f'{out} 에 이미 프레임이 있음 (지우고 다시 하거나 다른 --out)')
        out.mkdir(parents=True, exist_ok=True)
        fps = float(eval(subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
                                                  'stream=r_frame_rate', '-of', 'csv=p=0', str(v)], text=True).strip()))
        with tempfile.TemporaryDirectory() as tmp:
            subprocess.run(['ffmpeg', '-v', 'error', '-i', str(v), '-c:v', 'copy', '-bsf:v', 'mjpeg2jpeg', '-f', 'image2',
                            f'{tmp}/f_%05d.jpg'], check=True)
            fs = sorted(Path(tmp).glob('*.jpg'))
            sharp, small = [], []
            for f in fs:
                g = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
                sharp.append(float(cv2.Laplacian(g, cv2.CV_64F).var()))
                small.append(cv2.resize(g, (320, 180), interpolation=cv2.INTER_AREA).astype(np.float32))
            sharp = np.array(sharp)
            med, W = float(np.median(sharp)), max(1, int(round(a.every_s * fps)))
            keep, n_blur, n_dup = [], 0, 0
            for s in range(0, len(fs), W):
                i = s + int(np.argmax(sharp[s:s + W]))
                if sharp[i] < a.blur_ratio * med:
                    n_blur += 1
                    continue
                if keep and np.abs(small[i] - small[keep[-1]]).mean() < a.dup_diff:
                    n_dup += 1
                    continue
                keep.append(i)
            with open(out / 'frames.csv', 'w', newline='') as fh:
                w = csv.writer(fh)
                w.writerow(['file', 'frame', 'time_s', 'sharpness'])
                for i in keep:
                    name = f'{v.stem}_{i:05d}.jpg'
                    (out / name).write_bytes(trim_jpeg(fs[i].read_bytes()))
                    w.writerow([name, i, round(i / fps, 2), round(float(sharp[i]), 1)])
            th = [cv2.resize(cv2.imread(str(fs[i])), (256, 144)) for i in keep]
            for t, i in zip(th, keep):
                cv2.putText(t, f'{i / fps:.1f}s', (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            while len(th) % 10:
                th.append(np.zeros_like(th[0]))
            cv2.imwrite(str(out / 'contact.jpg'), np.vstack([np.hstack(th[r:r + 10]) for r in range(0, len(th), 10)]))
        print(f'{v.name}: {len(fs)} 프레임 ({len(fs) / fps:.1f} s) -> {len(keep)} 장 (흐림 {n_blur}, 비슷함 {n_dup} 버림) -> {out}')


if __name__ == '__main__':
    main()
