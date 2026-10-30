#!/usr/bin/env python3
"""손목 D435 실시간 퍽 자세 추정 · 시각화 (wrist_pose.py). pyrealsense2 필요.

화면: 왼쪽 = 컬러 + 퍽 윤곽 · 속 중심(노란 점) · 긴축(노란 선, 양 끝 = 끝 면 중심, 길이 6 cm) · 선 퍽 윗면 중심(흰 십자) · AprilTag 윤곽(보라)
      흰 화살표 = 월드 위쪽 (--up plane: 책상 · 선반 윗면 법선, 기본). 위쪽이 있으면 선/누운 퍽 판정 · 중심 높이 보정 (시뮬 평가: 긴축 90% 2.7°)
      CUT = 퍽이 화면 끝에 잘림 (자세 믿지 말 것)
      오른쪽 = 컬러에 정렬된 깊이 (0–1.2 m 색지도, 검정 = 깊이 없음)
키: q 끝, s 저장 (컬러 png · 깊이 npy(m) · 내부 파라미터 · 결과 json · 시각화), p 지금 퍽 점군 3D 그림 (matplotlib), r 확정 묶음 초기화 (--seg)
저장 위치: yolo/wrist/datasets/real_d435/<시각>/ (커밋 안 됨)

예) cd tools/d435_viewer && python3 d435_puck_viewer.py
    --frames 5 : 창 없이 1 초 간격 5 장만 처리해 저장 (원격 · 확인용)
    --offline <저장 폴더> : 카메라 없이 저장한 장면 다시 처리
    --seg ../../yolo/wrist/runs/wrist/seg8_sim/weights/best.pt : HSV 대신 손목 YOLO-seg 로 퍽 분할 (클래스 · 신뢰도 표시)
        + wrist_detector 파이프라인: 거르기(잘림 · 크기 · 점 수, 빨간 X + 이유) → 최근 5 장 중 3 장 확정 (흰 상자 'OK').
          확정 퍽은 planning Detection 형식으로 터미널 · result.json('detections') 에 나옴. 팔을 움직이면 r 키로 묶음 초기화
"""
import argparse
import json
import time
from pathlib import Path

import sys

import cv2
import numpy as np

WRIST_DIR = Path(__file__).resolve().parents[2] / 'yolo' / 'wrist'
sys.path.insert(0, str(WRIST_DIR / 'scripts'))   # wrist_pose · wrist_detector
import wrist_detector as wd  # noqa: E402
import wrist_pose as wp  # noqa: E402


def get_up(a, masks, depth_m, K, prev):
    """이번 장 월드 위쪽 (카메라 광학 좌표). plane 은 퍽 마스크를 빼고 평면을 찾고, 이전 값과 섞음. 못 찾으면 이전 값."""
    if a.up == 'none':
        return None
    if a.up == 'fixed':
        return wp.up_from_pitch_roll(a.pitch_deg, a.roll_deg)
    union = np.zeros(depth_m.shape, bool)                      # 퍽 마스크를 먼저 합친 뒤 한 번만 부풀림 (팽창은 합집합에 분배돼 결과 같음)
    for _, m, _, _ in masks:
        union |= m
    excl = cv2.dilate(union.astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
    up, _ = wp.estimate_up_from_plane(depth_m, K, excl)
    if up is None:
        return prev
    if prev is not None:
        up = a.up_smooth * prev + (1 - a.up_smooth) * up
        up /= np.linalg.norm(up)
    return up


def save(out_root: Path, bgr, depth_m, K, coeffs, pucks, tags, vis, up=None, dets=()) -> Path:
    d = out_root / time.strftime('%m%d_%H%M%S')
    d.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(d / 'color.png'), bgr)
    np.save(d / 'depth_m.npy', depth_m.astype(np.float32))
    cv2.imwrite(str(d / 'vis.png'), vis)
    (d / 'intrinsics.json').write_text(json.dumps({'K': K.tolist(), 'coeffs': list(coeffs)}))
    (d / 'result.json').write_text(json.dumps({
        'pucks': [{'label': p.label, 'kind': p.kind, 'conf': round(p.conf, 3), 'center_m': p.center.round(4).tolist(), 'axis': p.axis.round(4).tolist(),
                   'axis_source': p.axis_source, 'n_points': p.n_points, 'tag_id': p.tag.tag_id if p.tag else None,
                   'truncated': p.truncated}
                  for p in pucks],
        'tags': [{'id': t.tag_id, 't_m': t.t.round(4).tolist(), 'normal': t.R[:, 2].round(4).tolist()} for t in tags],
        'up_cam': None if up is None else up.round(4).tolist(),
        'detections': [d.to_dict() for d in dets],   # wrist_detector 확정 퍽 (planning Detection 형식, 카메라 광학 프레임)
    }, indent=1, ensure_ascii=False))
    return d


def plot3d(pucks, depth_m, K):
    import matplotlib.pyplot as plt
    ax = plt.figure(figsize=(7, 6)).add_subplot(projection='3d')
    for p in pucks:
        pts = wp.deproject(p.mask, depth_m, K)
        ax.scatter(pts[:, 0], pts[:, 2], -pts[:, 1], s=1, label=p.label)
        a, b = p.center - p.axis * wp.PUCK_H / 2, p.center + p.axis * wp.PUCK_H / 2
        ax.plot([a[0], b[0]], [a[2], b[2]], [-a[1], -b[1]], 'k-', lw=2)
    ax.set_xlabel('x 오른쪽 (m)'); ax.set_ylabel('z 앞 (m)'); ax.set_zlabel('위 (m)')
    ax.set_box_aspect((1, 1, 1)); ax.legend(); plt.title("퍽 점군 + 긴축 (카메라 좌표)"); plt.show(block=False); plt.pause(0.1)


class Viewer:
    """OpenCV 창을 쓰고, OpenCV 가 화면 기능 없는 판(opencv-python-headless)이면 Tk 창(PIL ImageTk)으로 대신 보여줌.
    YOLO venv 는 albumentations 설치 때 headless 판이 같이 깔려 cv2.imshow 가 안 됨 (확인함).
    matplotlib 창은 1280×480 한 장 그리는 데 100 ms 넘게 걸려 2 fps 밖에 안 나옴 -> Tk 사용"""

    def __init__(self, title: str):
        self.title, self.key, self.tk = title, None, None
        try:
            cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
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
        """그림 갱신 -> 눌린 키 ('q', 's', 'p' 또는 None)."""
        if self.tk is None:
            cv2.imshow(self.title, img)
            k = cv2.waitKey(1) & 0xFF
            return chr(k) if k != 255 else None
        root, label, Image, ImageTk = self.tk
        photo = ImageTk.PhotoImage(Image.fromarray(np.ascontiguousarray(img[..., ::-1])))
        label.configure(image=photo)
        label.image = photo                                      # 참조 안 잡아두면 그림이 지워짐
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
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--out', default=str(WRIST_DIR / 'datasets' / 'real_d435'))
    ap.add_argument('--frames', type=int, default=0, help='창 없이 1 초 간격으로 이만큼 처리 · 저장하고 끝')
    ap.add_argument('--offline', default=None, help='저장한 폴더(color.png, depth_m.npy, intrinsics.json) 다시 처리')
    ap.add_argument('--no_filter', action='store_true', help='깊이 후처리(공간 · 시간 · 구멍 메움) 끄기')
    ap.add_argument('--up', choices=['plane', 'fixed', 'none'], default='plane',
                    help='월드 위쪽(중력 반대) 얻는 법. plane = 깊이에서 가장 큰 평면(책상 · 선반 윗면) 법선, fixed = --pitch_deg/--roll_deg, '
                         'none = 안 씀. D435(i 아님)는 IMU 없음. 로봇에선 TF 로 넣을 것')
    ap.add_argument('--pitch_deg', type=float, default=60.0, help='--up fixed: 카메라 아래로 숙인 각 (손목 LOOK 자세 약 60°)')
    ap.add_argument('--roll_deg', type=float, default=0.0)
    ap.add_argument('--depth_bias_mm', type=float, default=0.0,
                    help='깊이에서 뺄 값 (mm). 정답 종이에선 깊이가 마커보다 2.4 mm 멀었지만 빼면 수평 오차가 커져 원인 미확정 -> 기본 0')
    ap.add_argument('--up_every', type=int, default=5,
                    help='--up plane: 이 장마다 한 번만 평면 다시 찾음 (RANSAC 약 20 ms). 평면을 못 찾은 상태에서도 이 간격으로만 재시도')
    ap.add_argument('--up_smooth', type=float, default=0.8, help='--up plane: 이전 값 비중 (흔들림 줄임, 0 = 매 장 새로)')
    ap.add_argument('--seg', default=None, help='손목 YOLO-seg 가중치 (예: ../../yolo/wrist/runs/wrist/seg8_sim/weights/best.pt). 없으면 HSV 색 분할')
    ap.add_argument('--seg_conf', type=float, default=wd.SEG_CONF)
    ap.add_argument('--min_hits', type=int, default=3, help='--seg: 최근 --window 장 중 이만큼 보여야 확정')
    ap.add_argument('--window', type=int, default=5)
    a = ap.parse_args()
    wp.DEPTH_BIAS_M = a.depth_bias_mm / 1000
    out_root = Path(a.out).expanduser()
    plt_font()
    det = wd.WristPuckDetector(a.seg, a.seg_conf, window=a.window, min_hits=a.min_hits) if a.seg else None
    seg = det.seg if det else None
    print('분할:', f'YOLO-seg {a.seg} + 확정 {a.min_hits}/{a.window} 장' if seg else 'HSV 색', flush=True)

    def run(bgr, depth, K, masks, up):
        """한 장 -> (퍽 자세 목록, 태그, 확정 Detection 목록, 시각화)"""
        if det is None:
            pucks, tags = wp.estimate(bgr, depth, K, up_cam=up, masks=masks)
            return pucks, tags, [], wp.draw(bgr, depth, K, pucks, tags, up, a.up)
        dets, raws, tags = det.process(bgr, depth, K, up, masks=masks)
        pucks = [r.pose for r in raws]
        vis = wp.draw(bgr, depth, K, pucks, tags, up, a.up)
        return pucks, tags, dets, wd.draw_detections(vis, K, dets, raws)

    if a.offline:
        d = Path(a.offline)
        bgr, depth = cv2.imread(str(d / 'color.png')), np.load(d / 'depth_m.npy')
        intr = json.loads((d / 'intrinsics.json').read_text())
        K = np.array(intr['K'])
        masks = wp.find_masks(bgr, seg)
        up = get_up(a, masks, depth, K, None)
        if det is not None:
            det.min_hits = 1                                   # 저장한 한 장뿐이라 여러 프레임 확인 안 함
        pucks, tags, dets, vis = run(bgr, depth, K, masks, up)
        cv2.imwrite(str(d / 'vis_offline.png'), vis)
        print(f'{d}: 퍽 {len(pucks)}, 태그 {[t.tag_id for t in tags]} -> {d / "vis_offline.png"}')
        for dd in dets:
            print('  ', dd.to_dict())
        return

    import pyrealsense2 as rs
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, a.width, a.height, rs.format.bgr8, a.fps)
    cfg.enable_stream(rs.stream.depth, a.width, a.height, rs.format.z16, a.fps)
    prof = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    ci = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([[ci.fx, 0, ci.ppx], [0, ci.fy, ci.ppy], [0, 0, 1.0]])
    filters = [] if a.no_filter else [rs.spatial_filter(), rs.temporal_filter(), rs.hole_filling_filter()]
    print(f'D435 {a.width}x{a.height}@{a.fps}  fx {ci.fx:.1f} fy {ci.fy:.1f} cx {ci.ppx:.1f} cy {ci.ppy:.1f}  깊이 단위 {scale}', flush=True)
    viewer = None                                               # 첫 프레임 전에 실패해도 finally 에서 읽을 수 있게
    try:
        for _ in range(30):                                     # 자동 노출 안정
            pipe.wait_for_frames()
        n_done, last_save, t_prev, up, i_frame, last_try = 0, 0.0, time.time(), None, 0, None
        while True:
            fs = align.process(pipe.wait_for_frames())
            df = fs.get_depth_frame()
            for f in filters:
                df = f.process(df)
            bgr = np.asanyarray(fs.get_color_frame().get_data()).copy()
            depth = np.asanyarray(df.get_data()).astype(np.float32)
            depth *= scale                                     # 제자리 곱 (임시 배열 하나 덜 만듦)
            masks = wp.find_masks(bgr, seg)
            # 평면 RANSAC 은 up_every 장에 한 번. 위쪽을 못 찾은 상태(None)에서 매 장 재시도하면 거의 수직으로 내려다볼 때
            # (평면이 '벽'으로 거부됨) 장마다 약 20 ms 가 붙음 -> 못 찾아도 같은 간격으로만 재시도
            if a.up != 'plane' or last_try is None or i_frame - last_try >= max(a.up_every, 1):
                last_try = i_frame
                up = get_up(a, masks, depth, K, up)
            i_frame += 1
            pucks, tags, dets, vis = run(bgr, depth, K, masks, up)
            now = time.time()
            cv2.putText(vis, f'{1 / max(now - t_prev, 1e-3):.0f} fps  pucks {len(pucks)} tags {len(tags)} OK {len(dets)}', (8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            t_prev = now
            if a.frames:
                if now - last_save >= 1.0:
                    d = save(out_root, bgr, depth, K, ci.coeffs, pucks, tags, vis, up, dets)
                    up_s = 'None' if up is None else f'{up.round(3).tolist()} (영상 위쪽과 {np.degrees(np.arccos(np.clip(up @ [0, -1, 0], -1, 1))):.0f}°)'
                    print(f'[{n_done + 1}/{a.frames}] 위쪽 {up_s}  퍽 {[(p.label, p.center.round(3).tolist(), p.axis_source, "CUT" if p.truncated else "") for p in pucks]} '
                          f'태그 {[t.tag_id for t in tags]} 확정 {[(dd.label, dd.shape, dd.id, dd.position.round(3).tolist()) for dd in dets]} -> {d}', flush=True)
                    n_done, last_save = n_done + 1, now
                    if n_done >= a.frames:
                        break
                continue
            if viewer is None:
                viewer = Viewer('D435 wrist pose (q 끝, s 저장, p 3D, r 확정 초기화)')
            k = viewer.show(vis)
            if k == 'q':
                break
            if k == 's':
                print('저장:', save(out_root, bgr, depth, K, ci.coeffs, pucks, tags, vis, up, dets), flush=True)
            if k == 'r' and det is not None:                    # 팔(카메라)을 옮긴 뒤 확정 묶음 초기화
                det.reset()
                print('확정 묶음 초기화', flush=True)
            if k == 'p' and pucks:
                plot3d(pucks, depth, K)
    finally:
        pipe.stop()
        if viewer is not None:
            viewer.close()


def plt_font():
    try:
        import matplotlib.pyplot as plt
        plt.rcParams['font.family'] = ['NanumGothic', 'Noto Sans CJK KR', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
    except ImportError:
        pass


if __name__ == '__main__':
    main()
