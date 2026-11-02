"""손목 D435 퍽 자세 추정 (ROS 없음, numpy · OpenCV 만). d435_puck_viewer.py · wrist_detector.py 가 씀. 로봇 노드는 따로 관리하는 C++ 구현 (ros2_ws/src/perception puck_geometry).

입력: 컬러(BGR) + 컬러에 정렬된 깊이(m) + 컬러 내부 파라미터 K
  1) 퍽 분할: 손목 YOLO-seg (Segmenter, yolo/wrist/scripts/train.py 결과) 또는 HSV 색 (color_masks, 모델 없을 때). puck_pose 는 마스크만 받음
  2) 마스크 + 깊이 -> 카메라 좌표 점군 -> 튀는 점 제거 -> 중심 · 긴축(PCA 가장 큰 고유벡터)
     중심 = 보이는 면 점 평균 그대로 (시선 방향 반지름 절반 밀기 보정은 뺌). up_cam 있으면 _gravity_refine 이 다시 정함
  3) AprilTag 36h11: ID 0–11 퍽 (검은 사각 16 mm), 12–14 장소 (48 mm). solvePnP(IPPE_SQUARE) 로 태그 면 자세
     태그가 퍽 마스크 안이면: 윗면/옆면을 영상 속 마스크 긴 방향으로 판정 (_tag_is_top)
       윗면 -> 긴축 = 법선, 중심 = 태그 - 법선 × 높이/2. 옆면 -> 긴축 = 태그 위쪽, 중심 = 태그 - 법선 × 폭/2
       (시뮬 make_props_usd.py 의 라벨 배치와 같은 약속: 옆면 라벨의 위쪽 = 퍽 +z) -> PCA 축 · 점 평균보다 정확
     시뮬 평가 (eval_sim.py, 60장 128개): 태그 없으면 긴축 오차 중앙 15° · 90% 83°, 태그 있으면 아래 결과 참고
     ※ 월드 위쪽(up_cam)을 알면 태그는 ID 읽기에만 씀: 실물 태그는 손으로 붙여 면과 나란하다는 보장이 없고
       붙인 면도 퍽마다 다름 → 긴축 · 선/누움 · 중심은 중력 + 점군, 선 퍽 면 방향은 윗면 윤곽(_upright_face)으로 정함
       태그로 방향을 정하는 위 방식은 up_cam 이 없을 때만 씀
출력 좌표: 카메라 광학 프레임 (x 오른쪽, y 아래, z 앞), m. 퍽 자세 약속은 planning 인터페이스(긴축 = z)와 같게
가정: 퍽 Ø30 × 60 mm (원기둥) / 30 × 30 × 60 mm (직육면체), 태그 크기는 시뮬 배포 표식 기준 (실물 다르면 TAG_SIZE 수정)
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

PUCK_D, PUCK_H = 0.030, 0.060
TAG_SIZE = {**{i: 0.016 for i in range(12)}, **{i: 0.048 for i in (12, 13, 14)}}   # 검은 사각 한 변 (m)
DEPTH_RANGE = (0.10, 1.20)        # 이 밖의 깊이는 버림 (D435 가까운 한계 약 0.1–0.3 m, 해상도에 따라)
DEPTH_BIAS_M = 0.0                # 깊이에서 뺄 값 (m). 실물 D435 는 정답 종이로 잰 값을 d435_puck_viewer --depth_bias_mm 로 줌.
                                  # 개발 PC D435, 0.31–0.42 m: 깊이가 마커 평면보다 중앙 2.4 mm 멂 (장면마다 0.6–4.3).
                                  # 그런데 2.4 mm 빼면 높이는 맞고 수평 오차가 1.5 -> 2.2 mm 로 커짐 (수평은 카메라 쪽으로 1.4 mm 오그라든 치우침)
                                  # -> 깊이 탓인지 마커 정답(인쇄 축척 · 내부 파라미터) 탓인지 미확정, 기본 0
HSV = {   # OpenCV HSV (H 0–179). 실물 조명에서 맞출 것 (d435_puck_viewer.py 의 --hsv_debug 로 확인)
    'green': [((40, 80, 40), (85, 255, 255))],
    'red': [((0, 140, 50), (6, 255, 255)), ((170, 140, 50), (179, 255, 255))],   # 채도 140↑: 피부(채도 약 80–150)를 퍽으로 잡던 것 뺌. 팀 빨강 (157,34,53) = H 175 · S 199. 실물 빨간 퍽 나오면 다시 맞출 것
}
MIN_AREA_PX = 150


@dataclass
class TagPose:
    tag_id: int
    corners: np.ndarray          # (4, 2) 화소
    R: np.ndarray                # 태그 -> 카메라 회전 (태그 z = 면 법선, 카메라 쪽)
    t: np.ndarray                # 태그 중심 (m)
    size: float
    alts: list = field(default_factory=list)   # IPPE 두 해 [(R, t), ...] (작은 태그는 법선이 뒤집힌 해를 고르기 쉬움 -> 깊이로 다시 고름)


@dataclass
class PuckPose:
    label: str
    mask: np.ndarray
    center: np.ndarray           # 퍽 중심 추정 (m)
    axis: np.ndarray             # 긴축 단위벡터 (부호 = orient_axis: 선 퍽 위쪽, 그 밖 화면 오른쪽)
    n_points: int
    extent: np.ndarray           # PCA 축 방향 점 퍼짐 (m, 큰 순) -> 보이는 크기 감
    tag: TagPose | None = None
    axis_source: str = 'pca'     # 'pca' / 'tag_normal' (윗·아랫면 태그) / 'tag_up' (옆면 태그) / 'gravity_up' / 'gravity_flat'
    bbox: tuple = field(default_factory=tuple)
    truncated: bool = False      # 마스크가 화면 가장자리에 닿음 = 일부만 보임 -> 자세를 믿지 말고 다시 볼 것 (시뮬: 잘린 퍽이 큰 오차 대부분)
    kind: str = ''               # seg 클래스 (red_cyl 등). HSV 분할이면 '' (모양 모름)
    conf: float = 1.0            # seg 신뢰도 (HSV 는 1)
    upright: bool | None = None  # 선 퍽인지 (up_cam 있을 때만 판정)
    face_dir: np.ndarray | None = None   # 사각기둥 면 법선 방향 (카메라 좌표 단위벡터, 90° 주기)
                                         #   선 퍽: 윗면 사각형의 한 변 방향 (수평). 누운 퍽: 위쪽 (_lying_face, 바닥에 닿은 면의 반대)
    face_size: tuple = ()        # 윗면에 맞춘 사각형 두 변 (m) — 확인용
    top_center: np.ndarray | None = None   # 윗면 중심 (m, up_cam 있을 때만). 선 퍽 집기 기준점 · 시각화용
    height_source: str = ''      # 중심 높이를 무엇으로 정했나: 'support' (놓인 면 + 크기 절반) / 'top' (윗면 - 크기 절반)
    points: np.ndarray | None = None     # 튀는 점 뺀 마스크 점군 (카메라 m). wrist_detector.point_spread 가 다시 뽑지 않고 씀


def color_masks(bgr: np.ndarray, hsv_ranges: dict = HSV, min_area: int = MIN_AREA_PX) -> list[tuple[str, np.ndarray]]:
    """색 이름별 연결 성분 마스크 [(label, mask bool)]. 작은 성분은 버림."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    out = []
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    for label, ranges in hsv_ranges.items():
        m = np.zeros(bgr.shape[:2], np.uint8)
        for lo, hi in ranges:
            m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
        n, lab, st, _ = cv2.connectedComponentsWithStats(m)
        for i in range(1, n):
            if st[i, cv2.CC_STAT_AREA] >= min_area:
                out.append((label, lab == i))
    return out


class Segmenter:
    """손목 YOLO-seg (ultralytics) -> 퍽 마스크. 클래스 이름 = '<색>_<모양>' (red_cyl, green_cyl, red_box, green_box)."""

    def __init__(self, weights: str, conf: float = 0.5, imgsz: int = 640, device=None):
        from ultralytics import YOLO
        self.model, self.conf, self.imgsz, self.device = YOLO(weights), conf, imgsz, device

    def __call__(self, bgr: np.ndarray) -> list[tuple[str, np.ndarray, str, float]]:
        r = self.model.predict(bgr, conf=self.conf, imgsz=self.imgsz, retina_masks=True, device=self.device, verbose=False)[0]
        if r.masks is None:
            return []
        out = []
        for m, c, s in zip(r.masks.data.cpu().numpy() > 0.5, r.boxes.cls.tolist(), r.boxes.conf.tolist()):
            kind = r.names[int(c)]
            if m.shape != bgr.shape[:2]:
                m = cv2.resize(m.astype(np.uint8), (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
            out.append((kind.split('_')[0], m, kind, float(s)))
        return out


def find_masks(bgr: np.ndarray, seg: Segmenter | None = None) -> list[tuple[str, np.ndarray, str, float]]:
    """퍽 마스크 [(색, mask, 클래스, 신뢰도)]: seg 있으면 YOLO-seg, 없으면 HSV (클래스 '', 신뢰도 1)."""
    if seg is not None:
        return seg(bgr)
    return [(lab, m, '', 1.0) for lab, m in color_masks(bgr)]


def mask_bbox(mask: np.ndarray, pad: int = 0) -> tuple:
    """마스크를 둘러싼 상자 (x0, y0, x1, y1, 끝 포함) + 사방 pad 화소 (화면 안으로 자름). 빈 마스크면 ()"""
    rows, cols = np.flatnonzero(mask.any(1)), np.flatnonzero(mask.any(0))
    if len(rows) == 0:
        return ()
    H, W = mask.shape
    return (max(0, int(cols[0]) - pad), max(0, int(rows[0]) - pad), min(W - 1, int(cols[-1]) + pad), min(H - 1, int(rows[-1]) + pad))


def deproject(mask: np.ndarray, depth_m: np.ndarray, K: np.ndarray, roi: tuple = ()) -> np.ndarray:
    """마스크 안 유효 깊이 화소 -> 카메라 좌표 점 (N, 3).
    roi = (x0, y0, x1, y1): 이 상자 안만 봄 (화면 전체를 훑지 않음). mask 는 화면 크기 또는 이미 roi 로 자른 크기"""
    x0, y0 = (roi[0], roi[1]) if roi else (0, 0)
    if roi:
        sl = (slice(roi[1], roi[3] + 1), slice(roi[0], roi[2] + 1))
        depth_m = depth_m[sl]
        if mask.shape != depth_m.shape:
            mask = mask[sl]
    v, u = np.nonzero(mask & (depth_m > DEPTH_RANGE[0]) & (depth_m < DEPTH_RANGE[1]))
    z = depth_m[v, u] - DEPTH_BIAS_M
    u, v = u + x0, v + y0
    return np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], -1)


_TAG_DET = None
_TAG_OBJ = {s: np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0], [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], np.float32) for s in set(TAG_SIZE.values())}


def detect_tags(gray: np.ndarray, K: np.ndarray, dist: np.ndarray | None = None) -> list[TagPose]:
    global _TAG_DET
    if _TAG_DET is None:                                           # 검출기는 한 번만 만듦
        _TAG_DET = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                           cv2.aruco.DetectorParameters())
    corners, ids, _ = _TAG_DET.detectMarkers(gray)
    out = []
    if ids is None:
        return out
    dist = np.zeros(5) if dist is None else dist
    for c, i in zip(corners, ids.flatten()):
        s = TAG_SIZE.get(int(i))
        if s is None:
            continue
        obj = _TAG_OBJ[s]
        n_sol, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, c.reshape(4, 2).astype(np.float32), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if n_sol:
            alts = [(cv2.Rodrigues(r)[0], t.reshape(3)) for r, t in zip(rvecs, tvecs)]
            out.append(TagPose(int(i), c.reshape(4, 2), alts[0][0], alts[0][1], s, alts))
    return out


def refine_tag_with_depth(t: TagPose, depth_m: np.ndarray, K: np.ndarray) -> TagPose:
    """태그 안쪽 깊이 점으로 평면 법선을 구해, IPPE 두 해 중 법선이 더 가까운 해를 고름. 점이 적으면 그대로."""
    poly = np.zeros(depth_m.shape, np.uint8)
    c = t.corners.mean(0)
    cv2.fillConvexPoly(poly, ((t.corners - c) * 0.8 + c).astype(np.int32), 1)   # 가장자리 깊이 튐 피하려고 80 % 안쪽
    pts = deproject(poly.astype(bool), depth_m, K)
    if len(pts) < 12 or len(t.alts) < 2:
        return t
    q = pts - pts.mean(0)
    nd = np.linalg.eigh(q.T @ q)[1][:, 0]                      # 가장 작은 고유벡터 = 평면 법선
    R, tt = max(t.alts, key=lambda a: abs(a[0][:, 2] @ nd))
    t.R, t.t = R, tt
    return t


def orient_axis(axis: np.ndarray, up: np.ndarray | None = None) -> np.ndarray:
    """긴축 부호 고정. 퍽은 양 끝이 같아 부호에 뜻은 없지만, 장마다 뒤집히면 화살표가 왔다 갔다 하고 평균이 깨짐.
    선 퍽(위쪽과 45° 안) = 위쪽, 그 밖(누운 퍽 · 위쪽 모름) = 화면 오른쪽(+x), 화면에서 거의 세로면 화면 아래(+y).
    '카메라에서 멀어지지 않는 쪽' 규칙은 누운 퍽처럼 긴축이 시선과 거의 수직이면 잡음으로 매 장 뒤집힘 (실물).
    누운 퍽이 화면에서 비스듬한 경계 근처면 여전히 뒤집힐 수 있음 -> wrist_detector 가 같은 퍽 이전 장 부호에 맞춤"""
    if up is not None and abs(axis @ up) >= np.cos(np.radians(45)):
        return axis if axis @ up >= 0 else -axis
    s = axis[0] if abs(axis[0]) >= 0.2 else axis[1]
    return axis if s >= 0 else -axis


_K41, _K15 = np.ones((41, 41), np.uint8), np.ones((15, 15), np.uint8)   # 둘레 띠 팽창 커널 (퍽 둘레 7–20 px)


def puck_pose(label: str, mask: np.ndarray, depth_m: np.ndarray, K: np.ndarray, tags: list[TagPose] = (),
              up_cam: np.ndarray | None = None) -> PuckPose | None:
    """마스크 하나 -> 퍽 자세. up_cam = 카메라 좌표에서 본 월드 위쪽(중력 반대) 단위벡터 (로봇에선 TF, 시뮬은 카메라 자세).
    up_cam 이 있으면 태그가 안 보일 때도 선/누움을 높이 범위로 나눠 긴축을 정하고, 중심 높이를 퍽 윗면에서 내려 잡음."""
    bb = mask_bbox(mask)
    if not bb:
        return None
    pts = deproject(mask, depth_m, K, bb)
    if len(pts) < 30:
        return None
    med = np.median(pts, 0)
    keep = np.linalg.norm(pts - med, axis=1) < 0.06            # 퍽 크기(대각 약 7 cm) 밖 튀는 점 제거
    pts = pts[keep]
    if len(pts) < 30:
        return None
    c = pts.mean(0)
    w, V = np.linalg.eigh(np.cov((pts - c).T))
    order = np.argsort(w)[::-1]
    axis = V[:, order[0]]
    axis = orient_axis(axis, up_cam)
    center = c                                                 # 보이는 면 점 평균 (시선 방향 밀기 보정 뺌)
    H, W = mask.shape
    bbox = bb
    p = PuckPose(label, mask, center, axis, len(pts), np.sqrt(np.maximum(w[order], 0)) * 2, bbox=bbox,
                 truncated=bbox[0] <= 2 or bbox[1] <= 2 or bbox[2] >= W - 3 or bbox[3] >= H - 3, points=pts)
    for t in tags:                                             # 마스크 안(또는 가장자리)에 태그 중심이 있으면 그 퍽의 태그
        cx, cy = t.corners.mean(0).astype(int)
        x0, y0, x1, y1 = p.bbox
        if not (x0 - 5 <= cx <= x1 + 5 and y0 - 5 <= cy <= y1 + 5 and t.tag_id < 12):
            continue
        if up_cam is not None:                                 # 위쪽을 알면 태그는 ID 만 (방향은 형상으로)
            p.tag = t
            break
        t = refine_tag_with_depth(t, depth_m, K)
        n, up = t.R[:, 2], t.R[:, 1]                           # 태그 법선, 태그 그림 위쪽
        if _tag_is_top(pts, n, up):                            # 윗면 · 아랫면 태그: 긴축 = 법선
            p.axis, p.axis_source = n.copy(), 'tag_normal'
        else:                                                  # 옆면 태그: 긴축 = 태그 위쪽
            p.axis, p.axis_source = up.copy(), 'tag_up'
        p.axis = orient_axis(p.axis, up_cam)
        p.tag = t
        break
    if up_cam is not None:
        r = (max(0, bb[0] - 20), max(0, bb[1] - 20), min(W - 1, bb[2] + 20), min(H - 1, bb[3] + 20))   # 둘레 띠까지 들어가는 상자만 잘라서 부풀림
        m8 = mask[r[1]:r[3] + 1, r[0]:r[2] + 1].astype(np.uint8)
        ring = cv2.dilate(m8, _K41) & ~cv2.dilate(m8, _K15)
        _gravity_refine(p, pts, up_cam, deproject(ring.astype(bool), depth_m, K, r))   # 퍽 둘레 7–20 px 띠 = 놓인 면 후보
    return p


UPRIGHT_MIN_H = 0.045             # 점 높이 범위가 이보다 크면 선 퍽 (선 퍽 6 cm, 누운 퍽 3 cm)
FLAT_MIN_ELONG = 1.5              # 수평 발자국 길쭉함(긴/짧은 표준편차 비)이 이보다 크면 누운 퍽
FLAT_MIN_LEN = 0.048              # 수평 발자국 긴 길이(PCA 긴 방향 3–97 %)가 이보다 길면 누운 퍽 (누운 퍽 60 mm, 선 퍽 · 윗면만 보이는 선 퍽 ≤ 35 mm).
                                  # 비율(FLAT_MIN_ELONG) 대신 씀: 실물에서 모서리 깊이가 뒤로 번진 점이 시선 방향으로 끌려 짧은 쪽이 부풀면
                                  # 누운 퍽 비율이 1.4 로 떨어져 선 퍽으로 오판함 (실물 팔 장착 촬영). 긴 길이는 55–59 mm 로 유지됨


TOP_WIN = 0.004                   # 윗면 높이 찾을 때 반복 창 (± m)


def _top_height(h: np.ndarray) -> float:
    """점 높이(위쪽 방향) -> 윗면 높이. 상위 5 % 에서 시작해 ±TOP_WIN 안 점의 중앙값으로 5 번 반복 = 윗면 점 덩어리의 가운데.
    상위 5 % 를 그대로 쓰면 깊이 잡음으로 위로 튄 점이 윗면이 돼 잡음 σ 의 약 1.7 배만큼 높게 나옴
    (시뮬: 잡음 0 / 2 / 4 mm 에서 p95 +1.3 / +3.4 / +5.6 mm, 이 방식 +1.1 / +0.9 / +1.0 mm)"""
    t = float(np.percentile(h, 95))
    for _ in range(5):
        s = h[np.abs(h - t) < TOP_WIN]
        if len(s) < 10:
            break
        t = float(np.median(s))
    return t


SUPPORT_TOL = 0.010               # 윗면 - 놓인 면 이 퍽 크기(60 / 30 mm)와 이만큼 넘게 다르면 놓인 면 안 믿음 (선반 끝에서 바닥을 잡는 경우 등)
SUPPORT_GAP = 0.010               # 놓인 면 후보 = 윗면보다 이만큼 이상 낮은 점 (옆 퍽 윗면 · 퍽 가장자리 번짐 뺌)


def _support_height(hr: np.ndarray, top_h: float) -> float | None:
    """퍽 둘레 띠 점 높이 -> 놓인 면(선반 · 책상) 높이. 윗면보다 SUPPORT_GAP 낮은 점 중 아래 20 % 에서 시작해 ±3 mm 중앙값 반복"""
    c = hr[hr < top_h - SUPPORT_GAP]
    if len(c) < 50:
        return None
    t = float(np.percentile(c, 20))
    for _ in range(5):
        s = c[np.abs(c - t) < 0.003]
        if len(s) < 20:
            return None
        t = float(np.median(s))
    return t


def _gravity_refine(p: PuckPose, pts: np.ndarray, up: np.ndarray, ring_pts: np.ndarray | None = None):
    """월드 위쪽을 알 때: 선/누움 판정 -> 긴축(태그 없을 때만) · 중심(항상) 보정.
    중심 = 수평 위치는 윗면 점(_top_face_points) 평균,
           높이는 놓인 면(퍽 둘레 띠, _support_height) + 선 퍽 H/2 · 누운 퍽 D/2. 놓인 면을 못 믿으면 윗면(_top_height)에서 그만큼 아래.
    놓인 면을 쓰는 이유 (정답 종이 실측): 퍽 윗면 깊이가 주변 면 대비 1.2–1.4 mm 더 멀게 나옴 (선 퍽 58.8 / 60, 누운 퍽 28.6 / 30,
    플라스틱 속으로 빛이 들어가는 탓으로 추정) -> 윗면 기준이면 중심이 그만큼 낮아짐. 놓인 면(종이 · 골판지)은 그 치우침이 없음"""
    up = up / np.linalg.norm(up)
    h = pts @ up
    top_h = _top_height(h)                                     # 윗면 높이 (깊이 잡음에 덜 흔들림)
    flat = pts - np.outer(h, up)                               # 수평면에 눌러 놓은 발자국
    q = flat - flat.mean(0)
    w, V = np.linalg.eigh(q.T @ q)
    elong = float(np.sqrt(max(w[2], 1e-12) / max(w[1], 1e-12)))
    # 선/누움: 높이 범위가 크면 선 퍽, 아니면 발자국 긴 길이 (누운 퍽 60 mm, 선 퍽 30 mm).
    # 높이만 보면 가려진 선 퍽(높이 범위 1–4 cm)을 누운 퍽으로 오판함 (시뮬 평가)
    pr = q @ V[:, 2]
    length = float(np.percentile(pr, 97) - np.percentile(pr, 3))
    upright = (top_h - np.percentile(h, 5)) > UPRIGHT_MIN_H or length < FLAT_MIN_LEN
    if upright:
        p.axis, p.axis_source = up.copy(), 'gravity_up'
    else:                                                      # 누운 퍽: 발자국 긴 방향
        p.axis, p.axis_source = V[:, 2], 'gravity_flat'
    p.upright = bool(upright)
    top = _top_face_points(pts, h, top_h, up)
    xy = top.mean(0) - up * (top.mean(0) @ up)                 # 윗면 점의 수평 위치
    p.top_center = xy + up * top_h
    half = (PUCK_H if upright else PUCK_D) / 2
    sup = None if ring_pts is None or len(ring_pts) == 0 else _support_height(ring_pts @ up, top_h)
    if sup is not None and abs(top_h - sup - 2 * half) < SUPPORT_TOL:
        p.center, p.height_source = xy + up * (sup + half), 'support'
    else:
        p.center, p.height_source = xy + up * (top_h - half), 'top'
    if upright:
        p.face_dir, p.face_size = _upright_face(top, up)
    else:
        p.face_dir, p.face_size = _lying_face(pts[h >= top_h - TOP_BAND], p.axis, up)
    p.axis = orient_axis(p.axis, up)


TOP_BAND = 0.008                  # 윗면 점 씨앗 = 윗면 높이에서 이만큼 아래까지 (옆면 점 섞임 방지)
PLANE_TOL = 0.004                 # 윗면 평면에서 이 거리 안 점 = 윗면 점
PLANE_MAX_TILT = 30.0             # 맞춘 윗면 법선이 위쪽과 이보다 벌어지면 (옆면을 잡은 것) 씨앗 그대로 씀 (deg)


def _top_face_points(pts: np.ndarray, h: np.ndarray, top_h: float, up: np.ndarray) -> np.ndarray:
    """윗면 점: 높이 띠(TOP_BAND)로 씨앗 -> 평면 맞춤 -> 평면 ±PLANE_TOL 안 점, 3 번 반복.
    높이(위쪽 방향) 기준으로만 고르면 윗면이 위쪽과 기울어 보일 때(실물 7–15°) 높은 쪽 띠만 골라
    중심이 7–10 mm 밀림 ('높이 상위 30 %' 로 고를 때). 평면 맞춤은 저장 장면에서 태그 중심과 2–4 px 안"""
    seed = pts[h >= top_h - TOP_BAND]
    if len(seed) < 30:
        return pts[h >= np.percentile(h, 70)]
    s = seed
    for _ in range(3):
        c = s.mean(0)
        q = s - c
        n = np.linalg.eigh(q.T @ q)[1][:, 0]                   # 가장 작은 고유벡터 = 평면 법선
        if abs(n @ up) < np.cos(np.radians(PLANE_MAX_TILT)):
            return seed
        nxt = pts[np.abs((pts - c) @ n) < PLANE_TOL]
        if len(nxt) < 30:
            return s
        s = nxt
    return s
FACE_SIDE = (0.020, 0.045)        # 윗면에 맞춘 사각형 한 변 허용 범위 (퍽 3 cm). 벗어나면 면 방향 모름
FACE_MAX_ASPECT = 1.4


def _upright_face(top: np.ndarray, up: np.ndarray):
    """선 퍽 윗면 점 → (면 방향 단위벡터 또는 None, (변1, 변2) m). 수평면에 눌러 최소 면적 사각형(cv2.minAreaRect).
    사각기둥이면 사각형 변 = 퍽 면 (90° 주기). 원기둥도 값은 나오지만 의미 없음 (호출하는 쪽에서 모양 보고 버림)"""
    if len(top) < 30:
        return None, ()
    e1 = np.cross(up, [1.0, 0.0, 0.0])
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(up, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    uv = np.stack([top @ e1, top @ e2], 1) * 1000.0           # mm
    rect = cv2.minAreaRect(uv.astype(np.float32))
    (_, _), (w, h), _ = rect
    corners = cv2.boxPoints(rect)
    edge = corners[1] - corners[0]
    size = (w / 1000.0, h / 1000.0)
    lo, hi = FACE_SIDE
    if not (lo <= min(size) and max(size) <= hi and max(size) / max(min(size), 1e-6) <= FACE_MAX_ASPECT):
        return None, size
    d = edge[0] * e1 + edge[1] * e2
    return d / np.linalg.norm(d), size


LYING_FACE_MIN_W = 0.022          # 누운 퍽 윗면 띠(TOP_BAND)의 폭(긴축 수직 수평 방향 3–97 %)이 이보다 넓으면 면이 위를 향함
                                  # 면이 위 = 약 30 mm, 모서리가 위(45° 기댐) = 약 16 mm (띠 8 mm × 2)


def _lying_face(band: np.ndarray, axis: np.ndarray, up: np.ndarray):
    """누운 퍽 윗면 띠 점 → (면 방향 = 위쪽 또는 None, (띠 폭 m,)). 누운 사각기둥은 한 면이 바닥에 닿아 면 하나가 위를 향함
    (시뮬 정답 3473 개 중 3471 개가 0.24° 안) → 면 법선 = 위쪽. 모서리로 기댄 경우는 띠가 좁아 None.
    원기둥도 값은 나오지만 의미 없음 (호출하는 쪽에서 모양 보고 버림)"""
    if len(band) < 30:
        return None, ()
    side = np.cross(up, axis)
    if np.linalg.norm(side) < 1e-6:
        return None, ()
    side /= np.linalg.norm(side)
    pr = band @ side
    w = float(np.percentile(pr, 97) - np.percentile(pr, 3))
    if w < LYING_FACE_MIN_W:
        return None, (w,)
    d = up - axis * (up @ axis)
    return d / np.linalg.norm(d), (w,)


def _tag_is_top(pts: np.ndarray, n: np.ndarray, up: np.ndarray) -> bool:
    """윗면(아랫면) 태그인지 옆면 태그인지: 퍽 점군이 어느 쪽으로 길게 퍼졌는지 (5–95 % 범위).
    윗면 태그면 긴축(= 법선)으로 옆면이 보여 최대 6 cm, 태그 위쪽 방향은 폭 3 cm. 옆면 태그면 반대.
    영상 마스크 모양 · PCA 축으로 정하면 직육면체를 위에서 볼 때 틀렸음 (시뮬 평가)"""
    q = pts - pts.mean(0)
    rng_ = lambda d: float(np.subtract(*np.percentile(q @ d, [95, 5])))
    return rng_(n) > rng_(up)


def estimate_up_from_plane(depth_m: np.ndarray, K: np.ndarray, exclude: np.ndarray | None = None, step: int = 4,
                           iters: int = 200, inlier_m: float = 0.006, rng: np.random.Generator | None = None, max_tilt_deg: float = 80.0):
    """깊이에서 가장 큰 평면(책상 · 선반 윗면)을 RANSAC 으로 찾아 그 법선을 월드 위쪽으로 봄 -> (up_cam, 평면 점 비율) 또는 (None, 0).
    D435(i 아님)는 IMU 가 없어 중력을 직접 못 읽음. 퍽 마스크(exclude)는 빼고, 법선은 카메라 쪽(점에서 카메라를 향함)으로.
    가정: 가장 큰 평면이 수평. 벽 · 선반 옆면을 크게 보면 틀림 -> 법선이 영상 위쪽(-y) 과 max_tilt_deg(80°) 넘게 벌어지면 버림."""
    rng = np.random.default_rng(0) if rng is None else rng
    ds = depth_m[::step, ::step]                                   # step 간격 화소만 (전체 화면 마스크를 만들지 않음, 점 순서 · 값은 같음)
    m = (ds > DEPTH_RANGE[0]) & (ds < DEPTH_RANGE[1])
    if exclude is not None:
        m &= ~exclude[::step, ::step]
    v, u = np.nonzero(m)
    z = ds[v, u] - DEPTH_BIAS_M
    u, v = u * step, v * step
    pts = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], -1)
    if len(pts) < 200:
        return None, 0.0
    best_n, best_cnt = None, 0
    for _ in range(iters):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        cnt = int((np.abs((pts - a) @ n) < inlier_m).sum())
        if cnt > best_cnt:
            best_n, best_cnt, best_a = n, cnt, a
    if best_n is None:
        return None, 0.0
    inl = pts[np.abs((pts - best_a) @ best_n) < inlier_m]          # 인라이어로 다시 맞춤
    q = inl - inl.mean(0)
    n = np.linalg.eigh(q.T @ q)[1][:, 0]
    if n @ (-inl.mean(0)) < 0:                                     # 카메라 쪽 = 위쪽
        n = -n
    if n @ np.array([0.0, -1.0, 0.0]) < np.cos(np.radians(max_tilt_deg)):   # 영상 위쪽과 너무 다름 = 벽 같은 면
        # 60° 였을 때 팔 관찰 자세(아래 58–60°)에서 선반 평면을 버림 -> 80° (관찰 자세 범위 35–80°)
        return None, best_cnt / len(pts)
    return n, best_cnt / len(pts)


def up_from_pitch_roll(pitch_deg: float, roll_deg: float = 0.0) -> np.ndarray:
    """카메라를 아래로 pitch, 시계 방향 roll 만큼 기울였을 때 카메라 광학 좌표(x 오른쪽 · y 아래 · z 앞)의 월드 위쪽."""
    p, r = np.radians(pitch_deg), np.radians(roll_deg)
    up = np.array([0.0, -np.cos(p), -np.sin(p)])                  # 수평일 때 (0, -1, 0), 아래로 숙이면 앞(z)쪽 성분이 음수
    c, s_ = np.cos(r), np.sin(r)
    return np.array([c * up[0] - s_ * up[1], s_ * up[0] + c * up[1], up[2]])


def estimate(bgr: np.ndarray, depth_m: np.ndarray, K: np.ndarray, dist: np.ndarray | None = None, up_cam: np.ndarray | None = None,
             seg: Segmenter | None = None, masks: list | None = None):
    """한 장 처리 -> (퍽 자세 목록, 태그 목록). up_cam: 카메라 좌표의 월드 위쪽 (없으면 중력 보정 안 함).
    masks: find_masks 결과를 이미 구했으면 넘김 (안 주면 seg 또는 HSV 로 구함)."""
    tags = detect_tags(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), K, dist)
    pucks = []
    for lab, m, kind, conf in (find_masks(bgr, seg) if masks is None else masks):
        pp = puck_pose(lab, m, depth_m, K, tags, up_cam)
        if pp is not None:
            pp.kind, pp.conf = kind, conf
            pucks.append(pp)
    return pucks, tags


def project(K: np.ndarray, P: np.ndarray) -> tuple[int, int]:
    return int(K[0, 0] * P[0] / P[2] + K[0, 2]), int(K[1, 1] * P[1] / P[2] + K[1, 2])


# ---------- 화면 문구 (표시만, axis_source 값 자체는 그대로 = json · 평가 코드가 씀) ----------
# axis_source -> 화면에 보일 "무엇으로 정했나"
_HOW = {'gravity_up': 'by gravity', 'gravity_flat': 'by gravity', 'tag_normal': 'by top tag', 'tag_up': 'by side tag', 'pca': 'by shape only'}
_UP_LABEL = {'plane': 'table plane', 'fixed': 'fixed angle', 'tf': 'robot TF'}


def pose_text(p: 'PuckPose', up_cam: np.ndarray | None) -> tuple[str, str]:
    """퍽 옆 두 줄. 1줄 = 클래스 · 신뢰도 · 거리, 2줄 = STANDING / LYING · 무엇으로 정했나 (태그 · 점군이면 기울기도)."""
    name = f'{p.kind} {p.conf:.2f}' if p.kind else p.label
    line1 = f'{name}  {np.linalg.norm(p.center):.2f} m'
    how = _HOW.get(p.axis_source, p.axis_source)
    if p.truncated:
        return line1, 'CUT at edge - pose not trusted'
    if up_cam is None:                                       # 위쪽 모름 -> 선/누움 판단 불가
        return line1, f'STANDING/LYING unknown (no up) - {how}'
    tilt = float(np.degrees(np.arccos(np.clip(abs(p.axis @ up_cam), 0, 1))))   # 긴축과 월드 위쪽 사이 각
    state = 'STANDING' if tilt < 45 else 'LYING'
    if p.axis_source.startswith('gravity'):                  # 중력 판정은 긴축을 위쪽 / 수평으로 딱 맞추므로 기울기는 늘 0° · 90° -> 안 씀
        return line1, f'{state} ({how})'
    return line1, f'{state} ({how}, tilt {tilt:.0f} deg)'


def put_texts(img: np.ndarray, items: list) -> np.ndarray:
    """[(문구, (x, y 왼쪽 위), BGR, 높이 px)] 를 그림. 검은 테두리(밝은 배경에서도 보임), 오른쪽 끝에서 안 잘리게 안쪽으로."""
    W = img.shape[1]
    for text, (x, y), c, size in items:
        sc = size / 33
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, sc, 1)
        org = (max(0, min(int(x), W - tw - 4)), int(y) + th)
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, sc, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, sc, c, 1, cv2.LINE_AA)
    return img


def draw(bgr: np.ndarray, depth_m: np.ndarray, K: np.ndarray, pucks, tags, up_cam: np.ndarray | None = None, up_label: str = '') -> np.ndarray:
    """컬러 위 시각화 + 깊이 색지도 나란히."""
    vis = bgr.copy()
    texts = []
    for t in tags:                                                 # 태그는 윤곽만 (축 · 거리는 퍽 중심 · 긴축을 가려서 뺌)
        cv2.polylines(vis, [t.corners.astype(int)], True, (255, 0, 255), 1)
    col = {'green': (0, 255, 0), 'red': (0, 0, 255)}
    for p in pucks:
        c = col.get(p.label, (255, 255, 0))
        if p.bbox:                                                 # 퍽 상자(+1 px) 안에서만 윤곽 (화면 전체 복사 안 함)
            x0, y0 = max(0, p.bbox[0] - 1), max(0, p.bbox[1] - 1)
            sub = p.mask[y0:p.bbox[3] + 2, x0:p.bbox[2] + 2].astype(np.uint8)
            cnts, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, offset=(x0, y0))
        else:
            cnts, _ = cv2.findContours(p.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, cnts, -1, c, 1)
        a, b = p.center - p.axis * PUCK_H / 2, p.center + p.axis * PUCK_H / 2   # 긴축 양 끝 면 중심 (부호에 뜻 없음 -> 화살촉 안 그림)
        if a[2] > 0 and b[2] > 0:
            cv2.line(vis, project(K, a), project(K, b), (0, 255, 255), 2)
            for e in (a, b):
                cv2.circle(vis, project(K, e), 3, (0, 255, 255), -1)
        cv2.circle(vis, project(K, p.center), 5, (0, 0, 0), -1)   # 속 중심 = 검은 테두리 노란 점
        cv2.circle(vis, project(K, p.center), 3, (0, 255, 255), -1)
        if p.upright and p.top_center is not None:            # 선 퍽 윗면 중심 = 흰 십자
            u, v = project(K, p.top_center)
            cv2.drawMarker(vis, (u, v), (255, 255, 255), cv2.MARKER_CROSS, 12, 2)
            if 'box' in p.kind and p.face_dir is not None:      # 선 사각기둥 윗면 = 추정 면 방향으로 돌린 30 mm 정사각형 (흰색)
                f = p.face_dir
                g = np.cross(p.axis, f)
                sq = [p.top_center + (su * f + sv * g) * PUCK_D / 2 for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
                cv2.polylines(vis, [np.array([project(K, q) for q in sq], np.int32)], True, (255, 255, 255), 1)
        x0, y0 = p.bbox[:2]
        l1, l2 = pose_text(p, up_cam)
        texts += [(l1, (x0, max(0, y0 - 34)), c, 14), (l2, (x0, max(16, y0 - 18)), (0, 0, 255) if p.truncated else c, 14)]
    if up_cam is not None:                                         # 위쪽: 영상 가운데에서 카메라 앞 0.3 m 지점 기준 5 cm 화살표
        o = np.array([0.0, 0.0, 0.3])
        cv2.arrowedLine(vis, project(K, o), project(K, o + up_cam * 0.05), (255, 255, 255), 2, tipLength=0.25)
        tilt = np.degrees(np.arccos(np.clip(up_cam @ np.array([0, -1.0, 0]), -1, 1)))
        src = _UP_LABEL.get(up_label, up_label)
        texts.append((f'white arrow = up ({src}), {tilt:.0f} deg from image-up', (8, vis.shape[0] - 22), (255, 255, 255), 14))
    vis = put_texts(vis, texts)
    dvis = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(depth_m, 0, 1.2), alpha=255 / 1.2), cv2.COLORMAP_TURBO)
    dvis[depth_m <= 0] = 0
    return np.hstack([vis, dvis])
