"""YOLO 밖에서 하는 후처리 — ROS 없이 테스트할 수 있게 여기 모음.

- 퍽 색: bbox 중앙 영역 HSV 중앙값 (가이드 2절)
- 신호등: bbox 를 세로로 나눠 칸별 밝기 비교 (가이드 2절)
- AprilTag: OpenCV aruco 의 36h11 사전 (OpenCV 4.6 / 4.7+ API 모두)
- 1D 바코드: OpenCV barcode 모듈 + 전처리 단계별 재시도 (가이드 7.4)
- 다수결: 최근 N 프레임 중 M 번 (가이드 7.3)
"""
from collections import Counter, deque

import cv2
import numpy as np


# ---------------------------------------------------------------- 퍽 색

def center_roi(img, box, frac):
    """bbox (x1, y1, x2, y2) 의 가운데 frac 비율만 잘라 냄. 가장자리 배경 · 그림자 제외용."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    mx, my = w * (1 - frac) / 2, h * (1 - frac) / 2
    xa, ya = int(max(0, x1 + mx)), int(max(0, y1 + my))
    xb, yb = int(min(img.shape[1], x2 - mx)), int(min(img.shape[0], y2 - my))
    if xb <= xa or yb <= ya:
        return None
    return img[ya:yb, xa:xb]


def classify_color(roi_bgr, cfg):
    """퍽 색 → ("red" | "green" | "beige" | "unknown", 유효 픽셀 수).

    cfg 키 (config/perception.yaml 의 color.*):
      v_min          너무 어두운 픽셀(그림자) 제외
      min_pixels     유효 픽셀이 이보다 적으면 unknown
      s_beige_max    채도 중앙값이 이보다 낮으면 무채색 계열 → beige 후보
      beige_hue      [lo, hi]  beige 로 볼 hue 범위 (OpenCV 0~179)
      red_hue        [lo, hi]  빨강. 0 을 넘나들어서 lo > hi 이면 감싸는 범위 (예: [170, 10])
      green_hue      [lo, hi]
    """
    if roi_bgr is None or roi_bgr.size == 0:
        return "unknown", 0
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    px = hsv[hsv[:, 2] > cfg["v_min"]]
    if len(px) < cfg["min_pixels"]:
        return "unknown", int(len(px))

    s_med = float(np.median(px[:, 1]))
    if s_med < cfg["s_beige_max"]:
        h = float(np.median(px[:, 0]))
        return ("beige" if _in_hue(h, cfg["beige_hue"]) else "unknown"), int(len(px))

    colored = px[px[:, 1] >= cfg["s_beige_max"]]
    h = _hue_median(colored[:, 0])
    for name in ("red", "green"):
        if _in_hue(h, cfg[name + "_hue"]):
            return name, int(len(px))
    return "unknown", int(len(px))


def _in_hue(h, rng):
    lo, hi = rng
    return lo <= h <= hi if lo <= hi else (h >= lo or h <= hi)


def _hue_median(hues):
    """빨강은 0/179 경계에 걸쳐 있어 그냥 중앙값을 내면 90 근처가 나옴 → 원형 평균 대신 90 만큼 돌려서 비교."""
    h = hues.astype(np.float32)
    shifted = (h + 90) % 180
    if np.std(shifted) < np.std(h):          # 경계에 걸친 분포면 돌린 쪽이 더 모여 있음
        return float((np.median(shifted) - 90) % 180)
    return float(np.median(h))


# ---------------------------------------------------------------- 신호등

def signal_state(roi_bgr, tiers, v_margin):
    """타워램프 bbox → ("red" | "orange" | "green" | "off", 칸별 점수).

    tiers: {"red": [a, b], ...} — bbox 높이 대비 위→아래 비율. 렌즈가 꺼져도 색이 보이므로 색이 아니라 밝기로 판단.
    """
    if roi_bgr is None or roi_bgr.size == 0:
        return "off", {}
    h, w = roi_bgr.shape[:2]
    v = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)[..., 2]
    cx0, cx1 = int(w * 0.25), max(int(w * 0.75), int(w * 0.25) + 1)   # 가장자리 반사 제외
    score = {}
    for name, (a, b) in tiers.items():
        band = v[int(a * h):max(int(b * h), int(a * h) + 1), cx0:cx1]
        score[name] = float(np.percentile(band, 90)) if band.size else 0.0
    lit = max(score, key=score.get)
    others = [s for k, s in score.items() if k != lit]
    if not others or score[lit] - max(others) > v_margin:
        return lit, score
    return "off", score


# ---------------------------------------------------------------- 다수결

class Voter:
    """최근 window 개 중 같은 값이 need 번 이상이면 그 값을 확정."""

    def __init__(self, window, need):
        self.buf = deque(maxlen=window)
        self.need = need

    def push(self, value):
        self.buf.append(value)
        return self.result()

    def result(self):
        """(확정값 또는 None, 그 값의 비율)"""
        if not self.buf:
            return None, 0.0
        value, n = Counter(self.buf).most_common(1)[0]
        return (value if n >= self.need else None), n / len(self.buf)

    def clear(self):
        self.buf.clear()


# ---------------------------------------------------------------- AprilTag

_TAG_DICTS = {
    "36h11": "DICT_APRILTAG_36h11",
    "25h9": "DICT_APRILTAG_25h9",
    "16h5": "DICT_APRILTAG_16h5",
}


class TagReader:
    """퍽 bbox 안의 AprilTag ID 와 (가능하면) 카메라 기준 자세."""

    def __init__(self, family="36h11", min_side_px=60):
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, _TAG_DICTS[family]))
        self.min_side = min_side_px
        if hasattr(cv2.aruco, "ArucoDetector"):                       # OpenCV 4.7+
            self._det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
            self._detect = lambda g: self._det.detectMarkers(g)[:2]
        else:                                                          # OpenCV 4.6 (Jetson 기본)
            params = cv2.aruco.DetectorParameters_create()
            self._detect = lambda g: cv2.aruco.detectMarkers(g, d, parameters=params)[:2]

    def read(self, img_bgr, box, pad=0.15):
        """→ (tag_id, 원본 이미지 좌표의 코너 4개 (4,2)) 또는 (-1, None). 여러 개면 가장 큰 것."""
        x1, y1, x2, y2 = box
        pw, ph = (x2 - x1) * pad, (y2 - y1) * pad
        xa, ya = int(max(0, x1 - pw)), int(max(0, y1 - ph))
        xb, yb = int(min(img_bgr.shape[1], x2 + pw)), int(min(img_bgr.shape[0], y2 + ph))
        if xb - xa < 4 or yb - ya < 4:
            return -1, None
        gray = cv2.cvtColor(img_bgr[ya:yb, xa:xb], cv2.COLOR_BGR2GRAY)
        scale = max(1.0, self.min_side / min(gray.shape))              # 작은 퍽은 키워서 검출
        if scale > 1.0:
            gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        corners, ids = self._detect(gray)
        if ids is None or len(ids) == 0:
            return -1, None
        areas = [cv2.contourArea(c.reshape(4, 2)) for c in corners]
        k = int(np.argmax(areas))
        pts = corners[k].reshape(4, 2) / scale + np.array([xa, ya], dtype=np.float32)
        return int(ids[k][0]), pts.astype(np.float32)


def tag_pose(corners, tag_size, K, D, fisheye):
    """태그 코너 → 카메라 기준 (t[3], R[3x3]). 실패하면 None. 태그 크기는 검은 테두리 바깥 한 변 (m)."""
    s = tag_size / 2.0
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
    img = corners.reshape(-1, 1, 2).astype(np.float64)
    if fisheye:                                   # fisheye 는 먼저 왜곡을 풀고 왜곡 없는 pinhole 로 PnP
        img = cv2.fisheye.undistortPoints(img, K, D, P=K)
        D = np.zeros(4)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return tvec.reshape(3), R


def depth_point(depth_m, box, K, frac=0.4):
    """정렬된 depth (m) 의 bbox 중앙 영역 → 카메라 기준 3D 점. 앞쪽(가까운) 25% 분위수를 써서 배경을 피함."""
    roi = center_roi(depth_m, box, frac)
    if roi is None:
        return None
    z = roi[np.isfinite(roi) & (roi > 0.05)]
    if z.size < 10:
        return None
    zq = float(np.percentile(z, 25))
    u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return np.array([(u - cx) * zq / fx, (v - cy) * zq / fy, zq])


def rot_to_quat(R):
    """3x3 회전행렬 → (x, y, z, w)."""
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return float(x), float(y), float(z), float(w)


# ---------------------------------------------------------------- 1D 바코드

class BarcodeReader:
    """OpenCV barcode 모듈 (EAN-8/13, UPC-A/E). 원본 → CLAHE → 샤프닝 → 이진화 순서로, 하나라도 읽히면 채택."""

    def __init__(self):
        self._det = cv2.barcode.BarcodeDetector()
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def _decode(self, img):
        if hasattr(self._det, "detectAndDecodeWithType"):              # OpenCV 4.8+
            ok, infos, types, _ = self._det.detectAndDecodeWithType(img)
        else:                                                          # OpenCV 4.6~4.7
            ok, infos, types, _ = self._det.detectAndDecode(img)
        if not ok or infos is None:
            return []
        types = list(types) if types is not None else [""] * len(infos)
        return [(t, _type_name(ty)) for t, ty in zip(infos, types) if t]

    def read(self, img_bgr, scales=(1.0, 0.5)):
        """→ [(text, symbology), ...] (첫 번째로 성공한 전처리 결과)

        OpenCV 검출기는 바코드가 화면을 크게 채우면 오히려 못 찾음 (가까이 정렬했을 때) → 줄여서도 시도.
        """
        gray0 = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY) if img_bgr.ndim == 3 else img_bgr
        for s in scales:
            gray = gray0 if s == 1.0 else cv2.resize(gray0, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            for f in (lambda g: g,
                      self._clahe.apply,
                      lambda g: cv2.addWeighted(g, 1.5, cv2.GaussianBlur(g, (0, 0), 3), -0.5, 0),
                      lambda g: cv2.adaptiveThreshold(g, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                      cv2.THRESH_BINARY, 31, 5)):
                found = self._decode(cv2.cvtColor(f(gray), cv2.COLOR_GRAY2BGR))
                if found:
                    return found
        return []


def _type_name(ty):
    if isinstance(ty, str):
        return ty
    # OpenCV 4.6 은 숫자 코드로 돌려줌 (cv::barcode::BarcodeType)
    return {1: "EAN_8", 2: "EAN_13", 3: "UPC_A", 4: "UPC_E", 5: "UPC_EAN_EXTENSION"}.get(int(ty), "")
