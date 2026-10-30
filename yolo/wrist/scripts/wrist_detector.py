"""손목 D435 퍽 검출 파이프라인: 손목 YOLO-seg → 자세(wrist_pose) → 거르기 → 여러 프레임 확인 → planning Detection 형식.

ROS 없음 (numpy · OpenCV · ultralytics). d435_puck_viewer.py · eval_detector.py 가 씀. 로봇 노드는 따로 관리하는 C++ 구현 (ros2_ws/src/perception wrist_puck_node)

한 장 처리 순서 (WristPuckDetector.process)
  1) YOLO-seg 마스크 (conf ≥ SEG_CONF, 기본 0.5 — experiments/train_seg8.ipynb 7절 기준값표 참고)
  2) wrist_pose.puck_pose: 마스크 + 정렬 깊이 → 중심 · 긴축, AprilTag 로 긴축 보정, 위쪽(up_cam) 있으면 중력 보정
  3) 거르기 (이번 장에서 버림, 이유는 RawPuck.reject)
       truncated : 화면 끝에 잘림 → 자세 오차 큼 (시뮬: 중심 90% 24 mm, 긴축 90% 90°). 다시 볼 것
       size      : 점군 퍼짐이 퍽보다 큼 (긴 방향 > 8.5 cm 또는 두 번째 > 5 cm) → 배경 · 반사 · 두 퍽 붙은 마스크
       points    : 유효 깊이 점 < 50 (너무 가깝거나 멀어 깊이 없음)
  4) 여러 프레임 확인: 카메라 좌표 중심이 2 cm 안이면 같은 퍽으로 묶음. 최근 window(5) 장 중 min_hits(3) 장 이상 보이면 확정
       확정 퍽 값 = 최근 관측 묶음: 중심 중앙값, 긴축 평균(부호 맞춤), 클래스 다수결(신뢰도 가중), 태그 ID 최빈값
     카메라가 움직이면 같은 퍽도 카메라 좌표가 바뀜 → 손목이 멈춘(LOOK) 상태에서 쓸 것. 움직이면 reset() 할 것
출력 (PuckDetection) = interfaces/Detection.msg 필드와 1:1
  kind "puck", label "red"/"green", shape "cylinder"/"box", id = AprilTag ID 문자열(못 읽으면 ""), confidence,
  position (카메라 광학 프레임 m), orientation 쿼터니언 xyzw (z 축 = 긴축), bbox [x0, y0, x1, y1]
  orientation x 축: 사각기둥 면 법선. 선 퍽 = 윗면 윤곽으로 구한 면 방향(wrist_pose._upright_face, 태그 안 씀),
                    누운 퍽 = 위쪽 (wrist_pose._lying_face, 면이 바닥에 닿음. 모서리로 기대 있으면 못 구함),
                    못 구했으면(또는 위쪽 모름 · 원기둥) 카메라 x 축을 긴축에 수직으로 눌러 놓은 방향
  → yaw_known = 사각기둥 면 방향을 구했는지 (선 · 누움 둘 다). 원기둥은 늘 False
     planar_pose 의 θ 는 누운 퍽이면 긴축 방향 (면 방향은 위쪽이라 θ 에 안 들어감)
  태그는 ID 읽기에만 씀 (위쪽을 알 때). 실물 태그는 손으로 붙여 면과 나란하다는 보장이 없음
로봇 프레임 집기 자세 (planar_pose): 촬영 시각 TF(base_link ← d435_color_optical_frame = 컬러 렌즈) 로 변환 → x, y, z, θ + 선/누움
  퍽은 수평면 위라 기울기는 0° 또는 90° 둘 중 하나 → 4 값이면 충분. θ 는 카메라 좌표가 아니라 로봇 프레임에서 정의
"""
from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field

import numpy as np

import wrist_pose as wp

SEG_CONF = 0.5
MAX_LONG_SPREAD = 0.085     # 점군 긴 방향 5–95 % 범위 (m). 직육면체 대각 7.3 cm + 깊이 잡음 여유 (시뮬 정답 최대 5.9 cm)
MAX_MID_SPREAD = 0.050      # 두 번째 방향 (시뮬 정답 최대 3.7 cm)
MIN_POINTS = 50
SHAPE_NAME = {'cyl': 'cylinder', 'box': 'box'}


def parse_kind(kind: str, fallback_color: str = '') -> tuple[str, str, bool | None]:
    """seg 클래스 이름 → (색, 모양 'cyl'/'box'/'', 누움 여부 또는 None).
    4클래스 'red_cyl', 8클래스 'red_cyl_up' · 'red_cyl_lying' (sim_to_seg.py --classes) 둘 다 받음. HSV 분할이면 kind = ''"""
    if not kind:
        return fallback_color, '', None
    parts = kind.split('_')
    lying = None
    if len(parts) >= 3:
        lying = parts[2] == 'lying'
    return parts[0], parts[1] if len(parts) > 1 else '', lying


@dataclass
class RawPuck:
    """한 장에서 나온 퍽 1개 (거르기 전). reject = '' 이면 통과"""
    pose: wp.PuckPose
    color: str                  # 'red' / 'green'
    shape: str                  # 'cylinder' / 'box' / '' (HSV 분할이면 모름)
    spread: tuple               # 점군 퍼짐 (긴, 두 번째, 세 번째) m
    reject: str = ''
    cls_lying: bool | None = None   # 8클래스 seg 가 본 선/누움 (4클래스 · HSV 면 None). 자세 계산은 중력 + 점군 판정을 씀, 이건 확인용


@dataclass
class PuckDetection:
    """planning 에 보낼 퍽 1개 (interfaces/Detection.msg 필드)"""
    label: str
    shape: str
    id: str
    confidence: float
    position: np.ndarray        # (3,) 카메라 광학 프레임 m
    orientation: np.ndarray     # (4,) xyzw, z 축 = 긴축
    bbox: tuple                 # (x0, y0, x1, y1) 픽셀
    yaw_known: bool             # 긴축 둘레 회전(사각기둥 면 방향)을 아는지
    hits: int                   # 최근 window 장 중 보인 장 수
    axis_source: str
    kind: str = 'puck'

    def to_dict(self) -> dict:
        """ROS 없이 저장 · 확인용 (msg 필드 이름 그대로)"""
        return {'kind': self.kind, 'label': self.label, 'shape': self.shape, 'id': self.id,
                'confidence': round(self.confidence, 3),
                'pose': {'position': self.position.round(4).tolist(), 'orientation_xyzw': self.orientation.round(4).tolist()},
                'bbox': list(self.bbox), 'yaw_known': self.yaw_known, 'hits': self.hits, 'axis_source': self.axis_source}


def point_spread(pose: wp.PuckPose, depth_m: np.ndarray, K: np.ndarray) -> tuple:
    """마스크 점군의 주축별 5–95 % 범위 (m, 큰 순). puck_pose 와 같은 튀는 점 제거"""
    if pose.points is not None:                     # puck_pose 가 이미 같은 방법으로 뽑은 점군
        pts = pose.points
    else:
        pts = wp.deproject(pose.mask, depth_m, K, pose.bbox)
        pts = pts[np.linalg.norm(pts - np.median(pts, 0), axis=1) < 0.06]
    if len(pts) < 3:
        return (0.0, 0.0, 0.0)
    q = pts - pts.mean(0)
    V = np.linalg.eigh(q.T @ q)[1][:, ::-1]
    return tuple(float(np.subtract(*np.percentile(q @ V[:, i], [95, 5]))) for i in range(3))


def axis_frame(axis: np.ndarray, face_dir: np.ndarray | None) -> tuple[np.ndarray, bool]:
    """긴축 = z 인 회전행렬 (열 = x, y, z)과 x 축을 면 방향으로 정했는지"""
    z = axis / np.linalg.norm(axis)
    yaw_known = face_dir is not None
    ref = face_dir if face_dir is not None else np.array([1.0, 0.0, 0.0])
    x = ref - z * (ref @ z)
    if np.linalg.norm(x) < 1e-6:                     # ref 가 긴축과 나란하면 다른 기준
        ref = np.array([0.0, 1.0, 0.0])
        x = ref - z * (ref @ z)
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], 1), yaw_known


def quat_xyzw(R: np.ndarray) -> np.ndarray:
    """회전행렬 → 쿼터니언 (x, y, z, w), w ≥ 0"""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q if q[3] >= 0 else -q


@dataclass
class _Track:
    obs: deque = field(default_factory=lambda: deque(maxlen=16))   # (frame, RawPuck)
    last_frame: int = -1


class WristPuckDetector:
    """seg 가중치 → 한 장씩 process() → (확정 퍽 목록, 이번 장 원시 퍽 목록, 태그 목록)"""

    def __init__(self, weights: str | None, conf: float = SEG_CONF, imgsz: int = 640, window: int = 5, min_hits: int = 3,
                 match_m: float = 0.02, device=None):
        self.seg = wp.Segmenter(weights, conf=conf, imgsz=imgsz, device=device) if weights else None   # None = HSV (모양 모름)
        self.window, self.min_hits, self.match_m = window, min_hits, match_m
        self.reset()

    def reset(self):
        """카메라(팔)가 움직인 뒤 호출 — 카메라 좌표 기준 묶음을 지움"""
        self.frame, self.tracks = -1, []

    # -- 한 장 ---------------------------------------------------------------
    def observe(self, bgr: np.ndarray, depth_m: np.ndarray, K: np.ndarray, up_cam: np.ndarray | None = None,
                dist: np.ndarray | None = None, masks: list | None = None):
        """묶음 없이 이번 장만: (원시 퍽 목록, 태그 목록)"""
        pucks, tags = wp.estimate(bgr, depth_m, K, dist, up_cam, seg=self.seg, masks=masks)
        raws = []
        for p in pucks:
            color, shape, cls_lying = parse_kind(p.kind, p.label)
            r = RawPuck(p, color, SHAPE_NAME.get(shape, ''), point_spread(p, depth_m, K), cls_lying=cls_lying)
            if p.truncated:
                r.reject = 'truncated'
            elif p.n_points < MIN_POINTS:
                r.reject = 'points'
            elif r.spread[0] > MAX_LONG_SPREAD or r.spread[1] > MAX_MID_SPREAD:
                r.reject = 'size'
            raws.append(r)
        return raws, tags

    def process(self, bgr: np.ndarray, depth_m: np.ndarray, K: np.ndarray, up_cam: np.ndarray | None = None,
                dist: np.ndarray | None = None, masks: list | None = None):
        """이번 장 처리 + 묶음 갱신 → (확정 PuckDetection 목록 (이번 장에 보인 것만), 원시 퍽, 태그)"""
        self.frame += 1
        raws, tags = self.observe(bgr, depth_m, K, up_cam, dist, masks)
        good = [r for r in raws if not r.reject]
        # 가까운 것부터 일대일로 기존 묶음에 붙임
        pairs = sorted(((np.linalg.norm(r.pose.center - t.obs[-1][1].pose.center), i, j)
                        for i, r in enumerate(good) for j, t in enumerate(self.tracks)), key=lambda x: x[0])
        used_r, used_t = set(), set()
        for d, i, j in pairs:
            if d > self.match_m or i in used_r or j in used_t:
                continue
            prev = self.tracks[j].obs[-1][1].pose
            if not good[i].pose.upright and good[i].pose.axis @ prev.axis < 0:   # 누운 퍽 · 위쪽 모름: 이전 장 부호에 맞춤 (화살표 안 뒤집히게)
                good[i].pose.axis = -good[i].pose.axis
            self.tracks[j].obs.append((self.frame, good[i]))
            self.tracks[j].last_frame = self.frame
            used_r.add(i)
            used_t.add(j)
        for i, r in enumerate(good):
            if i not in used_r:
                t = _Track()
                t.obs.append((self.frame, r))
                t.last_frame = self.frame
                self.tracks.append(t)
        self.tracks = [t for t in self.tracks if self.frame - t.last_frame < self.window]   # window 동안 안 보이면 버림
        out = []
        for t in self.tracks:
            recent = [r for f, r in t.obs if self.frame - f < self.window]
            if t.last_frame == self.frame and len(recent) >= self.min_hits:
                out.append(self._summarize(recent))
        return out, raws, tags

    # -- 묶음 → 확정 값 --------------------------------------------------------
    def _summarize(self, recent: list[RawPuck]) -> PuckDetection:
        # 선/누움 다수결: 장마다 판정이 갈리면(기준값 근처) 위쪽 · 수평 긴축을 섞어 평균해 30–45° 엉뚱한 축이 나왔음
        #   -> 많은 쪽 장만 남겨 중심 · 긴축 · 면 방향을 계산. 동점이면 최근 장의 판정. 위쪽 모름(upright None)은 그대로 다 씀
        known = [r for r in recent if r.pose.upright is not None]
        if known:
            n_up = sum(1 for r in known if r.pose.upright)
            n_flat = len(known) - n_up
            upright = n_up > n_flat if n_up != n_flat else bool(known[-1].pose.upright)
            group = [r for r in known if bool(r.pose.upright) == upright]
        else:
            group = recent
        last = group[-1]
        center = np.median(np.stack([r.pose.center for r in group]), 0)
        ref = last.pose.axis
        axis = np.sum([r.pose.axis * (1 if r.pose.axis @ ref >= 0 else -1) for r in group], 0)
        axis /= np.linalg.norm(axis)
        votes = Counter()
        for r in recent:
            votes[(r.color, r.shape)] += r.pose.conf
        color, shape = votes.most_common(1)[0][0]
        ids = Counter(r.pose.tag.tag_id for r in recent if r.pose.tag is not None)
        face = None
        if shape == 'box':                                     # 사각기둥 면 방향 (90° 주기 평균: 4θ 각으로 평균)
            faces = [r.pose.face_dir for r in group if r.pose.face_dir is not None]   # group = 선/누움 같은 장만 (선: 윗면 윤곽, 누움: 위쪽)
            if faces:
                ref = faces[-1]
                ref2 = np.cross(axis, ref)
                ang = np.mean([np.exp(4j * np.arctan2(f @ ref2, f @ ref)) for f in faces])
                a4 = np.angle(ang) / 4
                face = np.cos(a4) * ref + np.sin(a4) * ref2
        R, yaw_known = axis_frame(axis, face)
        return PuckDetection(label=color, shape=shape, id=str(ids.most_common(1)[0][0]) if ids else '',
                             confidence=float(np.mean([r.pose.conf for r in recent])), position=center,
                             orientation=quat_xyzw(R), bbox=last.pose.bbox, yaw_known=yaw_known, hits=len(recent),
                             axis_source=last.pose.axis_source)


UPRIGHT_COS = np.cos(np.radians(45))   # 긴축과 위쪽 사이가 45° 안이면 선 퍽


@dataclass
class PlanarPose:
    """로봇(base_link 등 z 위쪽인 프레임) 기준 집기용 자세. 퍽은 수평면 위라 x, y, z, θ + 선/누움이면 다 정해짐"""
    x: float
    y: float
    z: float                    # 퍽 중심 (m)
    theta: float                # rad. 선 사각기둥: 면 방향 [-π/4, π/4) (90° 주기), 누운 퍽: 긴축 방향 [-π/2, π/2) (180° 주기)
    upright: bool
    theta_known: bool           # False = 선 원기둥(θ 필요 없음) · 윗면 윤곽으로 면 방향을 못 구한 선 사각기둥

    def to_dict(self) -> dict:
        return {'x': round(self.x, 4), 'y': round(self.y, 4), 'z': round(self.z, 4), 'theta_deg': round(float(np.degrees(self.theta)), 1),
                'upright': self.upright, 'theta_known': self.theta_known}


def _wrap(a: float, period: float) -> float:
    """a 를 [-period/2, period/2) 로"""
    return (a + period / 2) % period - period / 2


def planar_pose(d: PuckDetection, R_bc: np.ndarray, t_bc: np.ndarray) -> PlanarPose:
    """카메라 광학 프레임 Detection → 로봇 프레임 (x, y, z, θ, 선/누움).
    R_bc, t_bc: 카메라 광학 프레임 → 로봇 프레임 변환 (촬영 시각 TF, base_link ← d435_color_optical_frame). 로봇 프레임 z = 위쪽 가정"""
    q = d.orientation
    x_, y_, z_, w_ = q
    R_cp = np.array([[1 - 2 * (y_ * y_ + z_ * z_), 2 * (x_ * y_ - z_ * w_), 2 * (x_ * z_ + y_ * w_)],
                     [2 * (x_ * y_ + z_ * w_), 1 - 2 * (x_ * x_ + z_ * z_), 2 * (y_ * z_ - x_ * w_)],
                     [2 * (x_ * z_ - y_ * w_), 2 * (y_ * z_ + x_ * w_), 1 - 2 * (x_ * x_ + y_ * y_)]])
    R_bp = R_bc @ R_cp                          # 퍽 축들 (열 = x, y, z=긴축) 을 로봇 프레임으로
    p = R_bc @ d.position + t_bc
    axis = R_bp[:, 2]
    upright = abs(axis[2]) > UPRIGHT_COS
    if upright:
        face = R_bp[:, 0]                       # 윗면 윤곽으로 정한 면 방향 (yaw_known 일 때만 의미 있음)
        theta = _wrap(float(np.arctan2(face[1], face[0])), np.pi / 2)
        known = d.shape == 'box' and d.yaw_known
        if not known:
            theta = 0.0
    else:
        theta = _wrap(float(np.arctan2(axis[1], axis[0])), np.pi)
        known = True
    return PlanarPose(float(p[0]), float(p[1]), float(p[2]), theta, bool(upright), known)


def draw_detections(vis, K: np.ndarray, dets: list[PuckDetection], raws: list[RawPuck] = ()):
    """wrist_pose.draw 결과 위에: 확정 퍽 = 굵은 흰 상자 + 'OK id', 거른 퍽 = 빨간 X + 이유"""
    import cv2
    for r in raws:
        if r.reject:
            x0, y0, x1, y1 = r.pose.bbox
            cv2.line(vis, (x0, y0), (x1, y1), (0, 0, 255), 1)
            cv2.line(vis, (x0, y1), (x1, y0), (0, 0, 255), 1)
            cv2.putText(vis, r.reject, (x0, y1 + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    for d in dets:
        x0, y0, x1, y1 = d.bbox
        cv2.rectangle(vis, (x0 - 3, y0 - 3), (x1 + 3, y1 + 3), (255, 255, 255), 2)
        cv2.putText(vis, f'OK {d.label} {d.shape} id{d.id or "?"} ({d.hits})', (x0, y1 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1)
    return vis
