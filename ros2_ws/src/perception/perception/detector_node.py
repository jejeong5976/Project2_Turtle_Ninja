"""YOLO 탐지 노드 — 카메라 하나당 노드 하나 (wide_detector / wrist_detector).

  image (+ 정렬된 depth)  →  /perception/detections   (interfaces/DetectionArray)
                          →  /perception/signals      (interfaces/SignalState, publish_signals:=true 인 카메라만)

- 모델: Ultralytics 로 .pt 또는 Jetson 에서 빌드한 .engine 을 그대로 불러 씀 (가이드 6절)
- 퍽: 색(HSV) · AprilTag ID · 카메라 기준 위치(depth 우선, 없으면 태그 PnP)
- 신호등: 칸별 밝기 + 다수결
- ~/enable (std_srvs/SetBool): 상태 머신이 손목 카메라를 켜고 끌 때 사용

`detectors` 실행 파일은 두 노드를 한 프로세스에 띄움 → CUDA context 하나로 GPU 메모리 절약.
"""
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import DetectionArray, PuckDetection, SignalState
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool

from perception import postproc


def image_qos(mode):
    """이미지 구독 QoS. 기본 reliable + depth 1 (최신 한 장만).

    1280x720 bgr8 (2.7MB) 은 프로세스 사이에서 best_effort 로 받으면 조각 손실로 거의 다 버려짐 (Fast DDS, 실측).
    드라이버가 best_effort 로만 보낼 때만 "best_effort".
    """
    rel = ReliabilityPolicy.BEST_EFFORT if mode == "best_effort" else ReliabilityPolicy.RELIABLE
    return QoSProfile(depth=1, reliability=rel)


class Detector(Node):

    def __init__(self, name="detector"):
        super().__init__(name)
        p = self.declare_parameter
        self.camera = p("camera", "wide").value
        image_topic = p("image_topic", "/wide_cam/image_raw").value
        depth_topic = p("depth_topic", "").value                     # "" 이면 depth 안 씀
        info_topic = p("camera_info_topic", "/wide_cam/camera_info").value
        # launch 인자는 모든 노드에 같이 들어가므로 카메라 이름이 붙은 wide_model / wrist_model 로 받음.
        # 손목 모델을 안 주면 광각 모델(.pt)을 같이 씀
        models = {k: p(f"{k}_model", "").value for k in ("wide", "wrist")}
        self.model_path = p("model", "").value or models.get(self.camera) or models["wide"]
        self.imgsz = list(p("imgsz", [736, 1280]).value)              # (h, w)
        self.conf = p("conf", 0.35).value
        self.device = p("device", "0").value
        self.max_hz = p("max_hz", 15.0).value
        self.enabled = p("enabled", True).value
        self.puck_classes = set(p("puck_classes", ["puck_cyl", "puck_box"]).value)
        self.signal_class = p("signal_class", "signal_tower").value
        self.publish_signals = p("publish_signals", False).value
        self.read_tags = p("read_tags", True).value
        self.tag_size = p("tag.size", 0.025).value                   # [확인 필요] 퍽 태그 한 변 (m)
        self.tag_reader = postproc.TagReader(p("tag.family", "36h11").value,
                                             p("tag.min_side_px", 60).value)
        self.color_frac = p("color.roi_frac", 0.6).value
        self.color_cfg = {
            "v_min": p("color.v_min", 40).value,
            "min_pixels": p("color.min_pixels", 30).value,
            "s_beige_max": p("color.s_beige_max", 70).value,
            "beige_hue": list(p("color.beige_hue", [5, 35]).value),
            "red_hue": list(p("color.red_hue", [170, 10]).value),
            "green_hue": list(p("color.green_hue", [40, 85]).value),
        }
        self.tiers = {
            "red": list(p("signal.red", [0.10, 0.40]).value),
            "orange": list(p("signal.orange", [0.40, 0.70]).value),
            "green": list(p("signal.green", [0.70, 1.00]).value),
        }
        self.v_margin = p("signal.v_margin", 40.0).value
        self.signal_vote = postproc.Voter(p("signal.window", 5).value, p("signal.need", 3).value)
        self.signal_timeout = p("signal.timeout", 1.0).value        # 이 시간 동안 안 보이면 unknown
        qos = image_qos(p("image_qos", "reliable").value)

        self.bridge = CvBridge()
        self.K = self.D = None
        self.fisheye = False
        self.model = None
        self.names = {}
        self._last = 0.0
        self._last_signal_seen = 0.0

        self.pub = self.create_publisher(DetectionArray, "/perception/detections", 10)
        self.sig_pub = (self.create_publisher(SignalState, "/perception/signals", 10)
                        if self.publish_signals else None)
        self.create_subscription(CameraInfo, info_topic, self._on_info, 1)
        self.create_service(SetBool, "~/enable", self._on_enable)

        if depth_topic:
            import message_filters
            subs = [message_filters.Subscriber(self, Image, image_topic, qos_profile=qos),
                    message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos)]
            self._sync = message_filters.ApproximateTimeSynchronizer(subs, 5, 0.05)
            self._sync.registerCallback(self._on_image)
        else:
            self.create_subscription(Image, image_topic, self._on_image, qos)

        self._load_model()
        self.get_logger().info(
            f"[{self.camera}] {image_topic}{' + ' + depth_topic if depth_topic else ''} → /perception/detections"
            f" (enabled={self.enabled})")

    # ------------------------------------------------------------ 준비

    def _load_model(self):
        if not self.model_path:
            self.get_logger().error("model 파라미터가 비어 있음 — 탐지 안 함")
            return
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error("ultralytics 가 없음: pip install ultralytics")
            return
        self.model = YOLO(self.model_path, task="detect")
        self.names = dict(self.model.names) if self.model.names else {}
        # 첫 추론은 엔진 초기화로 느리므로 빈 이미지로 미리 한 번
        self.model.predict(np.zeros((*self.imgsz, 3), np.uint8), imgsz=self.imgsz,
                           device=self.device, verbose=False)
        self.get_logger().info(f"[{self.camera}] 모델 {self.model_path} 클래스 {self.names}")

    def _on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.fisheye = msg.distortion_model == "equidistant"
        self.D = np.array(msg.d, dtype=np.float64)

    def _on_enable(self, req, res):
        self.enabled = req.data
        self.signal_vote.clear()
        res.success = True
        res.message = f"{self.camera} {'on' if self.enabled else 'off'}"
        self.get_logger().info(res.message)
        return res

    # ------------------------------------------------------------ 처리

    def _on_image(self, img_msg, depth_msg=None):
        now = time.monotonic()
        if not self.enabled or self.model is None or now - self._last < 1.0 / self.max_hz:
            return
        self._last = now

        img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        depth = self._depth_m(depth_msg) if depth_msg is not None else None
        res = self.model.predict(img, imgsz=self.imgsz, conf=self.conf, device=self.device, verbose=False)[0]

        out = DetectionArray()
        out.header = img_msg.header
        out.camera = self.camera
        best_signal = None
        for xyxy, score, cls in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.conf.cpu().numpy(),
                                    res.boxes.cls.cpu().numpy().astype(int)):
            name = self.names.get(cls, str(cls))
            det = self._make(img, depth, name, float(score), xyxy)
            out.detections.append(det)
            if name == self.signal_class and (best_signal is None or score > best_signal[1]):
                best_signal = (xyxy, score)
        self.pub.publish(out)

        if self.sig_pub is not None:
            self._publish_signal(img, img_msg.header, best_signal, now)

    def _make(self, img, depth, name, score, box):
        d = PuckDetection()
        d.class_name = name
        d.score = score
        d.tag_id = -1
        x1, y1, x2, y2 = (float(v) for v in box)
        d.cx, d.cy, d.width, d.height = (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1
        d.fallen = name.endswith("_fallen")
        base = name[:-len("_fallen")] if d.fallen else name
        if base not in self.puck_classes:
            return d

        d.color, _ = postproc.classify_color(postproc.center_roi(img, box, self.color_frac), self.color_cfg)

        corners = None
        if self.read_tags:
            d.tag_id, corners = self.tag_reader.read(img, box)

        if self.K is None:
            return d
        if depth is not None:                     # 손목: depth 로 퍽 중심 위치
            pt = postproc.depth_point(depth, box, self.K)
            if pt is not None:
                d.has_pose = True
                d.pose.position.x, d.pose.position.y, d.pose.position.z = (float(v) for v in pt)
                d.pose.orientation.w = 1.0
                return d
        if corners is not None:                   # depth 없으면 태그 PnP (태그 면의 자세)
            r = postproc.tag_pose(corners, self.tag_size, self.K, self.D, self.fisheye)
            if r is not None:
                t, R = r
                d.has_pose = True
                d.pose.position.x, d.pose.position.y, d.pose.position.z = (float(v) for v in t)
                (d.pose.orientation.x, d.pose.orientation.y,
                 d.pose.orientation.z, d.pose.orientation.w) = postproc.rot_to_quat(R)
        return d

    def _publish_signal(self, img, header, best, now):
        msg = SignalState()
        msg.header = header
        msg.camera = self.camera
        msg.cx = msg.cy = -1.0
        if best is not None:
            box = best[0]
            x1, y1, x2, y2 = (int(v) for v in box)
            state, _ = postproc.signal_state(img[max(0, y1):y2, max(0, x1):x2], self.tiers, self.v_margin)
            self.signal_vote.push(state)
            self._last_signal_seen = now
            msg.cx, msg.cy = float((x1 + x2) / 2), float((y1 + y2) / 2)
        elif now - self._last_signal_seen > self.signal_timeout:
            self.signal_vote.clear()
        value, ratio = self.signal_vote.result()
        msg.state = value or SignalState.UNKNOWN
        msg.confidence = float(ratio)
        self.sig_pub.publish(msg)

    def _depth_m(self, msg):
        d = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        if d.dtype == np.uint16:                  # realsense 기본: mm
            return d.astype(np.float32) / 1000.0
        return d.astype(np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():                   # Ctrl+C 로 이미 컨텍스트가 닫혔으면 destroy 하지 않음
            node.destroy_node()
        rclpy.try_shutdown()


def main_both(args=None):
    """광각 + 손목을 한 프로세스에서 (파라미터는 노드 이름 wide_detector / wrist_detector 로 구분)."""
    rclpy.init(args=args)
    nodes = [Detector("wide_detector"), Detector("wrist_detector")]
    ex = MultiThreadedExecutor(num_threads=2)
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            for n in nodes:
                n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
