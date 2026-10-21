"""
perception_master.py — 인지 마스터: 카메라 2대 → 퍽 · QR · 신호등
=====================================================================
카메라 영상을 **한 곳에서** 받고, 안의 부품(detectors.py)이 나눠 처리한다.

  D435 (팔 머리) ─color─▶ ┌──────────────────────┐ ──/perception/detections (DetectionArray)──▶ planning_master
                          │  perception_master    │ ──/perception/qr        (QrInstruction) ───▶
  bird-eye ──────image──▶ │  detectors.py 의 부품 │ ──/perception/signals   (DetectionArray)──▶
                          │  Puck · Qr · Signal   │
                          └──────────────────────┘

왜 노드 하나로 (D18)
  - YOLO 모델을 **한 번만** 불러서 퍽 · 신호등이 같이 씀 → Jetson Orin Nano GPU 메모리 아끼기
  - 같은 영상을 노드마다 따로 받지 않음 (복사 · 전송 한 번)
  대신 하나가 죽으면 인지 전체가 멈춤 → 부품마다 try 로 감싸서 한 부품 에러가 다른 부품을 안 죽이게

나눠 짜기: 부품은 detectors.py 의 클래스(PuckDetector · QrReader · SignalDetector) 단위로 고친다 (이 파일은 연결만).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from interfaces.msg import DetectionArray, QrInstruction
from perception.detectors import PuckDetector, QrReader, SignalDetector


class PerceptionMaster(Node):
    def __init__(self):
        super().__init__("perception_master")
        d435_topic = self.declare_parameter("d435_color_topic", "/camera/camera/color/image_raw").value
        bev_topic = self.declare_parameter("birdeye_topic", "/bird_camera/image_raw").value
        min_conf = self.declare_parameter("min_confidence", 0.5).value

        # TODO YOLO 모델을 여기서 한 번만 불러서 부품들에게 넘김
        model = None
        self.puck = PuckDetector(model, min_conf)
        self.qr = QrReader()
        self.signal = SignalDetector(model)
        self.last_qr = None

        self.pub_det = self.create_publisher(DetectionArray, "/perception/detections", 10)
        self.pub_qr = self.create_publisher(QrInstruction, "/perception/qr", 10)
        self.pub_sig = self.create_publisher(DetectionArray, "/perception/signals", 10)
        self.create_subscription(Image, d435_topic, self.on_d435, qos_profile_sensor_data)
        self.create_subscription(Image, bev_topic, self.on_birdeye, qos_profile_sensor_data)

        self.frames = {"d435": 0, "birdeye": 0}
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"perception_master 시작 (뼈대): D435 {d435_topic}, bird-eye {bev_topic}")

    # ---------------------------------------------------------------- D435: 퍽 · QR
    def on_d435(self, img: Image):
        self.frames["d435"] += 1
        out = DetectionArray(header=img.header)
        out.detections = self.safe("puck", self.puck.detect, img, default=[])
        self.pub_det.publish(out)          # 못 찾아도 빈 배열 보냄 (= 카메라 살아 있음, docs/design.md 3절)

        text = self.safe("qr", self.qr.read, img, default=None)
        if text is not None and text != self.last_qr:   # 같은 QR 을 매 프레임 보내지 않기
            self.last_qr = text
            msg = self.qr.parse(text)
            msg.header = img.header
            self.pub_qr.publish(msg)
            self.get_logger().info(f"QR 읽음: {text!r} → {msg.puck_color} → {msg.target_shelf_id}")

    # ---------------------------------------------------------------- bird-eye: 신호등 · 노브
    def on_birdeye(self, img: Image):
        self.frames["birdeye"] += 1
        out = DetectionArray(header=img.header)
        out.detections = self.safe("signal", self.signal.detect, img, default=[])
        self.pub_sig.publish(out)

    # ---------------------------------------------------------------- 도우미
    def safe(self, name, fn, *args, default=None):
        """부품 하나가 에러를 내도 마스터(와 다른 부품)는 계속 돌게."""
        try:
            return fn(*args)
        except Exception as e:  # noqa: BLE001 — 어떤 에러든 로그만 남기고 계속
            self.get_logger().error(f"[{name}] 에러: {e}", throttle_duration_sec=2.0)
            return default

    def report(self):
        d, b = self.frames["d435"], self.frames["birdeye"]
        self.get_logger().info(f"최근 5초 영상: D435 {d / 5.0:.1f} Hz, bird-eye {b / 5.0:.1f} Hz")
        self.frames = {"d435": 0, "birdeye": 0}


def main():
    rclpy.init()
    node = PerceptionMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
