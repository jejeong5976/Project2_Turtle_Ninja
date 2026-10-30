"""벽 1D 바코드 판독 노드.

  image  →  /perception/task_text   (interfaces/TaskText)

- 최근 /perception/detections 에 같은 카메라의 barcode bbox 가 있으면 그 영역을 키워서 먼저 읽고,
  못 읽으면 전체 화면을 읽음 (광각에서는 바코드가 작아서 ROI 확대가 중요)
- 여러 프레임에서 같은 문자열이 나와야 확정 (가이드 7.4). 확정한 문자열은 다시 보내지 않음
- ~/reset (std_srvs/Trigger): 확정 기록 비우기 (같은 바코드를 다시 읽어야 할 때)
"""
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import DetectionArray, TaskText
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger

from perception import postproc
from perception.detector_node import image_qos


class BarcodeNode(Node):

    def __init__(self):
        super().__init__("barcode")
        p = self.declare_parameter
        self.camera = p("camera", "wrist").value
        image_topic = p("image_topic", "/wrist_cam/color/image_raw").value
        self.max_hz = p("max_hz", 10.0).value
        self.enabled = p("enabled", True).value
        self.barcode_class = p("barcode_class", "barcode").value
        self.roi_pad = p("roi_pad", 0.3).value
        self.roi_scale = p("roi_scale", 2.0).value
        self.roi_timeout = p("roi_timeout", 0.5).value
        self.full_frame = p("full_frame", True).value
        self.voter = postproc.Voter(p("window", 5).value, p("need", 3).value)

        self.reader = postproc.BarcodeReader()
        self.bridge = CvBridge()
        self.sent = set()
        self._boxes, self._boxes_t = [], 0.0
        self._last = 0.0

        self.pub = self.create_publisher(TaskText, "/perception/task_text", 10)
        self.create_subscription(Image, image_topic, self._on_image,
                                 image_qos(p("image_qos", "reliable").value))
        self.create_subscription(DetectionArray, "/perception/detections", self._on_dets, 10)
        self.create_service(SetBool, "~/enable", self._on_enable)
        self.create_service(Trigger, "~/reset", self._on_reset)
        self.get_logger().info(f"[{self.camera}] {image_topic} → /perception/task_text")

    def _on_dets(self, msg):
        if msg.camera != self.camera:
            return
        boxes = [d for d in msg.detections if d.class_name == self.barcode_class]
        if boxes:
            self._boxes, self._boxes_t = boxes, time.monotonic()

    def _on_enable(self, req, res):
        self.enabled = req.data
        self.voter.clear()
        res.success, res.message = True, f"barcode {'on' if self.enabled else 'off'}"
        return res

    def _on_reset(self, _req, res):
        self.sent.clear()
        self.voter.clear()
        res.success, res.message = True, "cleared"
        return res

    def _on_image(self, msg):
        now = time.monotonic()
        if not self.enabled or now - self._last < 1.0 / self.max_hz:
            return
        self._last = now
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        found = []
        if now - self._boxes_t < self.roi_timeout:
            for b in self._boxes:
                found = self.reader.read(self._crop(img, b))
                if found:
                    break
        if not found and self.full_frame:
            found = self.reader.read(img)
        if not found:
            return

        text, sym = found[0]
        value, _ = self.voter.push((text, sym))
        if value is None or value[0] in self.sent:
            return
        self.sent.add(value[0])
        out = TaskText()
        out.header = msg.header
        out.camera = self.camera
        out.text, out.symbology = value
        out.votes = sum(1 for v in self.voter.buf if v == value)
        self.pub.publish(out)
        self.get_logger().info(f"바코드 확정: {out.text} ({out.symbology})")

    def _crop(self, img, d):
        w, h = d.width * (1 + 2 * self.roi_pad), d.height * (1 + 2 * self.roi_pad)
        x1, y1 = int(max(0, d.cx - w / 2)), int(max(0, d.cy - h / 2))
        x2, y2 = int(min(img.shape[1], d.cx + w / 2)), int(min(img.shape[0], d.cy + h / 2))
        roi = img[y1:y2, x1:x2]
        if roi.size == 0:
            return img
        return cv2.resize(roi, None, fx=self.roi_scale, fy=self.roi_scale, interpolation=cv2.INTER_CUBIC)


def main(args=None):
    rclpy.init(args=args)
    node = BarcodeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
