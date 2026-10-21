"""
detectors.py — perception_master 안에서 쓰는 부품 3개 (노드가 아님)
=========================================================================
  PuckDetector   : 퍽(빨·초·파) 찾기        D435 컬러 (+ 나중에 depth) → Detection(kind="puck")
  QrReader       : QR 읽어서 "무슨 색을 어디로"  D435 컬러 (멈춘 뒤, READ_QR)  → QrInstruction
  SignalDetector : 신호등 · 노브 찾기           bird-eye 영상                → Detection(kind="traffic_light"/"knob")

지금 상태: 뼈대. 셋 다 "아무것도 못 찾음" 으로 답한다 (빈 목록 / None).
나눠 짜기: 클래스 단위로 따로 고칠 수 있게. perception_master.py 는 연결만 (영상 받기 · 발행 · 예외 처리).
YOLO 를 쓰면 perception_master 가 **한 번만** 불러서 model 로 넘겨 줌 (Jetson GPU 메모리 아끼기, D18)
"""

from interfaces.msg import Detection, QrInstruction


class PuckDetector:
    """TODO
      1. YOLO 로 퍽 찾기 + 크기 검사 (너무 작거나 큰 상자는 버림)
      2. 색 3종 구분 추가 (YOLO 클래스 3개 또는 bbox 안 색 판정)
      3. depth 로 위치 → pose 채우기 (bbox 가운데 영역 깊이의 중앙값 — 가장자리 배경 깊이에 안 속게)
      4. bird-eye 영상으로 "멀리서 후보 찾기" — detect() 를 카메라별로 부르면 됨
    """

    def __init__(self, model=None, min_confidence: float = 0.5):
        self.model = model                  # perception_master 가 불러 둔 YOLO (지금은 None)
        self.min_confidence = min_confidence

    def detect(self, img) -> list:
        """img: sensor_msgs/Image → Detection 목록 (못 찾으면 빈 목록)."""
        _example = Detection(kind="puck", label="red", confidence=0.0)  # 채울 모양 예시
        return []


class QrReader:
    """TODO
      1. QR 찾기 · 읽기: OpenCV 의 cv2.QRCodeDetector (영상 → numpy 는 cv_bridge)
      2. parse() 를 대회 QR 포맷에 맞게 (포맷 미정)
    """

    def read(self, img):
        """img: sensor_msgs/Image → QR 글자 (못 찾으면 None)."""
        return None

    @staticmethod
    def parse(text: str) -> QrInstruction:
        """포맷 확정되면 수정. 지금은 'red:A' 같은 '색:선반' 모양이라고 가정."""
        msg = QrInstruction(raw_text=text)
        parts = text.strip().split(":")
        if len(parts) == 2 and parts[0] in ("red", "green", "blue"):
            msg.valid = True
            msg.puck_color, msg.target_shelf_id = parts
        return msg


class SignalDetector:
    """TODO
      1. 신호등(사인타워): 빨강 · 주황 · 초록 → label "red" / "orange" / "green". planning 은 빨강 · 주황이면 대기
         노브(셀렉트 스위치): 돌리면 무엇이 바뀌는지 미정 → 정해지면 kind="knob", label 에 상태
      2. 신호등은 멈춤 판단에 쓰임 → 놓치는 것보다 잘못 보는 게 덜 위험한 쪽으로 기준 잡기
         (빨강 · 주황을 초록으로 잘못 보면 위험, 초록을 주황으로 잘못 보면 잠깐 멈출 뿐)
      3. YOLO 를 쓰면 퍽과 같은 모델(클래스 추가) 하나로
    """

    def __init__(self, model=None):
        self.model = model

    def detect(self, img) -> list:
        """img: sensor_msgs/Image → Detection 목록."""
        return []
