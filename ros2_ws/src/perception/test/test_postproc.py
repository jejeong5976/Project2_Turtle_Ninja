"""후처리 함수 테스트 — 합성 이미지로 (카메라 · 모델 없이)."""
import cv2
import numpy as np

from perception import postproc

COLOR = {"v_min": 40, "min_pixels": 30, "s_beige_max": 70,
         "beige_hue": [5, 35], "red_hue": [170, 10], "green_hue": [40, 85]}
TIERS = {"red": [0.10, 0.40], "orange": [0.40, 0.70], "green": [0.70, 1.00]}


def _hsv_patch(h, s, v):
    return cv2.cvtColor(np.full((40, 40, 3), (h, s, v), np.uint8), cv2.COLOR_HSV2BGR)


def test_color_red_green_beige():
    assert postproc.classify_color(_hsv_patch(2, 200, 180), COLOR)[0] == "red"
    assert postproc.classify_color(_hsv_patch(176, 200, 180), COLOR)[0] == "red"
    assert postproc.classify_color(_hsv_patch(60, 200, 180), COLOR)[0] == "green"
    assert postproc.classify_color(_hsv_patch(20, 50, 200), COLOR)[0] == "beige"


def test_color_red_across_hue_wrap():
    # 빨강 픽셀이 0 과 179 양쪽으로 반씩 → 단순 중앙값이면 90 근처가 나와서 틀림
    img = np.concatenate([_hsv_patch(2, 200, 180), _hsv_patch(177, 200, 180)], axis=1)
    assert postproc.classify_color(img, COLOR)[0] == "red"


def test_color_dark_is_unknown():
    assert postproc.classify_color(_hsv_patch(60, 200, 10), COLOR)[0] == "unknown"


def _tower(lit):
    img = np.full((100, 30, 3), 60, np.uint8)
    if lit:
        a, b = TIERS[lit]
        img[int(a * 100):int(b * 100)] = 230
    return img


def test_signal_state():
    for s in ("red", "orange", "green"):
        assert postproc.signal_state(_tower(s), TIERS, 40)[0] == s
    assert postproc.signal_state(_tower(None), TIERS, 40)[0] == "off"


def test_voter():
    v = postproc.Voter(5, 3)
    assert v.push("red")[0] is None
    v.push("green")
    v.push("red")
    assert v.push("red")[0] == "red"


def _tag_image(tag_id, side=120):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    gen = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    tag = gen(d, tag_id, side)
    img = np.full((480, 640), 255, np.uint8)
    img[200:200 + side, 300:300 + side] = tag
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), (300, 200, 300 + side, 200 + side)


def test_tag_id_and_pose():
    img, box = _tag_image(7)
    tag_id, corners = postproc.TagReader().read(img, box)
    assert tag_id == 7
    K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], np.float64)
    t, R = postproc.tag_pose(corners, 0.025, K, np.zeros(5), fisheye=False)
    # 태그 한 변 120px, f=600 → 25mm 태그는 약 0.125m 앞
    assert abs(t[2] - 0.125) < 0.01
    q = postproc.rot_to_quat(R)
    assert abs(np.linalg.norm(q) - 1) < 1e-6


def test_tag_missing():
    img = np.full((480, 640, 3), 255, np.uint8)
    assert postproc.TagReader().read(img, (100, 100, 200, 200))[0] == -1


def test_depth_point():
    K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], np.float64)
    depth = np.full((480, 640), 2.0, np.float32)
    depth[200:280, 280:360] = 0.5                      # 퍽
    p = postproc.depth_point(depth, (280, 200, 360, 280), K)
    assert np.allclose(p, [0.0, 0.0, 0.5], atol=1e-3)


def test_barcode_ean13():
    # EAN-13 4006381333931 을 직접 그림 (OpenCV 4.6 에는 생성기가 없음)
    L = ["0001101", "0011001", "0010011", "0111101", "0100011",
         "0110001", "0101111", "0111011", "0110111", "0001011"]
    G = ["0100111", "0110011", "0011011", "0100001", "0011101",
         "0111001", "0000101", "0010001", "0001001", "0010111"]
    R = ["1110010", "1100110", "1101100", "1000010", "1011100",
         "1001110", "1010000", "1000100", "1001000", "1110100"]
    PARITY = ["LLLLLL", "LLGLGG", "LLGGLG", "LLGGGL", "LGLLGG",
              "LGGLLG", "LGGGLL", "LGLGLG", "LGLGGL", "LGGLGL"]
    code = "4006381333931"
    d = [int(c) for c in code]
    bits = "101"
    for i, par in enumerate(PARITY[d[0]]):
        bits += (L if par == "L" else G)[d[1 + i]]
    bits += "01010"
    for x in d[7:]:
        bits += R[x]
    bits += "101"
    reader = postproc.BarcodeReader()
    for m in (3, 6):                                   # 6 은 폭 570px 로 화면을 거의 채움
        row = np.array([0 if b == "1" else 255 for b in bits], np.uint8).repeat(m)
        img = np.full((480, 640), 255, np.uint8)
        x0 = (640 - len(row)) // 2
        img[100:380, x0:x0 + len(row)] = row
        img = cv2.GaussianBlur(img, (0, 0), 1.0)       # 실제 카메라처럼 경계를 약간 흐리게
        found = reader.read(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
        assert found and found[0][0] == code, m
