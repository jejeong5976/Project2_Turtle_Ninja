# Detection.msg — 인지가 찾은 "물체 하나"
# 퍽 · QR · 신호등 · 노브를 전부 이 한 모양으로 보낸다 (받는 쪽이 kind 로 구분)
# 보내는 쪽: perception/perception_master   받는 쪽: mission/planning_master

std_msgs/Header header        # stamp = 사진 찍은 시각, frame_id = 카메라 프레임 (예: camera_color_optical_frame)

string kind                   # "puck" | "qr" | "traffic_light" | "knob"
string label                  # puck: "red"/"green"/"blue"  traffic_light: "red"/"orange"/"green"  qr: 읽은 글자 그대로
float32 confidence            # 0.0 ~ 1.0

geometry_msgs/Pose pose       # 카메라 프레임 기준 위치 [m]. depth 를 못 구했으면 position 을 전부 NaN 으로
float32[4] bbox               # 사진 속 상자 [x_min, y_min, x_max, y_max] (픽셀)
