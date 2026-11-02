# Detection.msg — 퍽 검출 1 개
#
# 역할
#   - 영상 한 장에서 찾은 퍽 하나의 색 · 모양 · 태그 ID · 자세
# 토픽
#   - DetectionArray 안에 담겨 /perception/wrist/detections · /perception/wide/detections 로 나감
# 보내는 쪽 → 받는 쪽
#   - perception_node → mission_manager
# note
#   - 위치 모름 = position 전부 NaN
#   - 방향 모름 = orientation 0 · 0 · 0 · 0. msg 기본값이 w=1 이라 w=0 을 직접 넣어야 함
#   - 광각 검출은 깊이가 없어 pose 가 늘 모름

std_msgs/Header header        # stamp = 촬영 시각, frame_id = pose 기준 카메라 광학 프레임

string kind                   # "puck"
string label                  # "red" | "green"
string shape                  # "cylinder" | "box", 모르면 ""
string id                     # AprilTag ID 문자열 "0"–"11", 못 읽으면 ""
float32 confidence            # 0.0 ~ 1.0

geometry_msgs/Pose pose       # 퍽 중심 m, z 축 = 6 cm 긴축 방향
bool yaw_known                # x 축을 믿어도 되는지
                              #   true  = x 축이 사각기둥 면 법선, 90° 주기. 선 퍽 = 윗면 윤곽으로, 누운 퍽 = 위쪽
                              #   false = z 축만 믿음. 원기둥 · 면 방향 못 구한 사각기둥 · 모서리로 기댄 사각기둥
float32[4] bbox               # x_min, y_min, x_max, y_max, 원본 영상 픽셀
