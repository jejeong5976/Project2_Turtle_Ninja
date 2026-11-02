# PuckDetection.msg — 화면에서 찾은 물체 하나 (퍽 · 신호등 · 바코드 · QR)
# 보내는 쪽: perception/detector   받는 쪽: mission/planning_master
# 위치는 카메라 프레임 기준 (planning 요청). base_link 변환은 받는 쪽에서 tf 로

string class_name             # YOLO 클래스 이름 (puck_cyl, puck_box, signal_tower, barcode, qr ...)
float32 score                 # YOLO 신뢰도 0~1

int32 tag_id                  # 퍽 AprilTag ID (36h11). 못 읽었으면 -1
string color                  # 퍽 색: "red" | "green" | "beige" | "unknown" (퍽이 아니면 "")
bool fallen                   # 넘어진 퍽 (클래스 이름이 *_fallen 이면 true)

# bbox (원본 이미지 픽셀, 중심 + 크기)
float32 cx
float32 cy
float32 width
float32 height

bool has_pose                 # pose 를 구했나 (손목 depth 또는 AprilTag PnP 성공 시 true)
geometry_msgs/Pose pose       # 카메라 광학 프레임 기준 (header.frame_id 는 DetectionArray 쪽)
