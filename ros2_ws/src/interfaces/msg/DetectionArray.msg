# DetectionArray.msg — /perception/detections
# 카메라 한 프레임에서 찾은 것 전부. 두 카메라가 같은 토픽에 보내고 camera 로 구분
# header.stamp = 원본 이미지 시각, header.frame_id = 그 카메라의 광학 프레임

std_msgs/Header header
string camera                 # "wide" | "wrist"
PuckDetection[] detections
