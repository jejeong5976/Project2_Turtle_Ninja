# DetectionArray.msg — 사진 한 장에서 찾은 물체 전부
# 아무것도 못 찾았으면 detections 가 빈 배열 (메시지 자체는 보낸다 → "카메라는 살아 있음" 의 증거)

std_msgs/Header header
Detection[] detections
