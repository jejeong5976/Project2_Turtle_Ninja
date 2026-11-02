# DetectionArray.msg — 영상 한 장에서 찾은 퍽 검출 전부
#
# 역할
#   - 카메라 한 대의 영상 한 장 결과를 묶음
# 토픽
#   - /perception/wrist/detections  손목 D435
#   - /perception/wide/detections   광각, 위치 없음 → RViz · 디버깅용
# 보내는 쪽 → 받는 쪽
#   - perception_node → mission_manager, 기본은 손목만 구독
# note
#   - 못 찾아도 빈 배열로 발행 → 카메라가 살아 있다는 뜻
#   - 카메라마다 토픽을 나눔 → 한쪽 빈 배열이 다른 쪽 결과로 오해되지 않음

std_msgs/Header header        # 영상 header 그대로, frame_id 는 URDF 카메라 광학 프레임
Detection[] detections
