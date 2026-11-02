# TaskText.msg — 벽 1D 바코드에서 읽은 원문
#
# 역할
#   - 바코드 내용을 해석 없이 그대로 전달
# 토픽
#   - /perception/task_text
# 보내는 쪽 → 받는 쪽
#   - perception_node → mission_manager
# note
#   - 해석은 planning 이 함. 발제 v5 = P2-01 · P2-03 오름차순, P2-02 · P2-04 내림차순, 그 밖은 미등록
#   - perception 은 planning 이 켰을 때만 읽고, 다수결로 확정된 값만 보냄
#   - 켜져 있는 동안 계속 보냄 → planning 은 단계 시작 뒤 온 것만 씀

std_msgs/Header header        # stamp = 촬영 시각, frame_id = 읽은 카메라 광학 프레임
string raw_text               # 읽은 내용 그대로
geometry_msgs/Pose pose       # 바코드 위치. 지금은 안 구해서 늘 NaN
