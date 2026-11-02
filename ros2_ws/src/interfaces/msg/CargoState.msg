# CargoState.msg — 적재함 · 그리퍼 재고 전체
#
# 역할
#   - 지금 어느 칸에 어떤 퍽이 있는지 한 번에 보여줌
# 토픽
#   - /mission/cargo, 바뀔 때 + 1 Hz
# 보내는 쪽 → 받는 쪽
#   - mission_manager 의 Executor → 대시보드 · rosbag
# note
#   - 팔 작업 성공 결과로만 바뀜

std_msgs/Header header
CargoSlot[] slots             # 팔이 닿는 칸만
CargoSlot gripper             # slot = "gripper"
