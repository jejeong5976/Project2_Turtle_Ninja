# CargoSlot.msg — 적재함 칸 1 개 또는 그리퍼의 내용
#
# 역할
#   - planning 재고 기록 한 칸
# 토픽
#   - CargoState 안에 담겨 /mission/cargo 로 나감
# 보내는 쪽 → 받는 쪽
#   - mission_manager → 대시보드 · rosbag
# note
#   - 칸 이름은 control 과 같아야 함, planning 기본 S1–S12. 이름은 아직 control 과 미확정

string slot                   # 칸 이름, 그리퍼면 "gripper"
bool occupied                 # 퍽이 있는가
string puck_id                # AprilTag ID
string color                  # "red" | "green"
string shape                  # "cylinder" | "box" | ""
string task_id                # 이 퍽을 실은 task
bool verified                 # 카메라로 확인했으면 true, 기록만이면 false
