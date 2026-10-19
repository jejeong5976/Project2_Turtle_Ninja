# MissionStatus.msg — "지금 로봇이 무슨 일 중인가"
# 보내는 쪽: nt_planning/planning_master   받는 쪽: 대시보드 · rosbag 기록 · 발표 영상

std_msgs/Header header
string state                  # 상태 이름 (nt_planning/planning_master.py 의 State 참고)
string detail                 # 사람이 읽을 한 줄 설명
uint8 pucks_done              # 옮긴 퍽 개수
uint8 pucks_total             # 옮겨야 할 퍽 개수 (QR 을 읽어야 알 수 있음)
bool fault                    # true 면 FAULT — 로봇은 멈춰 있어야 함
