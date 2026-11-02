# MissionStatus.msg — 지금 로봇이 무슨 일 중인가
#
# 역할
#   - 미션 상태 · 단계 · 진행 개수 · 시간을 한 메시지로 보여줌
# 토픽
#   - /mission/status, 2 Hz
# 보내는 쪽 → 받는 쪽
#   - mission_manager → 대시보드 · rosbag · 발표 영상
# note
#   - 앞 5 개 필드는 main 원래 정의, 나머지는 planning 이 덧붙임
#   - pucks_* 는 퍽 단위, tasks_* 는 task 단위

std_msgs/Header header
string state                  # 로봇 운영 상태: STANDBY · RUNNING · DONE · FAULT
string detail                 # 사람이 읽을 한 줄 설명, 마지막 실패 이유 등
uint8 pucks_done              # 옮긴 퍽 개수 = PLACE 성공
uint8 pucks_total             # 옮겨야 할 퍽 개수, 바코드를 다 읽기 전엔 늘어날 수 있음
bool fault                    # true 면 FAULT → 로봇은 멈춰 있어야 함

string phase                  # RUNNING 일 때 미션 단계: HOME · READ_BARCODE · OBSERVE_SIGNAL · COLLECT · DELIVER · VERIFY · RETURN_HOME, 그 밖은 ""
string[] wait_reasons         # STANDBY = 준비 안 된 이유, RUNNING = 대기 원인 DRIVE_LOST · SCAN_LOST · DETECTION_LOST · LOCALIZATION_LOST, FAULT = 원인 코드
bool ready                    # STANDBY 에서 /mission/start 를 받을 수 있나
string result                 # DONE 일 때: COMPLETED · RETURN_FAILED
string task_id                # 실행 중인 task, 없으면 ""
string step                   # 지금 단계 이름, 예 "nav:puck:7" · "arm:PICK:7", 없으면 ""
string arm_phase              # 팔 작업 진행 = ArmTask feedback phase, 팔 작업 중이 아니면 ""
uint8 tasks_done              # 끝낸 task 수
uint8 tasks_failed            # 포기한 task 수, 해석 불가 포함
uint8 tasks_total             # 시도한 task 수 = done + failed + 진행 중
float32 elapsed_s             # 첫 /mission/start 뒤 경과 시간. FAULT 중에도 흐르고 DONE 에서 멈춤, 시작 전 0
float32 time_left_s           # 제한 시간까지 남은 시간, 제한 없으면 -1. 발제 v5 는 제한 없음
bool hold                     # /mission/hold 와 같은 값
