# MotionMode.msg
#
# 역할
#   - 바퀴와 팔을 동시에 움직이지 않게 차례를 알림. 모터 전원을 따로 빼서 순간 전류를 줄이려는 것
#       DRIVE = 바퀴 · 조향만, 팔은 접은 자세로 버팀
#       ARM   = 팔 · 그리퍼만, 몸통은 멈춰서 버팀
#       STOP  = 아무것도 새로 움직이지 않음: 켤 때 · 모드를 바꾸는 사이 · 팔 작업 실패 뒤
# 토픽
#   - /control/motion_mode      control_master → swerve_controller · mission_manager, 10 Hz 계속, transient_local
#   - /control/motion_mode_ack  swerve_controller → control_master, 20 Hz
# 보내는 쪽 → 받는 쪽
#   - 위 토픽별로 다름
# note
#   - swerve_controller 는 DRIVE 이고 0.5 s 안에 받은 것일 때만 바퀴를 움직임
#   - ack 의 mode = 이 request_id 의 차례를 제어 루프에 실제로 반영한 값, DRIVE 또는 STOP
#   - control_master 는 STOPPING request_id 에 대한 STOP 답을 받고 /odom 이 0.5 s 멈춘 뒤에야 팔을 시작
#   - 팔 작업이 들어와 바퀴를 세우는 중 · 팔 작업 실패 뒤도 STOP. reason 의 STOPPING · STOP 으로 구분

uint8 STOP=0
uint8 DRIVE=1
uint8 ARM=2

std_msgs/Header header
uint8 mode
uint32 request_id    # 차례가 바뀔 때마다 1 씩 증가. 예전 번호의 답으로 팔을 시작하지 않으려고 씀
string reason        # 사람이 읽는 이유, 예 "STOPPING"
bool ready           # /control/motion_mode 만 씀: control_master 가 지금 새 팔 작업을 받을 수 있나
                     #   = 보드 상태 · 관절 측정이 신선하고 정상 + 팔 · 그리퍼 JTC 가 떠 있음
                     #   고장 · 측정 끊김이면 다시 false
                     #   바퀴 허가가 아님. 바퀴는 mode 만 보고, ready=true 여도 HOME 성공 전에는 STOP 그대로
