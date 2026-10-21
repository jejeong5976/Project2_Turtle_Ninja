# MotionMode.msg — 지금 "어느 무리"만 움직여도 되나
# =========================================================
# D15: 모터 전원을 따로 빼서 쓰고, 순간 전류를 줄이려고 **한 번에 한 무리만** 움직인다.
#   DRIVE : 바퀴 · 조향만 (팔은 접은 자세로 버티기)
#   ARM   : 팔 · 그리퍼만 (몸통은 멈춰서 버티기)
#   STOP  : 아무것도 새로 움직이지 않음 (켤 때, 모드를 바꾸는 사이, 실패 후)
#   (팔 작업이 들어와 바퀴를 세우는 중 · 팔 작업 실패 후도 STOP — reason 에 STOPPING / STOP 으로 구분)
#
# 두 토픽이 이 메시지를 씀
#   /control/motion_mode      control_master → swerve_controller · planning   10 Hz 로 계속 (심장박동)
#                             swerve_controller 는 DRIVE 이고 0.5 s 안에 받은 것일 때만 바퀴를 움직임
#   /control/motion_mode_ack  swerve_controller → control_master               20 Hz
#                             "이 request_id 의 차례를 제어 루프에 반영했음". mode = 실제로 반영한 것 (DRIVE / STOP)
#   control_master 는 STOPPING 의 request_id 에 대한 STOP 답을 받고 + /odom 이 0.5 s 멈춘 뒤에야 팔을 시작

uint8 STOP=0
uint8 DRIVE=1
uint8 ARM=2

std_msgs/Header header
uint8 mode
uint32 request_id    # 차례가 바뀔 때마다 1 씩 증가. 예전 번호의 답으로 팔을 시작하지 않으려고
string reason        # 사람이 읽는 이유 (예: "STOPPING")
bool ready           # /control/motion_mode 만: control_master 가 **지금** 새 팔 작업을 받을 수 있나
                     #   = 보드 상태 · 관절 측정이 신선하고 정상 + 팔 · 그리퍼 재생기(JTC)가 떠 있음
                     #   고장 · 측정 끊김이면 다시 false. 바퀴 허가가 아님 — 바퀴는 mode(DRIVE)만 봄,
                     #   ready=true 여도 HOME 이 성공하기 전에는 STOP 그대로. (ack 토픽에서는 안 씀)
