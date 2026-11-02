# SignalRecord.msg — 신호등 점멸 기록 결과
#
# 역할
#   - 발제 v5 신호등 과제: 출발 구역에서 본 12 번 점멸의 순서 · 색별 횟수
# 토픽
#   - /planning/signal_record  mission_manager, 바뀔 때 + 1 Hz, transient_local
#   - /planning/signal_watch   signal_watch 단독 실행, transient_local
# 보내는 쪽 → 받는 쪽
#   - mission_manager · signal_watch → 대시보드 · rosbag, mission_manager 는 이 결과로 주사위 계산
# note
#   - 점멸 = 1 s 켜짐 + 1 s 소등, 12 번째 뒤 긴 소등 = 끝. 기록기는 2.5 s 소등을 끝으로 봄
#   - 불명 색은 추정하지 않고 '?'

uint8 WAITING=0               # 첫 점멸 전
uint8 RECORDING=1             # 기록 중
uint8 DONE=2                  # 12 개 · 끝 소등 확인 · 색 모두 확실 · 횟수 규칙 각 1–6, 합 12 맞음
uint8 INCOMPLETE=3            # 끝났지만 믿을 수 없음, 이유는 issues

std_msgs/Header header
uint8 status
string sequence               # 점멸 순서, 예 "RGRRGRGGYGYG". 색 불명은 '?'
uint8 count_red
uint8 count_yellow
uint8 count_green
uint8 count_unknown
string[] issues               # 못 믿게 하는 문제: 관측 끊김 · 색 불명 · 개수 · 횟수 규칙
