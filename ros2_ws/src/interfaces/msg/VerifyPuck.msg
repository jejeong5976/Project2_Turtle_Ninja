# VerifyPuck.msg — 재검증 때 장소에서 새로 본 퍽 1 개
#
# 역할
#   - 배치를 카메라로 다시 확인한 결과 중 퍽 하나
# 토픽
#   - VerifyRecord 안에 담겨 /planning/verify_record 로 나감
# 보내는 쪽 → 받는 쪽
#   - mission_manager → 대시보드 · rosbag · 기록 파일
# note
#   - 목표 상태를 복사하지 않고, VERIFY 단계 LOOK 뒤에 들어온 검출만 씀
#   - 좌표는 map 프레임

string place                  # "L1" | "L2" | "L3"
string id                     # AprilTag ID, 못 읽었으면 ""
string color                  # "red" | "green" | ""
string shape                  # "cylinder" | "box" | ""
float64 x                     # map m
float64 y
int8 cell                     # 맞은 칸 번호, 계획의 먼 칸부터 순서. 칸 밖이면 -1
float32 cell_error_m          # 가장 가까운 칸 중심까지 거리
bool ok                       # 칸 위 + 그룹 조건 맞음: R = 빨강, G = 초록, Y = 한 모양
