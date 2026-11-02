# VerifyRecord.msg — 세 장소 재검증 결과
#
# 역할
#   - 발제 v5 '이동 후 검증': 새 관측으로 장소별 ID · 위치를 기록하고 전체 중복 · 누락 판정
# 토픽
#   - /planning/verify_record, transient_local
# 보내는 쪽 → 받는 쪽
#   - mission_manager → 대시보드 · rosbag
# note
#   - 장소 하나를 볼 때마다 갱신, 세 장소를 다 보면 complete = true
#   - 같은 내용을 ~/planning_logs 아래 yaml 로도 남김 → log_file

uint8 PENDING=0               # 아직 다 못 봄
uint8 PASS=1                  # 세 장소 모두 개수 · 그룹 · 칸 맞음 + ID 중복 · 누락 없음
uint8 FAIL=2                  # 다 봤는데 하나라도 안 맞음

std_msgs/Header header
uint8 status
bool complete                 # 세 장소 다 봄
string[] places               # 본 장소 순서, "L1" …
string[] groups               # 장소별 그룹 글자 "R" | "Y" | "G", places 와 같은 순서
uint8[] expected              # 장소별 눈 수
uint8[] seen                  # 장소별 본 퍽 수
bool[] place_ok               # 장소별 개수 · 그룹 · 칸 모두 맞음
VerifyPuck[] pucks            # 본 퍽 전부
string[] duplicate_ids        # 두 번 이상 본 ID
string[] missing_ids          # 0–11 중 어디서도 못 본 ID, 세 장소 다 본 뒤에만
string[] issues               # 사람이 읽을 문제 목록
string log_file               # 기록 파일 경로, 없으면 ""
