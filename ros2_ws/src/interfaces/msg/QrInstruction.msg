# QrInstruction.msg — QR 하나를 "무슨 색을 어디로" 로 풀어 놓은 것
# ※ QR 포맷이 아직 미정. 정해지면 raw_text → 아래 칸으로 바꾸는 규칙만 고치면 됨
# 보내는 쪽: perception/perception_master (qr.py)   받는 쪽: mission/planning_master

std_msgs/Header header
string raw_text               # QR 에 적힌 글자 그대로 (디버깅용)
bool valid                    # 포맷에 맞게 풀렸나
string puck_color             # "red" | "green" | "blue"
string target_shelf_id        # 놓을 선반 이름/번호
