# SignalState.msg — /perception/signals
# 신호등(타워램프) 상태. 칸별 밝기 비교 + 최근 프레임 다수결로 확정한 값
# 신호등이 화면에 없으면 state = UNKNOWN 으로 계속 보냄 (planning 이 "못 봄" 과 "꺼짐" 을 구분하도록)

string RED="red"
string ORANGE="orange"
string GREEN="green"
string OFF="off"
string UNKNOWN="unknown"

std_msgs/Header header
string camera
string state
float32 confidence            # 다수결 창에서 이 상태가 차지한 비율 0~1
float32 cx                    # 신호등 bbox 중심 (픽셀). 못 봤으면 -1
float32 cy
