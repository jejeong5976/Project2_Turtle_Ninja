# TrafficLight.msg — 신호등 최종 상태
#
# 역할
#   - perception 이 판정 · 안정화까지 끝낸 신호등 상태 하나, 신호등은 1 개
# 토픽
#   - /perception/traffic_light
# 보내는 쪽 → 받는 쪽
#   - perception_node → mission_manager · signal_watch
# note
#   - 광각 영상을 처리할 때마다 발행, wide.max_hz 이하 → 끊기면 planning 이 perception 끊김으로 처리
#   - 이번 영상에서 신호등을 못 찾으면 이전 색을 내지 않고 UNKNOWN + visible=false
#   - OFF 는 발제 v5 점멸 기록에 필요함, 소등으로 점멸 하나를 구별

uint8 UNKNOWN=0               # 판정 못 함, 이유는 visible 로 구분
uint8 RED=1
uint8 YELLOW=2
uint8 GREEN=3
uint8 OFF=4                   # 신호등이 보이고 불이 모두 꺼짐

std_msgs/Header header        # stamp = 판단에 쓴 마지막 영상의 촬영 시각
uint8 state
bool visible                  # 신호등 상자를 찾았는가
                              #   OFF     + true  = 찾았고 불이 모두 꺼짐
                              #   UNKNOWN + true  = 찾았는데 색 판정이 애매함
                              #   UNKNOWN + false = 못 찾음, 가려짐 · 시야 밖 · 인식 실패
