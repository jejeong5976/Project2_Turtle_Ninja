# TaskText.msg — /perception/task_text
# 벽 1D 바코드 원문. 해석(task 변환)은 mission 이 한다
# 여러 프레임에서 같은 문자열이 votes 번 이상 나와야 보냄 (한 번 확정한 문자열은 다시 안 보냄)

std_msgs/Header header
string camera                 # 읽은 카메라
string text                   # 바코드 원문
string symbology              # 바코드 종류 (EAN_13, CODE_128 ...). 디코더가 모르면 ""
uint8 votes                   # 확정까지 일치한 프레임 수
