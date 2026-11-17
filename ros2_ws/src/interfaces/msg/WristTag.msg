# WristTag.msg — 손목 카메라 영상 한 장에서 읽은 AprilTag 하나 (tag36h11)
#
# 역할
#   - perception 이 영상 → 태그 ID · 모서리까지만 냄. 위치 (적재함 칸 · 쥔 퍽) 계산은 받는 쪽 (control_master) 이 함
# 보내는 쪽 → 받는 쪽
#   - perception_node (/perception/wrist/tags, WristTagArray) → control_master (적재함 칸 · 쥔 퍽 태그)
# note
#   - 모서리 순서 = 좌상 · 우상 · 우하 · 좌하 (OpenCV aruco 순서, AprilTag 3 는 바꿔서 넣음)
#   - corners_norm = 왜곡 푼 정규 좌표 (x/z, y/z). camera_info 의 ROI · 비닝을 풀고 전체 해상도 K · D 로 풂

int32 id
float32[8] corners_px      # 화소 (u0, v0, u1, v1, u2, v2, u3, v3)
float64[8] corners_norm    # 정규 좌표 (x0, y0, …, x3, y3)
bool rescue                # true = 원본 영상 OpenCV aruco 다시 찾기 (쥔 퍽 태그, 본 검출에서 가까운 퍽 태그를 못 찾았을 때만)
