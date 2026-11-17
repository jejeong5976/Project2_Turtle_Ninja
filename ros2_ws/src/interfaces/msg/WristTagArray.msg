# WristTagArray.msg — 손목 영상 한 장의 태그 전부 (/perception/wrist/tags)
#
# 역할
#   - perception 손목 태그 단계가 켜져 있을 때 (/perception/wrist_tags/enable) 처리한 장마다 하나, 태그 없으면 빈 배열
# note
#   - header.stamp = 촬영 시각, header.frame_id = 손목 카메라 광학 프레임 (URDF 이름, perception wrist.frame_id)

std_msgs/Header header
WristTag[] tags
