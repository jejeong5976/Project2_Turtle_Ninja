cmake_minimum_required(VERSION 3.10)
project(bringup)

find_package(ament_cmake REQUIRED)

# 저장소 공용 설정은 설치 후 share/bringup/config 에서 읽는다.
set(REPOSITORY_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../config")
install(DIRECTORY "${REPOSITORY_CONFIG_DIR}/"
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN "standup_paths.txt"
  PATTERN "camera_home_ref.png"   # 손목 카메라 기준 홈 사진 (camera_drift_node)   # 누운 퍽 세우기 경로 표 (control_master stand_lying)
  PATTERN "*.npz"   # RL 주행 정책 (config/rl, navigation nav2.launch.py local_planner:=rl)
  PATTERN "*.json"
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
