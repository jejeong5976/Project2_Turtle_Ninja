cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)

# 저장소 공용 설정은 설치 후 share/bringup/config 에서 읽는다.
set(REPOSITORY_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../config")
install(DIRECTORY "${REPOSITORY_CONFIG_DIR}/"
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# 앞 · 뒤 라이다 합치기 (slam.launch.py) — ros2 run bringup scan_merger.py
install(PROGRAMS scripts/scan_merger.py DESTINATION lib/${PROJECT_NAME})

ament_package()
