cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)

# 코드 없음. 로봇을 띄우는 launch 와 컨트롤러 설정(controllers.yaml)만
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
