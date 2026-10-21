cmake_minimum_required(VERSION 3.8)
project(description)

find_package(ament_cmake REQUIRED)

# 코드 없음. URDF · launch 를 share/ 로 복사만
install(DIRECTORY urdf launch meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
