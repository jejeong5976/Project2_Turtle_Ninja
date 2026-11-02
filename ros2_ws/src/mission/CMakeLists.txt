cmake_minimum_required(VERSION 3.8)
project(mission)

# mission = 발제 v5 '주사위 기억 로봇' 미션: 바코드 · 신호등 기록 · dice 계산 · 운반 계획 · 재검증 · 기록 파일
#   노드 mission_manager (2026-11-02 옛 planning_master 에서 바꿈, 토픽 · 서비스 이름은 그대로) = 이 패키지
#   실제 동작 (Nav2 · 팔 · 감시 · 기록부 · 재고) 은 planning 패키지 Executor 를 씀 (의존 방향 mission → planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(planning REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---- 경기장 파일 기본 위치 = 레포 최상위 maps/ (dice_map.yaml · dice_travel_times.yaml)
#   - 빌드할 때 소스 위치 기준 절대경로를 코드에 넣음 → 레포 안 ros2_ws 에서 빌드해야 맞음
#   - 다른 곳이면 --cmake-args -DMISSION_MAPS_DIR=<경로> (예전 이름 PLANNING_MAPS_DIR 도 받음)
if(DEFINED PLANNING_MAPS_DIR AND NOT DEFINED MISSION_MAPS_DIR)
  set(MISSION_MAPS_DIR "${PLANNING_MAPS_DIR}")
endif()
set(MISSION_MAPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../maps" CACHE PATH "dice_map.yaml 이 있는 폴더")
get_filename_component(MISSION_MAPS_DIR "${MISSION_MAPS_DIR}" ABSOLUTE)

# ---- 순수 계산 (ROS 없음) → 노드 · 테스트가 같이 씀
add_library(mission_core STATIC
  src/task_parser.cpp
  src/signal_recorder.cpp
  src/dice.cpp
  src/dice_map.cpp
  src/dice_steps.cpp
  src/dice_verify.cpp
  src/dice_record.cpp
  src/transfer_solver.cpp
  src/transfer_planner.cpp
)
target_include_directories(mission_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mission_core PUBLIC cxx_std_17)
target_link_libraries(mission_core PUBLIC planning::planning_core yaml-cpp::yaml-cpp)

# ---- 노드 mission_manager
add_executable(mission_manager src/mission_manager.cpp)
target_compile_definitions(mission_manager PRIVATE MISSION_MAPS_DIR="${MISSION_MAPS_DIR}")
target_link_libraries(mission_manager
  mission_core
  planning::planning_exec
  rclcpp::rclcpp
  ${interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

install(TARGETS mission_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ---- 테스트 (colcon test --packages-select mission)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_core test/test_mission_core.cpp)
  target_link_libraries(test_mission_core mission_core)
  ament_add_gtest(test_transfer test/test_transfer.cpp)
  target_link_libraries(test_transfer mission_core)
  target_compile_definitions(test_transfer PRIVATE
    TEST_TRAVEL_TABLE="${MISSION_MAPS_DIR}/dice_travel_times.yaml"
    TEST_DICE_MAP="${MISSION_MAPS_DIR}/dice_map.yaml")
endif()

ament_package()
