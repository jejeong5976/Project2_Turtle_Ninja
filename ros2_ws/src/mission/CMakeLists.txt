cmake_minimum_required(VERSION 3.10)
project(mission)

# mission = 발제 '주사위 기억 로봇' 미션: 바코드 · 신호등 기록 · dice 계산 · 운반 계획 · 재검증 · 기록 파일
#   노드 mission_manager = 이 패키지
#   실제 동작 (Nav2 · 팔 · 감시 · 기록부 · 재고) 은 planning 패키지 Executor 를 씀 (의존 방향 mission → planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(planning REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# ---- 경기장 파일 기본 위치 = 레포 최상위 maps/ (dice_map.yaml · dice_travel_times.yaml · dice_barcodes.json)
#   - 빌드할 때 소스 위치 기준 절대경로를 코드에 넣음 → 레포 안 ros2_ws 에서 빌드해야 맞음
#   - 다른 곳이면 --cmake-args -DMISSION_MAPS_DIR=<경로> (PLANNING_MAPS_DIR 도 받음)
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
  src/place_box.cpp
  src/barcode_search.cpp
  src/lying_backoff.cpp
)
target_include_directories(mission_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mission_core PUBLIC cxx_std_17)
target_link_libraries(mission_core PUBLIC planning::planning_core yaml-cpp::yaml-cpp)

# ---- 재검증 판정 그림 (OpenCV, ROS 없음)
add_library(mission_image STATIC src/verify_image.cpp)
target_link_libraries(mission_image PUBLIC mission_core ${OpenCV_LIBS})
target_include_directories(mission_image PUBLIC ${OpenCV_INCLUDE_DIRS})

# ---- 노드 mission_manager
add_executable(mission_manager src/mission_manager.cpp)
target_compile_definitions(mission_manager PRIVATE MISSION_MAPS_DIR="${MISSION_MAPS_DIR}")
target_link_libraries(mission_manager
  mission_core
  mission_image
  planning::planning_exec
  rclcpp::rclcpp
  ${interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)

install(TARGETS mission_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ---- 테스트 (colcon test --packages-select mission)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_core test/test_mission_core.cpp)
  target_link_libraries(test_mission_core mission_core)
  target_compile_definitions(test_mission_core PRIVATE TEST_BARCODE_TABLE="${MISSION_MAPS_DIR}/dice_barcodes.json")
  ament_add_gtest(test_barcode_search test/test_barcode_search.cpp)
  target_link_libraries(test_barcode_search mission_core)
  ament_add_gtest(test_lying_backoff test/test_lying_backoff.cpp)
  target_link_libraries(test_lying_backoff mission_core)
  ament_add_gtest(test_transfer test/test_transfer.cpp)
  target_link_libraries(test_transfer mission_core mission_image)
  target_compile_definitions(test_transfer PRIVATE
    TEST_TRAVEL_TABLE="${MISSION_MAPS_DIR}/dice_travel_times.yaml"
    TEST_DICE_MAP="${MISSION_MAPS_DIR}/dice_map.yaml")
endif()

ament_package()
