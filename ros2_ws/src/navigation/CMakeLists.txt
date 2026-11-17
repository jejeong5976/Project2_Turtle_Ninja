cmake_minimum_required(VERSION 3.10)
project(navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_executable(scan_merger src/scan_merger.cpp)
target_include_directories(scan_merger PRIVATE include)
target_compile_features(scan_merger PRIVATE cxx_std_17)
target_link_libraries(scan_merger rclcpp::rclcpp tf2::tf2 tf2_ros::tf2_ros ${sensor_msgs_TARGETS})

add_executable(pose_guard src/pose_guard.cpp)
target_include_directories(pose_guard PRIVATE include)
target_compile_features(pose_guard PRIVATE cxx_std_17)
target_link_libraries(pose_guard rclcpp::rclcpp tf2::tf2 tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} ${diagnostic_msgs_TARGETS})

# /mission/hold 를 받아 Nav2 속도 명령을 막는 문 (collision_monitor → cmd_vel_hold → /cmd_vel)
add_executable(cmd_vel_hold src/cmd_vel_hold.cpp)
target_compile_features(cmd_vel_hold PRIVATE cxx_std_17)
target_link_libraries(cmd_vel_hold rclcpp::rclcpp ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})

# RL 지역 계획기: 계산 (ROS 없음, 시험 대상) + 노드 (주행 nf · 정밀 접근 dock, nav2.launch.py local_planner:=rl)
#   정책 = 레포 config/rl/*.npz (export.py, numpy 형식). sim (Isaac) 은 같은 식의 파이썬 판 scripts/local_planner_node.py 를 씀
add_library(navigation_rl_core STATIC src/rl_core.cpp)
target_include_directories(navigation_rl_core PUBLIC include)
target_compile_features(navigation_rl_core PUBLIC cxx_std_17)
target_link_libraries(navigation_rl_core PUBLIC yaml-cpp::yaml-cpp)

# 규칙 정밀 접근 계산 (FaceTrack 은 rl_dock 박스 맞춤도 씀)
add_library(navigation_dock_core STATIC src/simple_dock_core.cpp)
target_link_libraries(navigation_dock_core PUBLIC navigation_rl_core)

add_executable(local_planner_node src/local_planner_node.cpp)
target_link_libraries(local_planner_node navigation_dock_core rclcpp::rclcpp rclcpp_action::rclcpp_action tf2::tf2 tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${nav2_msgs_TARGETS} ${sensor_msgs_TARGETS} ${rcl_interfaces_TARGETS})

# 규칙 정밀 접근 액션 노드임
add_executable(simple_dock_node src/simple_dock_node.cpp)
target_link_libraries(simple_dock_node navigation_dock_core rclcpp::rclcpp rclcpp_action::rclcpp_action tf2::tf2 tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${nav2_msgs_TARGETS} ${sensor_msgs_TARGETS} ${rcl_interfaces_TARGETS})

# localization:=amcl: 서 있는 동안 AMCL request_nomotion_update (켤 때 · /initialpose 뒤) — 출발 자리 1–2° 틀어 놓아도 STANDBY 전에 맞춤
add_executable(amcl_settle src/amcl_settle.cpp)
target_compile_features(amcl_settle PRIVATE cxx_std_17)
target_link_libraries(amcl_settle rclcpp::rclcpp ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_srvs_TARGETS}
  ${lifecycle_msgs_TARGETS})   # /amcl/transition_event (다시 켜지면 0,0,0 대신 꺼지기 전 자세)

# 시작 보조 (파이썬 scripts/auto_initial_pose.py · startup_lifecycle_watchdog.py 의 C++ 판, 같은 이름·파라미터·토픽·문구)
#   계산 (ROS 없음, 시험 대상): 지도 맞춤 · lifecycle 복구 순서. FMA 합침 끔 (파이썬 numpy 와 같은 반올림 순서, Jetson aarch64 기본은 합침)
add_library(navigation_startup_core STATIC src/pose_match_core.cpp src/lifecycle_recovery_core.cpp)
target_include_directories(navigation_startup_core PUBLIC include)
target_compile_features(navigation_startup_core PUBLIC cxx_std_17)
target_compile_options(navigation_startup_core PRIVATE -O2 -ffp-contract=off)

add_executable(auto_initial_pose src/auto_initial_pose.cpp)
target_compile_options(auto_initial_pose PRIVATE -ffp-contract=off)
target_link_libraries(auto_initial_pose navigation_startup_core rclcpp::rclcpp tf2::tf2 tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS} ${interfaces_TARGETS})

add_executable(startup_lifecycle_watchdog src/startup_lifecycle_watchdog.cpp)
target_link_libraries(startup_lifecycle_watchdog navigation_startup_core rclcpp::rclcpp tf2::tf2 tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS}
  ${nav2_msgs_TARGETS} ${lifecycle_msgs_TARGETS})

# Nav2 설정은 레포 config/nav2.yaml (bringup 이 share/bringup/config 로 설치) -> launch/nav2.launch.py 가 찾음
install(TARGETS scan_merger pose_guard cmd_vel_hold local_planner_node simple_dock_node amcl_settle
  auto_initial_pose startup_lifecycle_watchdog DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
# 파이썬 판도 그대로 설치 (launch 는 C++ 판을 씀. 되돌리기 = launch executable 이름만 .py 로)
install(PROGRAMS scripts/auto_initial_pose.py scripts/startup_lifecycle_watchdog.py scripts/startup_report.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/pose_match_core.py scripts/lifecycle_recovery_core.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_startup_python test/test_auto_initial_pose.py TIMEOUT 180)
  ament_add_pytest_test(test_auto_initial_pose_runtime test/test_auto_initial_pose_runtime.py TIMEOUT 90)
  ament_add_pytest_test(test_lifecycle_python test/test_lifecycle_recovery.py TIMEOUT 120)
  ament_add_pytest_test(test_startup_report test/test_startup_report.py TIMEOUT 15)
  # nav2.launch.py startup_recovery (true = autostart + manager respawn + 감시기 복구만)
  ament_add_pytest_test(test_startup_launch test/test_startup_launch.py TIMEOUT 30)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation test/test_scan_merge.cpp test/test_pose_guard.cpp)
  target_include_directories(test_navigation PRIVATE include)
  ament_add_gtest(test_rl_core test/test_rl_core.cpp)
  target_link_libraries(test_rl_core navigation_rl_core)
  # 레포 config/rl 정책 · cfg 를 그대로 읽어 봄
  target_compile_definitions(test_rl_core PRIVATE RL_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/rl")
  # 주행 노드 덧붙임: 막힘 기다리기 · 제자리 회전 반전 제한 (nav_guard.hpp)
  ament_add_gtest(test_nav_guard test/test_nav_guard.cpp)
  target_link_libraries(test_nav_guard navigation_rl_core)
  ament_add_gtest(test_simple_dock test/test_simple_dock.cpp)
  target_link_libraries(test_simple_dock navigation_dock_core)
  # 시작 보조 C++ 판: 파이썬 test_auto_initial_pose.py · test_lifecycle_recovery.py 를 옮김
  # recover_only (autostart 먼저, 실패한 manager 만 복구) · 프로세스 찾기 · 구독 다시 만들기 (test_startup_recover_only.cpp)
  ament_add_gtest(test_startup_core test/test_pose_match_core.cpp test/test_lifecycle_recovery_core.cpp
    test/test_startup_recover_only.cpp TIMEOUT 180)
  target_compile_options(test_startup_core PRIVATE -ffp-contract=off)
  target_link_libraries(test_startup_core navigation_startup_core)
  # test_auto_initial_pose_runtime.py 를 옮김 (rclcpp 노드, 장비 없음)
  ament_add_gtest(test_auto_initial_pose_node test/test_auto_initial_pose_node.cpp TIMEOUT 90)
  target_compile_options(test_auto_initial_pose_node PRIVATE -ffp-contract=off)
  target_link_libraries(test_auto_initial_pose_node navigation_startup_core rclcpp::rclcpp tf2::tf2 tf2_ros::tf2_ros
    ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS} ${interfaces_TARGETS})
endif()

ament_package()
