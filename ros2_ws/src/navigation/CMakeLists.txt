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

add_executable(local_planner_node src/local_planner_node.cpp)
target_link_libraries(local_planner_node navigation_rl_core rclcpp::rclcpp rclcpp_action::rclcpp_action tf2::tf2 tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${nav2_msgs_TARGETS} ${sensor_msgs_TARGETS} ${rcl_interfaces_TARGETS})

# 규칙 정밀 접근 계산과 액션 노드임
add_library(navigation_dock_core STATIC src/simple_dock_core.cpp)
target_link_libraries(navigation_dock_core PUBLIC navigation_rl_core)
add_executable(simple_dock_node src/simple_dock_node.cpp)
target_link_libraries(simple_dock_node navigation_dock_core rclcpp::rclcpp rclcpp_action::rclcpp_action tf2::tf2 tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${nav2_msgs_TARGETS} ${sensor_msgs_TARGETS} ${rcl_interfaces_TARGETS})

# Nav2 설정은 레포 config/nav2.yaml (bringup 이 share/bringup/config 로 설치) -> launch/nav2.launch.py 가 찾음
install(TARGETS scan_merger pose_guard cmd_vel_hold local_planner_node simple_dock_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation test/test_scan_merge.cpp test/test_pose_guard.cpp)
  target_include_directories(test_navigation PRIVATE include)
  ament_add_gtest(test_rl_core test/test_rl_core.cpp)
  target_link_libraries(test_rl_core navigation_rl_core)
  # 레포 config/rl 정책 · cfg 를 그대로 읽어 봄
  target_compile_definitions(test_rl_core PRIVATE RL_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/rl")
  ament_add_gtest(test_simple_dock test/test_simple_dock.cpp)
  target_link_libraries(test_simple_dock navigation_dock_core)
endif()

ament_package()
