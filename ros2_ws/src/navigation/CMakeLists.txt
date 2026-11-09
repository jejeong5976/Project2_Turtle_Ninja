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

# Nav2 설정은 레포 config/nav2.yaml (bringup 이 share/bringup/config 로 설치) -> launch/nav2.launch.py 가 찾음
install(TARGETS scan_merger pose_guard cmd_vel_hold DESTINATION lib/${PROJECT_NAME})
# RL 지역 계획기 노드 (주행 nf · 정밀 접근 dock, nav2.launch.py local_planner:=rl). 정책 = 레포 config/rl/*.npz (numpy, torch 없음)
install(PROGRAMS scripts/local_planner_node.py scripts/simple_dock_node.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/box_tracker.py scripts/numpy_policy.py scripts/simple_dock_core.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation test/test_scan_merge.cpp test/test_pose_guard.cpp)
  target_include_directories(test_navigation PRIVATE include)
endif()

ament_package()
