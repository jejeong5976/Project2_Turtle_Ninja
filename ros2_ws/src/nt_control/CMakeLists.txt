cmake_minimum_required(VERSION 3.8)
project(nt_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nt_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ① 순수 계산 라이브러리 (ROS 없음) — 노드 · 컨트롤러 · 테스트가 같이 씀
#    swerve.cpp : 바퀴 (기구학 · 자이로 · 끊김 · 차례 · 가속 · 조향 먼저 · odom)
#    arm.cpp    : 팔 (동작 순서표 · 차례 정하기 · 팔 작업 진행)
add_library(nt_control_core STATIC src/swerve.cpp src/arm.cpp)
set_target_properties(nt_control_core PROPERTIES POSITION_INDEPENDENT_CODE ON)   # 아래 플러그인(.so)에 넣으려고
target_include_directories(nt_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nt_control_core PUBLIC cxx_std_17)

# ② ROS 노드 control_master — 팔 동작 + 차례 (바퀴는 ③ 컨트롤러 안에서, D8)
add_executable(control_master src/control_master.cpp)
# (ament_target_dependencies 는 최신 배포판에서 사라지는 중이라 "대상(target)" 이름으로 직접 연결)
target_link_libraries(control_master
  nt_control_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${control_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${controller_manager_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nt_interfaces_TARGETS})

# ③ ros2_control 컨트롤러 플러그인 "nt_control/SwerveController" — 바퀴 (100 Hz 제어 루프 안)
add_library(nt_swerve_controller SHARED src/swerve_controller.cpp)
target_link_libraries(nt_swerve_controller PUBLIC
  nt_control_core
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nt_interfaces_TARGETS})
target_include_directories(nt_swerve_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
pluginlib_export_plugin_description_file(controller_interface nt_control_plugins.xml)

install(TARGETS nt_control_core nt_swerve_controller control_master
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# ④ 단위 테스트: colcon test --packages-select nt_control
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_swerve test/test_swerve.cpp)
  target_link_libraries(test_swerve nt_control_core)
  ament_add_gtest(test_arm test/test_arm.cpp)
  target_link_libraries(test_arm nt_control_core)
  ament_add_gtest(test_swerve_controller test/test_swerve_controller.cpp)
  target_link_libraries(test_swerve_controller
    controller_interface::controller_interface pluginlib::pluginlib
    rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle ${geometry_msgs_TARGETS})
endif()

ament_package()
