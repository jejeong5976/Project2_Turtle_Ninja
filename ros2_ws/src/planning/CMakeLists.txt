cmake_minimum_required(VERSION 3.8)
project(planning)

# planning = 실행 층 (미션을 모름): 단계 실행기 · 퍽 기록부 · 적재함 · 감시 · grasp 계산
#   미션 (발제 v5 dice 흐름 · mission_manager 노드) 은 mission 패키지 → 이 패키지 라이브러리를 씀

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---- 순수 계산 (ROS 없음) → 실행기 · 테스트 · mission 이 같이 씀
add_library(planning_core STATIC
  src/puck_registry.cpp
  src/grasp.cpp
  src/cargo.cpp
  src/inventory_policy.cpp
  src/health.cpp
)
set_target_properties(planning_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(planning_core PUBLIC cxx_std_17)
target_link_libraries(planning_core PUBLIC yaml-cpp::yaml-cpp)

# ---- 실행기 (ROS): mission 노드 안에서 같은 rclcpp::Node 로 동작
add_library(planning_exec STATIC src/executor.cpp)
set_target_properties(planning_exec PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(planning_exec PUBLIC
  planning_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${interfaces_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS planning_core planning_exec EXPORT export_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# ---- 테스트 (colcon test --packages-select planning)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planning_core test/test_planning_core.cpp)
  target_link_libraries(test_planning_core planning_core)
endif()

ament_export_targets(export_planning HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action interfaces nav2_msgs nav_msgs diagnostic_msgs geometry_msgs std_msgs
  visualization_msgs sensor_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs yaml-cpp)
ament_package()
