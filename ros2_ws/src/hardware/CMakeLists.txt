cmake_minimum_required(VERSION 3.10)
project(hardware)
# realtime_tools 가 부르는 find_package(Boost): CMake 3.30+ 의 CMP0167 경고 → 지금 동작 (FindBoost) 그대로
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

# 빌드 유형을 안 주면 최적화(Release) — 측정: 100 Hz 주기 계산이 -O0 대비 3~4 배 빠름 (최적화 없음이 기본이었음)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ★ 편지 규칙 opencr_packet.h 는 펌웨어 폴더에 원본이 하나만 있음 (펌웨어와 같은 파일을 씀 → 어긋날 수 없음)
#   저장소 구조: <repo>/firmware/opencr/opencr_packet.h,  <repo>/ros2_ws/src/hardware (여기)
#   ros2_ws 만 따로 복사했다면: colcon build --cmake-args -DNT_FIRMWARE_DIR=/경로/firmware/opencr
set(OPENCR_FIRMWARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../firmware/opencr"
  CACHE PATH "opencr_packet.h 가 있는 폴더 (firmware/opencr)")
if(NOT EXISTS "${OPENCR_FIRMWARE_DIR}/opencr_packet.h")
  message(FATAL_ERROR "opencr_packet.h 를 못 찾음: ${OPENCR_FIRMWARE_DIR}\n"
    "저장소 전체(firmware 폴더 포함)를 받았는지, 아니면 -DNT_FIRMWARE_DIR=... 로 알려 줄 것")
endif()

# ① 편지 주고받기 (ROS 없음) — 플러그인과 테스트가 같이 씀
add_library(opencr_link STATIC src/opencr_link.cpp)
target_include_directories(opencr_link PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${OPENCR_FIRMWARE_DIR}>)
target_compile_features(opencr_link PUBLIC cxx_std_17)
set_target_properties(opencr_link PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ② ros2_control 플러그인 (controller_manager 가 이름 "hardware/OpenCRHardware" 로 불러 씀)
add_library(hardware SHARED src/opencr_hardware.cpp src/mock_dynamics.cpp)   # mock_dynamics: 속도 · 가속 한계 가짜 모터
target_link_libraries(hardware PUBLIC
  opencr_link
  ${diagnostic_msgs_TARGETS}
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)

install(TARGETS hardware opencr_link
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ③ 단위 테스트 (로봇 없이): colcon test --packages-select hardware
#    test_opencr : 편지 규칙 (펌웨어와 같은 파일) + 가짜 OpenCR(pty) 로 악수 → 명령 → 상태, 단위 변환, 끊김
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_opencr test/test_opencr.cpp)
  target_link_libraries(test_opencr opencr_link util)
  ament_add_gtest(test_position_rate test/test_position_rate.cpp)
  target_link_libraries(test_position_rate opencr_link)
endif()

ament_export_dependencies(diagnostic_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
