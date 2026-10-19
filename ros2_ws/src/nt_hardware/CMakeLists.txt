cmake_minimum_required(VERSION 3.8)
project(nt_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ★ 편지 규칙 nt_packet.h 는 펌웨어 폴더에 원본이 하나만 있음 (펌웨어와 같은 파일을 씀 → 어긋날 수 없음)
#   저장소 구조: <repo>/firmware/nt_opencr/nt_packet.h,  <repo>/ros2_ws/src/nt_hardware (여기)
#   ros2_ws 만 따로 복사했다면: colcon build --cmake-args -DNT_FIRMWARE_DIR=/경로/firmware/nt_opencr
set(NT_FIRMWARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../firmware/nt_opencr"
  CACHE PATH "nt_packet.h 가 있는 폴더 (firmware/nt_opencr)")
if(NOT EXISTS "${NT_FIRMWARE_DIR}/nt_packet.h")
  message(FATAL_ERROR "nt_packet.h 를 못 찾음: ${NT_FIRMWARE_DIR}\n"
    "저장소 전체(firmware 폴더 포함)를 받았는지, 아니면 -DNT_FIRMWARE_DIR=... 로 알려 줄 것")
endif()

# ① 편지 주고받기 (ROS 없음) — 플러그인과 테스트가 같이 씀
add_library(nt_opencr_link STATIC src/opencr_link.cpp)
target_include_directories(nt_opencr_link PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${NT_FIRMWARE_DIR}>)
target_compile_features(nt_opencr_link PUBLIC cxx_std_17)
set_target_properties(nt_opencr_link PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ② ros2_control 플러그인 (controller_manager 가 이름 "nt_hardware/OpenCRHardware" 로 불러 씀)
add_library(nt_hardware SHARED src/opencr_hardware.cpp)
target_link_libraries(nt_hardware PUBLIC
  nt_opencr_link
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface nt_hardware.xml)

install(TARGETS nt_hardware nt_opencr_link
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ③ 단위 테스트 (로봇 없이): colcon test --packages-select nt_hardware
#    test_opencr : 편지 규칙 (펌웨어와 같은 파일) + 가짜 OpenCR(pty) 로 악수 → 명령 → 상태, 단위 변환, 끊김
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_opencr test/test_opencr.cpp)
  target_link_libraries(test_opencr nt_opencr_link util)
  ament_add_gtest(test_position_rate test/test_position_rate.cpp)
  target_link_libraries(test_position_rate nt_opencr_link)
endif()

ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
