cmake_minimum_required(VERSION 3.8)
project(perception)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(interfaces REQUIRED)
find_package(OpenCV REQUIRED)

# ONNX Runtime C++ (rosdep 키가 없어서 따로 설치). 기본 위치 /opt/onnxruntime, 다르면 -DONNXRUNTIME_ROOT=<폴더>
set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "onnxruntime 설치 폴더 (include/, lib/)")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include /opt/onnxruntime/include
  PATH_SUFFIXES onnxruntime onnxruntime/core/session)
find_library(ONNXRUNTIME_LIB onnxruntime HINTS ${ONNXRUNTIME_ROOT}/lib /opt/onnxruntime/lib)

# 후처리 (ROS · ONNX Runtime 없이 테스트 가능)
add_library(perception_core STATIC src/postproc.cpp)
set_target_properties(perception_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(perception_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(perception_core ${OpenCV_LIBS})

set(ROS_DEPS rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS}
  cv_bridge::cv_bridge ${interfaces_TARGETS})

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
  message(STATUS "onnxruntime: ${ONNXRUNTIME_LIB}")
  get_filename_component(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_LIB} DIRECTORY)
  # 노드 하나: 광각 + 손목 탐지 + 바코드
  add_executable(perception_node src/perception_node.cpp src/yolo.cpp)
  target_include_directories(perception_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(perception_node perception_core ${ROS_DEPS} ${ONNXRUNTIME_LIB})
  set_target_properties(perception_node PROPERTIES INSTALL_RPATH "${ONNXRUNTIME_LIB_DIR}")
  install(TARGETS perception_node RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "onnxruntime 을 못 찾음 → perception_node 는 빌드 안 함 (테스트만). "
    "설치 후 -DONNXRUNTIME_ROOT=<폴더> 또는 /opt/onnxruntime")
endif()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_postproc test/test_postproc.cpp)
  target_link_libraries(test_postproc perception_core)
endif()

ament_package()
