cmake_minimum_required(VERSION 3.10)
project(perception)
# 빌드 종류를 안 주면 Release: 최적화 없이 빌드하면 손목 처리 (분할 후처리 · 자세 · 박스 넓게 찾기) 가 몇 배 느려 실시간 주기 · 시간 상한을 넘음
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
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
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# ---- perception_core: 전처리 · 후처리 · 기하 (ROS 없음) → 개발 PC 에서 테스트
#   공개 헤더 include/perception/*.hpp (접두어 wide_ = 광각, wrist_ = 손목), 노드 내부 헤더는 src/
#   TensorRT 있으면 (Jetson) trt_engine.cpp 를 더하고 PERCEPTION_HAVE_TRT 를 켬 → BoxDetector · SegModel 구현이 들어감
add_library(perception_core STATIC
  src/common.cpp
  src/wide_detector.cpp
  src/wide_tag_reader.cpp
  src/traffic_light.cpp
  src/wrist_pose.cpp
  src/wrist_tracker.cpp
  src/wrist_seg.cpp
  src/wrist_tag_reader.cpp
  src/wrist_box.cpp
  src/wrist_tag_rescue.cpp
  src/wrist_held.cpp
  src/hang_watch.cpp
)
target_include_directories(perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${OpenCV_INCLUDE_DIRS})
target_compile_features(perception_core PUBLIC cxx_std_17)
target_link_libraries(perception_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS} Threads::Threads ${CMAKE_DL_LIBS})
set_target_properties(perception_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
# AprilTag 3 (ros-<distro>-apriltag) 있으면 손목 태그 단계 (wrist_tag_reader, 적재함 칸 · 쥔 퍽) · 손목 태그 다시 찾기 (wrist_tag_rescue) 에 씀
#   aruco 가 못 읽는 작은 퍽 태그를 CLAHE + AprilTag 3 는 읽음. 없으면 aruco 만
find_package(apriltag QUIET)
if(apriltag_FOUND)
  message(STATUS "손목 태그 다시 찾기: AprilTag 3")
  target_link_libraries(perception_core PRIVATE apriltag::apriltag)
  target_compile_definitions(perception_core PRIVATE PERCEPTION_HAVE_APRILTAG)
else()
  message(WARNING "apriltag 없음 → 손목 태그 다시 찾기는 OpenCV aruco 만. sudo apt install ros-$ENV{ROS_DISTRO}-apriltag")
endif()

# ---- perception_barcode: 바코드 판독기 + 노드 단계 (ROS 씀, TensorRT 없이 빌드 → 개발 PC 테스트)
add_library(perception_barcode STATIC src/barcode.cpp)
target_include_directories(perception_barcode PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)   # barcode.hpp · msg_utils.hpp
target_link_libraries(perception_barcode PUBLIC perception_core rclcpp::rclcpp
  ${std_msgs_TARGETS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${interfaces_TARGETS})
set_target_properties(perception_barcode PROPERTIES POSITION_INDEPENDENT_CODE ON)
# 바코드: zxing-cpp (Code128) 있으면 씀, 없으면 OpenCV (EAN/UPC 만)
find_package(ZXing QUIET)
if(ZXing_FOUND)
  message(STATUS "바코드: zxing-cpp ${ZXing_VERSION}")
  target_compile_definitions(perception_barcode PUBLIC PERCEPTION_HAVE_ZXING)
  target_link_libraries(perception_barcode PUBLIC ZXing::ZXing)
  # 이 zxing 판에 Barcode::position() 꼭짓점 (p[i].x) 이 있으면 읽은 바코드 자리를 zxing 으로 (없으면 OpenCV 자리 찾기만)
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_LIBRARIES ZXing::ZXing)
  check_cxx_source_compiles("
#include <ZXing/ReadBarcode.h>
int main() {
  const unsigned char px[4] = {0, 0, 0, 0};
  ZXing::ImageView iv(px, 2, 2, ZXing::ImageFormat::Lum);
  for (const auto & b : ZXing::ReadBarcodes(iv, ZXing::ReaderOptions())) {const auto & p = b.position(); int x = p[0].x + p[2].y; (void)x;}
  return 0;
}" PERCEPTION_ZXING_POSITION)
  unset(CMAKE_REQUIRED_LIBRARIES)
  if(PERCEPTION_ZXING_POSITION)
    target_compile_definitions(perception_barcode PUBLIC PERCEPTION_ZXING_POSITION)
  endif()
else()
  message(WARNING "zxing-cpp 없음 → 바코드는 OpenCV (EAN/UPC 만, 벽 바코드 Code128 못 읽음). sudo apt install libzxing-dev")
endif()

# TensorRT (Jetson JetPack) 있을 때만: 엔진 실행 + perception_node
find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATHS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/local/TensorRT/include)
find_library(TENSORRT_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/TensorRT/lib)
find_package(CUDAToolkit QUIET)
if(TENSORRT_INCLUDE_DIR AND TENSORRT_LIB AND CUDAToolkit_FOUND)
  message(STATUS "TensorRT: ${TENSORRT_LIB}")
  target_sources(perception_core PRIVATE src/trt_engine.cpp)
  target_compile_definitions(perception_core PUBLIC PERCEPTION_HAVE_TRT)
  target_include_directories(perception_core PRIVATE ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(perception_core PUBLIC ${TENSORRT_LIB} CUDA::cudart)

  # 노드 하나: 광각 · 손목 · 신호등 · 바코드
  add_executable(perception_node
    src/perception_node.cpp
    src/wide_pipeline.cpp
    src/wrist_pipeline.cpp
    src/wrist_tag_stage.cpp
  )
  target_link_libraries(perception_node
    perception_barcode
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${geometry_msgs_TARGETS} ${interfaces_TARGETS}
    message_filters::message_filters
    tf2::tf2
    tf2_ros::tf2_ros
  )
  # 멈춤 감시 기록: 실행 파일 함수 이름도 신호 스택 (dladdr) 에 나오게 -rdynamic
  set_target_properties(perception_node PROPERTIES ENABLE_EXPORTS ON)
  install(TARGETS perception_node RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "TensorRT 없음 → perception_node 는 빌드 안 함 (perception_core 테스트만)")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t test_wide test_wrist_pose test_wrist_seg test_wrist_box test_wrist_tag_rescue test_wrist_tag_reader)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} perception_core)
  endforeach()
  # 마스크 자세: 실물 LOOK 장면 (test/data/look_*: 정렬 깊이 mm PNG · 분할 마스크)
  target_compile_definitions(test_wrist_pose PRIVATE WRIST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  # 쥔 퍽 마스크 재기: 실물 분할 마스크 (test/data/held_*_mask.png)
  ament_add_gtest(test_wrist_held test/test_wrist_held.cpp)
  target_link_libraries(test_wrist_held perception_core)
  target_compile_definitions(test_wrist_held PRIVATE HELD_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  # 영상 끊김 → 구독 다시 만들기 규칙 (input_watch.hpp, 헤더만)
  ament_add_gtest(test_input_watch test/test_input_watch.cpp)
  target_link_libraries(test_input_watch perception_core)
  # 멈춤 감시 (하트비트 · 스택 기록) · respawn 상태 되살리기 (ROS 없음)
  ament_add_gtest(test_hang_watch test/test_hang_watch.cpp TIMEOUT 120)
  target_link_libraries(test_hang_watch perception_core)
  set_target_properties(test_hang_watch PROPERTIES ENABLE_EXPORTS ON)
  ament_add_gtest(test_barcode test/test_barcode.cpp)
  target_link_libraries(test_barcode perception_barcode)
  # 메시지 규칙 (msg_utils.hpp) 은 interfaces · geometry_msgs 필요
  ament_add_gtest(test_common test/test_common.cpp)
  target_include_directories(test_common PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)   # msg_utils.hpp 는 노드 내부 헤더 (src/)
  target_link_libraries(test_common perception_core ${interfaces_TARGETS} ${geometry_msgs_TARGETS})
endif()

ament_package()
