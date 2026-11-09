cmake_minimum_required(VERSION 3.10)
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
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

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
  src/wrist_rack.cpp
  src/wrist_box.cpp
)
target_include_directories(perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${OpenCV_INCLUDE_DIRS})
target_compile_features(perception_core PUBLIC cxx_std_17)
target_link_libraries(perception_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
set_target_properties(perception_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- perception_barcode: 바코드 판독기 + 노드 단계 (ROS 씀, TensorRT 없이 빌드 → 개발 PC 테스트)
add_library(perception_barcode STATIC src/barcode.cpp)
target_include_directories(perception_barcode PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)   # barcode.hpp · msg_utils.hpp
target_link_libraries(perception_barcode PUBLIC perception_core rclcpp::rclcpp
  ${std_msgs_TARGETS} ${geometry_msgs_TARGETS} ${interfaces_TARGETS})
set_target_properties(perception_barcode PROPERTIES POSITION_INDEPENDENT_CODE ON)
# 바코드: zxing-cpp (Code128) 있으면 씀, 없으면 OpenCV (EAN/UPC 만)
find_package(ZXing QUIET)
if(ZXing_FOUND)
  message(STATUS "바코드: zxing-cpp ${ZXing_VERSION}")
  target_compile_definitions(perception_barcode PUBLIC PERCEPTION_HAVE_ZXING)
  target_link_libraries(perception_barcode PUBLIC ZXing::ZXing)
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
  )
  target_link_libraries(perception_node
    perception_barcode
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${geometry_msgs_TARGETS} ${interfaces_TARGETS}
    message_filters::message_filters
    tf2::tf2
    tf2_ros::tf2_ros
  )
  install(TARGETS perception_node RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "TensorRT 없음 → perception_node 는 빌드 안 함 (perception_core 테스트만)")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t test_wide test_wrist_pose test_wrist_seg test_wrist_rack test_wrist_box)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} perception_core)
  endforeach()
  ament_add_gtest(test_barcode test/test_barcode.cpp)
  target_link_libraries(test_barcode perception_barcode)
  # 메시지 규칙 (msg_utils.hpp) 은 interfaces · geometry_msgs 필요
  ament_add_gtest(test_common test/test_common.cpp)
  target_include_directories(test_common PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)   # msg_utils.hpp 는 노드 내부 헤더 (src/)
  target_link_libraries(test_common perception_core ${interfaces_TARGETS} ${geometry_msgs_TARGETS})
endif()

ament_package()
