cmake_minimum_required(VERSION 3.16)
project(mapping_motion_trial LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
foreach(dependency rclcpp rosbag2_cpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_msgs lifecycle_msgs)
  find_package(${dependency} REQUIRED)
endforeach()
add_executable(run_mapping_motion ../run_mapping_motion.cpp)
target_compile_options(run_mapping_motion PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(run_mapping_motion rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
  ${tf2_msgs_TARGETS} ${lifecycle_msgs_TARGETS})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../test_mapping_motion.cpp")
  add_executable(test_mapping_motion ../test_mapping_motion.cpp)
  target_compile_options(test_mapping_motion PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_link_libraries(test_mapping_motion rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp
    ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
    ${tf2_msgs_TARGETS} ${lifecycle_msgs_TARGETS})
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../review_mapping_motion.cpp")
  add_executable(review_mapping_motion ../review_mapping_motion.cpp)
  target_compile_options(review_mapping_motion PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_link_libraries(review_mapping_motion rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp
    ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
    ${tf2_msgs_TARGETS})
endif()
