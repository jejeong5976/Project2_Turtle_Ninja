cmake_minimum_required(VERSION 3.16)
project(mapping_route_trial LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
foreach(dependency rclcpp rosbag2_cpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_msgs lifecycle_msgs)
  find_package(${dependency} REQUIRED)
endforeach()
foreach(target run_mapping_route test_mapping_route)
  add_executable(${target} ../${target}.cpp)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_link_libraries(${target} rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp
    ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
    ${tf2_msgs_TARGETS} ${lifecycle_msgs_TARGETS})
endforeach()
