// auto_initial_pose.cpp — AMCL 초기 자세 자동 맞춤 노드 실행 파일 (scripts/auto_initial_pose.py 의 C++ 판)
// 위치 메시지 (/initialpose) 만 발행하며 주행/FAULT 상태는 바꾸지 않음. 상태: /startup/pose_match_status (JSON)
// 실행: bringup mission_system.launch.py (localization amcl · auto_initial_pose:=true)
#include <cstdio>
#include <exception>
#include <memory>

#include "navigation/auto_initial_pose_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<navigation::AutoInitialPose>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    // 파이썬 판은 예외 traceback 뒤 종료 코드 1
    std::fprintf(stderr, "auto_initial_pose: %s\n", exc.what());
    code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return code;
}
