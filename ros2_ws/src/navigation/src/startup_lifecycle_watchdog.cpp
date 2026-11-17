// startup_lifecycle_watchdog.cpp — 시작 lifecycle 감시 노드 실행 파일 (scripts/startup_lifecycle_watchdog.py 의 C++ 판)
// AMCL→지도·TF→Nav2 시작과 lifecycle만 최대3회 복구함. 상태: /startup/lifecycle_status (JSON)
// 실행: nav2.launch.py startup_recovery:=true (respawn)
#include <cstdio>
#include <exception>
#include <memory>

#include "navigation/startup_lifecycle_watchdog_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<navigation::StartupLifecycleWatchdog>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    // 파이썬 판은 예외 traceback 뒤 종료 코드 1 (launch 가 respawn)
    std::fprintf(stderr, "startup_lifecycle_watchdog: %s\n", exc.what());
    code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return code;
}
