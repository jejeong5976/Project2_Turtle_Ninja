// tui.hpp — ncurses 상태 화면 + 명령 입력줄
//   맨 위 = 제목 · 메뉴 (0 전체 요약 · 1 위치·SLAM · 2 팔 · 3 주행·Nav2 · 4 인지 · 5 미션 · 6 진단 · 7 실험)
//   번호 + Enter = 그 쪽 (자세히), Esc 또는 0 = 전체 요약. 가운데 = 쪽 글 (10 Hz), 아래 = 기록 · 입력줄 (모든 쪽에서 같음)
//   teleop: w/s 앞뒤 · a/d 좌우 · q/e 회전 · +/- 속도 · space 정지 · Esc 끝 (키 하나 = 0.6 s 유지)
//   나갈 때 (quit · Ctrl-C) cycle · teleop · drive 를 멈추고, 이 CLI 의 goal 을 취소 (RobotLink::finish) 하고 /cmd_vel 0
#pragma once

#include <atomic>
#include <string>
#include <vector>

#include "robot_cli/robot_link.hpp"

namespace robot_cli
{

int run_tui(RobotLink & link, const std::atomic<bool> & interrupted, const std::vector<std::string> & startup,
  const std::string & mark_dir);

}  // namespace robot_cli
