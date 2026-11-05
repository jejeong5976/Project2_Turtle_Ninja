// standup_table.hpp — 누운 네모 퍽 세우기 경로 표 (config/standup_paths.txt) 읽기 · 가장 가까운 경로 고르기 (순수 C++, test/test_standup_table.cpp)
// =====================================================================
// 표 = 퍽 자리 (한 변 · 중심 x y · 긴축 방향) 마다 Isaac 시뮬에서 강화학습 정책을 돌려 뽑은 관절 경로
//   (robot_sim_rl/rl/standup_openloop.py record → export_standup_table.py). 실물은 움직이는 동안 퍽을 못 재서
//   LOOK 으로 잰 퍽에 가장 가까운 칸의 경로를 그대로 재생 (시뮬 재생 시험: 퍽 ±4 mm · ±4° 어긋나도 87 % 섬)
// 한 줄 = 경로 하나: side x y axis rate n release end_x end_y end_face  above[5] grasp[5] q[n × 5] (팀 명령 틱, 처짐 앞먹임 없음)
#pragma once

#include <array>
#include <string>
#include <vector>

namespace manipulation
{

struct StandupPath
{
  double side{0.0};           // 누운 네모 퍽 한 변 [m]
  double x{0.0}, y{0.0};      // 누운 퍽 중심 [m, base_link]
  double axis_deg{0.0};       // 긴축 방향 [°] (180° 마다 같음)
  double rate{-1.0};          // 시뮬 재생 시험 성공률 (모르면 -1)
  int release{0};             // 이 점 (q 번호) 에 도착하면 그리퍼를 엶
  double end_x{0.0}, end_y{0.0};   // 시뮬에서 퍽이 선 자리 [m, base_link]
  double end_face_deg{0.0};   // 선 퍽 면 방향 [°]
  std::array<int, 5> above{}, grasp{};   // 퍽 위 40 mm (열림) · 쥐기 (−20 mm) [팀 명령 틱]
  std::vector<std::array<int, 5>> q;     // 세우기 경로 [팀 명령 틱]
};

// 표 파일을 읽음. '#' 줄 · 빈 줄은 건너뜀. 실패하면 false (why)
bool load_standup_table(const std::string & file, std::vector<StandupPath> & out, std::string & why);

// (x, y, 긴축, 한 변) 에 가장 가까운 경로. 한 변은 가장 가까운 크기만, 거리 = (dx / step_x)² + (dy / step_y)² + (d각 / step_deg)²
//   d각 은 180° 마다 같음. 없으면 -1. dx · dy · dyaw 에 차이를 돌려줌 (실제 − 표)
int nearest_standup(const std::vector<StandupPath> & table, double x, double y, double axis_deg, double side,
  double & dx, double & dy, double & dyaw_deg, double step_x = 0.0075, double step_y = 0.01, double step_deg = 7.5);

}  // namespace manipulation
