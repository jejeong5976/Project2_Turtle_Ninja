// det_pick.hpp — 손목 인지 검출 하나로 PICK 목표 만들기 (pick det · pick tag 명령)
//   planning 실행기 (executor to_grasp_point) 와 같은 규칙: planning_core 의 집기 축 · 집기 점 · 목표 방향을 그대로 씀
//   검출은 카메라 광학 프레임 → 부르는 쪽이 TF (base_link ← 카메라) 를 줌. 순수 계산 (test/test_det_pick.cpp)
#pragma once

#include <string>
#include <vector>

#include "robot_cli/snapshot.hpp"

namespace robot_cli
{

// 그리퍼 중심 = 퍽 중심 + 쥘 끝 방향 × 이 값 (planning_interfaces.md 6 절, config/planning.yaml grasp_offset_m 과 같음)
constexpr double kGraspOffsetM = 0.015;

// base_link ← 카메라: 평행 이동 + 쿼터니언 (x, y, z, w)
struct Xform
{
  double tx{0}, ty{0}, tz{0};
  double qx{0}, qy{0}, qz{0}, qw{1};
};

struct DetPick
{
  double x{0}, y{0}, z{0};              // 그리퍼 중심이 갈 점 (base_link)
  double qx{0}, qy{0}, qz{0}, qw{0};    // 목표 방향, 방향 모르면 0,0,0,0 (control 기본 방향)
  double cx{0}, cy{0}, cz{0};           // 퍽 중심 (base_link)
  bool upright{true};
  bool theta_known{false};
  double theta_deg{0};
};

// 고를 검출 (dets 번호 0 부터): tag 가 있으면 그 태그 퍽, 없으면 퍽이 하나뿐일 때 그것. 못 고르면 −1
//   번호로 고르기는 없음 (화면을 그린 뒤 검출 순서가 바뀔 수 있음)
int choose_det(const std::vector<DetView> & dets, const std::string & tag, std::string & why);

// 검출 (카메라 프레임) → 집기 목표. 위치 모름 · 누운 퍽은 false (누운 퍽은 control_master 가 아직 안 함)
bool det_pick(const DetView & d, const Xform & T, double grasp_offset, DetPick & out, std::string & why);

}  // namespace robot_cli
