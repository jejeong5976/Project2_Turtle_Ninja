// 누운 네모 퍽 세우기 (그리퍼 받침, control_master stand_lying_mode park) 의 자세 계산 — ROS 없음 (시뮬 도구 · 테스트와 같은 코드)
//   1) lying_grasp: 누운 퍽을 (반지름 쪽이면 바깥 끝, 접선 쪽이면 J1 쪽 끝을) 손 기울임 0 · 10 · 20 · 30° (바깥) 중 닿는 것으로 쥠. 닫힘축 = 손 축 × 긴축
//      (손가락이 긴 옆면 두 개). 긴축이 반지름 방향이면 닫힘축은 수평 (예전 기울인 손), 접선 방향이면 닫힘축이 팔 평면 안에서
//      기울어 퍽이 긴축 둘레로 그만큼 굴러 손가락에 맞음 (아래 손가락 쪽 바닥 모서리로 섬). 손은 손 축을 따라 들어감
//   2) park_nodes: 쥔 퍽 (손 좌표) 을 받침 위에서 긴 쪽 끝이 J1 쪽 아래로 elev_deg 가 되게 손을 기울여 넣고 놓음 →
//      손 빼고 오므려 받침 바닥 위 push_h 로 밀어 세움 → 수직 손으로 다시 쥐고 듦
//   좌표 = 팔 모델 (xy = base_link, z = base_link − 0.039)
#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "manipulation/arm_kinematics.hpp"

namespace manipulation
{

struct LyingGraspCfg
{
  double side{0.0348};           // 누운 높이 = 네모 단면 한 변
  double length{0.060};          // 긴축 길이
  double grip_offset{0.025};     // 쥐는 점 = 중심에서 쥐는 끝 쪽으로 (반지름 쪽 퍽 = 바깥 끝, 접선 쪽 = J1 쪽 끝)
  // 쥐는 깊이: 수직 손 손끝 = 윗면 − depth. 기울인 손도 손가락 가장 낮은 점이 이 높이 아래로 안 감 (같은 바닥 여유)
  double depth{0.045};
  double open_inner{0.0255};     // 열린 손가락 안쪽 면 (손 축에서, 닫힘 방향)
  double open_outer{0.0437};     // 바깥 면
  double finger_half_w{0.0164};  // 손가락 폭 반 (손 b 방향)
  double min_margin{0.0025};     // 들어갈 때 열린 손가락 ↔ 퍽 최소 여유 (닫힘 방향 한쪽)
  double min_depth{0.020};       // 굴린 퍽 윗면 → 손끝 최소 (덜 쥐면 안 됨)
  double max_roll_deg{12.0};     // 퍽이 손가락에 맞으며 굴러도 되는 최대 (11-06 시뮬: 10° 는 쥠, 20° 는 들다 놓침)
  std::vector<double> tilts_deg{0.0, 10.0, 20.0, 30.0};
};

struct LyingGrasp
{
  std::vector<PickPose> poses;   // 위 · 내림 · 쥐기 (heights_mm 높이, 손 축을 따라). enc = 쥔 물체 없이 처짐 앞먹임
  Q5 q_grasp{};
  Eigen::Vector3d tip{Eigen::Vector3d::Zero()}, app{Eigen::Vector3d::Zero()}, close{Eigen::Vector3d::Zero()};   // 쥘 때 손
  Eigen::Vector3d c{Eigen::Vector3d::Zero()};       // 쥔 뒤 (굴린 뒤) 퍽 중심
  Eigen::Vector3d n{Eigen::Vector3d::Zero()};       // 쥔 뒤 퍽 단면의 위쪽 축
  Eigen::Vector3d a_end{Eigen::Vector3d::Zero()};   // 받침에 넣을 긴 쪽 끝 방향 (쥐는 끝의 반대)
  double tilt_deg{0.0}, roll_deg{0.0}, pitch_deg{0.0}, depth_mm{0.0}, margin_mm{0.0};
};

// 누운 퍽 (중심 x, y · 윗면 top_z · 긴축 axis_deg) 쥐기. heights_mm = 위 · 내림 높이 (윗면 위, 손끝) 2 개 + 쥐는 깊이 (−depth, cfg.depth 대신)
//   accept: 이웃 여유 등 바깥 검사 (false 면 다음 기울임). 다 안 되면 false + why
bool lying_grasp(
  const ArmModel & m, double x, double y, double top_z, double axis_deg, const std::vector<double> & heights_mm,
  const LyingGraspCfg & cfg, LyingGrasp & out, std::string & why, double max_tilt_deg = 30.0,
  const std::function<bool(const LyingGrasp &, std::string &)> & accept = nullptr);

struct ParkCfg
{
  Eigen::Vector2d center{0.1008, 0.00115};   // 받침 중심
  double floor_z{0.0975};        // 받침 바닥 윗면
  double elev_deg{45.0};         // 넣을 때 퍽 긴축 기울기 (긴 쪽 끝이 J1 쪽 아래로). IK 안 되면 5° 씩 min_elev_deg 까지
  double min_elev_deg{40.0};
  double insert_clear{0.004};    // 넣을 때 긴 쪽 끝 가장 낮은 모서리 = 바닥 + 이만큼
  // 긴 쪽 끝면 중심 = 받침 중심에서 J1 쪽으로 이만큼. 놓으면 퍽은 아래 모서리가 바닥에 닿은 자리에서 바깥 벽 위 모서리에 기댐 →
  //   기대는 각 = atan(벽 높이 / (안쪽 반 − 모서리 자리)). 밀어서 서려면 약 49° 넘게 기대야 함 (11-06 시뮬: 8 mm 면 47–50° 로 5 중 2,
  //   4 mm 면 49–62° 로 5 중 5, 0 mm 는 수직 손에서 45° 로 실패)
  double end_inset{0.004};
  double push_h{0.025};          // 미는 손끝 높이 = 바닥 위
  double push_dz{0.0};           // 밀기 시작 높이 = push_h + 이만큼 → 안쪽으로 가며 push_h 까지 내려감 (손끝이 퍽 면을 타고 오르지 않게)
  double push_r0{0.06};          // 밀기 시작 = 받침 중심에서 바깥으로 (높이 밀면 기댄 퍽 윗끝 밖에서 내려와야 함)
  double regrasp_depth{0.030};   // 선 퍽 다시 쥐는 깊이 (윗면 아래)
  double side{0.0348}, length{0.060};
  double held_kg{0.017};
  double inner_half{0.0235};     // 받침 안쪽 반 (반지름 방향)
  int tilt_steps{8}, push_steps{4};
};

struct ParkNode
{
  std::string name;
  Q5 q{};             // 실제 관절 (처짐 앞먹임 전)
  double held_kg{0.0};
  bool slow{true};    // 받침 근처 느린 구간 (false = 빈 공간 속도)
  std::string grip;   // "", "open", "close"
};

// lift 자세 q_from 뒤 받침 세우기 경유점. elev_used = 실제 넣은 기울기
bool park_nodes(
  const ArmModel & m, const ParkCfg & c, const LyingGrasp & g, const Q5 & q_from, std::vector<ParkNode> & out,
  double & elev_used, std::string & why);

}  // namespace manipulation
