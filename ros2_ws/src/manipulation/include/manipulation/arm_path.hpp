// arm_path.hpp — 실물에서 검사 · 실행한 팔 경로를 동작 목록으로 (순수 C++, test/test_arm_path.cpp)
// =====================================================================
// 경로 = 경유점 목록 (config/arm_paths.yaml, tools/arm 의 생성기가 실물 계획기 · 안전 검사 결과로 만듦)
//   경유점 하나 = 관절 5 개 명령값 [rad] + 받침 구간인가 + 그 점으로 가는 최고 속도 + 도착 뒤 그리퍼 동작 + 멈춤
// 시간표 = 실물 실행기(arm_pose_runner)와 같은 규칙
//   - 경로는 경유점을 곧게 잇는 꺾은선 그대로, 길이 = 가장 많이 움직이는 관절의 각도 [°]
//   - 구간 최고 속도: 받침 구간 stand_speed, 그 밖 speed, 경유점에 speed 가 있으면 그보다 작게
//   - 꺾이는 곳 속도 × cos²(꺾인 각 / 2), 가속 · 감속 accel, 처음 · 끝 0
//   - dt 마다 점 하나 (JTC 가 점 사이를 선형 보간)
// 동작 나누기: 그리퍼 동작이나 멈춤이 있는 경유점에서 끊음 → 팔 동작(여러 점) · 그리퍼 동작 차례로
//   grip_on_way 인 시간 맞춘 점은 안 끊고 그 팔 동작의 side_grip_t (도착 시각) 로

#pragma once

#include <map>
#include <string>
#include <vector>

#include "manipulation/arm.hpp"

namespace manipulation
{

struct PathNode
{
  std::string name;
  std::vector<double> q;     // 관절 5 개 명령값 [rad] (처짐 앞먹임 포함)
  bool stand{false};         // 받침 근처 세우기 · 복귀 구간 (느리게)
  double speed_dps{0.0};     // 이 점으로 가는 최고 속도 [°/s], 0 = 기본
  std::string grip;          // 도착 뒤 그리퍼: "" · open · release · close
  bool stop{false};          // 도착해서 멈춤 (동작을 여기서 끊음)
  double hold_s{0.0};        // 도착해서 이만큼 그 자세로 기다림 [s] (stop 과 같이 씀, 손목 카메라가 멈춘 장을 모으게)
  double dt_s{0.0};          // > 0: 앞 점에서 이 점까지 정확히 이 시간 [s] (속도 · 가속 한계 없이 — 세우기 표를 녹화 간격 그대로 재생)
  bool grip_on_way{false};   // grip 을 팔을 멈추지 않고 (그 점 도착 시각에 그리퍼만, 팔은 계속) — 손에 퍽이 없을 때 · 세우기 여는 점
  bool grip_early{false};    // grip 을 이 점으로 출발할 때 (앞 점 도착 시각) 팔을 멈추지 않고 — PICK 퍽 위로 가며 미리 열기
};

struct ArmPath
{
  std::string name;
  std::vector<PathNode> nodes;
  bool ends_home{false};     // 마지막 점 = 받침(home) → 끝나면 접힘 실측 확인 뒤 주행 허용
};

struct PathTiming
{
  double speed_dps{40.0};
  double stand_speed_dps{20.0};
  double accel_dps2{150.0};
  double dt_s{0.05};
  double grip_settle_s{0.0};   // 그리퍼를 여닫기 전 같은 자세로 기다림 (JTC 목표 허용 0.05–0.12 rad 라 도착 판정 뒤에도 수 ° 남음)
};

struct GripperValues
{
  double open{0.0164};
  double release{0.0093};
  double close{-0.0016};
  double loose{0.0060};   // 퍽이 손가락 사이에서 돌 수 있게 살짝 풂 (경로 grip "loose", 받침에 넣을 때). 퍽에 닿은 자리 약 0.0046
};

// start 에서 nodes 를 차례로 멈추지 않고 지나가는 시간표 (start 는 넣지 않음). 길이가 0 이면 마지막 점 하나를 0.3 s 에
//   nodes 가 모두 dt_s > 0 이면 경유점마다 그 시간 간격 그대로 (녹화 재생 — 시뮬: 속도 상한으로 다시 짜면 세우기 84 → 47 %)
//   출력 = dt 간격 점 + 경유점마다 그 도착 시각의 점 (JTC 가 점 사이를 직선으로 이어도 검사한 꺾은선 그대로)
std::vector<TrajPoint> time_path(
  const std::vector<double> & start, const std::vector<PathNode> & nodes, const PathTiming & timing);

// 경로 → 동작 목록. 첫 경유점 = 출발 자세 (지금 팔이 거기 있어야 함 — path_start_ok 로 확인)
//   last_pose: 마지막 팔 동작의 pose 이름 (ends_home 이면 "home" → ArmRunner 가 접힘 실측 확인)
//   q_now: 지금 실측 관절값. 주면 첫 구간을 첫 경유점 대신 여기서 출발해 계산 (허용 오차만큼 어긋난 시작도 같은 속도 · 가속 한계로)
std::vector<ArmAction> path_actions(
  const ArmPath & path, const PathTiming & timing, const GripperValues & grip,
  const std::vector<double> & q_now = {});

// 지금 관절값이 경로 첫 점에서 관절마다 tol 안인가 (NaN · 관절 수 다름 = 아님)
bool path_start_ok(const ArmPath & path, const std::vector<double> & q_now, double tol, std::string & why);

// 나중에 접기 (lazy_fold) 도우미. 바꿨으면 true
//   cut_fold_tail: 끝이 fold 경로와 같으면 P0 에서 끝냄 (ends_home = false)
//   skip_unfold: 앞이 unfold 경로와 같으면 P0 에서 시작
//   p0_distinct: P0 와 받침이 관절 하나라도 tol 넘게 다른가 (같으면 둘을 구분 못 함)
bool cut_fold_tail(ArmPath & path, const ArmPath & fold);
bool skip_unfold(ArmPath & path, const ArmPath & unfold);
bool p0_distinct(const ArmPath & fold, const ArmPath & unfold, double tol);

}  // namespace manipulation
