// approach.hpp — 박스 앞 마지막 접근 (Nav2 는 조금 앞에서 멈추고, 앞쪽 라이다로 박스 면까지 재며 똑바로 붙음)
//   Nav2 의 비용 지도는 2.5 cm 칸 · 위치 추정 오차가 수 cm 라 박스에서 2 cm 떨어진 목표에 정확히 서지 못하고 바퀴가 계속 움직임
//   → 목표를 back_m 앞에 두고, 그 뒤는 라이다로 잰 면 거리로 몸 x 방향만 움직임
//   면을 한 번도 못 봤으면 움직이지 않음. 본 뒤에 가까워져 라이다에서 사라지면 그 관측 + odom 직진 거리로 이어감
#pragma once

#include "planning/types.hpp"

namespace planning
{

// pose 에서 보는 방향 반대로 back_m 물러난 자세 (Nav2 목표)
Pose2D backed_off(const Pose2D & pose, double back_m);

struct ApproachConfig
{
  double target_face_m = 0.19;   // 멈출 때 몸 중심 → 앞 물체 면 거리 [m]
  double max_travel_m = 0.14;    // 이보다 많이 가면 실패
  double min_clear_m = 0.175;    // 차체 폭 앞쪽 가장 가까운 점이 몸 중심에서 이보다 가까우면 멈춤 (몸 앞 끝 0.16 + 여유)
  double obstacle_margin_m = 0.02;   // 차체 폭 앞쪽에 박스 면보다 이만큼 더 가까운 게 있으면 다른 물체로 봄
  double speed = 0.04;           // 최고 [m/s]
  double min_speed = 0.015;      // 최저 (조향 · 마찰로 안 움직이지 않게) [m/s]
  double gain = 1.0;             // [1/s] 남은 거리 → 속도
  double tol_m = 0.005;          // 이 안이면 도착
};

// 라이다 관측 하나 (새 스캔이 왔을 때만 갱신). travel_m = 그 스캔을 받았을 때의 직진 거리 (odom)
struct ApproachState
{
  bool have_face = false;
  double face_m = 0.0;           // 박스 면 (좁은 띠: 몸 중심선 근처)
  double front_m = 0.0;          // 차체 폭 띠의 가장 가까운 점 (NaN = 없음)
  double travel_m = 0.0;
};

struct ApproachCmd
{
  double vx = 0.0;               // 몸 x 방향 속도 [m/s] (vy · wz = 0)
  bool done = false;
  bool fail = false;
  bool waiting = false;          // 아직 면을 못 봄 (0 으로 기다림)
  double remaining = 0.0;        // 남은 거리 [m]
  const char * why = "";
};

// 새 스캔 관측을 기록 (face_m · front_m 은 NaN = 그 띠에 점 없음)
void approach_observe(ApproachState & st, double face_m, double front_m, double travel_m);

// travel_m: 지금 직진 거리 (odom, 새 측정일 때만 바뀜)
ApproachCmd approach_step(const ApproachState & st, const ApproachConfig & c, double travel_m);

}  // namespace planning
