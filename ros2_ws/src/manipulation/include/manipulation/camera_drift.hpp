// camera_drift.hpp — 홈 자세 손목 사진 (로봇 몸통) 을 기준 사진과 비교해 손목 카메라가 손목에서 얼마나 돌았나 (순수 C++ · Eigen, test/test_camera_drift.cpp)
// =====================================================================
// 11-05 실물: 세우기 때 손이 박스를 누르며 카메라 경첩이 3.8° 돌아 퍽을 앞 · 옆으로 잘못 봄 (팔 관절값은 그대로).
// 홈 자세는 늘 같은 관절이라 몸통 화소가 움직였으면 카메라가 돈 것. 맞은 화소 쌍의 광선을 Kabsch 로 맞춰 회전을 구함
//   (몸통은 5 ~ 20 cm 라 카메라가 옮겨진 만큼은 회전으로 섞여 들어감 — 경첩 축이 렌즈 가까이라 작음)
#pragma once

#include <vector>

#include <Eigen/Dense>

namespace manipulation
{

struct CameraDrift
{
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};   // 실제 카메라 = 명목 카메라 · R (카메라 좌표). 검출점 p (카메라) → R · p 가 맞는 점
  double deg{0.0};        // 회전 크기 [°]
  double resid_px{0.0};   // 회전으로 맞춘 뒤 남는 화소 차 평균
  int n{0};
};

// u_ref · u_now: 같은 몸통 점의 기준 · 지금 화소 (같은 순서), fx fy cx cy: 카메라 내부값. 점이 3 개 미만이면 false
bool camera_drift(const std::vector<Eigen::Vector2d> & u_ref, const std::vector<Eigen::Vector2d> & u_now,
  double fx, double fy, double cx, double cy, CameraDrift & out);

}  // namespace manipulation
