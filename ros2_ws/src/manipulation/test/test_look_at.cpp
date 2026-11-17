// test_look_at.cpp — LOOK_AT (손목 카메라로 한 점 보기) 계산 시험
//   카메라 = 배포 URDF camera_joint (wrist_link → wrist_cam_color_optical_frame), 모델 = 실물 계산기 (arm_kin_data)
//   look_barcode 끝 자세 = config/arm_paths.yaml home_from_look_barcode 첫 점 (팀 rad) → 모델 관절

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "manipulation/look_at.hpp"
#include "arm_kin_data.hpp"

using manipulation::LookAtGoal;
using manipulation::LookAtParams;
using manipulation::Q5;

namespace
{
constexpr double kDeg = M_PI / 180.0;

Eigen::Isometry3d urdf_T5c()
{
  // <origin rpy="-2.969292 -0.048951 -1.995706" xyz="0.085672 0.001345 0.004098"/>  R = Rz · Ry · Rx
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(-1.995706, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-0.048951, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-2.969292, Eigen::Vector3d::UnitX())).toRotationMatrix();
  T.translation() = Eigen::Vector3d(0.085672, 0.001345, 0.004098);
  return T;
}

// 팀 명령 rad → 모델 관절 (영점 4088 · 2048 …, 처짐 무시)
Q5 team_to_model(const manipulation::ArmModel & m, const std::vector<double> & team)
{
  manipulation::Ticks t;
  for (int j = 0; j < 5; ++j) {t[j] = static_cast<int>(std::lround(m.zero_tick[j] + team[j] * 4096.0 / (2.0 * M_PI)));}
  return manipulation::ticks_to_q(m, t);
}

const std::vector<double> kLookBarcodeTeam{0.13192, 0.30833, -0.85136, -1.17196, -0.57217};
const Eigen::Vector3d kWideCam(-0.0463, -0.0797, 0.598);   // URDF birdeye_camera_optical_frame (base_link)
}  // namespace

// 1) 카메라 자세 = URDF 순기구학 (look_barcode), 고정 자세 바코드 (앞 0.50 · 왼 0.15 · 바닥 위 0.45 m) 와 광축 28°
TEST(LookAt, CameraMatchesUrdfAndOldBarcodeOffset)
{
  const auto m = kin_data::model();
  const auto q = team_to_model(m, kLookBarcodeTeam);
  const auto cam = manipulation::look_camera_pose(m, urdf_T5c(), q);
  EXPECT_NEAR(cam.translation().x(), 0.2857, 2e-3);
  EXPECT_NEAR(cam.translation().y(), 0.0979, 2e-3);
  EXPECT_NEAR(cam.translation().z(), 0.4350, 2e-3);
  const Eigen::Vector3d z = cam.linear().col(2);
  EXPECT_NEAR(z.x(), 0.9363, 3e-3);
  EXPECT_NEAR(z.y(), -0.1920, 3e-3);
  const Eigen::Vector3d bc(0.50, 0.15, 0.45 - 0.033);
  const double err = std::acos(z.dot((bc - cam.translation()).normalized())) / kDeg;
  EXPECT_NEAR(err, 27.8, 1.0);
  std::printf("look_barcode: 카메라 (%.3f, %.3f, %.3f) 옛 바코드 광축 오차 %.1f° 화면 돌림 %.1f°\n", cam.translation().x(),
    cam.translation().y(), cam.translation().z(), err, manipulation::look_roll(cam, 0.0) / kDeg);
}

// 2) 옛 고정 바코드를 LOOK_AT 으로: 광축 < 2°, 거리 0.30 ± 2 cm, 돌림 ≤ 15°
TEST(LookAt, OldBarcodeSolved)
{
  const auto m = kin_data::model();
  const LookAtParams p;
  LookAtGoal g;
  g.target = Eigen::Vector3d(0.50, 0.15, 0.45 - 0.033);
  g.standoff_m = 0.30;
  g.view_yaw = 0.0;   // 벽이 앞 (−X 벽, 로봇이 보는 쪽)
  const auto start = team_to_model(m, kLookBarcodeTeam);
  const auto s = manipulation::solve_look_at(m, urdf_T5c(), g, p, start, kWideCam);
  ASSERT_TRUE(s.ok) << s.why;
  EXPECT_LT(s.axis_err_deg, 2.0);
  EXPECT_NEAR(s.dist_m, 0.30, 0.02);
  EXPECT_LE(s.roll_deg, 15.0);
  std::printf("옛 바코드: 광축 %.2f° 거리 %.3f 정면 %.1f° 돌림 %.1f°\n", s.axis_err_deg, s.dist_m, s.obliquity_deg, s.roll_deg);
}

// 3) 로봇 둘레 목표 (방위 · 거리 · 높이) — 풀린 것은 모두 광축 < 2° · 거리 범위 · 돌림 ≤ 15° (다시 평가로 확인). 앞쪽 반은 대부분 풀려야 함
TEST(LookAt, AxisErrorBelow2DegAroundRobot)
{
  const auto m = kin_data::model();
  const LookAtParams p;
  const auto T5c = urdf_T5c();
  const auto start = team_to_model(m, kLookBarcodeTeam);
  int n = 0, solved = 0, front = 0, front_solved = 0;
  std::printf("방위°  높이(바닥) 거리 → 풀림 (광축° 거리 m 정면° 돌림°)\n");
  for (double az = -150.0; az <= 150.0; az += 30.0) {
    for (double rng : {0.50, 0.65}) {
      for (double h : {0.25, 0.45, 0.60}) {
        LookAtGoal g;
        g.target = Eigen::Vector3d(rng * std::cos(az * kDeg), rng * std::sin(az * kDeg), h - 0.033);
        g.standoff_m = 0.30;
        const auto s = manipulation::solve_look_at(m, T5c, g, p, start, kWideCam);
        ++n;
        if (std::fabs(az) <= 60.0) {++front;}
        if (s.ok) {
          ++solved;
          if (std::fabs(az) <= 60.0) {++front_solved;}
          EXPECT_LT(s.axis_err_deg, 2.0);
          EXPECT_GE(s.dist_m, p.standoff_min_m);
          EXPECT_LE(s.dist_m, p.standoff_max_m);
          EXPECT_LE(s.roll_deg, p.max_roll_deg + 1e-9);
          const auto e = manipulation::eval_look_at(m, T5c, g, p, s.q);
          EXPECT_TRUE(e.ok) << e.why;
          std::string w;
          EXPECT_TRUE(manipulation::look_at_clear(m, T5c, s.q, p, g, kWideCam, w)) << w;
        }
        std::printf("%5.0f  %.2f  %.2f → %s (%.2f %.3f %.0f %.0f) %s\n", az, h, rng, s.ok ? "예" : "아니오", s.axis_err_deg, s.dist_m,
          s.obliquity_deg, s.roll_deg, s.ok ? "" : s.why.c_str());
      }
    }
  }
  std::printf("풀림 %d / %d (앞 ±60° %d / %d)\n", solved, n, front_solved, front);
  EXPECT_GE(front_solved * 10, front * 6);   // 앞쪽 60 % 이상 (바란 거리 ±3 cm · 벽 여유 — 먼 목표는 [REACH] → 몸 옮김)
}

// 4) 닿는 범위 밖: 1.5 m 먼 목표 (거리 범위 밖) · 차체 안 목표 · 너무 낮은 목표는 거절
TEST(LookAt, ReachLimitsRejected)
{
  const auto m = kin_data::model();
  const LookAtParams p;
  const auto start = team_to_model(m, kLookBarcodeTeam);
  LookAtGoal far;
  far.target = Eigen::Vector3d(1.5, 0.0, 0.40);
  far.standoff_m = 0.30;
  const auto s1 = manipulation::solve_look_at(m, urdf_T5c(), far, p, start, kWideCam);
  EXPECT_FALSE(s1.ok);
  EXPECT_FALSE(s1.why.empty());
  LookAtGoal floor;
  floor.target = Eigen::Vector3d(0.35, 0.0, -0.03);   // 바닥 (보는 방향 수평이면 카메라가 바닥 아래로)
  floor.standoff_m = 0.30;
  const auto s2 = manipulation::solve_look_at(m, urdf_T5c(), floor, p, start, kWideCam);
  EXPECT_FALSE(s2.ok);
  std::printf("먼 목표: %s / 바닥 목표: %s\n", s1.why.c_str(), s2.why.c_str());
}

// 5) 신호등 시야선: 풀린 카메라 자리를 지나는 시야선을 주면 그 자세는 거절 → 다른 해 또는 실패, 결과는 시야선에서 떨어짐
TEST(LookAt, LightLineOfSightKept)
{
  const auto m = kin_data::model();
  LookAtParams p;
  const auto T5c = urdf_T5c();
  const auto start = team_to_model(m, kLookBarcodeTeam);
  LookAtGoal g;
  g.target = Eigen::Vector3d(0.50, 0.15, 0.417);
  g.standoff_m = 0.30;
  g.view_yaw = 0.0;
  const auto s0 = manipulation::solve_look_at(m, T5c, g, p, start, kWideCam);
  ASSERT_TRUE(s0.ok);
  const Eigen::Vector3d l = (s0.cam_pos - kWideCam).normalized();
  g.has_light = true;
  g.light_yaw = std::atan2(l.y(), l.x());
  g.light_pitch = std::asin(l.z());
  std::string w;
  EXPECT_FALSE(manipulation::look_at_clear(m, T5c, s0.q, p, g, kWideCam, w));
  const auto s1 = manipulation::solve_look_at(m, T5c, g, p, start, kWideCam);
  if (s1.ok) {EXPECT_TRUE(manipulation::look_at_clear(m, T5c, s1.q, p, g, kWideCam, w)) << w;}
  std::printf("시야선 지나는 자리: 다른 해 %s (%s)\n", s1.ok ? "있음" : "없음", s1.why.c_str());
  // 신호등이 오른쪽 아래 (광각 앞 −20°) 면 앞 왼쪽 바코드는 그대로 풀림
  g.light_yaw = -20.0 * kDeg;
  g.light_pitch = -10.0 * kDeg;
  const auto s2 = manipulation::solve_look_at(m, T5c, g, p, start, kWideCam);
  EXPECT_TRUE(s2.ok) << s2.why;
}

// 6) 흔들기 dj1 · dj4: 해에서 그만큼만 다름, 여유 검사 통과
TEST(LookAt, PerturbationApplied)
{
  const auto m = kin_data::model();
  const LookAtParams p;
  const auto start = team_to_model(m, kLookBarcodeTeam);
  LookAtGoal g;
  g.target = Eigen::Vector3d(0.50, 0.15, 0.417);
  g.view_yaw = 0.0;
  const auto s0 = manipulation::solve_look_at(m, urdf_T5c(), g, p, start, kWideCam);
  ASSERT_TRUE(s0.ok);
  g.dj1 = 4.0 * kDeg;
  g.dj4 = -3.0 * kDeg;
  const auto s1 = manipulation::solve_look_at(m, urdf_T5c(), g, p, start, kWideCam);
  ASSERT_TRUE(s1.ok) << s1.why;
  EXPECT_NEAR(s1.q[0] - s0.q[0], 4.0 * kDeg, 1e-9);
  // J4 는 범위 끝이면 갈 수 있는 만큼만 (0 ~ −3°)
  EXPECT_LE(s1.q[3] - s0.q[3], 1e-9);
  EXPECT_GE(s1.q[3] - s0.q[3], -3.0 * kDeg - 1e-9);
  std::printf("흔들기: J1 %+.1f° J4 %+.1f° 광축 %.1f°\n", (s1.q[0] - s0.q[0]) / kDeg, (s1.q[3] - s0.q[3]) / kDeg, s1.axis_err_deg);
}

// 7) slot 글 읽기
TEST(LookAt, SlotParse)
{
  LookAtGoal g;
  std::string why;
  ASSERT_TRUE(manipulation::parse_look_at_slot("d=0.28, vyaw=12.5,dj1=4,dj4=-3,lyaw=-15,lpitch=5,new=1", g, why)) << why;
  EXPECT_DOUBLE_EQ(g.standoff_m, 0.28);
  EXPECT_NEAR(g.view_yaw, 12.5 * kDeg, 1e-12);
  EXPECT_NEAR(g.dj4, -3.0 * kDeg, 1e-12);
  EXPECT_TRUE(g.has_light);
  EXPECT_FALSE(manipulation::parse_look_at_slot("d=abc", g, why));
  LookAtGoal e;
  EXPECT_TRUE(manipulation::parse_look_at_slot("", e, why));
  EXPECT_FALSE(std::isfinite(e.standoff_m));
}

// 8) 실제 경기장: 벽 바코드 중심 바닥 위 약 25 cm, 광각에서 수평 0.66 m (base 앞 약 0.61 m), 벽 위 38 cm.
//    0.25 m 에서 정면으로 (손목 컬러 fx 606, 8 cm · 90 모듈 → 0.27 m 안이어야 2 px/모듈) 광축 < 2°
TEST(LookAt, LowWallBarcode)
{
  const auto m = kin_data::model();
  const LookAtParams p;
  const auto start = team_to_model(m, kLookBarcodeTeam);
  for (double y : {0.0, 0.15, -0.15}) {
    for (double d : {0.22, 0.25, 0.30}) {
      LookAtGoal g;
      g.target = Eigen::Vector3d(0.61, y, 0.25 - 0.033);
      g.standoff_m = d;
      g.view_yaw = 0.0;
      const auto s = manipulation::solve_look_at(m, urdf_T5c(), g, p, start, kWideCam);
      // 이 자리 (0.61 m 앞, 낮음) 에서 카메라는 약 0.26 m 보다 못 다가감 → 바란 거리 ±3 cm 가 아니면 [REACH] 거절 (몸을 옮김)
      if (d >= 0.30) {EXPECT_TRUE(s.ok) << "y " << y << " d " << d << ": " << s.why;}
      if (s.ok) {
        EXPECT_LT(s.axis_err_deg, 2.0);
        EXPECT_LE(std::fabs(s.dist_m - d), 0.03 + 1e-9);
      } else {
        EXPECT_EQ(s.why.rfind("[REACH]", 0), 0u) << s.why;
      }
      std::printf("벽 25 cm y %+.2f d %.2f: %s 광축 %.2f° 거리 %.3f 정면 %.0f° 돌림 %.0f° 카메라 z(바닥) %.3f %s\n", y, d, s.ok ? "예" : "아니오",
        s.axis_err_deg, s.dist_m, s.obliquity_deg, s.roll_deg, s.cam_pos.z() + 0.033, s.why.c_str());
    }
  }
}

// 9) 실제 로봇 시험 자세 (팀 rad 0.11375, 0.17963, -0.65079, -0.81745, -0.00859) = URDF FK 로 바코드 base (0.46, 0.08, 바닥 0.27) 를
//    약 24 cm 에서 겨눈 자세 → 이 모델에서도 광축 오차 작음 (모델 = URDF 확인)
TEST(LookAt, RealTestPoseAims)
{
  const auto m = kin_data::model();
  const auto q = team_to_model(m, {0.11375, 0.17963, -0.65079, -0.81745, -0.00859});
  const auto cam = manipulation::look_camera_pose(m, urdf_T5c(), q);
  const Eigen::Vector3d bc(0.46, 0.08, 0.27 - 0.033);
  const Eigen::Vector3d d = bc - cam.translation();
  const double err = std::acos(cam.linear().col(2).dot(d.normalized())) / kDeg;
  std::printf("실물 시험 자세: 카메라 (%.3f, %.3f, %.3f) 거리 %.3f 광축 오차 %.1f° 돌림 %.0f° J2 토크 %.2f N·m (look_barcode %.2f)\n",
    cam.translation().x(), cam.translation().y(), cam.translation().z(), d.norm(), err, manipulation::look_roll(cam, 0.0) / kDeg,
    std::fabs(manipulation::sag_torques(m, q, 0.0)[0]), std::fabs(manipulation::sag_torques(m, team_to_model(m, kLookBarcodeTeam), 0.0)[0]));
  EXPECT_LT(err, 3.0);
  EXPECT_NEAR(d.norm(), 0.24, 0.03);
}

// 10) 신호등이 바코드 옆 : 광각 → 신호등 시야 원뿔 (반각 lcone) 밖에 팔을 두고 바코드를 봄
//     실측 바코드 base (0.46, 0.08, 바닥 0.27). 신호등이 바코드 위 · 옆 (같은 벽) 이면 풀림. 신호등이 바코드와 같은 방향
//     (광각에서 겹침 · 원뿔 17° = 250 px) 이면 이 자리에서는 손목 카메라 · 손이 시야를 가려 못 풂 (거절)
TEST(LookAt, KeepLightClearInWide)
{
  const auto m = kin_data::model();
  LookAtParams p;
  const auto start = team_to_model(m, kLookBarcodeTeam);
  const Eigen::Vector3d bc(0.46, 0.08, 0.27 - 0.033);
  struct C {double dy, z_floor, cone; bool ok;};
  for (const C c : {C{-0.15, 0.35, 10, true}, C{-0.25, 0.15, 10, true}, C{0.0, 0.40, 10, true}, C{0.30, 0.35, 10, true},
                    C{0.40, 0.15, 10, true}, C{0.0, 0.27, 17, false}})
  {
    const Eigen::Vector3d light(0.47, bc.y() + c.dy, c.z_floor - 0.033);
    const Eigen::Vector3d l = (light - kWideCam).normalized();
    LookAtGoal g;
    g.target = bc;
    g.standoff_m = 0.25;
    g.view_yaw = 0.0;
    g.has_light = true;
    g.light_yaw = std::atan2(l.y(), l.x());
    g.light_pitch = std::asin(l.z());
    g.light_cone_deg = c.cone;
    const auto t0 = std::chrono::steady_clock::now();
    const auto s = manipulation::solve_look_at(m, urdf_T5c(), g, p, start, kWideCam);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("신호등 옆 %+.2f 높이 %.2f 원뿔 %2.0f°: %s 광축 %.2f° 거리 %.3f 정면 %.0f° (%.0f ms) %s\n", c.dy, c.z_floor, c.cone,
      s.ok ? "예" : "아니오", s.axis_err_deg, s.dist_m, s.obliquity_deg, ms, s.why.c_str());
    EXPECT_EQ(s.ok, c.ok) << s.why;
    if (s.ok) {
      std::string w;
      EXPECT_TRUE(manipulation::look_at_clear(m, urdf_T5c(), s.q, p, g, kWideCam, w)) << w;
      EXPECT_LT(s.axis_err_deg, 2.0);
    }
  }
}

// 11) 실제 로봇: 3 px/모듈 = 카메라 15–16 cm, 몸을 가까이 (바코드 base 0.40 m 앞) · 팔은 접은 쪽 (J2 토크 작게)
TEST(LookAt, CloseCompactLook)
{
  const auto m = kin_data::model();
  LookAtParams p;
  p.standoff_min_m = 0.12;
  const auto start = team_to_model(m, kLookBarcodeTeam);
  const double tau_far = std::fabs(manipulation::sag_torques(m, team_to_model(m, {0.11375, 0.17963, -0.65079, -0.81745, -0.00859}), 0.0)[0]);
  for (double ahead : {0.36, 0.40, 0.44}) {
    for (double h : {0.20, 0.27, 0.35}) {
      LookAtGoal g;
      g.target = Eigen::Vector3d(ahead, 0.05, h - 0.033);
      g.standoff_m = 0.16;
      g.view_yaw = 0.0;
      const auto s = manipulation::solve_look_at(m, urdf_T5c(), g, p, start, kWideCam);
      std::printf("앞 %.2f 높이 %.2f d 0.16: %s 광축 %.2f° 거리 %.3f 정면 %.0f° J2 %.2f N·m (실물 24 cm 자세 %.2f) %s\n", ahead, h,
        s.ok ? "예" : "아니오", s.axis_err_deg, s.dist_m, s.obliquity_deg, s.j2_torque_nm, tau_far, s.why.c_str());
      if (ahead <= 0.40 && h >= 0.27) {
        EXPECT_TRUE(s.ok) << s.why;
        EXPECT_NEAR(s.dist_m, 0.16, 0.03);
        EXPECT_LT(s.j2_torque_nm, tau_far);
      }
    }
  }
}

// 12) 실제 로봇에서 읽힌 자세 (팀 rad -0.13647, -0.15476, 0.43255, -1.65000, 0.00644): 벽 38 cm · 바코드 바닥 위 27 cm 를
//     약 15 cm 에서 (3.5 px/모듈) 2/2 읽음. 같은 목표를 LOOK_AT 으로 풀면 광축 < 2° · 15 cm · J2 토크가 그 자세 근처 (접은 팔)
TEST(LookAt, RealReadPose)
{
  const auto m = kin_data::model();
  const auto q_real = team_to_model(m, {-0.13647, -0.15476, 0.43255, -1.65000, 0.00644});
  const auto cam = manipulation::look_camera_pose(m, urdf_T5c(), q_real);
  const double tau_real = std::fabs(manipulation::sag_torques(m, q_real, 0.0)[0]);
  const Eigen::Vector3d z = cam.linear().col(2);
  std::printf("실물 읽힌 자세: 카메라 (%.3f, %.3f, 바닥 %.3f) 아래 %.0f° J2 %.2f N·m\n", cam.translation().x(), cam.translation().y(),
    cam.translation().z() + 0.033, std::asin(-z.z()) / kDeg, tau_real);
  EXPECT_NEAR(cam.translation().x(), 0.268, 0.01);   // 기준 FK (base_footprint 0.268, 0.001, 0.376)
  EXPECT_NEAR(cam.translation().z() + 0.033, 0.376, 0.01);
  // 광축이 벽 (x 0.38) 과 만나는 점 = 바코드 자리 (높이 ~0.27)
  const double s = (0.38 - cam.translation().x()) / z.x();
  const Eigen::Vector3d hit = cam.translation() + z * s;
  std::printf("  광축 ↔ 벽 x 0.38: (%.3f, %.3f, 바닥 %.3f) 거리 %.3f\n", hit.x(), hit.y(), hit.z() + 0.033, s);
  LookAtParams p;
  p.standoff_min_m = 0.12;
  LookAtGoal g;
  g.target = hit;
  g.standoff_m = 0.15;
  g.view_yaw = 0.0;
  const auto sol = manipulation::solve_look_at(m, urdf_T5c(), g, p, team_to_model(m, kLookBarcodeTeam), kWideCam);
  std::printf("  LOOK_AT 해: %s 광축 %.2f° 거리 %.3f 정면 %.0f° 돌림 %.0f° 카메라 (%.3f, %.3f, 바닥 %.3f) J2 %.2f N·m %s\n", sol.ok ? "예" : "아니오",
    sol.axis_err_deg, sol.dist_m, sol.obliquity_deg, sol.roll_deg, sol.cam_pos.x(), sol.cam_pos.y(), sol.cam_pos.z() + 0.033, sol.j2_torque_nm,
    sol.why.c_str());
  ASSERT_TRUE(sol.ok) << sol.why;
  EXPECT_LT(sol.axis_err_deg, 2.0);
  EXPECT_NEAR(sol.dist_m, 0.15, 0.02);
  EXPECT_LT(sol.j2_torque_nm, tau_real + 0.15);
  // 읽힌 자세처럼 위에서 (vpitch 43°, 그 자세를 씨앗 = control_master look_at_seed_team_rad 기본): 카메라가 그 자세 근처 (2 cm 안)
  g.view_pitch = 43.0 * kDeg;
  const auto sp = manipulation::solve_look_at(m, urdf_T5c(), g, p, team_to_model(m, kLookBarcodeTeam), kWideCam, {q_real});
  std::printf("  vpitch 43°: %s 광축 %.2f° 거리 %.3f 정면 %.0f° 돌림 %.0f° 카메라 (%.3f, %.3f, 바닥 %.3f) J2 %.2f N·m %s\n", sp.ok ? "예" : "아니오",
    sp.axis_err_deg, sp.dist_m, sp.obliquity_deg, sp.roll_deg, sp.cam_pos.x(), sp.cam_pos.y(), sp.cam_pos.z() + 0.033, sp.j2_torque_nm,
    sp.why.c_str());
  ASSERT_TRUE(sp.ok) << sp.why;
  EXPECT_LT((sp.cam_pos - cam.translation()).norm(), 0.02);
}

// 13) 읽힌 자세를 씨앗으로 다른 자리 (옆 · 높이) 바코드도 위에서 43° 로 — 벽 38 cm 앞 y ±0.12 · 높이 0.20–0.35
TEST(LookAt, TaughtSeedGeneralises)
{
  const auto m = kin_data::model();
  LookAtParams p;
  p.standoff_min_m = 0.12;
  const auto seed = team_to_model(m, {-0.13647, -0.15476, 0.43255, -1.65000, 0.00644});
  int ok = 0, n = 0;
  for (double y : {-0.12, 0.0, 0.12}) {
    for (double h : {0.20, 0.27, 0.35}) {
      LookAtGoal g;
      g.target = Eigen::Vector3d(0.38, y, h - 0.033);
      g.standoff_m = 0.15;
      g.view_yaw = 0.0;
      g.view_pitch = 43.0 * kDeg;
      const auto s = manipulation::solve_look_at(m, urdf_T5c(), g, p, team_to_model(m, kLookBarcodeTeam), kWideCam, {seed});
      ++n;
      if (s.ok) {++ok; EXPECT_LT(s.axis_err_deg, 2.0); EXPECT_NEAR(s.dist_m, 0.15, 0.02);}
      std::printf("벽 38 cm y %+.2f 높이 %.2f: %s 정면 %.0f° J2 %.2f %s\n", y, h, s.ok ? "예" : "아니오", s.obliquity_deg, s.j2_torque_nm, s.why.c_str());
    }
  }
  EXPECT_EQ(ok, n);
}

// 14) 손끝 · 카메라 ↔ 목표 벽 면 여유 ≥ 3 cm. 실제로 읽힌 자세의 여유를 찍고, 15 cm · 43° 해는 여유 통과,
//     13 cm 수평 보기는 여유 통과하거나 [WALL] · [REACH] 로 거절
TEST(LookAt, FingertipWallClearance)
{
  const auto m = kin_data::model();
  LookAtParams p;
  p.standoff_min_m = 0.12;
  const auto q_real = team_to_model(m, {-0.13647, -0.15476, 0.43255, -1.65000, 0.00644});
  LookAtGoal g;
  g.target = Eigen::Vector3d(0.38, -0.016, 0.271 - 0.033);
  g.view_yaw = 0.0;
  const double real_clear = manipulation::look_at_wall_clear(m, urdf_T5c(), q_real, g);
  const Eigen::Vector3d tip = manipulation::arm_grip(m, q_real).tip + Eigen::Vector3d(0, 0, manipulation::model_to_base_z(m));
  std::printf("실물 읽힌 자세: 손끝 (%.3f, %.3f, 바닥 %.3f) · 벽 면 여유 (손목 · 손끝 · 카메라 중 최소) %.3f m\n", tip.x(), tip.y(), tip.z() + 0.033, real_clear);
  g.standoff_m = 0.15;
  g.view_pitch = 43.0 * kDeg;
  const auto s = manipulation::solve_look_at(m, urdf_T5c(), g, p, team_to_model(m, kLookBarcodeTeam), kWideCam, {q_real});
  ASSERT_TRUE(s.ok) << s.why;
  EXPECT_GE(s.wall_clear_m, p.wall_margin_m - 1e-9);
  std::printf("15 cm · 43°: 벽 여유 %.3f m 정면 %.0f° J2 %.2f\n", s.wall_clear_m, s.obliquity_deg, s.j2_torque_nm);
  g.standoff_m = 0.13;
  g.view_pitch = 0.0;
  const auto f = manipulation::solve_look_at(m, urdf_T5c(), g, p, team_to_model(m, kLookBarcodeTeam), kWideCam, {q_real});
  std::printf("13 cm 수평: %s 벽 여유 %.3f %s\n", f.ok ? "예" : "아니오", f.wall_clear_m, f.why.c_str());
  if (f.ok) {EXPECT_GE(f.wall_clear_m, p.wall_margin_m - 1e-9);} else {EXPECT_EQ(f.why[0], '[');}
  // 벽 면을 넘는 자세는 [WALL]
  p.wall_margin_m = 0.30;
  std::string w;
  EXPECT_FALSE(manipulation::look_at_clear(m, urdf_T5c(), s.q, p, g, kWideCam, w));
  EXPECT_EQ(w.rfind("[WALL]", 0), 0u) << w;
}

// 15) 지나가는 길도 원뿔 — 원뿔 밖 → 안으로 들어가는 직선은 [OCCL], 안 → 밖 (떠남) 은 통과
TEST(LookAt, TransitConeChecked)
{
  const auto m = kin_data::model();
  const LookAtParams p;
  const auto qa = team_to_model(m, kLookBarcodeTeam);
  const auto qb = team_to_model(m, {-0.13647, -0.15476, 0.43255, -1.65000, 0.00644});
  // 원뿔 = 광각 → 두 자세 가운데 카메라 자리 (지나는 길이 지남)
  Q5 qm;
  for (int j = 0; j < 5; ++j) {qm[j] = 0.5 * (qa[j] + qb[j]);}
  const Eigen::Vector3d c = manipulation::look_camera_pose(m, urdf_T5c(), qm).translation();
  const Eigen::Vector3d l = (c - kWideCam).normalized();
  LookAtGoal g;
  g.target = Eigen::Vector3d(0.38, 0.0, 0.24);
  g.has_light = true;
  g.light_yaw = std::atan2(l.y(), l.x());
  g.light_pitch = std::asin(l.z());
  g.light_cone_deg = 3.0;
  std::string w;
  const bool a_in = !manipulation::look_at_cone_clear(m, urdf_T5c(), qa, p, g, kWideCam, w);
  const bool b_in = !manipulation::look_at_cone_clear(m, urdf_T5c(), qb, p, g, kWideCam, w);
  std::printf("원뿔 안: 시작 %d 끝 %d\n", a_in, b_in);
  if (!a_in && !b_in) {   // 둘 다 밖 → 가운데에서 들어감 = 실패
    EXPECT_FALSE(manipulation::look_at_path_clear(m, urdf_T5c(), qa, qb, p, g, kWideCam, w));
    EXPECT_NE(w.find("[OCCL]"), std::string::npos) << w;
  } else if (a_in && !b_in) {   // 시작이 안 → 떠나는 길은 통과, 거꾸로 (들어가며 끝남) 은 실패
    LookAtParams pb = p;
    pb.min_z_m = -1.0;   // 원뿔만 보려고 몸 검사 끔
    pb.body_box = {{9, 9, 9, 9, -9}};
    pb.rack_box = pb.body_box;
    pb.mast_box = pb.body_box;
    pb.wall_margin_m = 0.0;
    EXPECT_TRUE(manipulation::look_at_path_clear(m, urdf_T5c(), qa, qb, pb, g, kWideCam, w)) << w;
    EXPECT_FALSE(manipulation::look_at_path_clear(m, urdf_T5c(), qb, qa, pb, g, kWideCam, w));
    EXPECT_NE(w.find("[OCCL]"), std::string::npos) << w;
  }
}

// 16) 겨눔 화소 (손가락 피하기): aim (−60, −50) px 이면 목표가 화면 그 자리에 옴 (1 px 안)
TEST(LookAt, AimOffsetPixel)
{
  const auto m = kin_data::model();
  LookAtParams p;
  p.standoff_min_m = 0.12;
  p.aim_du_px = -60.0;
  p.aim_dv_px = -50.0;
  const auto q_real = team_to_model(m, {-0.13647, -0.15476, 0.43255, -1.65000, 0.00644});
  LookAtGoal g;
  g.target = Eigen::Vector3d(0.38, 0.0, 0.237);
  g.standoff_m = 0.15;
  g.view_yaw = 0.0;
  g.view_pitch = 43.0 * kDeg;
  const auto s = manipulation::solve_look_at(m, urdf_T5c(), g, p, team_to_model(m, kLookBarcodeTeam), kWideCam, {q_real});
  ASSERT_TRUE(s.ok) << s.why;
  const auto cam = manipulation::look_camera_pose(m, urdf_T5c(), s.q);
  const Eigen::Vector3d qc = cam.inverse() * g.target;
  const double u = p.fx * qc.x() / qc.z(), v = p.fy * qc.y() / qc.z();
  std::printf("겨눔 화소: (%.1f, %.1f) 기대 (−60, −50) · 거리 %.3f J2 %.2f 벽 여유 %.3f\n", u, v, s.dist_m, s.j2_torque_nm, s.wall_clear_m);
  EXPECT_NEAR(u, -60.0, 2.0);
  EXPECT_NEAR(v, -50.0, 2.0);
}
