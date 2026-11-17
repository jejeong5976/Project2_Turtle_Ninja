#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "manipulation/place_depth.hpp"
#include "manipulation/arm_kinematics.hpp"
#include "arm_kin_data.hpp"

TEST(PlaceDepth, ZeroKeepsExistingHeightBitForBit)
{
  for (double h : {60.0, 34.8}) {
    for (double depth : {20.0, 25.0, 30.0, 40.0, 45.0, 50.0}) {
      for (double tilt : {0.0, 10.0, 20.0, 30.0}) {
        const double d = std::abs(tilt - 10.0) * M_PI / 180.0;
        const double old = 2.0 - h + (h - depth) * std::cos(d) + 17.4 * std::sin(d);
        const double now = manipulation::place_drop_height_mm(
          2.0, h, manipulation::retrieve_place_depth_mm(depth, "S1", 0.0), tilt, 10.0);
        EXPECT_EQ(std::memcmp(&old, &now, sizeof(double)), 0);
      }
    }
  }
}

TEST(PlaceDepth, S1ShallowerGripRaisesOnlyDrop)
{
  const double assumed = 25.0 - (-20.0);
  const double corrected = manipulation::retrieve_place_depth_mm(assumed, "S1", 5.0);
  EXPECT_DOUBLE_EQ(assumed, 45.0);
  EXPECT_DOUBLE_EQ(corrected, 40.0);
  EXPECT_DOUBLE_EQ(manipulation::place_drop_height_mm(2.0, 60.0, assumed, 0.0, 0.0), -43.0);
  EXPECT_DOUBLE_EQ(manipulation::place_drop_height_mm(2.0, 60.0, corrected, 0.0, 0.0), -38.0);
}

TEST(PlaceDepth, S2UsesConfirmedRetrieveOffset)
{
  const double assumed = 25.0 - (-15.0);  // 실물에서 성공을 확인한 R2 값
  const double corrected = manipulation::retrieve_place_depth_mm(assumed, "S2", 4.0);
  EXPECT_DOUBLE_EQ(corrected, 36.0);
  EXPECT_DOUBLE_EQ(manipulation::place_drop_height_mm(2.0, 60.0, corrected, 0.0, 0.0), -34.0);
}

TEST(PlaceDepth, TiltedDropUsesProjection)
{
  const double old = manipulation::place_drop_height_mm(2.0, 60.0, 45.0, 20.0, 0.0);
  const double corrected = manipulation::retrieve_place_depth_mm(45.0, "S1", 5.0);
  const double now = manipulation::place_drop_height_mm(2.0, 60.0, corrected, 20.0, 0.0);
  EXPECT_NEAR(now - old, 5.0 * std::cos(20.0 * M_PI / 180.0), 1e-12);
}

TEST(PlaceDepth, PickAndRegraspHaveNoRetrieveCorrection)
{
  EXPECT_DOUBLE_EQ(manipulation::retrieve_place_depth_mm(50.0, "", 5.0), 50.0);
  EXPECT_DOUBLE_EQ(manipulation::retrieve_place_depth_mm(30.0, "", 5.0), 30.0);
}

TEST(PlaceDepth, InvalidMeasurementKeepsAssumption)
{
  for (double fix : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), 45.0, 46.0}) {
    EXPECT_DOUBLE_EQ(manipulation::retrieve_place_depth_mm(45.0, "S1", fix), 45.0);
  }
  EXPECT_DOUBLE_EQ(manipulation::retrieve_place_depth_mm(0.0, "S1", 5.0), 0.0);
}

TEST(PlaceDepth, SignedCorrectionIsNotClamped)
{
  EXPECT_DOUBLE_EQ(manipulation::retrieve_place_depth_mm(40.0, "S2", -2.0), 42.0);
}

// 박스 PICK (pick_top_bias_mm 10 → 명령 50, 실제 40) 뒤 PLACE 가 명령 깊이로 −48 에 놓으면 퍽 바닥이 면 8 mm 아래 (퍽을 밂).
//   실제 깊이 40 으로 −38 (바닥이 면 위 2 mm). RETRIEVE (25, 보정 0) → PLACE 는 그대로 −23
TEST(PlaceDepth, BoxPickTopBiasUsesRealDepth)
{
  EXPECT_DOUBLE_EQ(manipulation::place_real_depth_mm(50.0, 10.0), 40.0);
  EXPECT_DOUBLE_EQ(manipulation::place_drop_height_mm(2.0, 60.0, manipulation::place_real_depth_mm(50.0, 10.0), 0.0, 0.0), -38.0);
  EXPECT_DOUBLE_EQ(manipulation::place_drop_height_mm(2.0, 60.0, 50.0, 0.0, 0.0), -48.0);   // 명령 깊이 그대로면
  EXPECT_DOUBLE_EQ(manipulation::place_drop_height_mm(2.0, 60.0, manipulation::place_real_depth_mm(25.0, 0.0), 0.0, 0.0), -23.0);
  // 받침에서 다시 쥔 퍽 (30, 보정 0) · 누운 퍽 (20) 그대로
  EXPECT_DOUBLE_EQ(manipulation::place_real_depth_mm(30.0, 0.0), 30.0);
  EXPECT_DOUBLE_EQ(manipulation::place_drop_height_mm(2.0, 34.8, manipulation::place_real_depth_mm(20.0, 0.0), 0.0, 0.0), -18.0);
}

TEST(PlaceDepth, TopBiasInvalidKeepsHeldDepth)
{
  for (double b : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), 50.0, 60.0}) {
    EXPECT_DOUBLE_EQ(manipulation::place_real_depth_mm(50.0, b), 50.0);
  }
  EXPECT_DOUBLE_EQ(manipulation::place_real_depth_mm(0.0, 10.0), 0.0);   // 쥔 깊이 모름
  EXPECT_DOUBLE_EQ(manipulation::place_real_depth_mm(50.0, -5.0), 55.0);   // 윗면을 낮게 본 보정 (부호 그대로)
}

// 기울여 쥐고 다시 쥐지 않고 든 퍽 — PICK 은 위 · 내림은 실제 윗면, 쥐기는 박스 PICK 과 같은 보정한 윗면 (pick_top_bias_mm 10)
//   기준으로 기울인 손 깊이 30 (pick_tilted_depth_mm) → 실제 깊이 20. 기울인 손 보정을 0 으로 기록하면 PLACE 가 30 으로 놓음 (퍽 바닥 10 mm 낮음)
TEST(PlaceDepth, TiltedHoldTopBiasUsesRealDepth)
{
  using namespace manipulation;
  const auto m = kin_data::model();
  const double real_top = 0.058, bias_mm = 10.0;   // 박스 위 선 퍽 윗면 (모델) · 보정 (test_grasp_select run_spots 의 0.068 = 윗면 + 10)
  std::string why;
  const auto P = pick_poses(m, 0.30, -0.02, real_top + bias_mm / 1000.0, 0.0, {40.0 - bias_mm, 10.0 - bias_mm, -30.0}, why, std::nan(""), 30.0, 20.0,
    10.0, {}, false);
  ASSERT_EQ(P.size(), 3u) << why;
  EXPECT_GT(P.back().tilt_deg, 0.5);
  const double tip_z = arm_grip(m, ticks_to_q(m, P.back().ticks)).tip.z();
  const double real_mm = (real_top - tip_z) * 1000.0;   // 실제 윗면 아래 손끝 깊이
  EXPECT_NEAR(real_mm, 20.0, 0.5);
  EXPECT_NEAR(place_real_depth_mm(30.0, bias_mm), real_mm, 0.5);   // 명령 30 · 보정 10 → 실제 20 (pending_top_bias_mm_ = 보정 그대로)
  const double tilt = P.back().tilt_deg;
  const double now = place_drop_height_mm(2.0, 60.0, place_real_depth_mm(30.0, bias_mm), tilt, tilt);
  const double old = place_drop_height_mm(2.0, 60.0, place_real_depth_mm(30.0, 0.0), tilt, tilt);
  EXPECT_NEAR(now, -18.0, 1e-9);   // 퍽 바닥이 면 위 2 mm
  EXPECT_NEAR(old, -28.0, 1e-9);   // 보정 0 으로 기록하면: 면 아래 8 mm
  EXPECT_DOUBLE_EQ(place_real_depth_mm(30.0, 0.0), 30.0);   // 받침 · 누운 퍽 · RETRIEVE (보정 0) 은 그대로
}

TEST(FarRegraspPoint, ZeroOffsetKeepsPointBitForBit)
{
  for (double sx : {0.303, 0.299, 0.25}) {
    for (double sy : {0.026, -0.023, 0.0}) {
      for (const auto & off : {std::vector<double>{0.0, 0.0}, std::vector<double>{}, std::vector<double>{1.0},
          std::vector<double>{std::nan(""), 5.0}}) {
        double gx = 9.0, gy = 9.0;
        manipulation::far_regrasp_point(sx, sy, off, gx, gy);
        EXPECT_EQ(std::memcmp(&gx, &sx, sizeof(double)), 0);
        EXPECT_EQ(std::memcmp(&gy, &sy, sizeof(double)), 0);
      }
    }
  }
}

TEST(FarRegraspPoint, OffsetMovesRegraspOnly)
{
  double gx = 0.0, gy = 0.0;
  manipulation::far_regrasp_point(0.303, 0.026, {-4.0, 19.0}, gx, gy);
  EXPECT_NEAR(gx, 0.299, 1e-12);
  EXPECT_NEAR(gy, 0.045, 1e-12);
}

// ---------------------------------------------------------------- 먼 자리 PLACE 의 기울인 다시 쥐기 · 놓기 · 빼기
//   30° 손으로 다시 쥔 원기둥은 손가락 끝만으로 쥐어져 놓을 때 밀려 날아감 (실제 로봇 L2).
//   URDF 손가락 메시: 안쪽 면 = 손끝에서 손 축으로 0–35 mm · 폭 ±9 mm. 선 원기둥과는 세로 줄 하나로 닿음 →
//   손끝이 퍽 축 위면 30° 에서 닿는 퍽 높이 18 mm (21–39), 손끝을 4 mm (pad 8) 바깥으로 옮기면 27 mm (19–46) · 8 mm (pad 16) 33 mm (18–51)

TEST(CylFar, TiltedRegripShiftPutsPadCentreOnPuckAxis)
{
  using manipulation::tilted_regrip_shift_mm;
  EXPECT_NEAR(tilted_regrip_shift_mm(8.0, 30.0), 4.0, 1e-9);   // 설정값 (place_far_pad_mm 8)
  EXPECT_NEAR(tilted_regrip_shift_mm(16.0, 30.0), 8.0, 1e-9);
  EXPECT_NEAR(tilted_regrip_shift_mm(16.0, 40.0), 16.0 * std::sin(40.0 * M_PI / 180.0), 1e-9);
  EXPECT_DOUBLE_EQ(tilted_regrip_shift_mm(16.0, 0.0), 0.0);   // 수직 손 = 옮김 없음
  EXPECT_DOUBLE_EQ(tilted_regrip_shift_mm(0.0, 30.0), 0.0);    // 0 = 옮김 없음
  EXPECT_DOUBLE_EQ(tilted_regrip_shift_mm(std::nan(""), 30.0), 0.0);
  EXPECT_DOUBLE_EQ(tilted_regrip_shift_mm(16.0, std::nan("")), 0.0);
}

TEST(CylFar, FarPlaceReleasesLowerNotHigher)
{
  // 보통 PLACE 식: 6 − 60 + (60 − 40) = −34 (바닥이 면 위 6 mm 에서 떨어뜨림). 먼 자리 식: 1 − 40 = −39 (1 mm)
  const double old = manipulation::place_drop_height_mm(6.0, 60.0, 40.0, 30.0, 30.0);
  EXPECT_DOUBLE_EQ(old, -34.0);
  EXPECT_DOUBLE_EQ(manipulation::far_place_drop_mm(1.0, 40.0), -39.0);
  for (double clear : {0.0, 1.0, 6.0}) {
    EXPECT_LE(manipulation::far_place_drop_mm(clear, 40.0), old + 1e-12);   // 보통 PLACE 식보다 높게 놓지 않음 (여유 6 이하)
  }
}

TEST(CylFar, AxisRetreatMovesInwardByTan)
{
  double ox, oy;
  // J1 (0.0225, 0) 에서 +x 쪽 퍽: −39 → +10 mm 는 손 축으로 안쪽 49 · tan 30° = 28.3 mm
  manipulation::axis_retreat_xy(0.3305, 0.0, 0.0225, 0.0, 30.0, -39.0, 10.0, ox, oy);
  EXPECT_NEAR((0.3305 - ox) * 1000.0, 49.0 * std::tan(30.0 * M_PI / 180.0), 1e-9);
  EXPECT_NEAR(oy, 0.0, 1e-12);
  manipulation::axis_retreat_xy(0.3, 0.05, 0.0225, 0.0, 0.0, -39.0, 10.0, ox, oy);   // 수직 손 = 그대로
  EXPECT_DOUBLE_EQ(ox, 0.3);
  EXPECT_DOUBLE_EQ(oy, 0.05);
  manipulation::axis_retreat_xy(0.3, 0.05, 0.0225, 0.0, std::nan(""), -39.0, 10.0, ox, oy);
  EXPECT_DOUBLE_EQ(ox, 0.3);
}

namespace
{
// control_master place_far_plan 과 같은 계산 (목표 놓기 · 손 축 빼기)
struct FarPlan
{
  std::vector<manipulation::PickPose> P, Pu, Pc;
  double s_mm{0.0};
};
FarPlan far_plan(const manipulation::ArmModel & m, double x, double y, double top, double tilt, double pad, double clear, double depth)
{
  using namespace manipulation;
  FarPlan f;
  const Eigen::Vector2d d = (Eigen::Vector2d(x, y) - m.j1_xy).normalized();
  const double sf = tilted_regrip_shift_mm(pad, tilt);
  const double h = far_place_drop_mm(clear, depth);
  std::string why;
  for (double k : {1.0, 0.5, 0.0}) {
    const double tx = x + d.x() * sf * k / 1000.0, ty = y + d.y() * sf * k / 1000.0;
    const double cd = (std::atan2(ty - m.j1_xy.y(), tx - m.j1_xy.x()) + M_PI / 2.0) * 180.0 / M_PI;
    f.P = pick_poses(m, tx, ty, top, cd, {40.0, 10.0, h}, why, cd, 40.0, tilt, 0.5);
    if (f.P.size() != 3) {continue;}
    f.s_mm = sf * k;
    double ux, uy, cx, cy;
    axis_retreat_xy(tx, ty, m.j1_xy.x(), m.j1_xy.y(), tilt, h, 10.0, ux, uy);
    axis_retreat_xy(tx, ty, m.j1_xy.x(), m.j1_xy.y(), tilt, h, 40.0, cx, cy);
    f.Pu = pick_poses(m, ux, uy, top, cd, {10.0}, why, cd, 40.0, tilt, 0.5, {ticks_to_q(m, f.P[2].ticks)});
    if (f.Pu.size() == 1) {f.Pc = pick_poses(m, cx, cy, top, cd, {40.0}, why, cd, 40.0, tilt, 0.5, {ticks_to_q(m, f.Pu[0].ticks)});}
    break;
  }
  return f;
}
}  // namespace

// 박스 먼 줄 (실제 로봇 L2 두 목표 포함): 30° 손으로 손끝을 pad · sin 30° 바깥으로 옮겨 놓을 수 있음 · 손끝에서 손 축 pad mm 점이 퍽 축 위 ·
//   놓을 때 퍽 바닥 1 mm · 손 축 빼기 (J5 그대로, 관절 보간 중 손끝이 곧은 줄에서 닫힘축 쪽 0.5 mm · 손가락 면 안 3 mm 안, 손끝이 놓은 퍽 축 바깥쪽 · 위로만)
TEST(CylFar, FarRowTargetsReachShiftedAndRetreatAlongAxis)
{
  using namespace manipulation;
  const ArmModel m = kin_data::model();
  const double top = 0.058;   // 박스 위 선 퍽 윗면 (모델 z)
  int n = 0;
  for (double pad : {8.0, 16.0}) {   // 설정값 8 · 대안 16
    for (auto p : std::vector<std::pair<double, double>>{{0.328, -0.054}, {0.323, 0.066}, {0.318, 0.0}, {0.315, -0.04}, {0.31, 0.04}}) {
      SCOPED_TRACE(std::to_string(pad) + " / " + std::to_string(p.first) + ", " + std::to_string(p.second));
      const FarPlan f = far_plan(m, p.first, p.second, top, 30.0, pad, 1.0, 40.0);
      ASSERT_EQ(f.P.size(), 3u);
      EXPECT_NEAR(f.s_mm, pad / 2.0, 1e-9);   // 다 옮김으로 닿음 (기울임 30° 그대로)
      const Grip g = arm_grip(m, ticks_to_q(m, f.P[2].ticks));
      EXPECT_NEAR(g.tip.z(), top - 0.039, 1e-3);   // 놓는 손끝 = 윗면 아래 39 mm → 퍽 바닥 (40 mm 쥠) 이 면 위 1 mm
      const Eigen::Vector3d pad_c = g.tip - pad / 1000.0 * g.app;   // 손끝에서 손 축 pad mm
      EXPECT_LT(std::hypot(pad_c.x() - p.first, pad_c.y() - p.second) * 1000.0, 1.0);
      ASSERT_EQ(f.Pu.size(), 1u);
      ASSERT_EQ(f.Pc.size(), 1u);
      const Q5 q0 = ticks_to_q(m, f.P[2].ticks), q1 = ticks_to_q(m, f.Pu[0].ticks), q2 = ticks_to_q(m, f.Pc[0].ticks);
      EXPECT_LT(std::fabs(q1[4] - q0[4]), 0.35);
      EXPECT_LT(std::fabs(q2[4] - q0[4]), 0.35);
      // drop → up 관절 보간: 손끝이 손 축 줄 근처 · 기울임 그대로 (1° 안)
      const Grip g1 = arm_grip(m, q1);
      const Eigen::Vector3d ax = (g1.tip - g.tip).normalized();
      EXPECT_GT(ax.dot(-g.app), 0.999);   // 빼는 방향 = 손 축
      for (int k = 1; k < 10; ++k) {
        Q5 q;
        for (int j = 0; j < 5; ++j) {q[j] = q0[j] + (q1[j] - q0[j]) * k / 10.0;}
        const Grip gi = arm_grip(m, q);
        const Eigen::Vector3d v = gi.tip - g.tip, off = v - v.dot(ax) * ax;
        EXPECT_LT(std::fabs(off.dot(g.close)) * 1000.0, 0.5);   // 닫힘축 쪽 (손가락이 퍽에 닿는 쪽, 다 열면 여유 8.8 mm) 으로는 안 벗어남
        EXPECT_LT(off.norm() * 1000.0, 3.0);                      // 손가락 면 안 (반지름 · 위 평면) 으로 휨 2.5 mm 까지
        EXPECT_GT(gi.app.dot(g.app), std::cos(1.0 * M_PI / 180.0));
      }
      ++n;
    }
  }
  EXPECT_EQ(n, 10);
}

// 수직 손 (0°) 은 아무것도 안 바뀜 (먼 자리 놓기는 늘 기울인 손이지만 값 0 · 기울임 0 이면 옮김 없는 점)
TEST(CylFar, ZeroPadIsOldBehaviour)
{
  using namespace manipulation;
  const ArmModel m = kin_data::model();
  const FarPlan f = far_plan(m, 0.323, 0.066, 0.058, 30.0, 0.0, 6.0, 40.0);
  ASSERT_EQ(f.P.size(), 3u);
  EXPECT_DOUBLE_EQ(f.s_mm, 0.0);
  const Grip g = arm_grip(m, ticks_to_q(m, f.P[2].ticks));
  EXPECT_LT(std::hypot(g.tip.x() - 0.323, g.tip.y() - 0.066) * 1000.0, 1.0);
}
