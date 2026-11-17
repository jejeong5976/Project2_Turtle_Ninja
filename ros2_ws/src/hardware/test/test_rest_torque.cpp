// test_rest_torque.cpp — 쉬는 팔 (rest_torque.hpp): 요청 신선도 · 끄기/켜기 바꿈 · 다시 켠 뒤 옛 명령 막기
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "hardware/rest_torque.hpp"

using hardware::RestGate;
using hardware::RestTorque;
using hardware::rest_wanted;

namespace
{
const double NaN = std::numeric_limits<double>::quiet_NaN();
}

// 요청이 true · 신선 · 보드 깨끗일 때만 끔. 요청이 끊기면 (control_master 죽음) 다시 켬
TEST(RestTorque, WantedOnlyWithFreshRequestOnCleanBoard)
{
  EXPECT_TRUE(rest_wanted(true, 0.05, 1.0, true));
  EXPECT_TRUE(rest_wanted(true, 1.0, 1.0, true));
  EXPECT_FALSE(rest_wanted(false, 0.05, 1.0, true));   // 요청 끝
  EXPECT_FALSE(rest_wanted(true, 1.01, 1.0, true));    // 끊김
  EXPECT_FALSE(rest_wanted(true, NaN, 1.0, true));     // 받은 적 없음
  EXPECT_FALSE(rest_wanted(true, -0.1, 1.0, true));    // 시계 이상
  EXPECT_FALSE(rest_wanted(true, 0.05, 1.0, false));   // 보드 이상
}

// 끄기 · 켜기 바뀔 때만 OFF · ON. 켤 때 막기 시작, 끌 때 막기 지움
TEST(RestTorque, ChangesAndGateArming)
{
  RestTorque r;
  const std::vector<bool> gated{true, true, false};
  EXPECT_EQ(r.step(false, gated), RestTorque::Change::NONE);
  EXPECT_FALSE(r.gate().holding());
  EXPECT_EQ(r.step(true, gated), RestTorque::Change::OFF);
  EXPECT_TRUE(r.resting());
  EXPECT_FALSE(r.gate().holding());   // 끈 동안은 펌웨어가 목표를 안 씀
  EXPECT_EQ(r.step(true, gated), RestTorque::Change::NONE);
  EXPECT_EQ(r.step(false, gated), RestTorque::Change::ON);
  EXPECT_FALSE(r.resting());
  EXPECT_TRUE(r.gate().holding(0));
  EXPECT_TRUE(r.gate().holding(1));
  EXPECT_FALSE(r.gate().holding(2));   // 바퀴 (속도) 는 막지 않음
  r.reset();                           // 하드웨어 다시 켜기
  EXPECT_FALSE(r.resting());
  EXPECT_FALSE(r.gate().holding());
}

// 다시 켠 뒤: 옛 명령 (처지기 전 자세) 은 NaN (= 펌웨어가 지금 위치에서 버팀). 명령이 측정 tol 안에 오면 풀리고 그 뒤로는 그대로
TEST(RestTorque, GateHoldsOldCommandUntilCommandReachesMeasured)
{
  RestGate g;
  g.arm({true, true, true});
  const std::vector<double> meas{0.10, -1.30, 0.010};   // 처진 자세 (그리퍼 m)
  const std::vector<double> tol{0.02, 0.02, 0.02 * 0.01149};
  // JTC 가 계속 HOME 끝 자세를 명령 (끄기 전): 관절 0 은 0.15 rad 처짐 → 막음, 관절 1 은 거의 같음 → 풀림
  std::vector<double> t{-0.05, -1.31, 0.0119};
  EXPECT_EQ(g.filter(t, meas, tol), 1);
  EXPECT_TRUE(std::isnan(t[0]));
  EXPECT_DOUBLE_EQ(t[1], -1.31);
  EXPECT_TRUE(std::isnan(t[2]));       // 그리퍼 1.9 mm 차이 > 0.23 mm
  // 옛 명령이 계속 오는 동안은 계속 막음
  for (int k = 0; k < 50; ++k) {
    std::vector<double> u{-0.05, -1.32, 0.0119};
    EXPECT_EQ(g.filter(u, meas, tol), 0);
    EXPECT_TRUE(std::isnan(u[0]));
    EXPECT_DOUBLE_EQ(u[1], -1.32);   // 풀린 관절은 그대로
  }
  // 다시 앉히기 궤적: JTC 가 측정 자세에서 시작 → 첫 명령이 측정 근처 → 풀림 → 궤적 그대로 따라감
  std::vector<double> v{0.095, -1.30, 0.0119};
  EXPECT_EQ(g.filter(v, meas, tol), 1);
  EXPECT_DOUBLE_EQ(v[0], 0.095);
  std::vector<double> w{0.0, -1.30, 0.0119};
  g.filter(w, meas, tol);
  EXPECT_DOUBLE_EQ(w[0], 0.0);
  EXPECT_TRUE(g.holding());            // 그리퍼는 다음 그리퍼 명령까지
  std::vector<double> x{0.0, -1.30, 0.01001};
  EXPECT_EQ(g.filter(x, meas, tol), 1);
  EXPECT_FALSE(g.holding());
}

// 측정이 없거나 (NaN) 명령이 없으면 (NaN) 풀지 않음. 측정이 돌아오면 그때 판단
TEST(RestTorque, GateNeedsMeasurementAndCommand)
{
  RestGate g;
  g.arm({true, true});
  std::vector<double> t{0.10, NaN};
  EXPECT_EQ(g.filter(t, {NaN, 0.0}, {0.02, 0.02}), 0);
  EXPECT_TRUE(std::isnan(t[0]));
  EXPECT_TRUE(std::isnan(t[1]));
  std::vector<double> u{0.10, 0.0};
  EXPECT_EQ(g.filter(u, {0.11, 0.0}, {0.02, 0.02}), 2);
  EXPECT_FALSE(g.holding());
}

// 쉬는 중 · 막는 중 다시 끄기: 막기를 지움 (다음 켤 때 새로 막음). 짧은 끄기 → 켜기 반복에도 매번 처음부터 막음
TEST(RestTorque, RearmEveryWake)
{
  RestTorque r;
  const std::vector<bool> gated{true};
  const std::vector<double> tol{0.02};
  r.step(true, gated);
  r.step(false, gated);
  std::vector<double> t{0.0};
  r.gate().filter(t, {0.0}, tol);   // 같은 자리 → 풀림
  EXPECT_FALSE(r.gate().holding());
  r.step(true, gated);
  r.step(false, gated);
  std::vector<double> u{0.0};
  EXPECT_EQ(r.gate().filter(u, {0.2}, tol), 0);   // 이번엔 0.2 rad 처짐 → 다시 막음
  EXPECT_TRUE(std::isnan(u[0]));
}

// URDF 한계 지킴: 끈 동안 어느 관절이든 한계 margin 안 → 다시 켬 + 요청이 끝날 때까지 켠 채. 끄기 전부터 가까우면 안 끔
//   (lyrical JTC 는 측정이 URDF 한계 밖이면 컨트롤러를 끔 — HOME J2 −1.4511 은 한계 −1.5 에서 0.049 rad)
TEST(RestTorque, NearLimitWakesAndLatchesUntilRequestEnds)
{
  using hardware::near_limit;
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<double> lo{-3.14, -1.5, -1.5, -1.7, -1.5708, -0.003}, hi{3.14, 1.5, 1.4, 0.6735, 1.5708, 0.0175};
  const std::vector<double> margin{0.02, 0.02, 0.02, 0.02, 0.02, 0.02 * 0.01149};
  std::vector<double> q{-0.0061, -1.4511, 0.9511, -0.1181, 0.0230, -0.0007};   // HOME (그리퍼 빈손 닫힘)
  EXPECT_EQ(near_limit(q, lo, hi, margin), -1);
  q[1] = -1.479;
  EXPECT_EQ(near_limit(q, lo, hi, margin), -1);
  q[1] = -1.481;
  EXPECT_EQ(near_limit(q, lo, hi, margin), 1);
  q[1] = NaN;   // 모름은 안 봄 (읽기 실패는 하드웨어 ERROR 쪽)
  EXPECT_EQ(near_limit(q, lo, hi, margin), -1);
  EXPECT_EQ(near_limit({5.0}, {-inf}, {inf}, {0.02}), -1) << "한계 없는 관절";
  q[1] = -1.4511;
  q[5] = 0.0174;
  EXPECT_EQ(near_limit(q, lo, hi, margin), 5);

  RestTorque r;
  const std::vector<bool> gated(6, true);
  EXPECT_EQ(r.step(true, gated, -1, true), RestTorque::Change::OFF);
  EXPECT_EQ(r.step(true, gated, 1, true), RestTorque::Change::ON) << "J2 가 한계 가까이 → 다시 켬";
  EXPECT_EQ(r.limit_joint(), 1);
  EXPECT_EQ(r.step(true, gated, -1, true), RestTorque::Change::NONE) << "요청이 이어지는 동안 켠 채 (래치)";
  EXPECT_FALSE(r.resting());
  EXPECT_EQ(r.step(false, gated, -1, false), RestTorque::Change::NONE);   // control_master 가 켜기 요청 (작업) → 래치 풂
  EXPECT_EQ(r.limit_joint(), -1);
  EXPECT_EQ(r.step(true, gated, -1, true), RestTorque::Change::OFF) << "다음 쉼은 다시 끔";
  // 끄기 전부터 가까우면 안 끔
  RestTorque s;
  EXPECT_EQ(s.step(true, gated, 1, true), RestTorque::Change::NONE);
  EXPECT_FALSE(s.resting());
  EXPECT_EQ(s.limit_joint(), 1);
  // 쉬지 않는 동안 (요청 없음) 은 한계와 상관없음
  RestTorque u;
  EXPECT_EQ(u.step(false, gated, 1, false), RestTorque::Change::NONE);
  EXPECT_EQ(u.limit_joint(), -1);
}
