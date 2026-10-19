#include <gtest/gtest.h>
#include <cmath>
#include "nt_hardware/position_rate.hpp"

using nt_hardware::PositionRate;

TEST(PositionRate, LinearBothSignsAndWarmup)
{
  for (double rate : {-0.175, 0.175}) {
    PositionRate estimate;
    for (uint32_t ms = 0; ms <= 200; ms += 10) {
      const double v = estimate.feed(ms, 1.23 + rate * ms / 1000);
      if (ms < 30) {EXPECT_TRUE(std::isnan(v));}
      else {EXPECT_NEAR(v, rate, 1e-10);}
    }
  }
}

TEST(PositionRate, ClockWrapAndFullOneMillisecondWindow)
{
  PositionRate estimate;
  for (uint32_t i = 0; i <= 200; ++i) {
    const double v = estimate.feed(0xfffffff0u + i, i * 0.0002);
    if (i >= 30) {EXPECT_NEAR(v, 0.2, 1e-10);}
  }
}

TEST(PositionRate, ClockFaultStaysInvalidUntilReset)
{
  for (uint32_t bad : {30u, 29u, 81u}) {
    PositionRate estimate;
    for (uint32_t ms = 0; ms <= 30; ms += 10) {estimate.feed(ms, 0.0);}
    EXPECT_TRUE(std::isnan(estimate.feed(bad, 0.1)));
    for (uint32_t i = 1; i <= 10; ++i) {
      EXPECT_TRUE(std::isnan(estimate.feed(bad + 10 * i, 0.1)));
    }
    estimate = PositionRate{};
    for (uint32_t ms = 0; ms < 30; ms += 10) {estimate.feed(ms, 0.2);}
    EXPECT_DOUBLE_EQ(estimate.feed(30, 0.2), 0.0);
  }
}

TEST(PositionRate, MissingPositionRequiresFreshWindow)
{
  PositionRate estimate;
  for (uint32_t ms = 0; ms <= 30; ms += 10) {estimate.feed(ms, ms * 0.001);}
  EXPECT_TRUE(std::isnan(estimate.feed(40, NAN)));
  for (uint32_t ms = 50; ms < 80; ms += 10) {
    EXPECT_TRUE(std::isnan(estimate.feed(ms, 0.1)));
  }
  EXPECT_DOUBLE_EQ(estimate.feed(80, 0.1), 0.0);
}

TEST(PositionRate, StationaryTickNoiseAndStopTail)
{
  PositionRate estimate;
  constexpr double tick = 2 * 3.141592653589793 / 4096;
  for (uint32_t ms = 0; ms < 200; ms += 10) {
    const double v = estimate.feed(ms, (ms / 10 % 2) * tick);
    if (ms >= 30) {EXPECT_LT(std::abs(v), 0.04);}
  }
  for (uint32_t ms = 200; ms < 300; ms += 10) {estimate.feed(ms, ms * 0.001);}
  for (uint32_t ms = 300; ms <= 360; ms += 10) {estimate.feed(ms, 0.3);}
  EXPECT_DOUBLE_EQ(estimate.feed(370, 0.3), 0.0);
}
