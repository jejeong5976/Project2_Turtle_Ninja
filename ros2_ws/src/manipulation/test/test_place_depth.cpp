#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "manipulation/place_depth.hpp"

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
  const double assumed = 25.0 - (-15.0);  // 사용자 성공 확인한 R2 값임.
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
