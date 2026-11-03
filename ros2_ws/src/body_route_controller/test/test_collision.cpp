#include "body_route_controller/collision.hpp"
#include <gtest/gtest.h>
#include <nav2_costmap_2d/cost_values.hpp>

using namespace body_route_controller;
namespace
{
Footprint square()
{
  Footprint f;
  for (Point p : {Point{-.18, -.18}, Point{.18, -.18}, Point{.18, .18}, Point{-.18, .18}}) {
    geometry_msgs::msg::Point q; q.x = p.x; q.y = p.y; f.push_back(q);
  }
  return f;
}
void obstacle(nav2_costmap_2d::Costmap2D & m, double x, double y, unsigned char cost = 254)
{
  unsigned int mx, my; ASSERT_TRUE(m.worldToMap(x, y, mx, my)); m.setCost(mx, my, cost);
}
}
TEST(Collision, ChecksInteriorNotJustPolygonEdges)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .01, -1, -1, 0);
  EXPECT_TRUE(footprintSafe(map, square(), {}));
  obstacle(map, .05, .05);
  EXPECT_FALSE(footprintSafe(map, square(), {}));
}
TEST(Collision, RejectsUnknownOutOfMapAndMissingFootprint)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .01, -1, -1, 0);
  obstacle(map, .05, .05, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(footprintSafe(map, square(), {}));
  EXPECT_FALSE(footprintSafe(map, square(), {.95, 0, 0}));
  EXPECT_FALSE(footprintSafe(map, {}, {}));
}
TEST(Collision, ChecksEntireRotationIncludingIntermediateCorners)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .01, -1, -1, 0);
  obstacle(map, .24, 0);
  EXPECT_TRUE(footprintSafe(map, square(), {}));
  EXPECT_TRUE(footprintSafe(map, square(), {0, 0, 1.57079632679}));
  EXPECT_FALSE(turnSafe(map, square(), {}, 1.57079632679));
}
TEST(Collision, ChecksTranslationAndStoppingTail)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .01, -1, -1, 0);
  obstacle(map, .22, 0);
  EXPECT_TRUE(footprintSafe(map, square(), {}));
  EXPECT_FALSE(motionSafe(map, square(), {}, {.025, 0, 0}, 2));
  EXPECT_TRUE(motionSafe(map, square(), {}, {-.025, 0, 0}, 2));
}
TEST(Collision, ShiftedWallChangesOutcome)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .01, -1, -1, 0);
  obstacle(map, .25, 0);
  EXPECT_TRUE(motionSafe(map, square(), {}, {.025, 0, 0}, 2));
  map.resetMap(0, 0, 200, 200);
  obstacle(map, .20, 0);
  EXPECT_FALSE(motionSafe(map, square(), {}, {.025, 0, 0}, 2));
}
