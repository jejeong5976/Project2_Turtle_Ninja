// test_common.cpp — 공용 시험: 다수결 · 태그 ID 약속 · 메시지 규칙
//   실행: colcon build → source install/setup.bash → ./build/perception/test_common

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "perception/common.hpp"
#include "msg_utils.hpp"

using namespace perception;

TEST(Voter, MajorityOfWindow)
{
  Voter<std::string> v(5, 3);
  EXPECT_FALSE(v.push("red").first.has_value());
  v.push("green");
  v.push("red");
  const auto r = v.push("red");
  ASSERT_TRUE(r.first.has_value());
  EXPECT_EQ(*r.first, "red");
  v.clear();
  EXPECT_FALSE(v.result().first.has_value());
}

TEST(TagPolicy, PuckOnly)
{
  EXPECT_TRUE(is_puck_tag(0));
  EXPECT_TRUE(is_puck_tag(11));
  EXPECT_FALSE(is_puck_tag(12));   // 장소 태그
  EXPECT_FALSE(is_puck_tag(-1));
}

TEST(MsgUtils, UnknownPose)
{
  geometry_msgs::msg::Pose p;   // 기본값 w=1
  set_unknown_pose(p);
  EXPECT_TRUE(std::isnan(p.position.x) && std::isnan(p.position.y) && std::isnan(p.position.z));
  EXPECT_EQ(p.orientation.w, 0.0);
  EXPECT_EQ(p.orientation.x, 0.0);
}

TEST(MsgUtils, PuckIdString)
{
  EXPECT_EQ(puck_id_string(7), "7");
  EXPECT_EQ(puck_id_string(-1), "");
  EXPECT_EQ(puck_id_string(13), "");   // 장소 태그
}

TEST(MsgUtils, LabelShape)
{
  EXPECT_EQ(puck_label_shape("red_cyl"), std::make_pair(std::string("red"), std::string("cylinder")));
  EXPECT_EQ(puck_label_shape("green_box"), std::make_pair(std::string("green"), std::string("box")));
  EXPECT_EQ(puck_label_shape("green_box_up").second, "box");
  EXPECT_EQ(puck_label_shape("barcode").second, "");
}

TEST(MsgUtils, TrafficLightCode)
{
  using interfaces::msg::TrafficLight;
  EXPECT_EQ(traffic_light_code("red"), TrafficLight::RED);
  EXPECT_EQ(traffic_light_code("yellow"), TrafficLight::YELLOW);
  EXPECT_EQ(traffic_light_code("green"), TrafficLight::GREEN);
  EXPECT_EQ(traffic_light_code("off"), TrafficLight::OFF);
  EXPECT_EQ(traffic_light_code("unknown"), TrafficLight::UNKNOWN);
}
