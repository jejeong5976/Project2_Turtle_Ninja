// test_input_watch.cpp — 영상 끊김 → 구독 다시 만들기 규칙 (input_watch.hpp) 시험. ROS 없음.
//   카메라 30 Hz 로 오면 구독을 안 건드림 · 5 s 조용하면 다시 만듦 · 연달아면 2 배씩 (최대 60 s) · 받으면 처음으로
#include <gtest/gtest.h>

#include "perception/input_watch.hpp"

using perception::InputWatch;

TEST(InputWatch, FlowingCameraIsLeftAlone)
{
  InputWatch w;   // 기본 5 s
  w.start(0);
  for (double t = 0.0; t < 120.0; t += 1.0 / 30.0) {
    w.seen(t);
    EXPECT_FALSE(w.due(t + 0.01));
  }
  EXPECT_EQ(w.count(), 0);
}

TEST(InputWatch, SilentSubscriptionIsRecreatedWithBackoff)
{
  InputWatch w(5.0, 60.0);
  w.start(100);
  EXPECT_FALSE(w.due(104.9));
  EXPECT_TRUE(w.due(105));        // 한 번도 못 받음 → 5 s 뒤
  EXPECT_FALSE(w.ever());
  EXPECT_LT(w.since_msg(105), 0);
  w.start(105);                   // 구독 다시 만듦
  EXPECT_FALSE(w.due(114.9));
  EXPECT_TRUE(w.due(115));        // 10 s
  EXPECT_FALSE(w.due(134.9));
  EXPECT_TRUE(w.due(135));        // 20 s
  EXPECT_TRUE(w.due(175));        // 40 s
  EXPECT_TRUE(w.due(235));        // 최대 60 s
  EXPECT_EQ(w.count(), 5);
}

TEST(InputWatch, MessageAfterResubscribeResetsWait)
{
  InputWatch w(5.0, 60.0);
  w.start(0);
  EXPECT_TRUE(w.due(5));
  EXPECT_TRUE(w.due(15));
  w.seen(16);                     // 새 구독이 받기 시작
  EXPECT_FALSE(w.due(20.9));
  EXPECT_TRUE(w.due(21));         // 다시 끊기면 5 s 부터
  EXPECT_DOUBLE_EQ(w.since_msg(21), 5.0);
}

TEST(InputWatch, ZeroDisables)
{
  InputWatch w(0.0, 60.0);
  w.start(0);
  EXPECT_FALSE(w.due(1e6));
}
