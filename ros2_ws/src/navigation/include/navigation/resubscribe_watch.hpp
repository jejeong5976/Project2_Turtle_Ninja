// resubscribe_watch.hpp — 구독이 조용하면 다시 만들 때인지 판단 (ROS 없음, 시험 대상)
// 실물 시작 3 번 중 1 번꼴로 발행자는 도는데 어떤 노드 구독만 아무것도 못 받음 (DDS 짝짓기 유실, CPU 부하 큰 시작 때).
//   구독을 새로 만들면 새 reader 가 다시 짝을 찾음 → 끄고 다시 켜지 않고 풀림.
// 받음: 단조 시각 [s], 메시지 받은 시각. 보냄: due() = 지금 구독을 다시 만들 것.
// 규칙: 마지막 메시지 (또는 마지막 구독 생성) 뒤 wait 넘게 조용하면 due. 연달아 다시 만들 때마다 wait 2 배 (max_wait 까지),
//   메시지를 받으면 처음 wait 로. wait_s <= 0 = 안 봄. 지도처럼 한 번만 오는 토픽은 받은 뒤 부르는 쪽이 더 안 부름.
#pragma once

#include <algorithm>

namespace navigation
{

class ResubscribeWatch
{
public:
  explicit ResubscribeWatch(double wait_s = 10.0, double max_wait_s = 60.0)
  : base_(wait_s), max_(std::max(wait_s, max_wait_s)) {}

  // 구독을 만든 (또는 감시를 다시 시작한) 시각. 연달아 다시 만든 횟수 (기다림 2 배) 는 메시지를 받아야 처음으로
  void start(double now)
  {
    quiet_from_ = now;
  }

  void seen(double now)
  {
    quiet_from_ = now;
    last_msg_ = now;
    ever_ = true;
    streak_ = 0;
  }

  double wait() const
  {
    double w = base_;
    for (int i = 0; i < streak_ && w < max_; ++i) {
      w *= 2.0;
    }
    return std::min(w, max_);
  }

  // true 면 부르는 쪽이 구독을 다시 만듦 (여기서 그 시각을 기록)
  bool due(double now)
  {
    if (base_ <= 0.0 || now - quiet_from_ < wait()) {
      return false;
    }
    quiet_from_ = now;
    ++streak_;
    ++count_;
    return true;
  }

  int count() const {return count_;}
  bool ever() const {return ever_;}
  // 마지막 메시지 뒤 [s] (한 번도 안 왔으면 음수)
  double since_msg(double now) const {return ever_ ? now - last_msg_ : -1.0;}

private:
  double base_, max_;
  double quiet_from_ = 0.0;
  double last_msg_ = 0.0;
  bool ever_ = false;
  int streak_ = 0;
  int count_ = 0;
};

}  // namespace navigation
