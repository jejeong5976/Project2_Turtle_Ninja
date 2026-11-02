// signal_recorder.cpp — 신호등 점멸 순서 기록 구현, 설명은 signal_recorder.hpp

#include "mission/signal_recorder.hpp"

#include <algorithm>
#include <cstdio>

namespace mission
{

namespace
{
std::string fmt(const char * f, double a, double b = 0.0)
{
  char buf[160];
  std::snprintf(buf, sizeof(buf), f, a, b);
  return buf;
}
}  // namespace

void SignalRecorder::observe(double t, SignalObs o)
{
  if (finished()) {return;}
  if (last_t_ >= 0.0 && t < last_t_) {return;}   // 시각이 거꾸로 온 관측은 버림
  last_t_ = t;

  // 못 봄: 켜짐 · 소등 어느 쪽으로도 안 셈. 기록 중 오래 끊기면 점멸을 놓쳤을 수 있음
  if (o == SignalObs::NotSeen) {
    off_since_ = -1.0;                             // 소등은 끊기지 않고 이어져야 인정
    if (gap_since_ < 0.0) {gap_since_ = t;}
    if (status_ == SignalStatus::Recording && !gap_flagged_ && t - gap_since_ >= cfg_.max_gap_s) {
      issue(fmt("신호등 관측 끊김 %.1f s 이상 (t=%.1f s) - 점멸을 놓쳤을 수 있음", t - gap_since_, gap_since_));
      gap_flagged_ = true;
    }
    return;
  }
  // 다시 보임: 못 본 시간 = 처음 못 본 시각 ~ 지금 (마지막 못 본 관측까지로 재면 관측 간격만큼 짧게 잡힘)
  if (gap_since_ >= 0.0 && status_ == SignalStatus::Recording && !gap_flagged_ && t - gap_since_ >= cfg_.max_gap_s) {
    issue(fmt("신호등 관측 끊김 %.1f s (t=%.1f s) - 점멸을 놓쳤을 수 있음", t - gap_since_, gap_since_));
  }
  gap_since_ = -1.0;
  gap_flagged_ = false;

  if (o == SignalObs::Off) {
    if (off_since_ < 0.0) {off_since_ = t;}
    if (in_blink_ && t - off_since_ >= cfg_.min_off_s) {close_blink(off_since_);}
    if (!in_blink_ && status_ == SignalStatus::Recording && t - off_since_ >= cfg_.end_off_s) {finish(t);}
    return;
  }

  // 켜짐 (색 또는 애매)
  off_since_ = -1.0;
  const char c = o == SignalObs::Red ? 'R' : o == SignalObs::Yellow ? 'Y' : o == SignalObs::Green ? 'G' : 0;
  if (in_blink_ && c != 0 && cur_color_ != 0 && c != cur_color_ && t - blink_start_ >= cfg_.min_on_s) {
    issue(fmt("소등 없이 색이 바뀜 (t=%.1f s) - 점멸을 나눔", t));
    close_blink(t);
  }
  if (!in_blink_) {
    in_blink_ = true;
    blink_start_ = t;
    n_red_ = n_yellow_ = n_green_ = n_amb_ = 0;
    cur_color_ = 0;
    if (status_ == SignalStatus::Waiting) {status_ = SignalStatus::Recording;}
  }
  blink_last_color_ = t;
  switch (o) {
    case SignalObs::Red: ++n_red_; break;
    case SignalObs::Yellow: ++n_yellow_; break;
    case SignalObs::Green: ++n_green_; break;
    default: ++n_amb_; break;
  }
  if (c != 0) {cur_color_ = c;}
}

void SignalRecorder::close_blink(double t_end)
{
  in_blink_ = false;
  const double on_s = t_end - blink_start_;
  if (on_s < cfg_.min_on_s) {
    notes_.push_back(fmt("짧은 켜짐 %.2f s 버림 (t=%.1f s)", on_s, blink_start_));
    if (blinks_.empty()) {status_ = SignalStatus::Waiting;}   // 첫 점멸 전 깜빡임이면 아직 시작 안 한 것
    return;
  }
  SignalBlink b;
  b.start = blink_start_;
  b.end = blink_last_color_;
  const int total = n_red_ + n_yellow_ + n_green_ + n_amb_;
  const int best = std::max({n_red_, n_yellow_, n_green_});
  const char best_c = best == n_red_ ? 'R' : best == n_yellow_ ? 'Y' : 'G';
  if (total > 0 && best > 0 && best >= cfg_.majority * total && n_amb_ <= cfg_.max_ambiguous * total) {
    b.color = best_c;
  } else {
    b.color = '?';
    char buf[160];
    std::snprintf(buf, sizeof(buf), "점멸 %zu 색 불명 (R %d · Y %d · G %d · 애매 %d, t=%.1f s) - 추정 안 함",
      blinks_.size() + 1, n_red_, n_yellow_, n_green_, n_amb_, blink_start_);
    issue(buf);
  }
  blinks_.push_back(b);
  if (static_cast<int>(blinks_.size()) == cfg_.expected + 1) {
    issue(fmt("점멸이 %.0f 개보다 많음", cfg_.expected));
  }
}

void SignalRecorder::finish(double)
{
  const int n = static_cast<int>(blinks_.size());
  if (n != cfg_.expected) {
    issue(fmt("점멸 %.0f 개 (기대 %.0f 개)", n, cfg_.expected));
  }
  if (count('?') == 0 && n == cfg_.expected) {
    for (char c : {'R', 'Y', 'G'}) {
      const int k = count(c);
      if (k < cfg_.min_count || k > cfg_.max_count) {
        char buf[120];
        std::snprintf(buf, sizeof(buf), "%c 횟수 %d (규칙 %d-%d)", c, k, cfg_.min_count, cfg_.max_count);
        issue(buf);
      }
    }
  }
  status_ = issues_.empty() ? SignalStatus::Done : SignalStatus::Incomplete;
}

std::string SignalRecorder::sequence() const
{
  std::string s;
  for (const auto & b : blinks_) {s += b.color;}
  return s;
}

int SignalRecorder::count(char c) const
{
  return static_cast<int>(std::count_if(blinks_.begin(), blinks_.end(), [c](const SignalBlink & b) {return b.color == c;}));
}

const char * signal_status_name(SignalStatus s)
{
  switch (s) {
    case SignalStatus::Waiting: return "WAITING";
    case SignalStatus::Recording: return "RECORDING";
    case SignalStatus::Done: return "DONE";
    case SignalStatus::Incomplete: return "INCOMPLETE";
  }
  return "?";
}

}  // namespace mission
