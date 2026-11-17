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
    if (!in_blink_ && status_ == SignalStatus::Recording && t - off_since_ >= cfg_.end_off_s) {
      // 끝 소등: 12 개 이상이면 끝. 모자라면 중간부터 본 것 → 끝내지 않고 다음 시퀀스를 기다림
      if (static_cast<int>(blinks_.size()) >= cfg_.expected) {
        finish(t);
      } else {
        park_partial(off_since_);
      }
    }
    return;
  }

  // 켜짐 (색 또는 애매)
  off_since_ = -1.0;
  const char c = o == SignalObs::Red ? 'R' : o == SignalObs::Yellow ? 'Y' : o == SignalObs::Green ? 'G' : 0;
  if (in_blink_ && c != 0 && cur_color_ != 0 && c != cur_color_ && blink_last_on_ - blink_start_ >= cfg_.min_on_s) {
    close_blink(t);
    if (status_ == SignalStatus::Recording) {issue(fmt("소등 없이 색이 바뀜 (t=%.1f s) - 점멸을 나눔", t));}
  }
  if (!in_blink_) {
    in_blink_ = true;
    blink_start_ = t;
    n_red_ = n_yellow_ = n_green_ = n_amb_ = 0;
    cur_color_ = 0;
  }
  blink_last_on_ = t;
  switch (o) {
    case SignalObs::Red: ++n_red_; break;
    case SignalObs::Yellow: ++n_yellow_; break;
    case SignalObs::Green: ++n_green_; break;
    default: ++n_amb_; break;
  }
  if (c != 0) {cur_color_ = c;}
}

void SignalRecorder::close_blink(double)
{
  in_blink_ = false;
  // 켜짐 길이 = 첫 ~ 마지막 켜짐 관측. 다음 Off 도착 시각을 쓰지 않음 (관측 끊김만큼 길어짐)
  const double on_s = blink_last_on_ - blink_start_;
  if (on_s < cfg_.min_on_s) {
    notes_.push_back(fmt("짧은 켜짐 %.2f s 버림 (t=%.1f s)", on_s, blink_start_));
    return;
  }
  SignalBlink b;
  b.start = blink_start_;
  b.end = blink_last_on_;
  const int total = n_red_ + n_yellow_ + n_green_ + n_amb_;
  const int best = std::max({n_red_, n_yellow_, n_green_});
  const char best_c = best == n_red_ ? 'R' : best == n_yellow_ ? 'Y' : 'G';
  const bool known = total > 0 && best > 0 && best >= cfg_.majority * total && n_amb_ <= cfg_.max_ambiguous * total;
  char buf[200];
  std::snprintf(buf, sizeof(buf), "R %d · Y %d · G %d · 애매 %d, t=%.1f s", n_red_, n_yellow_, n_green_, n_amb_, blink_start_);
  if (status_ == SignalStatus::Waiting) {
    // 기록은 첫 쓸 만한 점멸 (색 확실 + min_on_s 이상) 부터. 광각이 막 켜질 때 '?' 점멸은 세지 않음
    if (!known) {
      notes_.push_back(std::string("시작 전 색 불명 점멸 버림 (") + buf + ")");
      return;
    }
    status_ = SignalStatus::Recording;
  }
  if (known) {
    b.color = best_c;
  } else {
    b.color = '?';
    issue("점멸 " + std::to_string(blinks_.size() + 1) + " 색 불명 (" + buf + ") - 추정 안 함");
  }
  blinks_.push_back(b);
  if (static_cast<int>(blinks_.size()) == cfg_.expected + 1) {
    issue(fmt("점멸이 %.0f 개보다 많음", cfg_.expected));
  }
}

// 12 개 전에 끝 소등: 시퀀스 처음을 못 봤거나 점멸을 놓침. 끝내지 않고 남겨 둔 뒤 다음 시퀀스를 처음부터 기다림
void SignalRecorder::park_partial(double t)
{
  ++n_partial_;
  partial_seq_ = sequence();
  partial_blinks_ = blinks_;
  partial_issues_ = issues_;
  SignalPartial h;
  h.n = n_partial_;
  h.parked = t;
  h.sequence = partial_seq_;
  h.blinks = blinks_;
  h.issues = issues_;
  partial_history_.push_back(std::move(h));
  char buf[200];
  std::snprintf(buf, sizeof(buf), "점멸 %zu 개 (%s) 뒤 끝 소등 (t=%.1f s) - 처음을 못 본 시퀀스로 보고 다음 시퀀스 기다림",
    blinks_.size(), partial_seq_.c_str(), t);
  notes_.push_back(buf);
  blinks_.clear();
  issues_.clear();
  status_ = SignalStatus::Waiting;
  gap_since_ = -1.0;
  gap_flagged_ = false;
}

void SignalRecorder::give_up(double t, const std::string & why)
{
  if (finished()) {return;}
  if (in_blink_) {close_blink(t);}
  if (blinks_.empty() && n_partial_ > 0) {   // 지금 기록이 없으면 마지막 partial 을 결과로 (추정 아님, 본 그대로)
    blinks_ = partial_blinks_;
    issues_ = partial_issues_;
    issue(fmt("점멸 %.0f 개 뒤 끝 소등이 옴 - 시퀀스 처음을 못 봤거나 점멸을 놓침", static_cast<double>(blinks_.size())));
  }
  issue(why);
  const int n = static_cast<int>(blinks_.size());
  if (n != cfg_.expected) {issue(fmt("점멸 %.0f 개 (기대 %.0f 개)", n, cfg_.expected));}
  status_ = SignalStatus::Incomplete;
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
