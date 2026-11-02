// 위치 감시 판정과 자이로 이동 치우침 학습 (ROS 없음, 시험 가능).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace navigation
{

inline double wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// 판정에 쓰는 지표. 없는 값(std::nullopt)은 "모름"
struct GuardMetrics
{
  std::optional<double> scan_age;       // 마지막 /scan 뒤 [s]
  std::optional<double> scan_hz;
  std::optional<int> scan_points;
  std::optional<double> match;          // 스캔 점 중 지도 벽 5 cm 안 비율 (최근 5 개 중앙값)
  double match_low_s{0.0};              // match < 0.60 이 이어진 시간 [s]
  double match_unknown_s{0.0};          // match 를 못 잰 시간 [s]
  std::optional<double> map_odom_age;   // map→odom TF 시각이 얼마나 오래됐나 [s]
  double jump_m{0.0};                   // 최근 1 s 안 map→odom 한 번 고친 양
  double jump_deg{0.0};
  std::optional<double> yaw_fix_rate;   // 최근 5 s map→odom 방향 고침 속도 [°/s]
  std::optional<double> imu_age;
  int gyro_spikes{0};                   // 최근 1 s 안 |ω| > 5 rad/s
  double acc_bad_s{0.0};                // 가속도 크기가 9.8 ± 3 밖에 머문 시간 [s]
};

struct GuardResult
{
  int level{0};                         // 0 정상 · 1 경고(느리게) · 2 정지
  std::vector<std::string> reasons;
};

// 기준: 지도 맞음 99 ~ 100 % · map→odom 한 번 고침 ≤ 6 cm · 1.4° · 라이다 10 Hz 약 260 점 인 실물 주행에서 정함
inline GuardResult evaluate(const GuardMetrics & m)
{
  std::vector<std::string> err, warn;
  char b[96];
  if (!m.scan_age || *m.scan_age > 0.5) {err.emplace_back("라이다 끊김");}
  if (m.map_odom_age && *m.map_odom_age > 1.0) {
    std::snprintf(b, sizeof(b), "SLAM TF %.1f s 멈춤", *m.map_odom_age);
    err.emplace_back(b);
  }
  if (!m.match) {
    (m.match_unknown_s > 2.0 ? err : warn).emplace_back("지도 맞음 모름 (지도 · TF · 스캔 기다림)");
  } else if (m.match_low_s >= 1.0) {
    std::snprintf(b, sizeof(b), "지도 맞음 %.0f%% (1 s 넘게 < 60%%)", *m.match * 100.0);
    err.emplace_back(b);
  } else if (*m.match < 0.85) {
    std::snprintf(b, sizeof(b), "지도 맞음 %.0f%%", *m.match * 100.0);
    warn.emplace_back(b);
  }
  const bool match_good = m.match && *m.match >= 0.85;
  if (m.jump_m > 0.25 || m.jump_deg > 10.0) {
    std::snprintf(b, sizeof(b), "SLAM 큰 보정 %.0f cm · %.1f°%s", m.jump_m * 100.0, m.jump_deg,
      match_good ? " 받아들임" : "");
    (match_good ? warn : err).emplace_back(b);
  } else if (m.jump_m > 0.10 || m.jump_deg > 3.0) {
    std::snprintf(b, sizeof(b), "SLAM 보정 %.0f cm · %.1f°", m.jump_m * 100.0, m.jump_deg);
    warn.emplace_back(b);
  }
  if (m.yaw_fix_rate) {
    std::snprintf(b, sizeof(b), "방향 고침 %.1f°/s", *m.yaw_fix_rate);
    if (*m.yaw_fix_rate > 3.0) {err.emplace_back(b);} else if (*m.yaw_fix_rate > 1.0) {warn.emplace_back(b);}
  }
  if (m.scan_hz && *m.scan_hz < 7.0) {
    std::snprintf(b, sizeof(b), "라이다 %.0f Hz", *m.scan_hz);
    warn.emplace_back(b);
  }
  if (m.scan_points && *m.scan_points < 150) {
    std::snprintf(b, sizeof(b), "라이다 점 %d", *m.scan_points);
    warn.emplace_back(b);
  }
  if (!m.imu_age || *m.imu_age > 0.5) {warn.emplace_back("IMU 끊김 (바퀴 회전값 사용)");}
  if (m.gyro_spikes > 0) {
    std::snprintf(b, sizeof(b), "자이로 튐 %d", m.gyro_spikes);
    warn.emplace_back(b);
  }
  if (m.acc_bad_s >= 1.0) {
    err.emplace_back("가속도 이상 1 s (들림 · 부딪힘?)");
  } else if (m.acc_bad_s > 0.0) {
    warn.emplace_back("가속도 튐");
  }
  GuardResult r;
  r.level = !err.empty() ? 2 : !warn.empty() ? 1 : 0;
  r.reasons = err;
  r.reasons.insert(r.reasons.end(), warn.begin(), warn.end());
  return r;
}

// 자이로 이동 치우침 학습: 움직이는 동안 자이로가 실제 회전을 덜 느끼는 만큼 [rad/s].
// window_s 동안 판정이 내내 정상이고 60 % 넘게 움직였으면, 그동안 map→odom 방향이 바뀐 속도(= 남은 치우침)에
// gain 을 곱해 더함. 판정이 한 번이라도 정상이 아니었던 창은 버림. 창은 겹치지 않음 (한 번의 SLAM 보정이 한 번만 반영).
class BiasLearner
{
public:
  BiasLearner(double initial, double gain, double max_abs, double window_s)
  : est_(std::clamp(initial, -max_abs, max_abs)), gain_(gain), max_(max_abs), window_(window_s) {}

  // t [s] · map→odom 방향 [rad] · 바퀴로 움직이는 중 · 판정 level. 창이 차면 학습하고 비움
  void add(double t, double map_odom_yaw, bool moving, int level)
  {
    hist_.push_back({t, map_odom_yaw, moving, level});
    if (hist_.back().t - hist_.front().t < window_) {return;}
    learn();
    hist_.clear();
  }

  double estimate() const {return est_;}

private:
  struct Sample
  {
    double t;
    double yaw;
    bool moving;
    int level;
  };

  void learn()
  {
    if (hist_.size() < 2) {return;}
    for (const auto & h : hist_) {if (h.level != 0) {return;}}
    double moving_s = 0.0;
    for (std::size_t k = 1; k < hist_.size(); ++k) {
      if (hist_[k].moving) {moving_s += hist_[k].t - hist_[k - 1].t;}
    }
    const double span = hist_.back().t - hist_.front().t;
    if (moving_s < 0.6 * span || moving_s <= 0.0) {return;}
    const double rate = wrap_angle(hist_.back().yaw - hist_.front().yaw) / moving_s;
    if (std::fabs(rate) > 0.02) {return;}   // 1.15 °/s 넘음 = 치우침이 아니라 위치 다시 잡기
    est_ = std::clamp(est_ + gain_ * rate, -max_, max_);
  }

  double est_;
  double gain_;
  double max_;
  double window_;
  std::vector<Sample> hist_;
};

// 점유 칸까지 거리 [칸] — 4 · 8 이웃을 번갈아 늘림(≈ 팔각), cap 칸에서 자름. occupied: 행 우선 (v · width + u)
inline std::vector<std::int16_t> wall_distance(
  const std::vector<std::uint8_t> & occupied, int width, int height, int cap)
{
  const std::size_t n = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  std::vector<std::int16_t> d(n, static_cast<std::int16_t>(cap));
  std::vector<std::uint8_t> reach(occupied), next;
  for (std::size_t i = 0; i < n; ++i) {if (reach[i]) {d[i] = 0;}}
  for (int k = 1; k < cap; ++k) {
    next = reach;
    for (int v = 0; v < height; ++v) {
      for (int u = 0; u < width; ++u) {
        const std::size_t i = static_cast<std::size_t>(v) * width + u;
        if (reach[i]) {continue;}
        auto at = [&](int uu, int vv) {
            return uu >= 0 && uu < width && vv >= 0 && vv < height &&
                   reach[static_cast<std::size_t>(vv) * width + uu];
          };
        bool hit = at(u - 1, v) || at(u + 1, v) || at(u, v - 1) || at(u, v + 1);
        if (!hit && k % 2 == 0) {
          hit = at(u - 1, v - 1) || at(u + 1, v + 1) || at(u - 1, v + 1) || at(u + 1, v - 1);
        }
        if (hit) {next[i] = 1; d[i] = static_cast<std::int16_t>(k);}
      }
    }
    reach.swap(next);
  }
  return d;
}

}  // namespace navigation
