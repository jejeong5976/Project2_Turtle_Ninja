// 앞 · 뒤 라이다를 몸 중심 360° 스캔 하나로 합치는 계산 (ROS 없음, 시험 가능).
#pragma once

#include <cmath>
#include <limits>
#include <vector>

namespace navigation
{

struct Point2
{
  double x;
  double y;
};

// 라이다가 몸(frame)에 붙은 자세
struct Mount
{
  double x;
  double y;
  double yaw;
};

// 자기 몸(바퀴 · 기둥 · 선)으로 보고 버리는 직사각형 반길이 · 반폭 [m]
struct BodyBox
{
  double half_x;
  double half_y;
};

inline bool inside_body(const Point2 & p, const BodyBox & b)
{
  return std::fabs(p.x) <= b.half_x && std::fabs(p.y) <= b.half_y;
}

// 스캔 한 장 → 몸 좌표 점. 범위 밖 · NaN · 몸 상자 안 점은 버림. out 은 비우고 다시 채움 (버퍼 재사용)
inline void scan_to_points(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, const Mount & m, const BodyBox & body, std::vector<Point2> & out)
{
  out.clear();
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r < range_min || r > range_max) {continue;}
    const double a = angle_min + angle_increment * static_cast<double>(i) + m.yaw;
    const Point2 p{m.x + r * std::cos(a), m.y + r * std::sin(a)};
    if (!inside_body(p, body)) {out.push_back(p);}
  }
}

// 점 → bins 칸 거리. 칸 k 의 가운데 각 = −π + k · 2π/bins (LaserScan angle_min = −π 와 같음), 칸마다 가장 가까운 점
inline void bin_points(const std::vector<Point2> & pts, int bins, std::vector<float> & ranges)
{
  ranges.assign(static_cast<std::size_t>(bins), std::numeric_limits<float>::infinity());
  const double inc = 2.0 * M_PI / bins;
  for (const auto & p : pts) {
    long k = std::lround((std::atan2(p.y, p.x) + M_PI) / inc) % bins;
    if (k < 0) {k += bins;}
    const float r = static_cast<float>(std::hypot(p.x, p.y));
    if (r < ranges[static_cast<std::size_t>(k)]) {ranges[static_cast<std::size_t>(k)] = r;}
  }
}

}  // namespace navigation
