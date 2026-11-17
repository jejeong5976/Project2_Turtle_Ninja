// 앞 · 뒤 라이다를 몸 중심 360° 스캔 하나로 합치는 계산 (ROS 없음, 시험 가능).
#pragma once

#include <algorithm>
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

// ---- 자기 몸 거르기 "wheels" (self_filter: wheels) ----
// 고정 큰 상자 (±0.17 × ±0.19) 는 옆 0.16–0.19 를 통째로 지워 PolygonStop (±0.18) 이 옆 사람을 못 봄.
// 대신: 차체 외곽 상자 (작게) + 바퀴 모듈마다 조향 각에 따라 도는 사각형만 버림.
//   모듈 사각형 = 조향축 기준 모듈 좌표 (u = 굴림 방향, v = 바퀴축 바깥쪽 +) 에서 |u| ≤ half_u, −in ≤ v ≤ out
//   조향 각을 모르면 (joint_states 끊김) 그 모듈이 어느 각도든 덮는 원 (반지름 = 사각형 모서리) 으로 버림
struct WheelModule
{
  double x;      // 조향축 [m] (base_footprint)
  double y;
  double side;   // +1 = 왼쪽 모듈 (조향 0° 에서 바깥 = +y), −1 = 오른쪽
};

struct SelfFilter
{
  bool box_mode = true;                 // true = body_half 상자만
  BodyBox box{0.17, 0.19};
  double chassis_x_min = -0.133, chassis_x_max = 0.141, chassis_half_y = 0.150;
  // 앞 · 뒤 면 띠 (wheels 만): 차체 폭 (|y| ≤ chassis_half_y) 안에서는 x 를 이 범위까지 버림 (상자 방식 앞뒤 ±0.17 과 같음).
  //   정밀 접근 정차 (박스 면 1 cm = 중심 0.152) 에서 박스 면이 FootprintApproach (±0.16) 안에 들어와 출발 명령을 다 0 으로 막던 것 (시뮬).
  //   옆 띠 (|y| > chassis_half_y) 는 그대로 봄 → 옆 사람
  double face_clear_x_min = -0.17, face_clear_x_max = 0.17;
  std::vector<WheelModule> modules;
  double wheel_half_u = 0.05, wheel_in = 0.05, wheel_out = 0.05;
  std::vector<double> steer;            // [rad] modules 순서. 크기가 다르면 (모름) 원으로

  double sweep_radius() const
  {
    return std::hypot(wheel_half_u, std::max(wheel_in, wheel_out));
  }
};

inline bool inside_wheel(const Point2 & p, const WheelModule & m, double steer,
  double half_u, double in, double out)
{
  const double dx = p.x - m.x, dy = p.y - m.y;
  const double c = std::cos(steer), s = std::sin(steer);
  const double u = c * dx + s * dy;
  const double v = (-s * dx + c * dy) * m.side;
  return std::fabs(u) <= half_u && v >= -in && v <= out;
}

inline bool inside_self(const Point2 & p, const SelfFilter & f)
{
  if (f.box_mode) {return inside_body(p, f.box);}
  if (std::fabs(p.y) <= f.chassis_half_y &&
    p.x >= std::min(f.chassis_x_min, f.face_clear_x_min) && p.x <= std::max(f.chassis_x_max, f.face_clear_x_max))
  {
    return true;
  }
  const bool known = f.steer.size() == f.modules.size();
  const double r = f.sweep_radius();
  for (std::size_t i = 0; i < f.modules.size(); ++i) {
    const auto & m = f.modules[i];
    if (known && std::isfinite(f.steer[i])) {
      if (inside_wheel(p, m, f.steer[i], f.wheel_half_u, f.wheel_in, f.wheel_out)) {return true;}
    } else if (std::hypot(p.x - m.x, p.y - m.y) <= r) {
      return true;
    }
  }
  return false;
}

// 스캔 한 장 → 몸 좌표 점. 범위 밖 · NaN · 자기 몸 (inside_self) 점은 버림. out 은 비우고 다시 채움 (버퍼 재사용)
inline void scan_to_points(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, const Mount & m, const SelfFilter & self, std::vector<Point2> & out)
{
  out.clear();
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r < range_min || r > range_max) {continue;}
    const double a = angle_min + angle_increment * static_cast<double>(i) + m.yaw;
    const Point2 p{m.x + r * std::cos(a), m.y + r * std::sin(a)};
    if (!inside_self(p, self)) {out.push_back(p);}
  }
}

// 몸 상자만 쓰는 판
inline void scan_to_points(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, const Mount & m, const BodyBox & body, std::vector<Point2> & out)
{
  SelfFilter f;
  f.box = body;
  scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, m, f, out);
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
