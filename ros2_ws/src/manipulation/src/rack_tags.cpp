#include "manipulation/rack_tags.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace manipulation
{

TagOnPlane tag_on_plane(
  const std::array<Eigen::Vector2d, 4> & corners_n, const Eigen::Isometry3d & T_base_cam, const TagPlaneConfig & c)
{
  TagOnPlane out;
  const Eigen::Vector3d o = T_base_cam.translation();
  std::array<Eigen::Vector2d, 4> p;
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d d = (T_base_cam.linear() * Eigen::Vector3d(corners_n[i].x(), corners_n[i].y(), 1.0)).normalized();
    if (!(d.z() < -c.min_view_cos)) {   // NaN 도 여기서 걸림
      out.why = "광선이 너무 비스듬하거나 아래를 안 봄";
      return out;
    }
    const double t = (c.plane_z - o.z()) / d.z();
    if (!(t > 0.0)) {
      out.why = "카메라가 칸 바닥보다 낮음";
      return out;
    }
    p[i] = (o + t * d).head<2>();
  }
  double sum = 0.0;
  for (int i = 0; i < 4; ++i) {
    const double s = (p[(i + 1) % 4] - p[i]).norm();
    if (std::fabs(s - c.tag_size_m) > c.size_tol * c.tag_size_m) {
      out.side_m = s;
      out.why = "태그 크기가 안 맞음 (칸 바닥 높이 · 손목 TF 확인)";
      return out;
    }
    sum += s;
  }
  out.side_m = sum / 4.0;
  out.center = (p[0] + p[1] + p[2] + p[3]) / 4.0;
  const Eigen::Vector2d ax = (p[1] - p[0]) + (p[2] - p[3]);
  out.yaw = std::atan2(ax.y(), ax.x());
  out.ok = true;
  return out;
}

RackFit fit_rack(const std::vector<Eigen::Vector2d> & expected, const std::vector<Eigen::Vector2d> & measured,
  const Eigen::Vector2d & pivot, double min_span_m)
{
  RackFit f;
  const size_t n = expected.size();
  if (n == 0 || measured.size() != n) {
    return f;
  }
  Eigen::Vector2d me = Eigen::Vector2d::Zero(), mm = Eigen::Vector2d::Zero();
  for (size_t i = 0; i < n; ++i) {
    me += expected[i];
    mm += measured[i];
  }
  me /= static_cast<double>(n);
  mm /= static_cast<double>(n);
  double span = 0.0;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      span = std::max(span, (expected[i] - expected[j]).norm());
    }
  }
  if (span >= min_span_m) {
    // 2 차원 Kabsch: 가운데를 뺀 두 점 묶음 사이 각
    double dot = 0.0, cross = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const Eigen::Vector2d a = expected[i] - me, b = measured[i] - mm;
      dot += a.dot(b);
      cross += a.x() * b.y() - a.y() * b.x();
    }
    f.dyaw = std::atan2(cross, dot);
    f.rotation = true;
  }
  const Eigen::Rotation2Dd R(f.dyaw);
  const Eigen::Vector2d t = mm - R * (me - pivot) - pivot;
  f.dx = t.x();
  f.dy = t.y();
  f.n = static_cast<int>(n);
  f.ok = true;
  double se = 0.0;
  for (size_t i = 0; i < n; ++i) {
    se += (expected[i] + rack_shift(f, expected[i], pivot) - measured[i]).squaredNorm();
  }
  f.rms = std::sqrt(se / static_cast<double>(n));
  return f;
}

Eigen::Vector2d rack_shift(const RackFit & f, const Eigen::Vector2d & expected, const Eigen::Vector2d & pivot)
{
  const Eigen::Rotation2Dd R(f.dyaw);
  return R * (expected - pivot) + pivot + Eigen::Vector2d(f.dx, f.dy) - expected;
}

double median(std::vector<double> v)
{
  if (v.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const size_t h = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + h, v.end());
  if (v.size() % 2) {
    return v[h];
  }
  const double hi = v[h];
  return (*std::max_element(v.begin(), v.begin() + h) + hi) / 2.0;
}

}  // namespace manipulation
