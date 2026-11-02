// grasp.cpp — 퍽 앞 자세 · 집기 축 · 그리퍼 목표 계산 구현, 설명은 grasp.hpp

#include "planning/grasp.hpp"

#include <cmath>

namespace planning
{

Pose2D approach_pose(double robot_x, double robot_y, double puck_x, double puck_y, double standoff)
{
  double dx = puck_x - robot_x;
  double dy = puck_y - robot_y;
  double d = std::hypot(dx, dy);
  if (d < 1e-6) {   // 로봇이 퍽 위에 있다고 나오는 이상한 경우 → +x 방향으로 가정
    dx = 1.0;
    dy = 0.0;
    d = 1.0;
  }
  const double ux = dx / d;
  const double uy = dy / d;
  return {puck_x - ux * standoff, puck_y - uy * standoff, std::atan2(uy, ux)};
}

GraspAxis choose_grasp_axis(bool has_q, const Quat & q, const Vec3 & center)
{
  if (!has_q) {return {{0.0, 0.0, 1.0}, false};}
  // 회전 행렬의 z 열 = 퍽 z 축(긴축)을 기준 프레임으로 돌린 것
  Vec3 a{2.0 * (q.x * q.z + q.w * q.y), 2.0 * (q.y * q.z - q.w * q.x), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)};
  const double n = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  if (n < 1e-9) {return {{0.0, 0.0, 1.0}, false};}
  a = {a.x / n, a.y / n, a.z / n};

  bool flip = false;
  if (std::fabs(a.z) >= std::sqrt(0.5)) {
    flip = a.z < 0.0;                                    // 서 있음 → 위쪽 끝
  } else {
    flip = a.x * center.x + a.y * center.y > 0.0;        // 누움 → 로봇(원점) 쪽 끝
  }
  if (flip) {a = {-a.x, -a.y, -a.z};}
  return {a, flip};
}

Vec3 grasp_point(const Vec3 & center, const Vec3 & axis, double grasp_offset)
{
  return {center.x + axis.x * grasp_offset, center.y + axis.y * grasp_offset, center.z + axis.z * grasp_offset};
}

namespace
{

// 쿼터니언 회전 행렬의 x 열 · z 열 (= 퍽 x 축 · 긴축을 기준 프레임으로 돌린 것)
Vec3 quat_col_x(const Quat & q)
{
  return {1.0 - 2.0 * (q.y * q.y + q.z * q.z), 2.0 * (q.x * q.y + q.w * q.z), 2.0 * (q.x * q.z - q.w * q.y)};
}

Vec3 quat_col_z(const Quat & q)
{
  return {2.0 * (q.x * q.z + q.w * q.y), 2.0 * (q.y * q.z - q.w * q.x), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)};
}

double norm(const Vec3 & v) {return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);}

// a 를 [-period/2, period/2) 로
double wrap(double a, double period)
{
  double r = std::fmod(a + period / 2.0, period);
  if (r < 0.0) {r += period;}
  return r - period / 2.0;
}

// v 를 단위 벡터 z 에 수직인 성분만 남겨 정규화. 너무 작으면 false
bool perp_unit(const Vec3 & v, const Vec3 & z, Vec3 & out)
{
  const double d = v.x * z.x + v.y * z.y + v.z * z.z;
  Vec3 p{v.x - d * z.x, v.y - d * z.y, v.z - d * z.z};
  const double n = norm(p);
  if (n < 1e-6) {return false;}
  out = {p.x / n, p.y / n, p.z / n};
  return true;
}

}  // namespace

PuckPlanar planar_from_pose(bool has_q, const Quat & q, const Vec3 & center, bool yaw_known, bool is_box)
{
  PuckPlanar p;
  p.center = center;
  if (!has_q) {return p;}   // 방향 모름 → 선 퍽, θ 모름
  Vec3 a = quat_col_z(q);
  const double n = norm(a);
  if (n < 1e-9) {return p;}
  a = {a.x / n, a.y / n, a.z / n};
  p.upright = std::fabs(a.z) >= std::sqrt(0.5);
  if (p.upright) {
    if (is_box && yaw_known) {
      const Vec3 f = quat_col_x(q);
      if (std::hypot(f.x, f.y) > 1e-6) {
        p.theta = wrap(std::atan2(f.y, f.x), M_PI / 2.0);
        p.theta_known = true;
      }
    }
  } else if (std::hypot(a.x, a.y) > 1e-6) {
    p.theta = wrap(std::atan2(a.y, a.x), M_PI);
    p.theta_known = true;
  }
  return p;
}

Quat grasp_orientation(const PuckPlanar & p, const GraspAxis & ga)
{
  const Vec3 z = ga.axis;
  Vec3 ref;
  if (!p.upright) {
    ref = {0.0, 0.0, 1.0};                                       // 누운 퍽: 위쪽
  } else if (p.theta_known) {
    ref = {std::cos(p.theta), std::sin(p.theta), 0.0};           // 면 방향
  } else if (std::hypot(p.center.x, p.center.y) > 1e-6) {
    ref = {p.center.x, p.center.y, 0.0};                         // 로봇 → 퍽
  } else {
    ref = {1.0, 0.0, 0.0};
  }
  Vec3 x;
  if (!perp_unit(ref, z, x) && !perp_unit({1.0, 0.0, 0.0}, z, x)) {
    perp_unit({0.0, 1.0, 0.0}, z, x);
  }
  return quat_from_xz(x, z);
}

Quat quat_from_xz(const Vec3 & x, const Vec3 & z)
{
  const Vec3 y{z.y * x.z - z.z * x.y, z.z * x.x - z.x * x.z, z.x * x.y - z.y * x.x};
  // 행렬 [x y z] (열) → 쿼터니언 (Shepperd)
  const double m00 = x.x, m01 = y.x, m02 = z.x;
  const double m10 = x.y, m11 = y.y, m12 = z.y;
  const double m20 = x.z, m21 = y.z, m22 = z.z;
  const double tr = m00 + m11 + m22;
  Quat q;
  if (tr > 0.0) {
    const double s = std::sqrt(tr + 1.0) * 2.0;
    q = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s};
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    q = {0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    q = {(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s};
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    q = {(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s};
  }
  if (q.w < 0.0) {q = {-q.x, -q.y, -q.z, -q.w};}
  return q;
}

}  // namespace planning
