// camera_drift.cpp — 홈 사진 화소 쌍 → 손목 카메라 회전 (camera_drift.hpp)

#include "manipulation/camera_drift.hpp"

#include <cmath>
#include <sstream>
#include <string>

namespace manipulation
{

bool camera_drift(const std::vector<Eigen::Vector2d> & u_ref, const std::vector<Eigen::Vector2d> & u_now,
  double fx, double fy, double cx, double cy, CameraDrift & out)
{
  out = CameraDrift{};
  if (u_ref.size() != u_now.size() || u_ref.size() < 3 || !(fx > 0.0) || !(fy > 0.0)) {return false;}
  auto ray = [&](const Eigen::Vector2d & u) {return Eigen::Vector3d((u.x() - cx) / fx, (u.y() - cy) / fy, 1.0).normalized();};
  // 지금 광선 b = M · 기준 광선 a 인 회전 M (Kabsch). 몸통 점 B 는 명목 카메라에서 a, 실제 카메라 (명목 · R) 에서 b = Rᵀ a → R = Mᵀ
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < u_ref.size(); ++i) {H += ray(u_ref[i]) * ray(u_now[i]).transpose();}
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
  D(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
  const Eigen::Matrix3d M = svd.matrixV() * D * svd.matrixU().transpose();
  out.R = M.transpose();
  out.deg = Eigen::AngleAxisd(out.R).angle() * 180.0 / M_PI;
  double s = 0.0;
  for (size_t i = 0; i < u_ref.size(); ++i) {
    const Eigen::Vector3d b = M * ray(u_ref[i]);
    const Eigen::Vector2d u(fx * b.x() / b.z() + cx, fy * b.y() / b.z() + cy);
    s += (u - u_now[i]).norm();
  }
  out.n = static_cast<int>(u_ref.size());
  out.resid_px = s / out.n;
  return true;
}

Eigen::Quaterniond chain_drift(const Eigen::Quaterniond & base, const Eigen::Quaterniond & meas)
{
  return (base.normalized() * meas.normalized()).normalized();
}

double drift_deg(const Eigen::Quaterniond & q)
{
  return Eigen::AngleAxisd(q.normalized()).angle() * 180.0 / M_PI;
}

std::string ref_base_tag(std::uintmax_t size, long long mtime_s)
{
  return "ref " + std::to_string(size) + " " + std::to_string(mtime_s);
}

bool ref_base_tag_matches(const std::string & stored, const std::string & now)
{
  std::istringstream a(stored), b(now);
  std::string ka, kb;
  unsigned long long sa = 0, sb = 0;
  long long ma = 0, mb = 0;
  if (!(a >> ka)) {return true;}   // 옛 형식 (사원수만, 짝 줄 없음) — 뒤 호환
  if (ka != "ref" || !(a >> sa >> ma)) {return false;}
  if (!(b >> kb >> sb >> mb) || kb != "ref") {return false;}
  return sa == sb && ma == mb;
}

}  // namespace manipulation
