// wrist_rack.cpp — 적재함 컵 바닥 태그 → 칸 중심 구현, 설명은 wrist_rack.hpp

#include "perception/wrist_rack.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "perception/common.hpp"

namespace perception::wrist
{

namespace
{
constexpr double kDeg = 180.0 / M_PI;

double angle_deg(const Vec3 & a, const Vec3 & b)
{
  return std::acos(std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0)) * kDeg;
}

Mat3 to_eigen(const cv::Mat & rvec)
{
  cv::Mat Rm;
  cv::Rodrigues(rvec, Rm);
  Mat3 R;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {R(r, c) = Rm.at<double>(r, c);}
  }
  return R;
}

double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
}  // namespace

std::vector<RackTag> detect_rack_tags(
  const cv::Mat & gray, const Intrinsics & K, const cv::Mat & dist, const RackParams & p,
  const Vec3 * up_cam, std::vector<std::pair<RackTag, std::string>> * rejected)
{
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  detect_apriltags(gray, corners, ids);   // common.hpp (광각 · 손목 같은 검출기)

  const cv::Matx33d Kc(K.fx, 0, K.cx, 0, K.fy, K.cy, 0, 0, 1);
  const cv::Mat D = dist.empty() ? cv::Mat::zeros(1, 5, CV_64F) : dist;
  const double h = p.tag_side_m / 2;
  const std::vector<cv::Point3f> obj{
    {static_cast<float>(-h), static_cast<float>(h), 0.f}, {static_cast<float>(h), static_cast<float>(h), 0.f},
    {static_cast<float>(h), static_cast<float>(-h), 0.f}, {static_cast<float>(-h), static_cast<float>(-h), 0.f}};
  // 태그 → 칸: 칸 x = 태그 y, 칸 y = −태그 x, z 같음 (z 축 +90° 회전)
  Mat3 Rz90;
  Rz90 << 0, -1, 0, 1, 0, 0, 0, 0, 1;

  std::vector<RackTag> out;
  std::set<int> seen;
  for (size_t k = 0; k < ids.size(); ++k) {
    const int cell = rack_cell_of(ids[k], p);
    if (cell == 0) {continue;}   // 퍽 · 장소 태그
    RackTag t;
    t.tag_id = ids[k];
    t.cell = cell;
    t.name = rack_cell_name(cell);

    // 모서리 부화소 보정 (aruco 4.6 기본은 보정 없음). 창 = 태그 한 변의 약 1/20
    std::vector<cv::Point2f> c = corners[k];
    const double side_px = cv::norm(c[0] - c[1]);
    const int win = std::clamp(static_cast<int>(side_px / 20.0), 2, 5);
    cv::cornerSubPix(gray, c, cv::Size(win, win), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
    for (int i = 0; i < 4; ++i) {t.corners[i] = c[i];}

    if (seen.count(ids[k])) {   // 같은 ID 두 개 = 반사 · 오검출, 둘 다 못 믿음
      if (rejected) {rejected->push_back({t, "dup"});}
      out.erase(std::remove_if(out.begin(), out.end(), [&](const RackTag & r) {return r.tag_id == ids[k];}), out.end());
      continue;
    }
    seen.insert(ids[k]);

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> errs;
    const int n = cv::solvePnPGeneric(obj, c, Kc, D, rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE,
        cv::noArray(), cv::noArray(), errs);
    if (n <= 0) {
      if (rejected) {rejected->push_back({t, "pnp"});}
      continue;
    }
    // 두 해 중: 위쪽을 알면 법선이 위쪽에 가까운 해, 모르면 재투영 오차 작은 해 (첫 해)
    int best = 0;
    if (up_cam && n > 1) {
      double best_dot = -2.0;
      for (int i = 0; i < n; ++i) {
        const double d = to_eigen(rvecs[i]).col(2).dot(*up_cam);
        if (d > best_dot) {best_dot = d; best = i;}
      }
    }
    const Mat3 R_tag = to_eigen(rvecs[best]);
    t.t = Vec3(tvecs[best].at<double>(0), tvecs[best].at<double>(1), tvecs[best].at<double>(2));
    t.R = R_tag * Rz90;
    t.reproj_px = errs.empty() ? 0.0 : errs[static_cast<size_t>(best)];
    const Vec3 nrm = R_tag.col(2);   // 태그 면 법선 (카메라 쪽)
    t.view_deg = angle_deg(nrm, -t.t);
    t.tilt_deg = up_cam ? angle_deg(nrm, *up_cam) : 0.0;

    std::string why;
    if (t.reproj_px > p.max_reproj_px) {why = "reproj";}
    else if (t.view_deg > p.max_view_deg) {why = "view";}
    else if (up_cam && t.tilt_deg > p.max_tilt_deg) {why = "tilt";}
    if (!why.empty()) {
      if (rejected) {rejected->push_back({t, why});}
      continue;
    }
    out.push_back(t);
  }
  std::sort(out.begin(), out.end(), [](const RackTag & a, const RackTag & b) {return a.cell < b.cell;});
  return out;
}

double planar_yaw(const Mat3 & R)
{
  return std::atan2(R(1, 0), R(0, 0));
}

void RackFilter::push(int cell, const Vec3 & position, double yaw)
{
  buf_.push_back({frame_, cell, position, yaw});
  while (!buf_.empty() && buf_.front().frame <= frame_ - window_) {buf_.pop_front();}
}

std::optional<RackEstimate> RackFilter::get(int cell) const
{
  std::vector<double> xs, ys, zs, yaws;
  double ref = 0.0;
  for (const auto & s : buf_) {
    if (s.cell != cell || s.frame <= frame_ - window_) {continue;}
    if (xs.empty()) {ref = s.yaw;}
    xs.push_back(s.p.x());
    ys.push_back(s.p.y());
    zs.push_back(s.p.z());
    yaws.push_back(wrap(s.yaw - ref));   // 첫 값 기준으로 펴서 ±π 경계 문제 없앰
  }
  if (static_cast<int>(xs.size()) < min_hits_) {return std::nullopt;}
  RackEstimate e;
  e.position = Vec3(median(xs), median(ys), median(zs));
  e.yaw = wrap(ref + median(yaws));
  e.hits = static_cast<int>(xs.size());
  return e;
}

std::vector<int> RackFilter::cells() const
{
  std::set<int> s;
  for (const auto & b : buf_) {s.insert(b.cell);}
  std::vector<int> out;
  for (int c : s) {
    if (get(c)) {out.push_back(c);}
  }
  return out;
}

}  // namespace perception::wrist
