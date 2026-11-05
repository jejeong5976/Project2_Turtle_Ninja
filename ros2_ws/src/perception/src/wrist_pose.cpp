// wrist_pose.cpp — 손목 퍽 자세 기하 계산 구현, 설명은 wrist_pose.hpp

#include "perception/wrist_pose.hpp"

#include "perception/common.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace perception::wrist
{

namespace
{
// 점 평균
Vec3 mean_of(const std::vector<Vec3> & pts)
{
  Vec3 m = Vec3::Zero();
  for (const auto & p : pts) {m += p;}
  return m / static_cast<double>(std::max<size_t>(pts.size(), 1));
}

// (pts − c) 의 산포 행렬 Σ q qᵀ
Mat3 scatter(const std::vector<Vec3> & pts, const Vec3 & c)
{
  Mat3 S = Mat3::Zero();
  for (const auto & p : pts) {
    const Vec3 q = p - c;
    S += q * q.transpose();
  }
  return S;
}

// 고유값 오름차순 (numpy eigh 와 같음)
Eigen::SelfAdjointEigenSolver<Mat3> eig(const Mat3 & S) {return Eigen::SelfAdjointEigenSolver<Mat3>(S);}

std::vector<double> dots(const std::vector<Vec3> & pts, const Vec3 & d, const Vec3 & c = Vec3::Zero())
{
  std::vector<double> out;
  out.reserve(pts.size());
  for (const auto & p : pts) {out.push_back((p - c).dot(d));}
  return out;
}

double range_5_95(const std::vector<double> & v) {return percentile(v, 95.0) - percentile(v, 5.0);}
}  // namespace

double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {return 0.0;}
  std::sort(v.begin(), v.end());
  const double pos = q / 100.0 * static_cast<double>(v.size() - 1);
  const auto lo = static_cast<size_t>(std::floor(pos));
  const size_t hi = std::min(lo + 1, v.size() - 1);
  const double f = pos - static_cast<double>(lo);
  return v[lo] + f * (v[hi] - v[lo]);
}

double median(std::vector<double> v) {return percentile(std::move(v), 50.0);}

std::vector<Vec3> deproject(const cv::Mat1b & mask, const cv::Mat1f & depth_m, const Intrinsics & K)
{
  std::vector<Vec3> out;
  for (int v = 0; v < mask.rows; ++v) {
    const uint8_t * m = mask.ptr<uint8_t>(v);
    const float * d = depth_m.ptr<float>(v);
    for (int u = 0; u < mask.cols; ++u) {
      const double z = d[u];
      if (!m[u] || !(z > DEPTH_MIN && z < DEPTH_MAX)) {continue;}
      out.emplace_back((u - K.cx) * z / K.fx, (v - K.cy) * z / K.fy, z);
    }
  }
  return out;
}

double top_height(const std::vector<double> & h)
{
  double t = percentile(h, 95.0);
  for (int it = 0; it < 5; ++it) {
    std::vector<double> s;
    for (double x : h) {
      if (std::abs(x - t) < TOP_WIN) {s.push_back(x);}
    }
    if (s.size() < 10) {break;}
    t = median(std::move(s));
  }
  return t;
}

std::vector<TagPose> detect_tags(const cv::Mat & gray, const Intrinsics & K, const cv::Mat & dist)
{
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  detect_apriltags(gray, corners, ids);   // common.hpp (광각과 같은 검출기 · 설정)
  const cv::Matx33d Kc(K.fx, 0, K.cx, 0, K.fy, K.cy, 0, 0, 1);
  const cv::Mat D = dist.empty() ? cv::Mat::zeros(1, 5, CV_64F) : dist;
  std::vector<TagPose> out;
  for (size_t k = 0; k < ids.size(); ++k) {
    if (!is_puck_tag(ids[k])) {continue;}   // 장소 태그 등은 안 씀 (퍽 축 보정은 퍽 태그만)
    const float h = static_cast<float>(kPuckTagSideM / 2);   // 회전만 쓰므로 크기는 결과 회전에 영향 없음
    const std::vector<cv::Point3f> obj{{-h, h, 0}, {h, h, 0}, {h, -h, 0}, {-h, -h, 0}};
    std::vector<cv::Mat> rvecs, tvecs;
    const int n = cv::solvePnPGeneric(obj, corners[k], Kc, D, rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE);
    if (n <= 0) {continue;}
    TagPose t;
    t.id = ids[k];
    for (int i = 0; i < 4; ++i) {t.corners[i] = corners[k][i];}
    for (int i = 0; i < n; ++i) {
      cv::Mat Rm;
      cv::Rodrigues(rvecs[i], Rm);
      Mat3 R;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {R(r, c) = Rm.at<double>(r, c);}
      }
      const Vec3 tv(tvecs[i].at<double>(0), tvecs[i].at<double>(1), tvecs[i].at<double>(2));
      t.alts.emplace_back(R, tv);
    }
    t.R = t.alts[0].first;
    t.t = t.alts[0].second;
    out.push_back(std::move(t));
  }
  return out;
}

TagPose refine_tag_with_depth(TagPose t, const cv::Mat1f & depth_m, const Intrinsics & K)
{
  cv::Point2f c(0, 0);
  for (const auto & p : t.corners) {c += p * 0.25f;}
  std::vector<cv::Point> poly;
  for (const auto & p : t.corners) {
    const cv::Point2f q = (p - c) * 0.8f + c;                // 가장자리 깊이 튐 피하려고 80 % 안쪽
    poly.emplace_back(static_cast<int>(q.x), static_cast<int>(q.y));   // numpy astype(int32) = 0 쪽 버림
  }
  cv::Mat1b m = cv::Mat1b::zeros(depth_m.size());
  cv::fillConvexPoly(m, poly, cv::Scalar(1));
  const auto pts = deproject(m, depth_m, K);
  if (pts.size() < 12 || t.alts.size() < 2) {return t;}
  const Vec3 nd = eig(scatter(pts, mean_of(pts))).eigenvectors().col(0);   // 가장 작은 고유벡터 = 평면 법선
  size_t best = 0;
  double best_v = -1.0;
  for (size_t i = 0; i < t.alts.size(); ++i) {               // max() 와 같이 같으면 앞 해
    const double v = std::abs(t.alts[i].first.col(2).dot(nd));
    if (v > best_v) {best_v = v; best = i;}
  }
  t.R = t.alts[best].first;
  t.t = t.alts[best].second;
  return t;
}

bool tag_is_top(const std::vector<Vec3> & pts, const Vec3 & n, const Vec3 & up)
{
  const Vec3 c = mean_of(pts);
  return range_5_95(dots(pts, n, c)) > range_5_95(dots(pts, up, c));
}

std::pair<std::optional<Vec3>, std::array<double, 2>> upright_face(const std::vector<Vec3> & top, const Vec3 & up)
{
  if (top.size() < 30) {return {std::nullopt, {0.0, 0.0}};}
  Vec3 e1 = up.cross(Vec3::UnitX());
  if (e1.norm() < 1e-6) {e1 = up.cross(Vec3::UnitY());}
  e1.normalize();
  const Vec3 e2 = up.cross(e1);
  std::vector<cv::Point2f> uv;
  uv.reserve(top.size());
  for (const auto & p : top) {                                // mm
    uv.emplace_back(static_cast<float>(p.dot(e1) * 1000.0), static_cast<float>(p.dot(e2) * 1000.0));
  }
  const cv::RotatedRect rr = cv::minAreaRect(uv);
  cv::Point2f corners[4];
  rr.points(corners);
  const cv::Point2f edge = corners[1] - corners[0];
  const std::array<double, 2> size{rr.size.width / 1000.0, rr.size.height / 1000.0};
  const double lo = std::min(size[0], size[1]), hi = std::max(size[0], size[1]);
  if (!(FACE_SIDE_MIN <= lo && hi <= FACE_SIDE_MAX && hi / std::max(lo, 1e-6) <= FACE_MAX_ASPECT)) {
    return {std::nullopt, size};
  }
  const Vec3 d = edge.x * e1 + edge.y * e2;
  return {d.normalized(), size};
}

Vec3 orient_axis(const Vec3 & axis, const Vec3 * up)
{
  if (up != nullptr && std::abs(axis.dot(*up)) >= std::cos(M_PI / 4)) {return axis.dot(*up) >= 0 ? axis : -axis;}
  const double s = std::abs(axis.x()) >= 0.2 ? axis.x() : axis.y();
  return s >= 0 ? axis : -axis;
}

std::vector<Vec3> top_face_points(const std::vector<Vec3> & pts, const std::vector<double> & h, double top_h, const Vec3 & up)
{
  std::vector<Vec3> seed;
  for (size_t i = 0; i < pts.size(); ++i) {
    if (h[i] >= top_h - TOP_BAND) {seed.push_back(pts[i]);}
  }
  if (seed.size() < 30) {                                     // 씨앗 부족 → 높이 상위 30 %
    const double h70 = percentile(h, 70.0);
    std::vector<Vec3> out;
    for (size_t i = 0; i < pts.size(); ++i) {
      if (h[i] >= h70) {out.push_back(pts[i]);}
    }
    return out;
  }
  std::vector<Vec3> s = seed;
  for (int it = 0; it < 3; ++it) {
    const Vec3 c = mean_of(s);
    const Vec3 n = eig(scatter(s, c)).eigenvectors().col(0);   // 가장 작은 고유벡터 = 평면 법선
    if (std::abs(n.dot(up)) < std::cos(PLANE_MAX_TILT * M_PI / 180.0)) {return seed;}   // 옆면을 잡음
    std::vector<Vec3> nxt;
    for (const auto & p : pts) {
      if (std::abs((p - c).dot(n)) < PLANE_TOL) {nxt.push_back(p);}
    }
    if (nxt.size() < 30) {return s;}
    s = std::move(nxt);
  }
  return s;
}

std::pair<std::optional<Vec3>, std::array<double, 2>> lying_face(
  const std::vector<Vec3> & band, const Vec3 & axis, const Vec3 & up)
{
  if (band.size() < 30) {return {std::nullopt, {0.0, 0.0}};}
  Vec3 side = up.cross(axis);
  if (side.norm() < 1e-6) {return {std::nullopt, {0.0, 0.0}};}
  side.normalize();
  const auto pr = dots(band, side);
  const double w = percentile(pr, 97.0) - percentile(pr, 3.0);
  if (w < LYING_FACE_MIN_W) {return {std::nullopt, {w, 0.0}};}
  return {(up - axis * up.dot(axis)).normalized(), {w, 0.0}};
}

std::optional<double> support_height(const cv::Mat1b & mask, const cv::Mat1f & depth_m, const Intrinsics & K, const Vec3 & up_in,
  double top_h)
{
  const Vec3 up = up_in.normalized();
  cv::Mat1b out_d, in_d;
  cv::dilate(mask, out_d, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * RING_OUT + 1, 2 * RING_OUT + 1)));
  cv::dilate(mask, in_d, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * RING_IN + 1, 2 * RING_IN + 1)));
  const cv::Mat1b ring = out_d & ~in_d;
  std::vector<double> h;
  for (const auto & q : deproject(ring, depth_m, K)) {
    const double hq = q.dot(up);
    if (hq < top_h - 0.010) {h.push_back(hq);}
  }
  if (static_cast<int>(h.size()) < RING_MIN_POINTS) {return std::nullopt;}
  return percentile(h, 50.0);   // 둘레 대부분이 놓인 면 (옆 퍽 · 박스 끝 바닥은 일부)
}

void gravity_refine(PuckPose & p, const std::vector<Vec3> & pts, const Vec3 & up_in, std::optional<double> support_h)
{
  const Vec3 up = up_in.normalized();
  const auto h = dots(pts, up);
  const double top_h = top_height(h);
  // 발자국 = 수평면에 눌러 놓은 점
  std::vector<Vec3> flat;
  flat.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {flat.push_back(pts[i] - h[i] * up);}
  const Vec3 fm = mean_of(flat);
  const auto es = eig(scatter(flat, fm));
  const Vec3 long_dir = es.eigenvectors().col(2);
  const auto pr = dots(flat, long_dir, fm);
  const double length = percentile(pr, 97.0) - percentile(pr, 3.0);
  const bool upright = support_h ? (top_h - *support_h) > SUPPORT_UPRIGHT_H
                                  : ((top_h - percentile(h, 5.0)) > UPRIGHT_MIN_H || length < FLAT_MIN_LEN);
  if (upright) {
    p.axis = up;
    p.axis_source = "gravity_up";
  } else {
    p.axis = long_dir;
    p.axis_source = "gravity_flat";
  }
  p.upright = upright;
  // 중심: 윗면 점 수평 위치 + 윗면에서 H/2 · D/2 아래
  const auto top = top_face_points(pts, h, top_h, up);
  const Vec3 tm = mean_of(top);
  const Vec3 xy = tm - up * tm.dot(up);
  p.top_center = xy + up * top_h;
  p.center = xy + up * (top_h - (upright ? PUCK_H : PUCK_D) / 2.0);
  if (upright) {
    auto [face, size] = upright_face(top, up);
    p.face_dir = face;
    p.face_size = size;
  } else {
    std::vector<Vec3> band;
    for (size_t i = 0; i < pts.size(); ++i) {
      if (h[i] >= top_h - TOP_BAND) {band.push_back(pts[i]);}
    }
    auto [face, size] = lying_face(band, p.axis, up);
    p.face_dir = face;
    p.face_size = size;
  }
  p.axis = orient_axis(p.axis, &up);
}

bool tag_top_refine(PuckPose & p, const Intrinsics & K, const Vec3 & up_in)
{
  if (!p.tag || !p.upright || !*p.upright || !p.top_center || !(K.fx > 0.0) || !(K.fy > 0.0)) {return false;}
  const Vec3 up = up_in.normalized();
  const double top_h = p.top_center->dot(up);
  std::array<Vec3, 4> c;
  for (int k = 0; k < 4; ++k) {
    const Vec3 r((p.tag->corners[k].x - K.cx) / K.fx, (p.tag->corners[k].y - K.cy) / K.fy, 1.0);
    const double den = r.dot(up);
    if (std::abs(den) < 1e-6) {return false;}
    const double s = top_h / den;
    if (!(s > 0.0) || !std::isfinite(s)) {return false;}
    c[k] = r * s;
  }
  for (int k = 0; k < 4; ++k) {
    if (std::abs((c[(k + 1) % 4] - c[k]).norm() - kPuckTagSideM) > 0.004) {return false;}
  }
  const Vec3 tc = (c[0] + c[1] + c[2] + c[3]) / 4.0;
  if ((tc - *p.top_center).norm() > 0.025) {return false;}
  Vec3 e = (c[1] - c[0]) + (c[2] - c[3]);                    // 마주 보는 두 변 평균 = 면 방향
  e -= up * e.dot(up);
  if (e.norm() < 1e-9) {return false;}
  p.top_center = tc;
  p.center = tc - up * (PUCK_H / 2.0);
  p.face_dir = e.normalized();
  p.tag_centre = true;
  return true;
}

std::optional<PuckPose> puck_pose(
  const cv::Mat1b & mask, const cv::Mat1f & depth_m, const Intrinsics & K,
  const std::vector<TagPose> & tags, const Vec3 * up_cam)
{
  auto all = deproject(mask, depth_m, K);
  if (static_cast<int>(all.size()) < MIN_POSE_POINTS) {return std::nullopt;}
  // 중앙값(좌표별)에서 6 cm 밖 튀는 점 제거
  Vec3 med;
  for (int k = 0; k < 3; ++k) {
    std::vector<double> c;
    c.reserve(all.size());
    for (const auto & q : all) {c.push_back(q[k]);}
    med[k] = median(std::move(c));
  }
  std::vector<Vec3> pts;
  pts.reserve(all.size());
  for (const auto & q : all) {
    if ((q - med).norm() < OUTLIER_R) {pts.push_back(q);}
  }
  if (static_cast<int>(pts.size()) < MIN_POSE_POINTS) {return std::nullopt;}

  PuckPose p;
  const Vec3 c = mean_of(pts);
  const auto es = eig(scatter(pts, c) / static_cast<double>(pts.size() - 1));   // np.cov (ddof 1)
  p.axis = orient_axis(es.eigenvectors().col(2), up_cam);
  p.center = c;                                               // 보이는 면 점 평균 (위쪽 알면 gravity_refine 이 다시 잡음)
  p.n_points = static_cast<int>(pts.size());
  for (int k = 0; k < 3; ++k) {
    p.extent[k] = 2.0 * std::sqrt(std::max(es.eigenvalues()[2 - k], 0.0));
    p.spread[k] = range_5_95(dots(pts, es.eigenvectors().col(2 - k), c));
  }
  // 마스크 상자 · 잘림
  cv::Mat nz;
  cv::findNonZero(mask, nz);
  const cv::Rect r = cv::boundingRect(nz);
  p.bbox = {r.x, r.y, r.x + r.width - 1, r.y + r.height - 1};
  p.truncated = p.bbox[0] <= 2 || p.bbox[1] <= 2 || p.bbox[2] >= mask.cols - 3 || p.bbox[3] >= mask.rows - 3;

  // 태그: 상자(±5 px) 안에 중심이 있는 퍽 태그(ID 0–11, common.hpp) 하나
  for (const auto & t0 : tags) {
    cv::Point2f tc(0, 0);
    for (const auto & q : t0.corners) {tc += q * 0.25f;}
    const int cx = static_cast<int>(tc.x), cy = static_cast<int>(tc.y);
    if (!(p.bbox[0] - 5 <= cx && cx <= p.bbox[2] + 5 && p.bbox[1] - 5 <= cy && cy <= p.bbox[3] + 5 && is_puck_tag(t0.id))) {
      continue;
    }
    if (up_cam != nullptr) {                                  // 위쪽 알면 태그는 ID 만
      p.tag = t0;
      break;
    }
    TagPose t = refine_tag_with_depth(t0, depth_m, K);
    const Vec3 n = t.R.col(2), tup = t.R.col(1);              // 태그 법선, 태그 그림 위쪽
    if (tag_is_top(pts, n, tup)) {
      p.axis = n;
      p.axis_source = "tag_normal";
    } else {
      p.axis = tup;
      p.axis_source = "tag_up";
    }
    p.axis = orient_axis(p.axis, up_cam);
    p.tag = t;
    break;
  }
  if (up_cam != nullptr) {
    const Vec3 upn = up_cam->normalized();
    const auto hs = dots(pts, upn);
    gravity_refine(p, pts, *up_cam, support_height(mask, depth_m, K, upn, top_height(hs)));
    tag_top_refine(p, K, *up_cam);
  }
  return p;
}

}  // namespace perception::wrist
