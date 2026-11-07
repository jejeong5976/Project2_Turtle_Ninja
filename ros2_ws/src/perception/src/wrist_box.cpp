// wrist_box.cpp — 손목 깊이로 박스 윗면 중심 · 방향 구현, 설명은 wrist_box.hpp

#include "perception/wrist_box.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace perception::wrist
{

double fold_quarter(double a, double ref)
{
  const double q = M_PI / 2;
  double d = std::remainder(a - ref, q);   // (−45°, 45°]
  if (d <= -q / 2) {d += q;}
  return ref + d;
}

BoxFit fit_place_box(
  const cv::Mat1f & depth, const Intrinsics & K, const Mat3 & R, const Vec3 & t,
  const BoxParams & p, const Vec3 & guess, cv::Mat1b * edge_mask)
{
  BoxFit out;
  const int s = std::max(1, p.stride);
  const int gh = depth.rows / s, gw = depth.cols / s;
  if (gh < 3 || gw < 3) {
    out.why = "깊이 영상 작음";
    return out;
  }
  // 1) 화소 → 바닥 프레임 점, 높이로 나눔 (0 = 없음 · 1 = 윗면 · 2 = 바닥 쪽 · 3 = 그 밖)
  cv::Mat1b cls(gh, gw, uchar(0)), low(gh, gw, uchar(0));
  std::vector<Vec3> P(static_cast<size_t>(gh) * gw);
  for (int r = 0; r < gh; ++r) {
    const float * row = depth.ptr<float>(r * s);
    for (int c = 0; c < gw; ++c) {
      const double z = row[c * s];
      if (!(z > 0.0) || z > p.max_range) {continue;}
      const double u = c * s + 0.5, v = r * s + 0.5;
      const Vec3 pc((u - K.cx) / K.fx * z, (v - K.cy) / K.fy * z, z);
      const Vec3 pb = R * pc + t;
      P[static_cast<size_t>(r) * gw + c] = pb;
      if (std::fabs(pb.z() - p.top_z) <= p.band) {
        cls(r, c) = 1;
      } else if (pb.z() < p.top_z - p.drop) {
        cls(r, c) = 2;
        low(r, c) = 255;
      } else {
        cls(r, c) = 3;
      }
    }
  }
  // 2) 테두리 점 = 윗면 화소 중 dilate 안에 바닥 쪽 화소가 있는 것
  cv::Mat1b lowd;
  const int k = 2 * std::max(1, p.dilate) + 1;
  cv::dilate(low, lowd, cv::Mat::ones(k, k, CV_8U));
  std::vector<Eigen::Vector2d> pts;
  for (int r = 0; r < gh; ++r) {
    for (int c = 0; c < gw; ++c) {
      if (cls(r, c) != 1 || !lowd(r, c)) {continue;}
      const Vec3 & pb = P[static_cast<size_t>(r) * gw + c];
      if (pb.x() < p.min_x) {continue;}
      pts.emplace_back(pb.x(), pb.y());
      if (edge_mask) {
        if (edge_mask->size() != depth.size()) {*edge_mask = cv::Mat1b::zeros(depth.size());}
        cv::rectangle(*edge_mask, cv::Rect(c * s, r * s, s, s), 255, cv::FILLED);
      }
    }
  }
  out.edge_pts = static_cast<int>(pts.size());
  if (out.edge_pts < 2 * p.min_side_pts) {
    out.why = "테두리 점 부족 (" + std::to_string(out.edge_pts) + ")";
    return out;
  }

  // 3) 정사각형 맞춤: 상태 (cx, cy, yaw), 점마다 가까운 변 → 잔차 = 박스 좌표에서 그 변까지 (Huber 가중 가우스-뉴턴)
  const double h = p.size / 2;
  double cx = guess.x(), cy = guess.y(), yaw = guess.z();
  std::vector<int> side(pts.size());   // 0 앞 · 1 뒤 · 2 왼 · 3 오른
  for (int it = 0; it < p.iters; ++it) {
    const double co = std::cos(yaw), si = std::sin(yaw);
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < pts.size(); ++i) {
      const double dx = pts[i].x() - cx, dy = pts[i].y() - cy;
      const double qx = co * dx + si * dy, qy = -si * dx + co * dy;
      const bool xside = std::fabs(std::fabs(qx) - h) < std::fabs(std::fabs(qy) - h);
      double r;
      Eigen::Vector3d J;
      if (xside) {
        const double sg = qx < 0 ? -1.0 : 1.0;
        side[i] = qx < 0 ? 0 : 1;
        r = qx - sg * h;
        J << -co, -si, qy;
      } else {
        const double sg = qy < 0 ? -1.0 : 1.0;
        side[i] = qy < 0 ? 3 : 2;
        r = qy - sg * h;
        J << si, -co, -qx;
      }
      const double w = std::fabs(r) <= p.huber ? 1.0 : p.huber / std::fabs(r);
      A += w * J * J.transpose();
      b += w * J * r;
    }
    const Eigen::Vector3d dlt = A.ldlt().solve(-b);
    if (!dlt.allFinite()) {
      out.why = "맞춤 발산";
      return out;
    }
    cx += dlt(0);
    cy += dlt(1);
    yaw += dlt(2);
    if (std::hypot(dlt(0), dlt(1)) < 1e-5 && std::fabs(dlt(2)) < 1e-5) {break;}
  }

  // 4) 변마다 점 수 · 잔차
  const double co = std::cos(yaw), si = std::sin(yaw);
  double ss = 0.0;
  int n_in = 0;
  out.side_pts = {0, 0, 0, 0};
  for (const auto & q0 : pts) {
    const double dx = q0.x() - cx, dy = q0.y() - cy;
    const double qx = co * dx + si * dy, qy = -si * dx + co * dy;
    const double ex = std::fabs(std::fabs(qx) - h), ey = std::fabs(std::fabs(qy) - h);
    const double e = std::min(ex, ey);
    if (e <= p.side_tol) {
      if (ex < ey) {++out.side_pts[qx < 0 ? 0 : 1];} else {++out.side_pts[qy < 0 ? 3 : 2];}
    }
    if (e <= p.huber) {ss += e * e; ++n_in;}
  }
  out.rms = n_in ? std::sqrt(ss / n_in) : 0.0;
  // 앞 · 뒤 축과 좌우 축이 정해져야 함. 박스 yaw 를 처음 추정 근처로 접으면 변 이름도 바뀔 수 있어 접기 전에 셈
  const auto & sp = out.side_pts;
  const bool x_ok = sp[0] >= p.min_side_pts || sp[1] >= p.min_side_pts;
  const bool y_ok = sp[2] >= p.min_side_pts || sp[3] >= p.min_side_pts;
  out.x = cx;
  out.y = cy;
  out.yaw = fold_quarter(yaw, guess.z());
  if (!x_ok || !y_ok) {
    out.why = "변 부족 (앞 " + std::to_string(sp[0]) + " 뒤 " + std::to_string(sp[1]) + " 왼 " + std::to_string(sp[2]) +
      " 오른 " + std::to_string(sp[3]) + ")";
    return out;
  }
  // 뒤 변을 못 봤으면 앞뒤 위치는 앞 변으로만 정해짐 → 로봇 쪽 변이 차체에 붙었으면 그 변이 가려졌을 수 있음
  //   (뒤 변을 보면 앞 변이 가려져도 앞뒤가 정해지므로 검사 안 함)
  if (sp[1] >= p.min_side_pts) {
    out.ok = true;
    return out;
  }
  double near_x = 1e9;
  for (int k2 = 0; k2 < 4; ++k2) {
    const double a = out.yaw + k2 * M_PI / 2;
    near_x = std::min(near_x, cx + h * std::cos(a));
  }
  if (near_x < p.front_min_x) {
    out.why = "박스 앞 변이 차체에 너무 가까움 (" + std::to_string(near_x) + " m)";
    return out;
  }
  out.ok = true;
  return out;
}

}  // namespace perception::wrist
