// wrist_held.cpp — 손에 쥔 퍽 마스크 재기, 설명은 wrist_held.hpp

#include "perception/wrist_held.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace perception::wrist
{

namespace
{
constexpr double kDeg = 180.0 / M_PI;

// v = k u + b 최소제곱, 한 번 다시 맞춤 (잔차 > max(3 px, 2.5 σ) 인 점 빼고)
bool fit_line(const std::vector<double> & u, const std::vector<double> & v, double & k, double & b)
{
  std::vector<char> use(u.size(), 1);
  auto ls = [&](double & kk, double & bb) {
      double n = 0, su = 0, sv = 0, suu = 0, suv = 0;
      for (size_t i = 0; i < u.size(); ++i) {
        if (!use[i]) {continue;}
        n += 1; su += u[i]; sv += v[i]; suu += u[i] * u[i]; suv += u[i] * v[i];
      }
      const double d = n * suu - su * su;
      if (n < 2 || std::fabs(d) < 1e-9) {return false;}
      kk = (n * suv - su * sv) / d;
      bb = (sv - kk * su) / n;
      return true;
    };
  if (!ls(k, b)) {return false;}
  double ss = 0;
  for (size_t i = 0; i < u.size(); ++i) {const double r = v[i] - (k * u[i] + b); ss += r * r;}
  const double thr = std::max(3.0, 2.5 * std::sqrt(ss / static_cast<double>(u.size())));
  for (size_t i = 0; i < u.size(); ++i) {use[i] = std::fabs(v[i] - (k * u[i] + b)) < thr;}
  double k2 = k, b2 = b;
  if (ls(k2, b2)) {k = k2; b = b2;}
  return true;
}
}  // namespace

int pick_held(const std::vector<SegInstance> & inst, const HeldParams & p)
{
  int best = -1, best_area = 0;
  for (size_t i = 0; i < inst.size(); ++i) {
    const auto & s = inst[i];
    if (s.conf < p.conf || s.mask.empty()) {continue;}
    const int W = s.mask.cols, H = s.mask.rows;
    const int x0 = std::clamp(p.roi_x0, 0, W), y0 = std::clamp(p.roi_y0, 0, H);
    if (x0 >= W || y0 >= H) {continue;}
    const int area = cv::countNonZero(s.mask(cv::Rect(x0, y0, W - x0, H - y0)));
    if (area < p.min_area_px) {continue;}
    const int band = std::min(10, W - x0);
    const int right = cv::countNonZero(s.mask(cv::Rect(W - band, y0, band, H - y0)));
    if (right < p.right_touch_px) {continue;}
    if (area > best_area) {best_area = area; best = static_cast<int>(i);}
  }
  return best;
}

HeldMeasure measure_held(const cv::Mat1b & mask, const HeldParams & p)
{
  HeldMeasure m;
  if (mask.empty()) {m.why = "마스크 없음"; return m;}
  const int W = mask.cols, H = mask.rows;
  const int x0 = std::clamp(p.roi_x0, 0, W - 1), y0 = std::clamp(p.roi_y0, 0, H - 1);
  const cv::Rect roi(x0, y0, W - x0, H - y0);
  m.area = cv::countNonZero(mask(roi));
  if (m.area < p.min_area_px) {m.why = "ROI 안 마스크 " + std::to_string(m.area) + " px < " + std::to_string(p.min_area_px); return m;}
  {
    cv::Mat1b nz = mask(roi) > 0;
    const cv::Rect r = cv::boundingRect(nz);
    m.bbox = {r.x + x0, r.y + y0, r.x + r.width + x0, r.y + r.height + y0};
  }
  // 열마다 맨 위 마스크 줄
  std::vector<int> top(static_cast<size_t>(W), -1);
  int xmin = W;
  for (int x = x0; x < W; ++x) {
    for (int y = y0; y < H; ++y) {
      if (mask(y, x)) {top[static_cast<size_t>(x)] = y; xmin = std::min(xmin, x); break;}
    }
  }
  std::vector<double> xs, ys;
  for (int x = xmin + 15; x <= W - 6; ++x) {
    if (top[static_cast<size_t>(x)] >= 0) {xs.push_back(x); ys.push_back(top[static_cast<size_t>(x)]);}
  }
  m.top_cols = static_cast<int>(xs.size());
  double k = 0, b = 0;
  if (m.top_cols < p.min_rows || !fit_line(xs, ys, k, b)) {m.why = "위 모서리 열 " + std::to_string(m.top_cols) + " 개"; return m;}
  m.top_y = k * p.top_ref_x + b;
  m.top_deg = std::atan(k) * kDeg;
  const int t0 = static_cast<int>(*std::min_element(ys.begin(), ys.end()));
  // 왼쪽 모서리 (손가락 위 줄만)
  std::vector<double> ly, lx;
  const int r0 = std::max(t0 + 8, y0), r1 = std::min({t0 + p.left_rows, p.finger_y, H});
  for (int y = r0; y < r1; ++y) {
    for (int x = x0; x < W; ++x) {
      if (mask(y, x)) {ly.push_back(y); lx.push_back(x); break;}
    }
  }
  m.left_rows = static_cast<int>(ly.size());
  if (m.left_rows >= p.min_rows && fit_line(ly, lx, k, b)) {
    m.left_x = k * (t0 + 40) + b;
    m.left_deg = std::atan(k) * kDeg;
  }
  // 아래 끝
  const int bx0 = std::clamp(p.bottom_x0, 0, W), bx1 = std::clamp(p.bottom_x1, 0, W);
  for (int y = H - 1; y >= y0 && bx1 > bx0; --y) {
    if (cv::countNonZero(mask(cv::Rect(bx0, y, bx1 - bx0, 1))) > 0) {m.bottom_y = y; break;}
  }
  m.ok = true;
  return m;
}

HeldMeasure measure_held(const std::vector<SegInstance> & inst, const HeldParams & p)
{
  const int i = pick_held(inst, p);
  if (i < 0) {
    HeldMeasure m;
    m.why = "쥔 퍽 후보 없음 (분할 " + std::to_string(inst.size()) + " 개)";
    return m;
  }
  auto m = measure_held(inst[static_cast<size_t>(i)].mask, p);
  m.kind = inst[static_cast<size_t>(i)].kind;
  m.conf = inst[static_cast<size_t>(i)].conf;
  return m;
}

}  // namespace perception::wrist
