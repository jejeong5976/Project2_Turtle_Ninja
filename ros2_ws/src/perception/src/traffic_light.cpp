// traffic_light.cpp — 신호등 칸 판정 구현, 설명은 traffic_light.hpp

#include "perception/traffic_light.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace perception
{

namespace
{
// numpy.percentile (linear) 과 같은 계산. 비어 있으면 0
double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {return 0.0;}
  std::sort(v.begin(), v.end());
  const double pos = (static_cast<double>(v.size()) - 1.0) * q / 100.0;
  const auto lo = static_cast<size_t>(std::floor(pos));
  const size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (v[hi] - v[lo]) * (pos - static_cast<double>(lo));
}

// 가장 큰 값의 위치와 (그 값 − 나머지 중 최댓값). 하나뿐이면 차이 = 그 값
std::pair<size_t, double> best_and_gap(const std::vector<double> & x)
{
  const auto best = static_cast<size_t>(std::max_element(x.begin(), x.end()) - x.begin());
  double other = -1e9;
  for (size_t i = 0; i < x.size(); ++i) {
    if (i != best) {other = std::max(other, x[i]);}
  }
  return {best, x.size() == 1 ? x[best] : x[best] - other};
}
}  // namespace

std::string classify_traffic_light(
  const cv::Mat & roi_bgr, const std::vector<TrafficLightTier> & tiers, const TrafficLightRule & rule,
  std::map<std::string, TrafficLightScore> * score_out)
{
  if (roi_bgr.empty() || tiers.empty()) {return "unknown";}
  const int h = roi_bgr.rows, w = roi_bgr.cols;
  cv::Mat hsv, ch[3];
  cv::cvtColor(roi_bgr, hsv, cv::COLOR_BGR2HSV);
  cv::split(hsv, ch);
  const cv::Mat & s = ch[1];
  const cv::Mat & v = ch[2];
  const int cx0 = static_cast<int>(w * 0.25);
  const int cx1 = std::min(w, std::max(static_cast<int>(w * 0.75), cx0 + 1));   // 가장자리 반사 제외

  std::vector<double> vs, whites;
  for (const auto & t : tiers) {
    const int r0 = std::min(h, static_cast<int>(t.top * h));
    const int r1 = std::min(h, std::max(static_cast<int>(t.bottom * h), static_cast<int>(t.top * h) + 1));
    std::vector<double> vals;
    int n_white = 0;
    for (int r = r0; r < r1; ++r) {
      const auto * pv = v.ptr<uchar>(r);
      const auto * ps = s.ptr<uchar>(r);
      for (int c = cx0; c < cx1; ++c) {
        vals.push_back(pv[c]);
        if (pv[c] >= rule.white_v && ps[c] <= rule.white_s) {++n_white;}
      }
    }
    vs.push_back(vals.empty() ? 0.0 : percentile(vals, 90));
    whites.push_back(vals.empty() ? 0.0 : static_cast<double>(n_white) / static_cast<double>(vals.size()));
    if (score_out) {(*score_out)[t.name] = {vs.back(), whites.back()};}
  }
  const auto [wi, w_gap] = best_and_gap(whites);
  if (whites[wi] >= rule.white_min && w_gap > rule.white_margin) {return tiers[wi].name;}
  const auto [vi, v_gap] = best_and_gap(vs);
  if (v_gap > rule.v_margin) {return tiers[vi].name;}
  return "off";
}

}  // namespace perception
