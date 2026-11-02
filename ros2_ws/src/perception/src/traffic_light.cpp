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
}  // namespace

std::string classify_traffic_light(
  const cv::Mat & roi_bgr, const std::vector<TrafficLightTier> & tiers, double v_margin,
  std::map<std::string, double> * score_out)
{
  if (roi_bgr.empty() || tiers.empty()) {return "unknown";}
  const int h = roi_bgr.rows, w = roi_bgr.cols;
  cv::Mat hsv, ch[3];
  cv::cvtColor(roi_bgr, hsv, cv::COLOR_BGR2HSV);
  cv::split(hsv, ch);
  const cv::Mat & v = ch[2];
  const int cx0 = static_cast<int>(w * 0.25);
  const int cx1 = std::min(w, std::max(static_cast<int>(w * 0.75), cx0 + 1));   // 가장자리 반사 제외

  std::vector<double> score;
  for (const auto & t : tiers) {
    const int r0 = std::min(h, static_cast<int>(t.top * h));
    const int r1 = std::min(h, std::max(static_cast<int>(t.bottom * h), static_cast<int>(t.top * h) + 1));
    std::vector<double> vals;
    for (int r = r0; r < r1; ++r) {
      const auto * p = v.ptr<uchar>(r);
      for (int c = cx0; c < cx1; ++c) {vals.push_back(p[c]);}
    }
    score.push_back(vals.empty() ? 0.0 : percentile(vals, 90));
    if (score_out) {(*score_out)[t.name] = score.back();}
  }
  const size_t lit = static_cast<size_t>(std::max_element(score.begin(), score.end()) - score.begin());
  double other = -1e9;
  for (size_t i = 0; i < score.size(); ++i) {
    if (i != lit) {other = std::max(other, score[i]);}
  }
  if (score.size() == 1 || score[lit] - other > v_margin) {return tiers[lit].name;}
  return "off";
}

}  // namespace perception
