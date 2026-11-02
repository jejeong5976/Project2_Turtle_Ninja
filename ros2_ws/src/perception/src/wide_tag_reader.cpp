// wide_tag_reader.cpp — 광각 퍽 태그 읽기 구현, 설명은 wide_tag_reader.hpp

#include "perception/wide_tag_reader.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

#include "perception/common.hpp"

namespace perception::wide
{

std::pair<int, std::vector<cv::Point2f>> PuckTagReader::read(const cv::Mat & img_bgr, const Box & box, double pad) const
{
  const double pw = (box.x2 - box.x1) * pad, ph = (box.y2 - box.y1) * pad;
  const int xa = static_cast<int>(std::max(0.0, box.x1 - pw));
  const int ya = static_cast<int>(std::max(0.0, box.y1 - ph));
  const int xb = static_cast<int>(std::min(static_cast<double>(img_bgr.cols), box.x2 + pw));
  const int yb = static_cast<int>(std::min(static_cast<double>(img_bgr.rows), box.y2 + ph));
  if (xb - xa < 4 || yb - ya < 4) {return {-1, {}};}

  cv::Mat gray;
  cv::cvtColor(img_bgr(cv::Range(ya, yb), cv::Range(xa, xb)), gray, cv::COLOR_BGR2GRAY);
  const double scale = std::max(1.0, static_cast<double>(min_side_) / std::min(gray.rows, gray.cols));
  if (scale > 1.0) {   // 작은 퍽은 키워서 검출
    cv::resize(gray, gray, cv::Size(), scale, scale, cv::INTER_CUBIC);
  }
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  detect_apriltags(gray, corners, ids);

  size_t k = 0;
  double best = -1;
  for (size_t i = 0; i < corners.size(); ++i) {
    if (!is_puck_tag(ids[i])) {continue;}   // 장소 태그 등은 버림
    const double a = cv::contourArea(corners[i]);
    if (a > best) {best = a; k = i;}
  }
  if (best < 0) {return {-1, {}};}
  std::vector<cv::Point2f> pts;
  for (const auto & p : corners[k]) {
    pts.emplace_back(static_cast<float>(p.x / scale + xa), static_cast<float>(p.y / scale + ya));
  }
  return {ids[k], pts};
}

}  // namespace perception::wide
