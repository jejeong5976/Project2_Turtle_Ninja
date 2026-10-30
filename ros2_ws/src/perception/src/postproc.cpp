#include "postproc.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>

// OpenCV 4.7 부터 aruco 가 objdetect 로, 4.8 부터 barcode 가 objdetect 로 옮겨짐 (그 전은 contrib 모듈)
#define PERCEPTION_CV_AT_LEAST(maj, min) \
  (CV_VERSION_MAJOR > (maj) || (CV_VERSION_MAJOR == (maj) && CV_VERSION_MINOR >= (min)))

#if PERCEPTION_CV_AT_LEAST(4, 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#if PERCEPTION_CV_AT_LEAST(4, 8)
#include <opencv2/objdetect/barcode.hpp>
#else
#include <opencv2/barcode.hpp>
#endif

namespace perception
{

double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {return 0.0;}
  std::sort(v.begin(), v.end());
  const double pos = (static_cast<double>(v.size()) - 1.0) * q / 100.0;
  const auto lo = static_cast<size_t>(std::floor(pos));
  const size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (v[hi] - v[lo]) * (pos - static_cast<double>(lo));
}

namespace
{

double pstd(const std::vector<double> & v)
{
  if (v.empty()) {return 0.0;}
  const double m = std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
  double s = 0.0;
  for (double x : v) {s += (x - m) * (x - m);}
  return std::sqrt(s / static_cast<double>(v.size()));
}

bool in_hue(double h, const std::array<double, 2> & rng)
{
  const double lo = rng[0], hi = rng[1];
  return lo <= hi ? (lo <= h && h <= hi) : (h >= lo || h <= hi);
}

// 빨강은 0/179 경계에 걸쳐 있어 그냥 중앙값을 내면 90 근처가 나옴 → 90 만큼 돌린 쪽이 더 모여 있으면 그쪽을 씀
double hue_median(const std::vector<double> & hues)
{
  std::vector<double> shifted(hues.size());
  for (size_t i = 0; i < hues.size(); ++i) {shifted[i] = std::fmod(hues[i] + 90.0, 180.0);}
  if (pstd(shifted) < pstd(hues)) {
    return std::fmod(percentile(shifted, 50) - 90.0 + 180.0, 180.0);
  }
  return percentile(hues, 50);
}

float box_iou(const Box & a, const Box & b)
{
  const float ix = std::max(0.f, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
  const float iy = std::max(0.f, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
  const float inter = ix * iy;
  const float ua = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
  return ua > 0 ? inter / ua : 0.f;
}

}  // namespace

cv::Mat letterbox(const cv::Mat & img, cv::Size size, double & r, int & pad_x, int & pad_y)
{
  r = std::min(static_cast<double>(size.height) / img.rows, static_cast<double>(size.width) / img.cols);
  const int nw = static_cast<int>(std::round(img.cols * r)), nh = static_cast<int>(std::round(img.rows * r));
  const double dw = (size.width - nw) / 2.0, dh = (size.height - nh) / 2.0;
  cv::Mat resized;
  if (nw != img.cols || nh != img.rows) {
    cv::resize(img, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
  } else {
    resized = img;
  }
  const int top = static_cast<int>(std::round(dh - 0.1)), bottom = static_cast<int>(std::round(dh + 0.1));
  const int left = static_cast<int>(std::round(dw - 0.1)), right = static_cast<int>(std::round(dw + 0.1));
  cv::Mat out;
  cv::copyMakeBorder(resized, out, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  pad_x = left;
  pad_y = top;
  return out;
}

std::vector<Detection> nms(std::vector<Detection> dets, float iou)
{
  std::sort(dets.begin(), dets.end(), [](const Detection & a, const Detection & b) {return a.score > b.score;});
  std::vector<Detection> keep;
  std::vector<bool> removed(dets.size(), false);
  for (size_t i = 0; i < dets.size(); ++i) {
    if (removed[i]) {continue;}
    keep.push_back(dets[i]);
    for (size_t j = i + 1; j < dets.size(); ++j) {
      if (!removed[j] && dets[j].cls == dets[i].cls && box_iou(dets[i].box, dets[j].box) > iou) {
        removed[j] = true;
      }
    }
  }
  return keep;
}

// ---------------------------------------------------------------- 퍽 색

cv::Mat center_roi(const cv::Mat & img, const Box & box, double frac)
{
  const double w = box.x2 - box.x1, h = box.y2 - box.y1;
  const double mx = w * (1 - frac) / 2, my = h * (1 - frac) / 2;
  const int xa = static_cast<int>(std::max(0.0, box.x1 + mx));
  const int ya = static_cast<int>(std::max(0.0, box.y1 + my));
  const int xb = static_cast<int>(std::min(static_cast<double>(img.cols), box.x2 - mx));
  const int yb = static_cast<int>(std::min(static_cast<double>(img.rows), box.y2 - my));
  if (xb <= xa || yb <= ya) {return cv::Mat();}
  return img(cv::Range(ya, yb), cv::Range(xa, xb));
}

std::pair<std::string, int> classify_color(const cv::Mat & roi_bgr, const ColorCfg & cfg)
{
  if (roi_bgr.empty()) {return {"unknown", 0};}
  cv::Mat hsv;
  cv::cvtColor(roi_bgr, hsv, cv::COLOR_BGR2HSV);
  std::vector<double> H, S;
  H.reserve(hsv.total());
  S.reserve(hsv.total());
  for (int r = 0; r < hsv.rows; ++r) {
    const auto * p = hsv.ptr<cv::Vec3b>(r);
    for (int c = 0; c < hsv.cols; ++c) {
      if (p[c][2] > cfg.v_min) {
        H.push_back(p[c][0]);
        S.push_back(p[c][1]);
      }
    }
  }
  const int n = static_cast<int>(H.size());
  if (n < cfg.min_pixels) {return {"unknown", n};}

  if (percentile(S, 50) < cfg.s_beige_max) {
    return {in_hue(percentile(H, 50), cfg.beige_hue) ? "beige" : "unknown", n};
  }
  std::vector<double> colored;
  for (size_t i = 0; i < H.size(); ++i) {
    if (S[i] >= cfg.s_beige_max) {colored.push_back(H[i]);}
  }
  const double h = hue_median(colored);
  if (in_hue(h, cfg.red_hue)) {return {"red", n};}
  if (in_hue(h, cfg.green_hue)) {return {"green", n};}
  return {"unknown", n};
}

// ---------------------------------------------------------------- 신호등

std::string signal_state(
  const cv::Mat & roi_bgr, const std::vector<Tier> & tiers, double v_margin,
  std::map<std::string, double> * score_out)
{
  if (roi_bgr.empty() || tiers.empty()) {return "off";}
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

// ---------------------------------------------------------------- AprilTag

struct TagReader::Impl
{
#if PERCEPTION_CV_AT_LEAST(4, 7)
  std::unique_ptr<cv::aruco::ArucoDetector> det;
#else
  cv::Ptr<cv::aruco::Dictionary> dict;
  cv::Ptr<cv::aruco::DetectorParameters> params;
#endif
};

TagReader::TagReader(const std::string & family, int min_side_px)
: impl_(std::make_unique<Impl>()), min_side_(min_side_px)
{
  int id;
  if (family == "36h11") {
    id = cv::aruco::DICT_APRILTAG_36h11;
  } else if (family == "25h9") {
    id = cv::aruco::DICT_APRILTAG_25h9;
  } else if (family == "16h5") {
    id = cv::aruco::DICT_APRILTAG_16h5;
  } else {
    throw std::invalid_argument("지원하지 않는 태그 종류: " + family);
  }
#if PERCEPTION_CV_AT_LEAST(4, 7)
  impl_->det = std::make_unique<cv::aruco::ArucoDetector>(
    cv::aruco::getPredefinedDictionary(id), cv::aruco::DetectorParameters());
#else
  impl_->dict = cv::aruco::getPredefinedDictionary(id);
  impl_->params = cv::aruco::DetectorParameters::create();
#endif
}

TagReader::~TagReader() = default;

std::pair<int, std::vector<cv::Point2f>> TagReader::read(const cv::Mat & img_bgr, const Box & box, double pad) const
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
#if PERCEPTION_CV_AT_LEAST(4, 7)
  impl_->det->detectMarkers(gray, corners, ids);
#else
  cv::aruco::detectMarkers(gray, impl_->dict, corners, ids, impl_->params);
#endif
  if (ids.empty()) {return {-1, {}};}

  size_t k = 0;
  double best = -1;
  for (size_t i = 0; i < corners.size(); ++i) {
    const double a = cv::contourArea(corners[i]);
    if (a > best) {best = a; k = i;}
  }
  std::vector<cv::Point2f> pts;
  for (const auto & p : corners[k]) {
    pts.emplace_back(static_cast<float>(p.x / scale + xa), static_cast<float>(p.y / scale + ya));
  }
  return {ids[k], pts};
}

std::optional<Pose> tag_pose(
  const std::vector<cv::Point2f> & corners, double tag_size,
  const cv::Matx33d & K, const cv::Mat & D, bool fisheye)
{
  if (corners.size() != 4) {return std::nullopt;}
  const float s = static_cast<float>(tag_size / 2.0);
  const std::vector<cv::Point3f> obj{{-s, s, 0}, {s, s, 0}, {s, -s, 0}, {-s, -s, 0}};
  std::vector<cv::Point2d> img(corners.begin(), corners.end());
  cv::Mat dist = D.empty() ? cv::Mat::zeros(1, 4, CV_64F) : D;
  if (fisheye) {   // fisheye 는 먼저 왜곡을 풀고 왜곡 없는 pinhole 로 PnP
    std::vector<cv::Point2d> und;
    cv::fisheye::undistortPoints(img, und, K, dist, cv::noArray(), K);
    img = und;
    dist = cv::Mat::zeros(1, 4, CV_64F);
  }
  cv::Vec3d rvec, tvec;
  if (!cv::solvePnP(obj, img, K, dist, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE)) {return std::nullopt;}
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  return Pose{tvec, R};
}

std::optional<cv::Vec3d> depth_point(const cv::Mat & depth_m, const Box & box, const cv::Matx33d & K, double frac)
{
  const cv::Mat roi = center_roi(depth_m, box, frac);
  if (roi.empty()) {return std::nullopt;}
  std::vector<double> z;
  for (int r = 0; r < roi.rows; ++r) {
    const auto * p = roi.ptr<float>(r);
    for (int c = 0; c < roi.cols; ++c) {
      if (std::isfinite(p[c]) && p[c] > 0.05f) {z.push_back(p[c]);}
    }
  }
  if (z.size() < 10) {return std::nullopt;}
  const double zq = percentile(z, 25);
  const double u = (box.x1 + box.x2) / 2.0, v = (box.y1 + box.y2) / 2.0;
  const double fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);
  return cv::Vec3d((u - cx) * zq / fx, (v - cy) * zq / fy, zq);
}

std::array<double, 4> rot_to_quat(const cv::Matx33d & R)
{
  const double w = std::sqrt(std::max(0.0, 1 + R(0, 0) + R(1, 1) + R(2, 2))) / 2;
  double x = std::sqrt(std::max(0.0, 1 + R(0, 0) - R(1, 1) - R(2, 2))) / 2;
  double y = std::sqrt(std::max(0.0, 1 - R(0, 0) + R(1, 1) - R(2, 2))) / 2;
  double z = std::sqrt(std::max(0.0, 1 - R(0, 0) - R(1, 1) + R(2, 2))) / 2;
  x = std::copysign(x, R(2, 1) - R(1, 2));
  y = std::copysign(y, R(0, 2) - R(2, 0));
  z = std::copysign(z, R(1, 0) - R(0, 1));
  return {x, y, z, w};
}

// ---------------------------------------------------------------- 1D 바코드

struct BarcodeReader::Impl
{
  cv::barcode::BarcodeDetector det;
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));

  std::vector<std::pair<std::string, std::string>> decode(const cv::Mat & bgr)
  {
    std::vector<std::string> infos;
    std::vector<std::string> names;
    cv::Mat points;
#if PERCEPTION_CV_AT_LEAST(4, 8)
    const bool ok = det.detectAndDecodeWithType(bgr, infos, names, points);
#else
    std::vector<cv::barcode::BarcodeType> types;
    const bool ok = det.detectAndDecode(bgr, infos, types, points);
    for (auto t : types) {   // OpenCV 4.6~4.7 은 숫자 코드로 돌려줌
      switch (static_cast<int>(t)) {
        case 1: names.emplace_back("EAN_8"); break;
        case 2: names.emplace_back("EAN_13"); break;
        case 3: names.emplace_back("UPC_A"); break;
        case 4: names.emplace_back("UPC_E"); break;
        case 5: names.emplace_back("UPC_EAN_EXTENSION"); break;
        default: names.emplace_back(""); break;
      }
    }
#endif
    std::vector<std::pair<std::string, std::string>> out;
    if (!ok) {return out;}
    for (size_t i = 0; i < infos.size(); ++i) {
      if (!infos[i].empty()) {out.emplace_back(infos[i], i < names.size() ? names[i] : "");}
    }
    return out;
  }
};

BarcodeReader::BarcodeReader()
: impl_(std::make_unique<Impl>()) {}

BarcodeReader::~BarcodeReader() = default;

std::vector<std::pair<std::string, std::string>> BarcodeReader::read(
  const cv::Mat & img_bgr, const std::vector<double> & scales) const
{
  cv::Mat gray0;
  if (img_bgr.channels() == 3) {
    cv::cvtColor(img_bgr, gray0, cv::COLOR_BGR2GRAY);
  } else {
    gray0 = img_bgr;
  }
  for (double s : scales) {
    cv::Mat gray;
    if (s == 1.0) {
      gray = gray0;
    } else {
      cv::resize(gray0, gray, cv::Size(), s, s, cv::INTER_AREA);
    }
    for (int step = 0; step < 4; ++step) {   // 원본 → CLAHE → 샤프닝 → 이진화
      cv::Mat g;
      if (step == 0) {
        g = gray;
      } else if (step == 1) {
        impl_->clahe->apply(gray, g);
      } else if (step == 2) {
        cv::Mat blur;
        cv::GaussianBlur(gray, blur, cv::Size(0, 0), 3);
        cv::addWeighted(gray, 1.5, blur, -0.5, 0, g);
      } else {
        cv::adaptiveThreshold(gray, g, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 31, 5);
      }
      cv::Mat bgr;
      cv::cvtColor(g, bgr, cv::COLOR_GRAY2BGR);
      auto found = impl_->decode(bgr);
      if (!found.empty()) {return found;}
    }
  }
  return {};
}

}  // namespace perception
