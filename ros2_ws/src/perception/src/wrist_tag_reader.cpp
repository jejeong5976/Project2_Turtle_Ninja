// wrist_tag_reader.cpp — 손목 영상 태그 ID · 모서리 읽기 구현, 설명은 wrist_tag_reader.hpp

#include "perception/wrist_tag_reader.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/version.hpp>
// OpenCV 4.7 부터 aruco 가 objdetect 로 옮겨짐 (그 전은 contrib 모듈)
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#define WRIST_TAG_ARUCO_NEW 1
#else
#include <opencv2/aruco.hpp>
#endif

#ifdef PERCEPTION_HAVE_APRILTAG   // ros-<distro>-apriltag (AprilTag 3)
extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
}
#endif

namespace perception::wrist
{

bool undistort_corners(const TagCamera & cam, const std::vector<cv::Point2f> & px, std::vector<cv::Point2f> & out, std::string * why)
{
  // K · D 는 전체 해상도 기준 → 영상이 ROI 로 잘렸거나 비닝됐으면 화소를 전체 해상도로 되돌린 뒤 풂
  std::vector<cv::Point2f> full;
  for (const auto & p : px) {full.emplace_back(p.x * cam.binning_x + cam.roi_x, p.y * cam.binning_y + cam.roi_y);}
  const auto & m = cam.model;
  if (m == "equidistant") {   // 계수가 0 이어도 핀홀과 투영이 다름
    if (cam.D.size() < 4) {
      if (why) {*why = "camera_info equidistant 인데 계수가 4 개보다 적음";}
      return false;
    }
    cv::fisheye::undistortPoints(full, out, cam.K, cv::Mat(std::vector<double>(cam.D.begin(), cam.D.begin() + 4), true));
    return true;
  }
  bool zero = true;
  for (double d : cam.D) {zero = zero && d == 0.0;}
  if (zero || m == "plumb_bob" || m == "rational_polynomial") {
    cv::undistortPoints(full, out, cam.K, zero ? cv::Mat() : cv::Mat(cam.D, true));
    return true;
  }
  if (why) {*why = "camera_info 왜곡 모델 " + m + " 를 모름";}
  return false;
}

bool tag_near(const std::vector<cv::Point2f> & n, double size, double max_dist)
{
  const float h = static_cast<float>(size / 2.0);
  const std::vector<cv::Point3f> obj{{-h, h, 0.0f}, {h, h, 0.0f}, {h, -h, 0.0f}, {-h, -h, 0.0f}};   // SOLVEPNP_IPPE_SQUARE 순서
  double best = 1e9;
  cv::Vec3d c(0, 0, 0);
  for (int order = 0; order < 2; ++order) {
    std::vector<cv::Point2f> img = n;
    if (order == 1) {std::reverse(img.begin() + 1, img.end());}
    cv::Mat rvec, tvec;
    if (!cv::solvePnP(obj, img, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE)) {continue;}
    std::vector<cv::Point2f> back;
    cv::projectPoints(obj, rvec, tvec, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), back);
    double e = 0.0;
    for (int j = 0; j < 4; ++j) {e += cv::norm(back[j] - img[j]);}
    if (e < best && tvec.at<double>(2) > 0.0) {
      best = e;
      c = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    }
  }
  return best < 1e8 && c[2] > 0.02 && cv::norm(c) < max_dist;
}

cv::Rect project_box(const std::vector<cv::Point3d> & cam_pts, const cv::Matx33d & K, double pad_px, double min_z)
{
  double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  bool any = false;
  for (const auto & p : cam_pts) {
    if (!(p.z >= min_z)) {continue;}
    const double u = K(0, 0) * p.x / p.z + K(0, 2), v = K(1, 1) * p.y / p.z + K(1, 2);
    x0 = std::min(x0, u);
    y0 = std::min(y0, v);
    x1 = std::max(x1, u);
    y1 = std::max(y1, v);
    any = true;
  }
  if (!any) {return {};}
  // 화면 밖 먼 값은 merge_rois 가 자름. int 넘침만 막음
  auto c = [](double v) {return static_cast<int>(std::clamp(v, -1e5, 1e5));};
  return cv::Rect(cv::Point(c(std::floor(x0 - pad_px)), c(std::floor(y0 - pad_px))),
           cv::Point(c(std::ceil(x1 + pad_px)), c(std::ceil(y1 + pad_px))));
}

bool merge_rois(std::vector<cv::Rect> & rois, const cv::Size & img, double max_frac)
{
  const cv::Rect full(cv::Point(0, 0), img);
  std::vector<cv::Rect> out;
  for (const auto & r : rois) {
    const cv::Rect c = r & full;
    if (c.width >= 8 && c.height >= 8) {out.push_back(c);}
  }
  // 겹치면 감싼 상자로 합침, 더 안 바뀔 때까지 (영역 수십 개라 단순 반복)
  for (bool changed = true; changed;) {
    changed = false;
    for (size_t i = 0; i < out.size() && !changed; ++i) {
      for (size_t j = i + 1; j < out.size(); ++j) {
        if ((out[i] & out[j]).area() > 0) {
          out[i] |= out[j];
          out.erase(out.begin() + static_cast<long>(j));
          changed = true;
          break;
        }
      }
    }
  }
  rois = out;
  double area = 0.0;
  for (const auto & r : out) {area += r.area();}
  return area <= max_frac * full.area();
}

TagReader::TagReader(const Params & p)
: p_(p), clahe_(cv::createCLAHE(3.0, cv::Size(8, 8))), clahe_roi_(cv::createCLAHE(3.0, cv::Size(8, 8)))
{
#ifdef PERCEPTION_HAVE_APRILTAG
  auto * fam = tag36h11_create();
  auto * det = apriltag_detector_create();
  apriltag_detector_add_family(det, fam);
  det->quad_decimate = 1.0f;   // 640 × 480 그대로 (태그 약 100 화소)
  det->nthreads = p_.nthreads;
  det->refine_edges = true;
  fam_ = fam;
  det_ = det;
#endif
}

TagReader::~TagReader()
{
#ifdef PERCEPTION_HAVE_APRILTAG
  apriltag_detector_destroy(static_cast<apriltag_detector_t *>(det_));
  tag36h11_destroy(static_cast<apriltag_family_t *>(fam_));
#endif
}

bool TagReader::have_apriltag()
{
#ifdef PERCEPTION_HAVE_APRILTAG
  return true;
#else
  return false;
#endif
}

const char * TagReader::name()
{
  return have_apriltag() ? "CLAHE + AprilTag 3" : "CLAHE + OpenCV aruco (apriltag 없음: sudo apt install ros-<distro>-apriltag)";
}

void TagReader::detect(const cv::Mat & gray, std::vector<std::vector<cv::Point2f>> & corners, std::vector<int> & ids)
{
  corners.clear();
  ids.clear();
  cv::Mat eq;
  clahe_->apply(gray, eq);
  detect_eq(eq, cv::Point2f(0, 0), corners, ids);
}

void TagReader::detect_in(
  const cv::Mat & gray, const std::vector<cv::Rect> & rois, std::vector<std::vector<cv::Point2f>> & corners,
  std::vector<int> & ids)
{
  corners.clear();
  ids.clear();
  const cv::Rect full(0, 0, gray.cols, gray.rows);
  for (const auto & r0 : rois) {
    const cv::Rect r = r0 & full;
    if (r.width < 8 || r.height < 8) {continue;}
    // 전체 영상 8 × 8 조각 = 640 × 480 에서 조각 80 × 60 px → 영역에서도 조각 크기를 비슷하게 (wrist_tag_rescue 와 같음)
    clahe_roi_->setTilesGridSize(cv::Size(std::max(1, (r.width + 40) / 80), std::max(1, (r.height + 30) / 60)));
    cv::Mat eq;
    clahe_roi_->apply(gray(r), eq);
    std::vector<std::vector<cv::Point2f>> c;
    std::vector<int> i;
    detect_eq(eq, cv::Point2f(static_cast<float>(r.x), static_cast<float>(r.y)), c, i);
    for (size_t k = 0; k < i.size(); ++k) {
      if (std::find(ids.begin(), ids.end(), i[k]) != ids.end()) {continue;}
      ids.push_back(i[k]);
      corners.push_back(c[k]);
    }
  }
}

void TagReader::detect_eq(
  const cv::Mat & eq0, const cv::Point2f & off, std::vector<std::vector<cv::Point2f>> & corners, std::vector<int> & ids)
{
  const cv::Mat eq = eq0.isContinuous() ? eq0 : eq0.clone();
#ifdef PERCEPTION_HAVE_APRILTAG
  image_u8_t img{eq.cols, eq.rows, static_cast<int32_t>(eq.step), eq.data};
  zarray_t * dets = apriltag_detector_detect(static_cast<apriltag_detector_t *>(det_), &img);
  for (int k = 0; k < zarray_size(dets); ++k) {
    apriltag_detection_t * d = nullptr;
    zarray_get(dets, k, &d);
    if (d->hamming > p_.max_hamming || d->decision_margin < p_.min_margin) {continue;}
    std::vector<cv::Point2f> c;   // AprilTag 3 는 반시계 → aruco 순서 (좌상 · 우상 · 우하 · 좌하)
    for (int j : {1, 0, 3, 2}) {c.emplace_back(static_cast<float>(d->p[j][0]) + off.x, static_cast<float>(d->p[j][1]) + off.y);}
    corners.push_back(c);
    ids.push_back(d->id);
  }
  apriltag_detections_destroy(dets);
#else
  detect_aruco(eq, corners, ids);
  for (auto & c : corners) {for (auto & p : c) {p += off;}}
#endif
}

void TagReader::detect_aruco(const cv::Mat & eq, std::vector<std::vector<cv::Point2f>> & corners, std::vector<int> & ids)
{
#if defined(WRIST_TAG_ARUCO_NEW)
  static const cv::aruco::ArucoDetector det = [] {
      cv::aruco::DetectorParameters p;
      p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;   // 모서리를 화소 아래까지 (평면 교점 정확도)
      return cv::aruco::ArucoDetector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), p);
    }();
  det.detectMarkers(eq, corners, ids);
#else
  static const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  static const auto prm = [] {
      auto p = cv::aruco::DetectorParameters::create();
      p->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
      return p;
    }();
  cv::aruco::detectMarkers(eq, dict, corners, ids, prm);
#endif
}

}  // namespace perception::wrist
