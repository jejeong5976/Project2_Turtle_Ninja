// common.cpp — AprilTag 검출 구현, 설명은 common.hpp

#include "perception/common.hpp"

#include <opencv2/core/version.hpp>

// OpenCV 4.7 부터 aruco 가 objdetect 로 옮겨짐 (그 전은 contrib 모듈)
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#define PERCEPTION_ARUCO_NEW 1
#else
#include <opencv2/aruco.hpp>
#endif

namespace perception
{

void detect_apriltags(const cv::Mat & gray, std::vector<std::vector<cv::Point2f>> & corners, std::vector<int> & ids)
{
  corners.clear();
  ids.clear();
#ifdef PERCEPTION_ARUCO_NEW
  thread_local const cv::aruco::ArucoDetector det(
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), cv::aruco::DetectorParameters());
  det.detectMarkers(gray, corners, ids);
#else
  // OpenCV 4.6 기본값은 작은 퍽 태그(16 mm)를 4.7+ (Python cv2 5.0) 보다 덜 읽음 (손목 시뮬 300 장: 246 ↔ 346)
  //   → 칸 가장자리 무시 비율 · 모서리 거리 기준을 낮춤 (같은 시험에서 337)
  thread_local const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  thread_local const auto prm = [] {
      auto p = cv::aruco::DetectorParameters::create();
      p->perspectiveRemoveIgnoredMarginPerCell = 0.3;
      p->minCornerDistanceRate = 0.02;
      p->minMarkerDistanceRate = 0.02;
      p->minDistanceToBorder = 1;
      return p;
    }();
  cv::aruco::detectMarkers(gray, dict, corners, ids, prm);
#endif
}

}  // namespace perception
