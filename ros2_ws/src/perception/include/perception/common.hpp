// common.hpp — 광각 · 손목 · 바코드 공용 도구
//
// 역할
//   - AprilTag 36h11 검출 detect_apriltags
//   - 태그 ID 약속: 0–11 = 퍽, 12–14 = 장소 → is_puck_tag 로 퍽 태그만 고름
//   - 다수결 Voter: 최근 window 개 중 같은 값이 need 번 이상이면 확정, 신호등 · 바코드 안정화에 씀
// 입력
//   - detect_apriltags: 회색 영상
//   - Voter::push: 한 장의 판정값
// 출력
//   - detect_apriltags: 태그마다 모서리 4 개 · ID, ID 거르기는 부르는 쪽이 함
//   - Voter: 확정값 또는 nullopt, 가장 많은 값의 비율
// 실행
//   - perception_core 라이브러리. 시험 test_common
// note
//   - OpenCV 4.6 contrib aruco 는 작은 퍽 태그에 맞춰 검출 설정을 낮춤, 4.7 이상 objdetect 는 기본값
//   - 검출기는 스레드마다 하나씩 둠
//   - 장소 태그는 퍽 옆 · 아래에 같이 보일 수 있어 퍽 ID 로 붙이면 안 됨
//   - kPuckTagSideM 16 mm 는 손목 PnP 에만 쓰고 결과 회전에는 영향 없음. 인쇄 실측은 약 15.2–15.4 mm

#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

// ---------------------------------------------------------------- AprilTag ID
constexpr int kPuckTagMinId = 0;
constexpr int kPuckTagMaxId = 11;
// 퍽 태그 검은 사각 한 변 (m). 손목 PnP 는 회전만 쓰므로 (거리는 깊이 점군) 이 값은 결과 회전에 영향 없음
//   인쇄 실측 약 15.2–15.4 mm (미정밀 실측). 태그로 거리를 재게 되면 실측값으로 바꿀 것
constexpr double kPuckTagSideM = 0.016;

inline bool is_puck_tag(int id) {return id >= kPuckTagMinId && id <= kPuckTagMaxId;}

// AprilTag 36h11 검출 (광각 퍽 태그 · 손목 공용, ID 거르기는 부르는 쪽). 회색 영상 → 모서리 4 개씩 · ID
//   OpenCV 4.6 (contrib aruco) 는 작은 퍽 태그용으로 설정을 낮춤, 4.7+ (objdetect) 는 기본값. 스레드마다 검출기 하나
void detect_apriltags(const cv::Mat & gray, std::vector<std::vector<cv::Point2f>> & corners, std::vector<int> & ids);

// ---------------------------------------------------------------- 다수결
// 최근 window 개 중 같은 값이 need 번 이상이면 그 값을 확정 (신호등 · 바코드)
template<typename T>
class Voter
{
public:
  Voter(int window, int need)
  : window_(static_cast<size_t>(std::max(window, 1))), need_(need) {}

  // (확정값 또는 nullopt, 가장 많은 값의 비율)
  std::pair<std::optional<T>, double> push(const T & value)
  {
    buf_.push_back(value);
    while (buf_.size() > window_) {buf_.pop_front();}
    return result();
  }

  std::pair<std::optional<T>, double> result() const
  {
    if (buf_.empty()) {return {std::nullopt, 0.0};}
    // 가장 많은 값 (같으면 먼저 들어온 값)
    const T * best = nullptr;
    int best_n = 0;
    for (const auto & v : buf_) {
      const int n = count(v);
      if (n > best_n) {best_n = n; best = &v;}
    }
    const double ratio = static_cast<double>(best_n) / static_cast<double>(buf_.size());
    if (best_n >= need_) {return {*best, ratio};}
    return {std::nullopt, ratio};
  }

  int count(const T & value) const
  {
    int n = 0;
    for (const auto & v : buf_) {n += (v == value) ? 1 : 0;}
    return n;
  }

  void clear() {buf_.clear();}

private:
  size_t window_;
  int need_;
  std::deque<T> buf_;
};

}  // namespace perception
