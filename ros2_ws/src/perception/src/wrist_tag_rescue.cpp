// wrist_tag_rescue.cpp — 손목 퍽 태그 다시 찾기 구현, 설명은 wrist_tag_rescue.hpp

#include "perception/wrist_tag_rescue.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>

#include "perception/common.hpp"

#ifdef PERCEPTION_HAVE_APRILTAG   // apriltag 패키지 (AprilTag 3), manipulation rack_tag_node 와 같은 것
extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
}
#endif

namespace perception::wrist
{

cv::Point2f RawTag::centre() const
{
  cv::Point2f c(0, 0);
  for (const auto & p : corners) {c += p * 0.25f;}
  return c;
}

bool apriltag3_available()
{
#ifdef PERCEPTION_HAVE_APRILTAG
  return true;
#else
  return false;
#endif
}

int tag_owner(const cv::Point2f & centre, const std::vector<RescuePuck> & pucks, int margin_px)
{
  const int x = static_cast<int>(std::lround(centre.x)), y = static_cast<int>(std::lround(centre.y));
  auto box_dist = [&centre](const cv::Rect & b) {
      const double dx = b.x + b.width / 2.0 - centre.x, dy = b.y + b.height / 2.0 - centre.y;
      return dx * dx + dy * dy;
    };
  // 1) 마스크 안 (여유 없이): 여럿이면 상자 중심이 가까운 퍽
  int best = -1;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < pucks.size(); ++i) {
    const auto & m = pucks[i].mask;
    if (m.empty() || x < 0 || y < 0 || x >= m.cols || y >= m.rows || !m(y, x)) {continue;}
    const double d = box_dist(pucks[i].box);
    if (d < best_d) {best_d = d; best = static_cast<int>(i);}
  }
  if (best >= 0 || margin_px <= 0) {return best;}
  // 2) margin_px 안 가장 가까운 마스크 화소, 거리가 같은 퍽이 둘이면 애매 → 안 붙임
  int best_d2 = margin_px * margin_px + 1;
  bool tie = false;
  for (size_t i = 0; i < pucks.size(); ++i) {
    const auto & m = pucks[i].mask;
    if (m.empty()) {continue;}
    const cv::Rect win = cv::Rect(x - margin_px, y - margin_px, 2 * margin_px + 1, 2 * margin_px + 1) &
      cv::Rect(0, 0, m.cols, m.rows);
    int d2_min = std::numeric_limits<int>::max();
    for (int v = win.y; v < win.y + win.height; ++v) {
      const uint8_t * row = m.ptr<uint8_t>(v);
      for (int u = win.x; u < win.x + win.width; ++u) {
        if (!row[u]) {continue;}
        const int d2 = (u - x) * (u - x) + (v - y) * (v - y);
        d2_min = std::min(d2_min, d2);
      }
    }
    if (d2_min > margin_px * margin_px) {continue;}
    if (d2_min < best_d2) {best_d2 = d2_min; best = static_cast<int>(i); tie = false;} else if (d2_min == best_d2) {tie = true;}
  }
  return tie ? -1 : best;
}

#ifdef PERCEPTION_HAVE_APRILTAG
struct TagRescuer::At3
{
  apriltag_family_t * fam{nullptr};
  apriltag_detector_t * det{nullptr};
  explicit At3(int max_hamming)
  {
    fam = tag36h11_create();
    det = apriltag_detector_create();
    // 고칠 비트 수 = 받을 hamming 한계 (기본 2 는 빠른 표가 수십 MB · 시작 수백 ms, 어차피 1 넘으면 버림)
    apriltag_detector_add_family_bits(det, fam, std::clamp(max_hamming, 0, 2));
    det->quad_decimate = 1.0f;   // 640 × 480 그대로 (퍽 태그 약 20–40 px)
    det->nthreads = 1;           // 작은 상자 하나라 스레드 나눌 일 없음 (손목 콜백 안)
    det->refine_edges = true;
  }
  ~At3()
  {
    apriltag_detector_destroy(det);
    tag36h11_destroy(fam);
  }
};
#else
struct TagRescuer::At3 {};
#endif

TagRescuer::TagRescuer(const TagRescueParams & p)
: p_(p), clahe_(cv::createCLAHE(3.0, cv::Size(8, 8)))
{
#ifdef PERCEPTION_HAVE_APRILTAG
  if (p_.apriltag3) {at3_ = std::make_unique<At3>(p_.at3_max_hamming);}
#endif
}

TagRescuer::~TagRescuer() = default;

std::vector<std::string> TagRescuer::methods() const
{
  // 합성 시험 (test_wrist_tag_rescue MethodUnion, 1차 실패 705 장): AprilTag 3 원본 125 → + CLAHE 147 → + aruco 원본 147
  //   → + aruco CLAHE 149. aruco CLAHE 는 못 찾는 상자에서 비쌈 (잡음이 커져 후보가 많음) → AprilTag 3 없을 때만
  std::vector<std::string> m;
  if (at3_) {
    m.emplace_back("at3");
    if (p_.clahe) {m.emplace_back("at3_clahe");}
  }
  if (p_.aruco) {
    m.emplace_back("aruco");
    if (p_.clahe && !at3_) {m.emplace_back("aruco_clahe");}
  }
  return m;
}

cv::Mat TagRescuer::equalize(const cv::Mat & sub)
{
  // rack_tag_node 는 전체 640 × 480 에 8 × 8 조각 (조각 80 × 60 px) → 상자에서도 조각 크기를 비슷하게
  const int gx = std::clamp(static_cast<int>(std::lround(sub.cols / 80.0)), 1, 8);
  const int gy = std::clamp(static_cast<int>(std::lround(sub.rows / 60.0)), 1, 8);
  clahe_->setTilesGridSize(cv::Size(gx, gy));
  cv::Mat eq;
  clahe_->apply(sub, eq);
  return eq;
}

std::vector<RawTag> TagRescuer::detect_once(const cv::Mat & gray, const cv::Rect & roi, const std::string & method)
{
  std::vector<RawTag> out;
  const cv::Rect r = roi & cv::Rect(0, 0, gray.cols, gray.rows);
  if (r.width < 8 || r.height < 8) {return out;}
  const cv::Mat sub = gray(r);
  const cv::Point2f off(static_cast<float>(r.x), static_cast<float>(r.y));
  if (method == "at3" || method == "at3_clahe") {
#ifdef PERCEPTION_HAVE_APRILTAG
    if (!at3_) {return out;}
    cv::Mat img = method == "at3_clahe" ? equalize(sub) : sub;
    image_u8_t im{img.cols, img.rows, static_cast<int32_t>(img.step), img.data};
    zarray_t * dets = apriltag_detector_detect(at3_->det, &im);
    for (int k = 0; k < zarray_size(dets); ++k) {
      apriltag_detection_t * d = nullptr;
      zarray_get(dets, k, &d);
      if (!is_puck_tag(d->id) || d->hamming > p_.at3_max_hamming || d->decision_margin < p_.at3_min_margin) {continue;}
      RawTag t;
      t.id = d->id;
      // AprilTag 3 p[0..3] → aruco 순서 (좌상 · 우상 · 우하 · 좌하), rack_tag_node 와 같은 대응
      //   AprilTag 3 좌표는 화소 왼쪽 위 모서리가 0 (화소 중심 +0.5) → OpenCV 규약 (화소 중심 = 정수) 으로 −0.5 (시험: 참값 투영과 비교)
      int j = 0;
      for (int q : {1, 0, 3, 2}) {
        t.corners[j++] = cv::Point2f(static_cast<float>(d->p[q][0] - 0.5), static_cast<float>(d->p[q][1] - 0.5)) + off;
      }
      out.push_back(t);
    }
    apriltag_detections_destroy(dets);
#endif
    return out;
  }
  if (method == "aruco" || method == "aruco_clahe") {
    const cv::Mat img = method == "aruco_clahe" ? equalize(sub) : sub;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    detect_apriltags(img, corners, ids);   // common.hpp (1차와 같은 사전 · 설정)
    for (size_t k = 0; k < ids.size(); ++k) {
      if (!is_puck_tag(ids[k]) || corners[k].size() != 4) {continue;}
      RawTag t;
      t.id = ids[k];
      for (int i = 0; i < 4; ++i) {t.corners[i] = corners[k][i] + off;}
      out.push_back(t);
    }
  }
  return out;
}

std::vector<std::optional<TagPose>> TagRescuer::rescue(
  const cv::Mat & gray, const Intrinsics & K, const std::vector<RescuePuck> & pucks,
  const std::vector<TagPose> & primary, RescueStats * stats, const cv::Mat & dist)
{
  const auto t0 = std::chrono::steady_clock::now();
  RescueStats local;
  RescueStats & st = stats ? *stats : local;
  st = RescueStats{};
  st.how.assign(pucks.size(), "");
  st.tried.assign(pucks.size(), 0);
  std::vector<std::optional<TagPose>> out(pucks.size());
  std::set<int> taken;   // 이번 장 다른 퍽이 가진 ID · 다른 퍽 묶음 ID · 이미 붙인 ID
  for (const auto & pk : pucks) {
    if (pk.own_id >= 0) {taken.insert(pk.own_id);}
    st.need += pk.need ? 1 : 0;
    st.resting += pk.need && pk.rest ? 1 : 0;
  }
  // 태그 하나를 주인 퍽에 붙여 봄 → 붙인 퍽 번호, 못 붙이면 -1
  auto try_assign = [&](int id, const std::array<cv::Point2f, 4> & corners, const std::string & how) {
      if (!is_puck_tag(id) || taken.count(id)) {return -1;}
      cv::Point2f c(0, 0);
      for (const auto & q : corners) {c += q * 0.25f;}
      const int o = tag_owner(c, pucks, p_.margin_px);
      if (o < 0 || !pucks[static_cast<size_t>(o)].need || out[static_cast<size_t>(o)]) {return -1;}
      auto t = tag_pose_from_corners(id, corners, K, dist);
      if (!t) {return -1;}
      out[static_cast<size_t>(o)] = std::move(t);
      taken.insert(id);
      st.how[static_cast<size_t>(o)] = how;
      ++st.found;
      return o;
    };
  // 0) 1차 검출에서 안 붙은 태그 (붙은 ID 는 own_id 로 taken 에 있음)
  for (const auto & t : primary) {
    if (try_assign(t.id, t.corners, "primary") >= 0) {++st.from_primary;}
  }
  // 1–4) 남은 퍽마다 넓힌 상자로 다시 검출, 그 퍽 태그를 찾으면 멈춤 (찾은 다른 퍽 태그도 주인에게 붙임)
  const auto ms = methods();
  for (size_t i = 0; i < pucks.size(); ++i) {
    if (!pucks[i].need || pucks[i].rest || out[i]) {continue;}
    if (st.rois > 0 && p_.budget_ms > 0.0 &&
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() > p_.budget_ms)
    {
      ++st.over_budget;   // 다음 장으로 (쉬지 않음)
      continue;
    }
    const auto & b = pucks[i].box;
    const cv::Rect roi = cv::Rect(b.x - p_.pad_px, b.y - p_.pad_px, b.width + 2 * p_.pad_px, b.height + 2 * p_.pad_px) &
      cv::Rect(0, 0, gray.cols, gray.rows);
    if (roi.area() <= 0) {continue;}
    ++st.rois;
    st.tried[i] = 1;
    st.roi_rects.push_back(roi);
    for (const auto & m : ms) {
      if (m.rfind("at3", 0) == 0) {++st.at3_runs;} else {++st.aruco_runs;}
      for (const auto & t : detect_once(gray, roi, m)) {try_assign(t.id, t.corners, m);}
      if (out[i]) {break;}
      if (p_.budget_ms > 0.0 &&   // 한 상자 안 단계 사이에도 시간 한도 (젯슨에서 못 찾는 퍽 하나가 단계를 다 돌면 10 ms 넘음)
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() > p_.budget_ms)
      {
        ++st.over_budget;
        break;
      }
    }
  }
  return out;
}

bool RescueCooldown::resting(const Vec3 & c) const
{
  for (const auto & [p, until] : failed_) {
    if (frame_ < until && (p - c).norm() <= match_m_) {return true;}
  }
  return false;
}

void RescueCooldown::failed(const Vec3 & c)
{
  failed_.erase(std::remove_if(failed_.begin(), failed_.end(),
    [this, &c](const std::pair<Vec3, long> & f) {return f.second <= frame_ || (f.first - c).norm() <= match_m_;}), failed_.end());
  if (retry_frames_ > 0) {failed_.emplace_back(c, frame_ + 1 + retry_frames_);}
}

int rescue_raw_pucks(
  TagRescuer & rescuer, const cv::Mat & gray, const cv::Mat1f & depth_m, const Intrinsics & K, const Vec3 & up_cam,
  const std::vector<SegInstance> & inst, const std::vector<int> & raw_of, std::vector<RawPuck> & raws,
  const std::vector<TagPose> & primary, const PuckTracker & tracker, double min_rise_m, bool tag_centre,
  RescueCooldown * cooldown, RescueStats * stats, PoseTiming * timing)
{
  RescueStats local;
  RescueStats & st = stats ? *stats : local;
  st = RescueStats{};
  st.how.assign(inst.size(), "");
  st.tried.assign(inst.size(), 0);
  if (cooldown) {cooldown->next_frame();}
  std::vector<RescuePuck> pucks(inst.size());
  bool any = false;
  for (size_t k = 0; k < inst.size(); ++k) {
    auto & pk = pucks[k];
    const auto & s = inst[k];
    pk.mask = s.mask;
    pk.box = cv::Rect(cv::Point(static_cast<int>(std::floor(s.box[0])), static_cast<int>(std::floor(s.box[1]))),
      cv::Point(static_cast<int>(std::ceil(s.box[2])), static_cast<int>(std::ceil(s.box[3]))));
    const int ri = k < raw_of.size() ? raw_of[k] : -1;
    if (ri < 0 || ri >= static_cast<int>(raws.size())) {continue;}   // 자세 없는 분할: 마스크만 (옆 퍽 판정용)
    const auto & r = raws[static_cast<size_t>(ri)];
    pk.box = cv::Rect(cv::Point(r.pose.bbox[0], r.pose.bbox[1]), cv::Point(r.pose.bbox[2] + 1, r.pose.bbox[3] + 1));
    pk.own_id = r.pose.tag ? r.pose.tag->id : tracker.known_id_near(r.pose.center);
    // truncated 퍽은 태그 중심으로 잡을 때 (tag_centre) 만: 아니면 다시 찾은 태그 중심 장이 묶음의 모양 중심 장을 밀어냄 (wrist_tracker summarize)
    pk.need = !r.pose.tag && pk.own_id < 0 && (r.reject.empty() || (tag_centre && r.reject == "truncated"));
    pk.rest = pk.need && cooldown && cooldown->resting(r.pose.center);
    any = any || pk.need;
  }
  if (!any) {return 0;}
  const auto found = rescuer.rescue(gray, K, pucks, primary, &st);
  int n = 0;
  for (size_t k = 0; k < inst.size(); ++k) {
    if (!found[k]) {
      if (cooldown && st.tried[k] && raw_of[k] >= 0) {cooldown->failed(raws[static_cast<size_t>(raw_of[k])].pose.center);}
      continue;
    }
    auto & r = raws[static_cast<size_t>(raw_of[k])];
    // 1차에서 이 태그를 찾았을 때와 같게: 자세 다시 (태그 중심 보정 · truncated 풀림 포함). 상자 ±5 px 밖 태그면 ID 만 붙임
    if (auto p = puck_pose(inst[k].mask, depth_m, K, {*found[k]}, &up_cam, min_rise_m, tag_centre, timing)) {
      r.pose = std::move(*p);
    }
    if (!r.pose.tag) {r.pose.tag = *found[k];}
    r.reject = reject_reason(r.pose);
    ++n;
  }
  return n;
}

}  // namespace perception::wrist
