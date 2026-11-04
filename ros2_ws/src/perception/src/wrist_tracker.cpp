// wrist_tracker.cpp — 손목 퍽 거르기 · 여러 장 확인 구현, 설명은 wrist_tracker.hpp

#include "perception/wrist_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <map>
#include <tuple>

namespace perception::wrist
{

namespace
{
std::vector<std::string> split(const std::string & s, char sep)
{
  std::vector<std::string> out(1);
  for (char c : s) {
    if (c == sep) {out.emplace_back();} else {out.back() += c;}
  }
  return out;
}

std::string shape_name(const std::string & s)
{
  if (s == "cyl") {return "cylinder";}
  if (s == "box") {return "box";}
  return "";
}

double wrap(double a, double period)
{
  double r = std::fmod(a + period / 2, period);
  if (r < 0) {r += period;}
  return r - period / 2;
}

// 넣은 순서를 지키는 다수결 (같은 점수면 먼저 나온 것 = Python Counter.most_common)
template<typename K>
struct OrderedVote
{
  std::vector<std::pair<K, double>> v;
  void add(const K & k, double w)
  {
    for (auto & e : v) {
      if (e.first == k) {e.second += w; return;}
    }
    v.emplace_back(k, w);
  }
  const K * best() const
  {
    const std::pair<K, double> * b = nullptr;
    for (const auto & e : v) {
      if (b == nullptr || e.second > b->second) {b = &e;}
    }
    return b ? &b->first : nullptr;
  }
};
}  // namespace

KindInfo parse_kind(const std::string & kind, const std::string & fallback_color)
{
  KindInfo k;
  if (kind.empty()) {
    k.color = fallback_color;
    return k;
  }
  const auto parts = split(kind, '_');
  k.color = parts[0];
  k.shape = parts.size() > 1 ? shape_name(parts[1]) : "";
  if (parts.size() >= 3) {k.lying = parts[2] == "lying";}
  return k;
}

std::string reject_reason(const PuckPose & p)
{
  if (p.truncated && !p.tag_centre) {return "truncated";}   // 태그로 중심을 잡았으면 마스크가 잘려도 씀 (박스 끝 퍽, 11-04)
  if (p.n_points < MIN_POINTS) {return "points";}
  if (p.spread[0] > MAX_LONG_SPREAD || p.spread[1] > MAX_MID_SPREAD) {return "size";}
  return "";
}

Mat3 axis_frame(const Vec3 & axis, const std::optional<Vec3> & face_dir)
{
  const Vec3 z = axis.normalized();
  Vec3 ref = face_dir ? *face_dir : Vec3::UnitX();
  Vec3 x = ref - z * ref.dot(z);
  if (x.norm() < 1e-6) {                                     // ref 가 긴축과 나란하면 다른 기준
    ref = Vec3::UnitY();
    x = ref - z * ref.dot(z);
  }
  x.normalize();
  Mat3 R;
  R.col(0) = x;
  R.col(1) = z.cross(x);
  R.col(2) = z;
  return R;
}

std::array<double, 4> quat_xyzw(const Mat3 & R)
{
  std::array<double, 4> q{};
  const double tr = R.trace();
  if (tr > 0) {
    const double s = std::sqrt(tr + 1.0) * 2;
    q = {(R(2, 1) - R(1, 2)) / s, (R(0, 2) - R(2, 0)) / s, (R(1, 0) - R(0, 1)) / s, 0.25 * s};
  } else {
    int i = 0;
    if (R(1, 1) > R(i, i)) {i = 1;}
    if (R(2, 2) > R(i, i)) {i = 2;}
    const int j = (i + 1) % 3, k = (i + 2) % 3;
    const double s = std::sqrt(1.0 + R(i, i) - R(j, j) - R(k, k)) * 2;
    q[3] = (R(k, j) - R(j, k)) / s;
    q[i] = 0.25 * s;
    q[j] = (R(j, i) + R(i, j)) / s;
    q[k] = (R(k, i) + R(i, k)) / s;
  }
  if (q[3] < 0) {
    for (auto & v : q) {v = -v;}
  }
  return q;
}

Mat3 quat_to_mat(const std::array<double, 4> & q)
{
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  Mat3 R;
  R << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y);
  return R;
}

PuckTracker::PuckTracker(int window, int min_hits, double match_m)
: window_(window), min_hits_(min_hits), match_m_(match_m) {}

void PuckTracker::reset()
{
  frame_ = -1;
  tracks_.clear();
}

std::vector<PuckDetection> PuckTracker::update(const std::vector<RawPuck> & raws)
{
  ++frame_;
  std::vector<const RawPuck *> good;
  for (const auto & r : raws) {
    if (r.reject.empty()) {good.push_back(&r);}
  }
  // 가까운 것부터 일대일로 기존 묶음에 붙임
  std::vector<std::tuple<double, size_t, size_t>> pairs;
  for (size_t i = 0; i < good.size(); ++i) {
    for (size_t j = 0; j < tracks_.size(); ++j) {
      pairs.emplace_back((good[i]->pose.center - tracks_[j].obs.back().second.pose.center).norm(), i, j);
    }
  }
  std::sort(pairs.begin(), pairs.end());
  std::vector<bool> used_r(good.size(), false), used_t(tracks_.size(), false);
  for (const auto & [d, i, j] : pairs) {
    if (d > match_m_ || used_r[i] || used_t[j]) {continue;}
    auto & t = tracks_[j];
    t.obs.emplace_back(frame_, *good[i]);
    // 누운 퍽 · 위쪽 모름: 이전 장 부호에 맞춤 (화살표 안 뒤집히게)
    auto & pose = t.obs.back().second.pose;
    const auto & prev = t.obs[t.obs.size() - 2].second.pose;
    if (!(pose.upright && *pose.upright) && pose.axis.dot(prev.axis) < 0) {pose.axis = -pose.axis;}
    if (t.obs.size() > 16) {t.obs.pop_front();}
    t.last_frame = frame_;
    used_r[i] = used_t[j] = true;
  }
  for (size_t i = 0; i < good.size(); ++i) {
    if (used_r[i]) {continue;}
    Track t;
    t.obs.emplace_back(frame_, *good[i]);
    t.last_frame = frame_;
    tracks_.push_back(std::move(t));
  }
  // window 동안 안 보이면 버림
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
    [this](const Track & t) {return frame_ - t.last_frame >= window_;}), tracks_.end());
  std::vector<PuckDetection> out;
  for (const auto & t : tracks_) {
    std::vector<const RawPuck *> recent;
    for (const auto & [f, r] : t.obs) {
      if (frame_ - f < window_) {recent.push_back(&r);}
    }
    if (t.last_frame == frame_ && static_cast<int>(recent.size()) >= min_hits_) {out.push_back(summarize(recent));}
  }
  return out;
}

PuckDetection PuckTracker::summarize(const std::vector<const RawPuck *> & recent) const
{
  // 선/누움 다수결: 많은 쪽 장만 남겨 중심 · 긴축 · 면 방향 계산 (동점이면 최근 장). 위쪽 모름이면 전부
  std::vector<const RawPuck *> known, group;
  for (const auto * r : recent) {
    if (r->pose.upright) {known.push_back(r);}
  }
  bool upright = false;
  if (!known.empty()) {
    int n_up = 0;
    for (const auto * r : known) {n_up += *r->pose.upright ? 1 : 0;}
    const int n_flat = static_cast<int>(known.size()) - n_up;
    upright = n_up != n_flat ? n_up > n_flat : *known.back()->pose.upright;
    for (const auto * r : known) {
      if (*r->pose.upright == upright) {group.push_back(r);}
    }
  } else {
    group = recent;
  }
  const RawPuck & last = *group.back();
  // 중심 = 좌표별 중앙값
  Vec3 center;
  for (int k = 0; k < 3; ++k) {
    std::vector<double> c;
    for (const auto * r : group) {c.push_back(r->pose.center[k]);}
    center[k] = median(std::move(c));
  }
  // 긴축 = 마지막 장 부호에 맞춰 평균
  const Vec3 ref = last.pose.axis;
  Vec3 axis = Vec3::Zero();
  for (const auto * r : group) {axis += r->pose.axis * (r->pose.axis.dot(ref) >= 0 ? 1.0 : -1.0);}
  axis.normalize();
  // 색 · 모양 (신뢰도 가중), 태그 ID 최빈값
  OrderedVote<std::pair<std::string, std::string>> votes;
  OrderedVote<int> ids;
  double conf_sum = 0.0;
  for (const auto * r : recent) {
    votes.add({r->info.color, r->info.shape}, r->conf);
    if (r->pose.tag) {ids.add(r->pose.tag->id, 1.0);}
    conf_sum += r->conf;
  }
  const auto [color, shape] = *votes.best();
  // 사각기둥 면 방향 (선 = 윗면 윤곽, 누움 = 위쪽): 90° 주기라 4θ 각으로 평균. group = 선/누움 같은 장만
  std::optional<Vec3> face;
  if (shape == "box") {
    std::vector<Vec3> faces;
    for (const auto * r : group) {
      if (r->pose.face_dir) {faces.push_back(*r->pose.face_dir);}
    }
    if (!faces.empty()) {
      const Vec3 f_ref = faces.back();
      const Vec3 f_ref2 = axis.cross(f_ref);
      std::complex<double> acc(0.0, 0.0);
      for (const auto & f : faces) {acc += std::polar(1.0, 4.0 * std::atan2(f.dot(f_ref2), f.dot(f_ref)));}
      const double a4 = std::arg(acc / static_cast<double>(faces.size())) / 4.0;
      face = std::cos(a4) * f_ref + std::sin(a4) * f_ref2;
    }
  }
  PuckDetection d;
  d.label = color;
  d.shape = shape;
  if (const int * id = ids.best()) {d.id = std::to_string(*id);}
  d.confidence = conf_sum / static_cast<double>(recent.size());
  d.position = center;
  d.orientation = quat_xyzw(axis_frame(axis, face));
  d.bbox = last.pose.bbox;
  d.yaw_known = face.has_value();
  d.hits = static_cast<int>(recent.size());
  d.axis_source = last.pose.axis_source;
  if (!known.empty()) {d.upright = upright;}
  return d;
}

PlanarPose planar_pose(const PuckDetection & d, const Mat3 & R_bc, const Vec3 & t_bc)
{
  static const double upright_cos = std::cos(M_PI / 4);       // 긴축과 위쪽 45° 안이면 선 퍽
  const Mat3 R_bp = R_bc * quat_to_mat(d.orientation);
  const Vec3 p = R_bc * d.position + t_bc;
  const Vec3 axis = R_bp.col(2);
  PlanarPose out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  out.upright = std::abs(axis.z()) > upright_cos;
  if (out.upright) {
    const Vec3 face = R_bp.col(0);
    out.theta_known = d.shape == "box" && d.yaw_known;
    out.theta = out.theta_known ? wrap(std::atan2(face.y(), face.x()), M_PI / 2) : 0.0;
  } else {
    out.theta = wrap(std::atan2(axis.y(), axis.x()), M_PI);
    out.theta_known = true;
  }
  return out;
}

}  // namespace perception::wrist
