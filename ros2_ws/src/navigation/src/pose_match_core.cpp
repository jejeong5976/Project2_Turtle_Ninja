// pose_match_core.cpp — scripts/pose_match_core.py 의 C++ 판 (같은 식·같은 계산 순서). ROS 없음.
#include "navigation/pose_match_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace navigation
{
namespace pose_match
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kDegToRad = M_PI / 180.0;   // CPython math.radians 와 같음 (x * (pi / 180))

double radians(double deg) {return deg * kDegToRad;}

// 1 차원 제곱 거리 변환 (Felzenszwalb). f = 제곱거리 또는 kBig (점유 없음). 정수라 결과가 정확함
constexpr int64_t kBig = std::numeric_limits<int64_t>::max() / 4;

void dt1d(const int64_t * f, int64_t n, int64_t * d, int64_t * v, double * z)
{
  int64_t k = -1;
  for (int64_t q = 0; q < n; ++q) {
    if (f[q] >= kBig) {
      continue;
    }
    if (k < 0) {
      k = 0;
      v[0] = q;
      z[0] = -kInf;
      z[1] = kInf;
      continue;
    }
    double s = 0.0;
    while (true) {
      const int64_t p = v[k];
      s = static_cast<double>((f[q] + q * q) - (f[p] + p * p)) / static_cast<double>(2 * q - 2 * p);
      if (s <= z[k]) {
        --k;   // z[0] = -inf 라 k 는 0 아래로 안 감
        continue;
      }
      break;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  if (k < 0) {
    for (int64_t q = 0; q < n; ++q) {
      d[q] = kBig;
    }
    return;
  }
  int64_t j = 0;
  for (int64_t q = 0; q < n; ++q) {
    while (z[j + 1] < static_cast<double>(q)) {
      ++j;
    }
    const int64_t p = v[j];
    d[q] = (q - p) * (q - p) + f[p];
  }
}

// 정확한 유클리드 거리 (scipy distance_transform_edt(~occupied) 와 같은 값)
std::vector<float> distance_field(const std::vector<int16_t> & grid, int64_t w, int64_t h, double resolution)
{
  const int64_t n = std::max(w, h);
  std::vector<int64_t> f(n), d(n), v(n);
  std::vector<double> z(n + 1);
  std::vector<int64_t> col(static_cast<std::size_t>(w * h));
  // 열 방향
  for (int64_t x = 0; x < w; ++x) {
    for (int64_t y = 0; y < h; ++y) {
      f[y] = grid[y * w + x] > 50 ? 0 : kBig;
    }
    dt1d(f.data(), h, d.data(), v.data(), z.data());
    for (int64_t y = 0; y < h; ++y) {
      col[y * w + x] = d[y];
    }
  }
  // 행 방향
  std::vector<float> out(static_cast<std::size_t>(w * h));
  for (int64_t y = 0; y < h; ++y) {
    for (int64_t x = 0; x < w; ++x) {
      f[x] = col[y * w + x];
    }
    dt1d(f.data(), w, d.data(), v.data(), z.data());
    for (int64_t x = 0; x < w; ++x) {
      out[y * w + x] = static_cast<float>(std::sqrt(static_cast<double>(d[x])) * resolution);
    }
  }
  return out;
}

bool all_finite(const Pose & p)
{
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

// numpy lexsort((residual, -scores)): 점수 큰 순, 같으면 잔차 작은 순, 같으면 원래 순서
std::vector<std::size_t> order_of(const std::vector<double> & scores, const std::vector<double> & residual)
{
  std::vector<std::size_t> order(scores.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      const double sa = -scores[a];
      const double sb = -scores[b];
      if (sa < sb) {
        return true;
      }
      if (sb < sa) {
        return false;
      }
      return residual[a] < residual[b];
    });
  return order;
}
}  // namespace

double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double pairwise_sum(const double * a, std::size_t n)
{
  if (n < 8) {
    double res = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      res += a[i];
    }
    return res;
  }
  if (n <= 128) {
    double r[8];
    for (int j = 0; j < 8; ++j) {
      r[j] = a[j];
    }
    std::size_t i = 8;
    for (; i < n - (n % 8); i += 8) {
      for (int j = 0; j < 8; ++j) {
        r[j] += a[i + j];
      }
    }
    double res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
    for (; i < n; ++i) {
      res += a[i];
    }
    return res;
  }
  std::size_t n2 = n / 2;
  n2 -= n2 % 8;
  return pairwise_sum(a, n2) + pairwise_sum(a + n2, n - n2);
}

void MatchConfig::validate() const
{
  const double positive[] = {search_x, search_y, search_yaw_deg, coarse_xy, coarse_yaw_deg, fine_xy,
    fine_yaw_deg, wall_distance, distinct_xy, distinct_yaw_deg};
  for (double v : positive) {
    if (!(std::isfinite(v) && v > 0)) {
      throw std::invalid_argument("탐색 거리·각도·간격은 유한한 양수여야 함");
    }
  }
  for (double v : {min_score, min_improvement, ambiguity_margin}) {
    if (!(std::isfinite(v) && 0 <= v && v <= 1)) {
      throw std::invalid_argument("점수는 0~1이어야 함");
    }
  }
  if (min_points < 3 || max_points < min_points || refine_modes < 2) {
    throw std::invalid_argument("점 개수·재탐색 개수 확인 필요");
  }
}

std::shared_ptr<const MapModel> MapModel::create(
  std::vector<int16_t> grid, int64_t width, int64_t height, double resolution, const Pose & origin)
{
  if (width <= 0 || height <= 0 || static_cast<int64_t>(grid.size()) != width * height ||
    !std::isfinite(resolution) || resolution <= 0)
  {
    throw std::invalid_argument("잘못된 지도 크기/해상도");
  }
  if (!all_finite(origin)) {
    throw std::invalid_argument("잘못된 지도 원점");
  }
  if (std::none_of(grid.begin(), grid.end(), [](int16_t g) {return g > 50;})) {
    throw std::invalid_argument("지도에 점유 벽 없음");
  }
  auto m = std::make_shared<MapModel>();
  m->distance = distance_field(grid, width, height, resolution);
  m->grid = std::move(grid);
  m->width = width;
  m->height = height;
  m->resolution = resolution;
  m->origin = origin;
  m->oc_ = std::cos(origin[2]);
  m->os_ = std::sin(origin[2]);
  return m;
}

bool MapModel::cell(double x, double y, int64_t & u, int64_t & v) const
{
  const double dx = x - origin[0];
  const double dy = y - origin[1];
  const double fu = std::floor((oc_ * dx + os_ * dy) / resolution);
  const double fv = std::floor((-os_ * dx + oc_ * dy) / resolution);
  if (!(fu >= 0.0 && fv >= 0.0 && fu < static_cast<double>(width) && fv < static_cast<double>(height))) {
    return false;
  }
  u = static_cast<int64_t>(fu);
  v = static_cast<int64_t>(fv);
  return true;
}

double MapModel::distance_at(double x, double y) const
{
  int64_t u = 0, v = 0;
  if (!cell(x, y, u, v)) {
    return kInf;
  }
  return static_cast<double>(distance[v * width + u]);
}

bool MapModel::free_center(double x, double y) const
{
  int64_t u = 0, v = 0;
  if (!cell(x, y, u, v)) {
    return false;
  }
  const int16_t g = grid[v * width + u];
  return g >= 0 && g <= 50;
}

void MapModel::score_one(const Points & points, const Pose & q, double wall_distance,
  double & score, double & residual, std::vector<double> & scratch) const
{
  if (!free_center(q[0], q[1])) {
    score = -1.0;
    residual = kInf;
    return;
  }
  const std::size_t n = points.size();
  scratch.resize(n);
  const double c = std::cos(q[2]);
  const double s = std::sin(q[2]);
  std::size_t hits = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const double px = points.x[i];
    const double py = points.y[i];
    const double x = q[0] + c * px - s * py;
    const double y = q[1] + s * px + c * py;
    const double d = distance_at(x, y);
    hits += d <= wall_distance ? 1 : 0;
    scratch[i] = std::min(d, 0.25);
  }
  score = static_cast<double>(hits) / static_cast<double>(n);
  residual = pairwise_sum(scratch.data(), n) / static_cast<double>(n);
}

void MapModel::score_batch(const Points & points, const std::vector<Pose> & poses, double wall_distance,
  std::vector<double> & scores, std::vector<double> & residual) const
{
  scores.resize(poses.size());
  residual.resize(poses.size());
  std::vector<double> scratch;
  scratch.reserve(points.size());
  for (std::size_t i = 0; i < poses.size(); ++i) {
    score_one(points, poses[i], wall_distance, scores[i], residual[i], scratch);
  }
}

double MapModel::score(const Points & points, const Pose & pose, double wall_distance) const
{
  double s = 0.0, r = 0.0;
  std::vector<double> scratch;
  score_one(points, pose, wall_distance, s, r, scratch);
  return s;
}

Points prepare_points(const Points & points, const MatchConfig & cfg)
{
  Points finite;
  finite.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (std::isfinite(points.x[i]) && std::isfinite(points.y[i])) {
      finite.push(points.x[i], points.y[i]);
    }
  }
  const int64_t n = static_cast<int64_t>(finite.size());
  if (n > cfg.max_points) {
    // np.linspace(0, n - 1, max_points).astype(int)
    Points picked;
    picked.reserve(static_cast<std::size_t>(cfg.max_points));
    const double step = static_cast<double>(n - 1) / static_cast<double>(cfg.max_points - 1);
    for (int64_t i = 0; i < cfg.max_points; ++i) {
      const double yi = (i == cfg.max_points - 1) ? static_cast<double>(n - 1) :
        static_cast<double>(i) * step + 0.0;
      const auto k = static_cast<std::size_t>(static_cast<int64_t>(yi));
      picked.push(finite.x[k], finite.y[k]);
    }
    finite = std::move(picked);
  }
  if (static_cast<int64_t>(finite.size()) < cfg.min_points) {
    throw std::invalid_argument("유효한 스캔 점 부족");
  }
  return finite;
}

template<typename RangeT>
Points scan_points(const std::vector<RangeT> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, ScanTrigCache * cache)
{
  if (!std::isfinite(angle_min) || !std::isfinite(angle_increment) || angle_increment == 0) {
    throw std::invalid_argument("잘못된 스캔 각도");
  }
  const std::size_t n = ranges.size();
  ScanTrigCache local;
  ScanTrigCache & t = cache ? *cache : local;
  if (!t.valid || t.n != n || t.angle_min != angle_min || t.angle_increment != angle_increment) {
    t.c.resize(n);
    t.s.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      const double a = angle_min + static_cast<double>(i) * angle_increment;
      t.c[i] = std::cos(a);
      t.s[i] = std::sin(a);
    }
    t.n = n;
    t.angle_min = angle_min;
    t.angle_increment = angle_increment;
    t.valid = true;
  }
  Points out;
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double r = static_cast<double>(ranges[i]);
    if (std::isfinite(r) && r > range_min && r < range_max) {
      out.push(r * t.c[i], r * t.s[i]);
    }
  }
  return out;
}

template Points scan_points<float>(const std::vector<float> &, double, double, double, double, ScanTrigCache *);
template Points scan_points<double>(const std::vector<double> &, double, double, double, double, ScanTrigCache *);

std::vector<double> offsets(double radius, double step)
{
  const auto n = static_cast<int64_t>(std::floor(radius / step + 1e-9));
  std::vector<double> v;
  v.reserve(static_cast<std::size_t>(2 * n + 4));
  for (int64_t k = -n; k <= n; ++k) {
    v.push_back(static_cast<double>(k) * step);
  }
  v.push_back(-radius);
  v.push_back(radius);
  v.push_back(0.0);
  std::sort(v.begin(), v.end());
  v.erase(std::unique(v.begin(), v.end()), v.end());
  return v;
}

std::vector<Pose> mesh_poses(const Pose & center, double xr, double yr, double ar,
  double xy_step, double yaw_step)
{
  const auto ox = offsets(xr, xy_step);
  const auto oy = offsets(yr, xy_step);
  const auto oa = offsets(ar, yaw_step);
  std::vector<Pose> out;
  out.reserve(ox.size() * oy.size() * oa.size());
  for (double x : ox) {
    for (double y : oy) {
      for (double a : oa) {
        out.push_back(Pose{x + center[0], y + center[1], a + center[2]});
      }
    }
  }
  return out;
}

bool distinct(const Pose & a, const Pose & b, const MatchConfig & cfg)
{
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  return std::sqrt(dx * dx + dy * dy) >= cfg.distinct_xy ||
         std::fabs(wrap(a[2] - b[2])) >= radians(cfg.distinct_yaw_deg);
}

MatchResult match(const MapModel & model, const Points & points_in, const Pose & center,
  const Pose & current_pose, const MatchConfig & cfg)
{
  const auto started = std::chrono::steady_clock::now();
  auto elapsed = [&]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    };
  cfg.validate();
  const Points points = prepare_points(points_in, cfg);
  if (!all_finite(center) || !all_finite(current_pose)) {
    throw std::invalid_argument("잘못된 탐색 중심/현재 자세");
  }
  const auto coarse = mesh_poses(center, cfg.search_x, cfg.search_y,
    radians(cfg.search_yaw_deg), cfg.coarse_xy, radians(cfg.coarse_yaw_deg));
  std::vector<double> scores, residual;
  model.score_batch(points, coarse, cfg.wall_distance, scores, residual);
  std::vector<Pose> seeds;
  for (std::size_t i : order_of(scores, residual)) {
    if (scores[i] < 0) {
      break;
    }
    bool all_distinct = true;
    for (const auto & p : seeds) {
      if (!distinct(coarse[i], p, cfg)) {
        all_distinct = false;
        break;
      }
    }
    if (all_distinct) {
      seeds.push_back(coarse[i]);
      if (static_cast<int64_t>(seeds.size()) >= cfg.refine_modes) {
        break;
      }
    }
  }
  MatchResult r;
  if (seeds.empty()) {
    r.reason = "no_free_pose";
    r.score = 0.0;
    r.ambiguous = false;
    r.elapsed_s = elapsed();
    return r;
  }
  const double limits[3] = {cfg.search_x, cfg.search_y, radians(cfg.search_yaw_deg)};
  std::vector<Pose> fine;
  for (const auto & seed : seeds) {
    for (const auto & p : mesh_poses(seed, cfg.coarse_xy, cfg.coarse_xy,
      radians(cfg.coarse_yaw_deg), cfg.fine_xy, radians(cfg.fine_yaw_deg)))
    {
      if (std::fabs(p[0] - center[0]) <= limits[0] + 1e-9 &&
        std::fabs(p[1] - center[1]) <= limits[1] + 1e-9 &&
        std::fabs(p[2] - center[2]) <= limits[2] + 1e-9)
      {
        fine.push_back(p);
      }
    }
  }
  if (fine.empty()) {
    throw std::out_of_range("index 0 is out of bounds for axis 0 with size 0");
  }
  std::vector<double> fs, fr;
  model.score_batch(points, fine, cfg.wall_distance, fs, fr);
  const auto order = order_of(fs, fr);
  const std::size_t best_index = order[0];
  const Pose best = fine[best_index];
  const double score = fs[best_index];
  std::optional<std::size_t> other;
  for (std::size_t i : order) {
    if (distinct(best, fine[i], cfg)) {
      other = i;
      break;
    }
  }
  const double runner = other ? fs[*other] : -1.0;
  const bool ambiguous = runner >= cfg.min_score && score - runner < cfg.ambiguity_margin;
  r.reason = ambiguous ? "ambiguous" : "matched";
  r.pose = best;
  r.score = score;
  r.current_score = model.score(points, current_pose, cfg.wall_distance);
  r.runner_up_score = runner;
  if (other) {
    r.runner_up_pose = fine[*other];
  }
  r.ambiguous = ambiguous;
  r.point_count = static_cast<int64_t>(points.size());
  r.candidate_count = static_cast<int64_t>(coarse.size() + fine.size());
  r.elapsed_s = elapsed();
  return r;
}

Decision decide_publication(const MatchResult & result, const MapModel & model, const Points & points_in,
  const Pose & current_pose, const MatchConfig & cfg)
{
  const Points points = prepare_points(points_in, cfg);
  const double current = model.score(points, current_pose, cfg.wall_distance);
  const double score = result.pose ? model.score(points, *result.pose, cfg.wall_distance) : 0.0;
  Decision d;
  if (current >= cfg.min_score && score - current < cfg.min_improvement) {
    d.reason = "already_aligned"; d.publish = false; d.retry = false;
  } else if (!result.pose || score < cfg.min_score) {
    d.reason = "low_score"; d.publish = false; d.retry = true;
  } else if (result.ambiguous) {
    d.reason = "ambiguous"; d.publish = false; d.retry = true;
  } else if (score - current < cfg.min_improvement) {
    d.reason = "small_improvement"; d.publish = false; d.retry = true;
  } else {
    d.reason = "improved"; d.publish = true; d.retry = false;
  }
  d.score = score;
  d.current_score = current;
  d.improvement = score - current;
  return d;
}

bool pose_changed(const Pose & before, const Pose & after, double xy, double yaw)
{
  return std::hypot(after[0] - before[0], after[1] - before[1]) > xy ||
         std::fabs(wrap(after[2] - before[2])) > yaw;
}

JobOutput run_match_job(const MapData & map, std::shared_ptr<const MapModel> model,
  const Points & points, const Pose & center, const Pose & current_pose, const MatchConfig & cfg)
{
  // 같은 지도 판의 거리장은 다시 쓰고 (값 같음), 없으면 이 스레드에서 만듦
  if (!model) {
    model = MapModel::create(map.grid, map.width, map.height, map.resolution, map.origin);
  }
  JobOutput out;
  out.result = match(*model, points, center, current_pose, cfg);
  out.model = std::move(model);
  return out;
}

}  // namespace pose_match
}  // namespace navigation
