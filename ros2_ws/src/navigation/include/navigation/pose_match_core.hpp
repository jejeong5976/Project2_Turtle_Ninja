// pose_match_core.hpp — 정지 라이다/점유 지도로 초기 위치를 찾는 계산. ROS·모터 명령 없음.
// scripts/pose_match_core.py (numpy/scipy) 와 같은 식·같은 순서의 C++ 판. auto_initial_pose 가 씀.
//   - 거리장 = scipy distance_transform_edt 와 같은 정확한 유클리드 거리 (정수 제곱거리 → sqrt × 해상도 → float32)
//   - 평균 = numpy pairwise 합과 같은 순서 (점수 동점 정렬이 파이썬과 같게)
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace navigation
{
namespace pose_match
{

using Pose = std::array<double, 3>;   // x, y, yaw

// N×2 스캔 점 (열 따로 저장 → 점수 계산이 연속 메모리를 읽음)
struct Points
{
  std::vector<double> x;
  std::vector<double> y;
  std::size_t size() const {return x.size();}
  void clear() {x.clear(); y.clear();}
  void reserve(std::size_t n) {x.reserve(n); y.reserve(n);}
  void push(double px, double py) {x.push_back(px); y.push_back(py);}
};

double wrap(double a);

struct MatchConfig
{
  double search_x = 0.6;
  double search_y = 0.5;
  double search_yaw_deg = 25.0;
  double coarse_xy = 0.04;
  double coarse_yaw_deg = 2.0;
  double fine_xy = 0.01;
  double fine_yaw_deg = 0.5;
  double wall_distance = 0.04;
  double min_score = 0.85;
  double min_improvement = 0.08;
  double ambiguity_margin = 0.03;
  double distinct_xy = 0.12;
  double distinct_yaw_deg = 8.0;
  int64_t max_points = 360;
  int64_t min_points = 30;
  int64_t refine_modes = 8;

  void validate() const;   // std::invalid_argument (파이썬 ValueError 와 같은 문구)
};

class MapModel
{
public:
  // grid = 행 우선 (height 행 × width 열), 값 = OccupancyGrid data (-1..100)
  static std::shared_ptr<const MapModel> create(
    std::vector<int16_t> grid, int64_t width, int64_t height, double resolution, const Pose & origin);

  // 한 자세의 점수 (벽 거리 이하 비율, 중심이 빈칸 아니면 -1) 와 잔차 (min(d, 0.25) 평균, 아니면 inf)
  void score_one(const Points & points, const Pose & pose, double wall_distance,
    double & score, double & residual, std::vector<double> & scratch) const;
  void score_batch(const Points & points, const std::vector<Pose> & poses, double wall_distance,
    std::vector<double> & scores, std::vector<double> & residual) const;
  double score(const Points & points, const Pose & pose, double wall_distance) const;

  // 칸 번호. 지도 밖이면 false
  bool cell(double x, double y, int64_t & u, int64_t & v) const;
  double distance_at(double x, double y) const;   // 지도 밖 = inf
  bool free_center(double x, double y) const;

  std::vector<int16_t> grid;
  std::vector<float> distance;
  int64_t width = 0;
  int64_t height = 0;
  double resolution = 0.0;
  Pose origin{0.0, 0.0, 0.0};

private:
  double oc_ = 1.0;
  double os_ = 0.0;
};

// 유한한 점만, max_points 넘으면 linspace 간격으로 고름, min_points 보다 적으면 std::invalid_argument
Points prepare_points(const Points & points, const MatchConfig & cfg);

// LaserScan → 점 (range_min < r < range_max, 유한값). 같은 각도 배치의 cos/sin 을 다시 쓰는 캐시 (값은 같음)
struct ScanTrigCache
{
  double angle_min = 0.0;
  double angle_increment = 0.0;
  std::size_t n = 0;
  bool valid = false;
  std::vector<double> c;
  std::vector<double> s;
};
template<typename RangeT>
Points scan_points(const std::vector<RangeT> & ranges, double angle_min, double angle_increment,
  double range_min = 0.25, double range_max = 4.0, ScanTrigCache * cache = nullptr);

std::vector<double> offsets(double radius, double step);
std::vector<Pose> mesh_poses(const Pose & center, double xr, double yr, double ar,
  double xy_step, double yaw_step);
bool distinct(const Pose & a, const Pose & b, const MatchConfig & cfg);

struct MatchResult
{
  std::string reason;               // matched / ambiguous / no_free_pose
  std::optional<Pose> pose;
  double score = 0.0;
  std::optional<double> current_score;
  double runner_up_score = -1.0;
  std::optional<Pose> runner_up_pose;
  bool ambiguous = false;
  int64_t point_count = 0;
  int64_t candidate_count = 0;
  double elapsed_s = 0.0;
};

// 위치 후보만 반환함. 발행 판단은 최신 스캔/TF로 다시 계산함.
MatchResult match(const MapModel & model, const Points & points, const Pose & center,
  const Pose & current_pose, const MatchConfig & cfg = MatchConfig());

struct Decision
{
  std::string reason;
  bool publish = false;
  bool retry = false;
  double score = 0.0;
  double current_score = 0.0;
  double improvement = 0.0;
};

// 현재 시점의 스캔/TF와 비교. 불확실하면 위치를 쓰지 않고 다음 정지 때 재탐색함.
Decision decide_publication(const MatchResult & result, const MapModel & model, const Points & points,
  const Pose & current_pose, const MatchConfig & cfg = MatchConfig());

bool pose_changed(const Pose & before, const Pose & after, double xy = 0.01, double yaw = 0.02);

// 실행기와 다른 스레드에서 부름. model 이 없으면 (지도 바뀜) 새로 만듦
struct MapData
{
  std::vector<int16_t> grid;
  int64_t width = 0;
  int64_t height = 0;
  double resolution = 0.0;
  Pose origin{0.0, 0.0, 0.0};
  std::string frame;
};
struct JobOutput
{
  MatchResult result;
  std::shared_ptr<const MapModel> model;
};
JobOutput run_match_job(const MapData & map, std::shared_ptr<const MapModel> model,
  const Points & points, const Pose & center, const Pose & current_pose, const MatchConfig & cfg);

// numpy pairwise 합 (np.add.reduce 와 같은 더하기 순서)
double pairwise_sum(const double * a, std::size_t n);

}  // namespace pose_match
}  // namespace navigation
