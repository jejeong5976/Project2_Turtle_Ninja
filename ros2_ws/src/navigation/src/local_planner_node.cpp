// local_planner_node.cpp — 학습한 지역 계획기 정책 (RL) 을 ROS 로 돌리는 노드. nav2.launch.py local_planner:=rl 이 둘 띄움
//   rl_nav = 주행 (navigate_to_pose, nf5), rl_dock = 정밀 접근 (dock_to_pose, dock3)
//
// 받는 것
//   - 액션 <action_name> (nav2_msgs/NavigateToPose 형): 목표
//   - /odom (몸 속도), /joint_states (조향 위치 · 바퀴 속도), /scan (scan_merger 360°, 몸 상자 안 점 버림)
//   - TF map → base_footprint (goal_frame odom 이면 odom → base_footprint + 목표 받을 때 한 번 map → odom)
//   - /collision_monitor_state (near_disable_polygons 또는 stop_abort_s 일 때)
// 부르는 것
//   - compute_path_to_pose (Nav2 planner_server, 1 s 마다 따로 스레드에서 다시)
//   - <cm_node>/set_parameters (collision_monitor 영역 끄기 · 켜기)
// 보내는 것
//   - cmd_vel_nav (TwistStamped, 20 Hz) → collision_monitor → /cmd_vel
// 실행
//   - 명령줄 인자는 파이썬 판 (scripts/local_planner_node.py) 과 같음, 예:
//     local_planner_node --node_name rl_dock --policy dock3.npz --cfg dock3.json --swerve_yaml team_swerve.yaml
//       --action_name dock_to_pose --tol_pos 0.015 --tol_yaw_deg 2 --disable_polygons PolygonStop,FootprintApproach
//       --goal_frame odom --box_track
// note
//   - 관측 · 명령 · 박스 맞춤 계산은 rl_core (파이썬 판 · 학습 env 와 같은 식). sim (Isaac) 은 파이썬 판을 그대로 씀
//   - 정책은 npz 만 (torch 판 · 그림자 모드는 파이썬 판에만 있음)
//   - 성공 = 도착 범위 안에서 goal_hold_s 멈춤. 범위 2 배 안 near_ok_s 머물러도 성공
//   - 목표 수락부터 복구와 결과 전송까지 한 목표만 실행함. 실행 · 재계획 스레드는 목표 종료 때 회수함
//   - 안전 영역 복구 실패 = 정지 · 새 목표 거절, 1 s마다 복구 재시도함
//   - SIGINT · SIGTERM = 정지 · 영역 복구 · 스레드 회수 뒤 ROS context를 종료함
//   - 목표 좌표계 odom = 목표를 받는 순간 map → odom 으로 한 번 바꿔 고정 (접근 중 slam_toolbox 보정에 안 흔들림)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <csignal>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "navigation/rl_core.hpp"

namespace navigation
{

using namespace std::chrono_literals;
namespace rl = navigation::rl;
using rl::Pose2;
using rl::Vec2;
using Nav = nav2_msgs::action::NavigateToPose;
using NavGoal = rclcpp_action::ServerGoalHandle<Nav>;
using Plan = nav2_msgs::action::ComputePathToPose;
using CmState = nav2_msgs::msg::CollisionMonitorState;
using SetParameters = rcl_interfaces::srv::SetParameters;

// ================================================================ 명령줄 인자 (파이썬 판 argparse 와 같은 이름 · 기본값)

struct Args
{
  std::string node_name = "rl_local_planner", policy, cfg, swerve_yaml, scan_topic = "/scan";
  double tol_pos = 0.02, tol_yaw_deg = 2.0, scan_stale_s = 0.5;
  bool sim_time = false, box_track = false, no_settle = false;
  std::string disable_polygons, cm_node = "/collision_monitor", near_disable_polygons, cm_state_topic = "/collision_monitor_state";
  double near_leave_m = 0.25, near_clear_m = 0.05, near_clear_pad = 0.03, near_vmax = 0.10, stop_half = 0.18, stop_abort_s = 8.0;
  std::string dock_body_half = "0.145,0.153";
  int dock_block_n = 3, box_reject_n = 10;
  double goal_back_m = 0.45, straight_max = 0.6, near_ok_s = 3.0, min_cmd_lin = 0.0, min_cmd_ang = 0.0, escape_m = 0.20;
  std::string goal_frame = "map", action_name = "navigate_to_pose", planner_id;
  double box_dist = 0.2575;
};

// argv 중 --ros-args 앞만 읽음 (launch Node 가 뒤에 붙임). 모르는 인자 · 값 없는 인자는 오류
Args parse_args(int argc, char ** argv)
{
  Args a;
  std::map<std::string, std::string *> str{
    {"--node_name", &a.node_name}, {"--policy", &a.policy}, {"--cfg", &a.cfg}, {"--swerve_yaml", &a.swerve_yaml},
    {"--scan_topic", &a.scan_topic}, {"--disable_polygons", &a.disable_polygons}, {"--cm_node", &a.cm_node},
    {"--near_disable_polygons", &a.near_disable_polygons}, {"--cm_state_topic", &a.cm_state_topic},
    {"--dock_body_half", &a.dock_body_half}, {"--goal_frame", &a.goal_frame}, {"--action_name", &a.action_name},
    {"--planner_id", &a.planner_id}};
  std::map<std::string, double *> num{
    {"--tol_pos", &a.tol_pos}, {"--tol_yaw_deg", &a.tol_yaw_deg}, {"--scan_stale_s", &a.scan_stale_s},
    {"--near_leave_m", &a.near_leave_m}, {"--near_clear_m", &a.near_clear_m}, {"--near_clear_pad", &a.near_clear_pad},
    {"--near_vmax", &a.near_vmax}, {"--stop_half", &a.stop_half}, {"--stop_abort_s", &a.stop_abort_s},
    {"--goal_back_m", &a.goal_back_m}, {"--straight_max", &a.straight_max}, {"--near_ok_s", &a.near_ok_s},
    {"--min_cmd_lin", &a.min_cmd_lin}, {"--min_cmd_ang", &a.min_cmd_ang}, {"--escape_m", &a.escape_m},
    {"--box_dist", &a.box_dist}};
  std::map<std::string, int *> integer{{"--dock_block_n", &a.dock_block_n}, {"--box_reject_n", &a.box_reject_n}};
  std::map<std::string, bool *> flag{{"--sim_time", &a.sim_time}, {"--box_track", &a.box_track}, {"--no_settle", &a.no_settle}};
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    if (k == "--ros-args") {break;}
    if (k == "--shadow") {throw std::runtime_error("--shadow (그림자 모드) 는 파이썬 판 scripts/local_planner_node.py 에만 있음");}
    if (auto f = flag.find(k); f != flag.end()) {*f->second = true; continue;}
    if (i + 1 >= argc) {throw std::runtime_error("값 없음: " + k);}
    const std::string v = argv[++i];
    if (auto s = str.find(k); s != str.end()) {*s->second = v;} else if (auto n = num.find(k); n != num.end()) {
      *n->second = std::stod(v);
    } else if (auto m = integer.find(k); m != integer.end()) {
      *m->second = std::stoi(v);
    } else {
      throw std::runtime_error("모르는 인자: " + k);
    }
  }
  if (a.policy.empty() || a.cfg.empty()) {throw std::runtime_error("--policy · --cfg 필요");}
  if (a.policy.size() < 4 || a.policy.substr(a.policy.size() - 4) != ".npz") {
    throw std::runtime_error("정책은 .npz 만 (torch .pt 는 파이썬 판): " + a.policy);
  }
  if (a.goal_frame != "map" && a.goal_frame != "odom") {throw std::runtime_error("--goal_frame 은 map | odom");}
  return a;
}

std::vector<std::string> split(const std::string & s, char sep)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  for (std::string t; std::getline(ss, t, sep);) {if (!t.empty()) {out.push_back(t);}}
  return out;
}

std::string fmt(const char * f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char * f, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

double deg(double r) {return r * 180.0 / M_PI;}

// ================================================================ 노드

class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const Args & a)
  : Node(a.node_name, rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", a.sim_time)})),
    a_(a), cfg_(rl::load_env_cfg(a.cfg)), sw_(rl::load_swerve(a.swerve_yaml.empty() ? cfg_.swerve_yaml : a.swerve_yaml)),
    policy_(a.policy)
  {
    // 정책 입력 크기 = 관측 크기 (경로 2N + 목표 5 + 라이다 + 이력 22 × history)
    const int dim = 2 * cfg_.path_obs_points + 5 + cfg_.lidar_rays + 22 * cfg_.history;
    if (policy_.input_dim() != dim) {
      throw std::runtime_error(fmt("정책 입력 %d ≠ cfg 관측 %d (정책 · cfg 짝 확인)", policy_.input_dim(), dim));
    }
    tol_p_ = a.tol_pos;
    tol_y_ = a.tol_yaw_deg * M_PI / 180.0;
    // 정밀 접근 (정차 0.2575 = 차체 앞 끝 ↔ 박스 면 1 cm) 동안 collision_monitor 영역 끄기: PolygonStop ±0.18 · FootprintApproach 가
    //   박스에 걸려 마지막 몇 cm 를 못 감. 목표마다 끄고 끝나면 (성공 · 실패 · 취소) 다시 켬
    cm_polygons_ = split(a.disable_polygons, ',');
    // 정밀 접근 정차 (박스 면 1–2 cm) 에서 출발할 때: PolygonStop 이 박스에 걸려 0 속도 → 그때만 영역 끄고 출발 자리에서 벗어나면 다시 켬
    //   끄는 조건 = 빠져나오기 경로를 씀 또는 PolygonStop 이 지금 막음 (collision_monitor_state STOP · /scan 점이 PolygonStop + 1 cm 안)
    //   ('스캔 최소 < 0.30' 같은 거리 조건은 출발 구역 벽에도 걸려 영역 끈 채 출발함)
    //   켜는 조건 = near_leave_m 멀어짐, 또는 near_clear_m 움직였고 PolygonStop + near_clear_pad 안에 점 없음
    //   끈 동안 병진 near_vmax 상한 (몸 상자 안은 /scan 에 안 보여 벽에 붙어도 모름)
    near_polygons_ = split(a.near_disable_polygons, ',');
    const auto bh = split(a.dock_body_half, ',');
    if (bh.size() != 2) {throw std::runtime_error("--dock_body_half 는 x,y");}
    dock_body_ = {std::stod(bh[0]), std::stod(bh[1])};

    cb_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions so;
    so.callback_group = cb_;
    if (!cm_polygons_.empty() || !near_polygons_.empty()) {
      cm_client_ = create_client<SetParameters>(a.cm_node + "/set_parameters", rclcpp::ServicesQoS(), cb_);
    }
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          std::lock_guard lk(in_mtx_);
          j_.odom = {m->twist.twist.linear.x, m->twist.twist.linear.y, m->twist.twist.angular.z};
        }, so);
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {on_js(*m);}, so);
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(a.scan_topic, rclcpp::QoS(5).best_effort(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
          const double t = now_s();
          std::lock_guard lk(in_mtx_);
          scan_ = m;
          scan_t_ = t;
        }, so);
    if (!near_polygons_.empty() || a.stop_abort_s > 0) {
      sub_cm_ = create_subscription<CmState>(a.cm_state_topic, 10, [this](CmState::ConstSharedPtr m) {
            const double t = now_s();
            std::lock_guard lk(in_mtx_);
            if (m->action_type != CmState::STOP) {
              stop_since_.reset();
            } else if (!stop_since_) {
              stop_since_ = t;
            }
            cm_state_ = m;
          }, so);
    }
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_nav", 10);
    planner_ = rclcpp_action::create_client<Plan>(this, "compute_path_to_pose", cb_);
    server_ = rclcpp_action::create_server<Nav>(this, a.action_name,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Nav::Goal>) {
          std::lock_guard lk(goal_mtx_);
          if (stopping_ || busy_ || zones_off_) {return rclcpp_action::GoalResponse::REJECT;}
          busy_ = true;
          plan_stop_ = false;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<NavGoal>) {
          plan_stop_ = true;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<NavGoal> gh) {
          std::lock_guard lk(worker_mtx_);
          if (stopping_) {gh->abort(std::make_shared<Nav::Result>()); return;}
          if (worker_.joinable()) {worker_.join();}
          worker_ = std::thread([this, gh] {execute(gh);});
        },
        rcl_action_server_get_default_options(), cb_);
    restore_timer_ = create_wall_timer(1s, [this] {retry_restore();}, cb_);
    RCLCPP_INFO(get_logger(), "정책 %s 불러옴, 주행 (%s → cmd_vel_nav), 한계 [%.3f, %.3f, %.3f], 성공 %.0f cm · %.1f°",
      a.policy.c_str(), a.action_name.c_str(), cfg_.vmax, cfg_.vmax, cfg_.wmax, tol_p_ * 100, a.tol_yaw_deg);
  }

  ~LocalPlannerNode() override {shutdown_worker();}

  // executor 종료 뒤 멤버 해제 전에 작업 스레드를 회수함
  void shutdown_worker()
  {
    stopping_ = true;
    plan_stop_ = true;
    if (restore_timer_) {restore_timer_->cancel();}
    std::lock_guard lk(worker_mtx_);
    if (worker_.joinable()) {worker_.join();}
  }

private:
  bool interrupted() const {return stopping_ || plan_stop_ || !rclcpp::ok();}

  // 취소와 종료 중에는 planner 응답을 무한정 기다리지 않음
  template<typename Future>
  bool wait_plan(Future & f)
  {
    const auto until = std::chrono::steady_clock::now() + 3s;
    while (!interrupted() && std::chrono::steady_clock::now() < until) {
      if (f.wait_for(20ms) == std::future_status::ready) {return true;}
    }
    return false;
  }

  // ------------------------------------------------------------ 입력
  double now_s() {return get_clock()->now().seconds();}

  void on_js(const sensor_msgs::msg::JointState & m)
  {
    std::map<std::string, size_t> idx;
    for (size_t i = 0; i < m.name.size(); ++i) {idx[m.name[i]] = i;}
    for (const auto & n : sw_.steer) {if (!idx.count(n) || idx[n] >= m.position.size()) {return;}}
    for (const auto & n : sw_.drive) {if (!idx.count(n)) {return;}}
    std::array<double, 4> q{}, w{};
    for (int i = 0; i < 4; ++i) {
      q[i] = m.position[idx[sw_.steer[i]]];
      const size_t k = idx[sw_.drive[i]];
      w[i] = k < m.velocity.size() ? m.velocity[k] : 0.0;
    }
    std::lock_guard lk(in_mtx_);
    j_.q = q;
    j_.w = w;
  }

  rl::JointObs joints()
  {
    std::lock_guard lk(in_mtx_);
    return j_;
  }

  std::pair<sensor_msgs::msg::LaserScan::ConstSharedPtr, double> scan()
  {
    std::lock_guard lk(in_mtx_);
    return {scan_, scan_t_};
  }

  Pose2 tf2d(const std::string & target, const std::string & source, const tf2::TimePoint & t = tf2::TimePointZero)
  {
    const auto tr = tf_->lookupTransform(target, source, t);   // 없으면 tf2::TransformException
    return {tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation)};
  }

  Pose2 pose(const std::string & frame) {return tf2d(frame, "base_footprint");}

  static std::vector<Vec2> polar(const sensor_msgs::msg::LaserScan & m)
  {
    std::vector<Vec2> pts;
    pts.reserve(m.ranges.size());
    for (size_t i = 0; i < m.ranges.size(); ++i) {
      const double r = m.ranges[i];
      if (!std::isfinite(r) || r < m.range_min || r > m.range_max) {continue;}
      const double a = m.angle_min + static_cast<double>(i) * m.angle_increment;
      pts.push_back({r * std::cos(a), r * std::sin(a)});
    }
    return pts;
  }

  static bool body_frame(const std::string & f) {return f.empty() || f == "base_footprint";}

  // /scan → frame xy (촬영 시각 TF, 없으면 최신). (점, 스캔 시각) 또는 nullopt
  std::optional<std::pair<std::vector<Vec2>, double>> scan_points(const std::string & frame)
  {
    const auto [m, t] = scan();
    if (!m) {return std::nullopt;}
    const std::string src = m->header.frame_id.empty() ? "base_footprint" : m->header.frame_id;
    Pose2 T;
    try {
      T = tf2d(frame, src, tf2::TimePoint(std::chrono::nanoseconds(rclcpp::Time(m->header.stamp).nanoseconds())));
    } catch (const tf2::TransformException &) {
      try {
        T = tf2d(frame, src);
      } catch (const tf2::TransformException &) {
        return std::nullopt;
      }
    }
    auto pts = polar(*m);
    for (auto & p : pts) {p = rl::apply(T, p);}
    return std::make_pair(std::move(pts), t);
  }

  // /scan 점 → base_footprint xy (몸 상자 안 점은 scan_merger 가 이미 버림). 없거나 오래되면 nullopt
  std::optional<std::vector<Vec2>> body_points()
  {
    const auto [m, t] = scan();
    if (!m || now_s() - t > a_.scan_stale_s) {return std::nullopt;}
    if (!body_frame(m->header.frame_id)) {
      auto sp = scan_points("base_footprint");
      if (!sp) {return std::nullopt;}
      return std::move(sp->first);
    }
    return polar(*m);
  }

  // 몸 기준 10° 칸 최솟값 (학습 env 라이다와 같은 칸 · 같은 상자). 스캔 없거나 오래되면 nullopt
  std::optional<std::vector<double>> lidar()
  {
    const auto [m, t] = scan();
    if (!m || now_s() - t > a_.scan_stale_s) {return std::nullopt;}
    auto pts = polar(*m);
    if (!body_frame(m->header.frame_id)) {             // scan_merger frame 을 바꿨을 때만
      Pose2 T;
      try {
        T = tf2d("base_footprint", m->header.frame_id);
      } catch (const tf2::TransformException &) {
        return std::nullopt;
      }
      for (auto & p : pts) {p = rl::apply(T, p);}
    }
    return rl::lidar_bins(pts, cfg_);
  }

  static bool in_square(const std::vector<Vec2> & pts, double h)
  {
    return std::any_of(pts.begin(), pts.end(), [h](const Vec2 & p) {return std::abs(p.x) <= h && std::abs(p.y) <= h;});
  }

  // PolygonStop 이 지금 막고 있는지: collision_monitor 상태 (STOP · 그 영역) 또는 /scan 점이 PolygonStop + 1 cm 안
  bool stop_blocking()
  {
    {
      std::lock_guard lk(in_mtx_);
      if (cm_state_ && cm_state_->action_type == CmState::STOP &&
        std::find(near_polygons_.begin(), near_polygons_.end(), cm_state_->polygon_name) != near_polygons_.end())
      {
        return true;
      }
    }
    const auto pts = body_points();
    return pts && in_square(*pts, a_.stop_half + 0.01);
  }

  bool stop_zone_clear()
  {
    const auto pts = body_points();
    return pts && !in_square(*pts, a_.stop_half + a_.near_clear_pad);
  }

  // ------------------------------------------------------------ 경로
  geometry_msgs::msg::PoseStamped stamped(const std::string & frame, double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame;
    ps.header.stamp = get_clock()->now();
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.orientation.z = std::sin(yaw / 2);
    ps.pose.orientation.w = std::cos(yaw / 2);
    return ps;
  }

  std::optional<rl::RlPath> plan_from(const geometry_msgs::msg::PoseStamped & goal,
    const std::optional<geometry_msgs::msg::PoseStamped> & start)
  {
    if (interrupted()) {return std::nullopt;}
    const auto until = std::chrono::steady_clock::now() + 2s;
    while (!interrupted() && !planner_->wait_for_action_server(20ms)) {
      if (std::chrono::steady_clock::now() >= until) {return std::nullopt;}
    }
    if (interrupted()) {return std::nullopt;}
    Plan::Goal g;
    g.goal = goal;
    g.planner_id = a_.planner_id;
    g.use_start = start.has_value();
    if (start) {g.start = *start;}
    auto fh = planner_->async_send_goal(g);
    if (!wait_plan(fh)) {return std::nullopt;}
    const auto h = fh.get();
    if (!h) {return std::nullopt;}
    auto fr = planner_->async_get_result(h);
    if (!wait_plan(fr)) {planner_->async_cancel_goal(h); return std::nullopt;}
    const auto r = fr.get();
    if (!r.result || r.result->path.poses.size() < 2) {return std::nullopt;}
    std::vector<Vec2> pts;
    for (const auto & p : r.result->path.poses) {pts.push_back({p.pose.position.x, p.pose.position.y});}
    pts.push_back({goal.pose.position.x, goal.pose.position.y});   // 끝 = 정확한 목표 (학습과 같음)
    return rl::RlPath(std::move(pts));
  }

  // 목표 그대로 계획 → 실패하면 목표를 보는 방향 반대로 0.05 m 씩 goal_back_m 까지 물려 계획 + 끝에 목표 점 (직선)
  //   목표가 지도 장애물 (박스를 지도에 넣고 매핑) 일 때. 정한 물림 거리는 이 목표 동안 먼저 쓰고, 안 되면 더 먼 쪽만 봄
  //   (실물 L3: 1 s 마다 35 ↔ 45 cm 로 바뀌어 직선 끝점이 10 cm 씩 앞뒤로 움직임)
  std::optional<rl::RlPath> plan_goal(const geometry_msgs::msg::PoseStamped & goal,
    const std::optional<geometry_msgs::msg::PoseStamped> & start)
  {
    if (interrupted()) {return std::nullopt;}
    std::optional<double> gb;
    {
      std::lock_guard lk(plan_mtx_);
      gb = goal_back_;
    }
    if (!gb) {
      auto p = plan_from(goal, start);
      if (p || a_.goal_back_m <= 0) {return p;}
    }
    const double gyaw = yaw_of(goal.pose.orientation);
    for (int k = 1; !interrupted() && 0.05 * k <= a_.goal_back_m + 1e-6; ++k) {
      const double b = 0.05 * k;
      if (gb && b < *gb - 1e-6) {continue;}
      auto ps = goal;
      ps.pose.position.x = goal.pose.position.x - b * std::cos(gyaw);
      ps.pose.position.y = goal.pose.position.y - b * std::sin(gyaw);
      auto p = plan_from(ps, start);
      if (p) {
        if (!gb || std::abs(b - *gb) > 1e-6) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "목표가 지도 장애물 → 목표 뒤 %.0f cm 까지 계획 + 직선", b * 100);
        }
        {
          std::lock_guard lk(plan_mtx_);
          goal_back_ = b;
        }
        auto pts = p->points();
        pts.push_back({goal.pose.position.x, goal.pose.position.y});
        return rl::RlPath(std::move(pts));
      }
    }
    if (gb) {                                           // 물린 자리도 막힘 → 다음엔 처음부터
      {
        std::lock_guard lk(plan_mtx_);
        goal_back_.reset();
      }
      return plan_from(goal, start);
    }
    return std::nullopt;
  }

  // planner_server 경로 (map). 실패하면 빠져나오기: 정밀 접근 정차 (박스 면 1 cm) 는 costmap 치명 영역 안이라 지금 자리에서는
  //   경로가 안 나옴 → 몸 뒤 escape_m 이내 점에서 다시 계획해 '지금 자리 → 뒤 점 직선 + 그 경로' 로 씀
  std::optional<rl::RlPath> plan(const geometry_msgs::msg::PoseStamped & goal)
  {
    auto path = plan_goal(goal, std::nullopt);
    if (path || a_.escape_m <= 0) {return path;}
    Pose2 p;
    try {
      p = pose("map");
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
    for (int k = 1; !interrupted() && 0.05 * k <= a_.escape_m + 1e-6; ++k) {
      const double d = 0.05 * k;
      const auto st = stamped("map", p.x - d * std::cos(p.yaw), p.y - d * std::sin(p.yaw), p.yaw);
      path = plan_goal(goal, st);
      if (path) {
        escaped_ = true;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "지금 자리에서 경로 안 나옴 → 뒤 %.0f cm 로 빠져나와 감", d * 100);
        std::vector<Vec2> pts{{p.x, p.y}};
        pts.insert(pts.end(), path->points().begin(), path->points().end());
        return rl::RlPath(std::move(pts));
      }
    }
    return std::nullopt;
  }

  // goal_frame 기준 경로. odom = 고정한 odom 목표를 지금 map → odom 으로 map 목표로 바꿔 planner 에 보내고 받은 경로를 다시 odom 으로
  std::optional<rl::RlPath> plan_frame(const geometry_msgs::msg::PoseStamped & goal_ps, const Pose2 & goal)
  {
    if (a_.goal_frame == "map") {return plan(goal_ps);}
    Pose2 T_mo;
    try {
      T_mo = tf2d("map", "odom");
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
    const Pose2 gm = rl::compose(T_mo, goal);
    auto path = plan(stamped("map", gm.x, gm.y, gm.yaw));
    if (!path) {return std::nullopt;}
    const Pose2 T_om = rl::inverse(T_mo);
    std::vector<Vec2> pts;
    for (const auto & p : path->points()) {pts.push_back(rl::apply(T_om, p));}
    pts.back() = {goal.x, goal.y};
    return rl::RlPath(std::move(pts));
  }

  // planner 실패 때 대체: 목표가 straight_max 안이면 지금 자리 → 목표 직선 (박스가 밀려 고친 목표가 옛 박스 치명 영역일 때)
  std::optional<rl::RlPath> straight(const Pose2 & goal)
  {
    Pose2 p;
    try {
      p = pose(a_.goal_frame);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
    const double d = std::hypot(goal.x - p.x, goal.y - p.y);
    if (d > a_.straight_max || d < 1e-3) {return std::nullopt;}
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "경로 못 받음 → 목표 %.0f cm 직선 경로로 대신", d * 100);
    return rl::RlPath({{p.x, p.y}, {goal.x, goal.y}});
  }

  // collision_monitor 의 <영역>.enabled 를 바꿈 (실행 중 바꾸기 지원). 실패하면 경고만
  bool set_polygons(bool enabled, const std::vector<std::string> & names)
  {
    if (names.empty()) {return true;}
    if (!cm_client_ || !cm_client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "collision_monitor set_parameters 없음 → 영역 못 바꿈");
      return false;
    }
    auto req = std::make_shared<SetParameters::Request>();
    std::string joined;
    for (const auto & n : names) {
      req->parameters.push_back(rclcpp::Parameter(n + ".enabled", enabled).to_parameter_msg());
      joined += (joined.empty() ? "" : ",") + n;
    }
    auto fut = cm_client_->async_send_request(req);
    bool ok = fut.wait_for(2s) == std::future_status::ready;
    if (ok) {
      const auto res = fut.get();
      ok = res && std::all_of(res->results.begin(), res->results.end(), [](const auto & r) {return r.successful;});
    }
    RCLCPP_INFO(get_logger(), "collision_monitor %s %s%s", joined.c_str(), enabled ? "켬" : "끔", ok ? "" : " (실패)");
    return ok;
  }

  // ------------------------------------------------------------ 보내기
  void send(const std::array<double, 3> & cmd)
  {
    // 명령 사이 틈 기록: swerve_controller cmd_timeout_s 0.3 s 를 넘기면 바퀴가 섬
    const auto t = std::chrono::steady_clock::now();
    const bool moving = std::hypot(cmd[0], cmd[1], cmd[2]) > 1e-6;
    if (last_send_) {
      const double gap = std::chrono::duration<double>(t - *last_send_).count();
      if (gap > 0.25 && gap < 5.0 && moving) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "명령 틈 %.2f s (0.3 s 넘으면 swerve_controller 가 멈춤)", gap);
      }
    }
    last_send_ = t;
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = get_clock()->now();
    m.header.frame_id = "base_footprint";
    m.twist.linear.x = cmd[0];
    m.twist.linear.y = cmd[1];
    m.twist.angular.z = cmd[2];
    pub_->publish(m);
  }

  // 관측 → 몸 속도 명령 (학습 env _command 와 같은 한계)
  std::array<double, 3> act(const std::vector<float> & o, double goal_dist)
  {
    const auto sc = lidar();
    std::optional<double> smin;
    if (sc) {smin = *std::min_element(sc->begin(), sc->end());}
    return rl::command(cfg_, policy_(o), goal_dist, near_off_ && a_.near_vmax > 0 ? a_.near_vmax : 0.0, smin);
  }

  // ------------------------------------------------------------ 실행
  struct Rot                                           // 목표 하나 회전 기록
  {
    double yaw0 = 0, need = 0, odom = 0, last_wz = 0;
    int rev = 0;
    std::string fr;
  };

  void log_rotation()
  {
    if (!rot_) {return;}
    double net = NAN;
    try {
      net = std::abs(rl::wrap(pose(rot_->fr).yaw - rot_->yaw0));
    } catch (const tf2::TransformException &) {
    }
    RCLCPP_INFO(get_logger(), "회전: odom ∫|ω| %.0f° · 필요 %.0f° · 실제 순 %.0f° · 남는 회전 %.0f° · 명령 반전 %d 번",
      deg(rot_->odom), deg(rot_->need), deg(net), deg(std::max(rot_->odom - rot_->need, 0.0)), rot_->rev);
    rot_.reset();
  }

  // 목표 하나. disable_polygons 면 이 목표 동안만 영역을 끔 (끝나면 반드시 다시 켬)
  //   주행 노드 (near_disable_polygons): 출발 전에 그 영역을 켬으로 확인 → 못 켜면 주행 안 하고 실패
  //   (정밀 접근 노드가 영역을 끈 뒤 다시 못 켰으면 비상 정지 없이 달리게 되므로)
  enum class End {Abort, Succeed, Cancel};

  void stop() noexcept
  {
    try {send({0, 0, 0});} catch (const std::exception &) {}  // ROS 종료 뒤 발행 실패 허용함
  }

  // 모든 영역 복구 성공 전에는 새 목표와 속도 제한 해제를 허용하지 않음
  bool restore_zones()
  {
    try {
      const bool dock_ok = set_polygons(true, cm_polygons_);
      const bool near_ok = set_polygons(true, near_polygons_);
      if (near_ok) {near_off_ = false;}
      return dock_ok && near_ok;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "영역 복구 예외: %s", e.what());
      return false;
    }
  }

  void retry_restore()
  {
    {
      std::lock_guard lk(goal_mtx_);
      if (stopping_ || busy_ || !zones_off_) {return;}
      busy_ = true;
    }
    if (restore_zones()) {zones_off_ = false;}
    std::lock_guard lk(goal_mtx_);
    busy_ = false;
  }

  void execute(std::shared_ptr<NavGoal> gh)
  {
    end_ = End::Abort;
    try {
      if (cm_polygons_.empty() && !set_polygons(true, near_polygons_)) {
        RCLCPP_ERROR(get_logger(), "영역 켬을 확인 못 함 → 주행 안 함");
      } else if (!interrupted()) {
        if (!cm_polygons_.empty() && !set_polygons(false, cm_polygons_)) {
          RCLCPP_WARN(get_logger(), "영역을 못 꺼서 그대로 진행");
        }
        rot_.reset();
        if (!interrupted()) {run(gh);}
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "목표 실행 중 오류 → 실패: %s", e.what());
      end_ = End::Abort;
    }
    stop();
    plan_stop_ = true;
    if (replan_.joinable()) {replan_.join();}           // 이전 목표가 다음 목표 상태를 바꾸지 못하게 함
    try {log_rotation();} catch (const std::exception &) {}
    bool restored = false;
    for (int i = 0; i < 3 && !restored; ++i) {restored = restore_zones();}
    zones_off_ = !restored;
    if (!restored) {
      end_ = End::Abort;
      RCLCPP_ERROR(get_logger(), "영역 복구 실패 → 정지 · 새 목표 거절 · 주기적으로 복구함");
    }
    // 결과 직후 도착한 다음 목표가 busy 상태를 잘못 보지 않게 함
    std::lock_guard lk(goal_mtx_);
    // 복구와 스레드 회수가 끝난 뒤에만 최종 결과를 보냄
    try {
      auto result = std::make_shared<Nav::Result>();
      if (gh->is_canceling()) {gh->canceled(result);}
      else if (end_ == End::Succeed) {gh->succeed(result);}
      else {gh->abort(result);}
    } catch (const std::exception &) {}
    busy_ = false;
  }

  void finish(const std::shared_ptr<NavGoal> &, bool ok)
  {
    stop();
    end_ = ok ? End::Succeed : End::Abort;              // 최종 결과는 복구 뒤 execute에서 보냄
  }

  void run(const std::shared_ptr<NavGoal> & gh)
  {
    const auto goal_ps = gh->get_goal()->pose;
    Pose2 goal{goal_ps.pose.position.x, goal_ps.pose.position.y, yaw_of(goal_ps.pose.orientation)};
    escaped_ = false;
    {
      std::lock_guard lk(plan_mtx_);
      goal_back_.reset();
    }
    const std::string fr = a_.goal_frame;
    if (fr == "odom") {                                // 목표를 지금 map → odom 으로 한 번 바꿔 고정
      const std::string src = goal_ps.header.frame_id.empty() ? "map" : goal_ps.header.frame_id;
      try {
        goal = rl::compose(tf2d("odom", src), goal);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(get_logger(), "%s → odom 변환 없음 (%s)", src.c_str(), e.what());
        end_ = End::Abort;
        return;
      }
      RCLCPP_INFO(get_logger(), "목표 odom 고정 (%.3f, %.3f, %.1f°)", goal.x, goal.y, deg(goal.yaw));
    }
    std::optional<rl::BoxTracker> tracker;
    double scan_seen = -1.0, start_dist = 0.0;
    int n_block = 0, n_off = 0;
    std::optional<rl::RlPath> path;
    if (a_.box_track) {                                // 박스 맞춤: 경로 = 직선 (학습과 같음), 목표는 스캔마다 고침
      tracker.emplace(Vec2{goal.x, goal.y}, goal.yaw, a_.box_dist);
      const Pose2 p = pose(fr);
      path.emplace(std::vector<Vec2>{{p.x, p.y}, {goal.x, goal.y}});
      start_dist = std::hypot(goal.x - p.x, goal.y - p.y);
    } else {
      path = plan_frame(goal_ps, goal);
    }
    if (!path) {path = straight(goal);}
    if (!path) {
      RCLCPP_WARN(get_logger(), "경로 못 받음");
      end_ = End::Abort;
      return;
    }
    std::vector<std::vector<double>> hist;
    std::array<double, 3> last_cmd{0, 0, 0};
    double hold = 0.0, near_t = 0.0;
    struct PlanJob
    {
      std::atomic<bool> done{false};
      std::optional<rl::RlPath> path;
      double dt = 0.0;
    };
    std::shared_ptr<PlanJob> job;                      // 따로 도는 경로 계획 (1 s 마다)
    bool near_checked = near_polygons_.empty(), settled = false;
    std::optional<Vec2> near_xy;
    Pose2 p0 = pose(fr);
    double s = path->project({p0.x, p0.y}).first;
    // 회전 기록 (목표 하나): odom 각속도 적분 · wz 명령 부호 반전 · 필요한 회전 하한 (출발 yaw ↔ 목표 yaw)
    rot_ = Rot{p0.yaw, std::abs(rl::wrap(goal.yaw - p0.yaw)), 0.0, 0.0, 0, fr};
    const double t0 = now_s();
    double t_plan = t0;
    const double T = 2 * path->length() / cfg_.vmax + 30.0;
    const double dt = cfg_.ctrl_dt * cfg_.decimation;
    double nxt = now_s();
    const auto wait_next = [&]() {
        while (now_s() < nxt && !interrupted()) {std::this_thread::sleep_for(2ms);}
      };
    while (!interrupted()) {
      if (gh->is_canceling()) {
        send({0, 0, 0});
        end_ = End::Cancel;
        return;
      }
      const double scan_t = scan().second;
      if (tracker && scan_t > scan_seen) {             // 새 스캔 → 박스 앞면 맞춤 → 목표 고침
        scan_seen = scan_t;
        const auto sp = scan_points(fr);
        const Pose2 pr = pose(fr);
        if (sp && tracker->update(sp->first, {pr.x, pr.y})) {
          const Pose2 g = tracker->goal();
          if (std::hypot(g.x - goal.x, g.y - goal.y) > 0.003 || std::abs(rl::wrap(g.yaw - goal.yaw)) > 0.2 * M_PI / 180.0) {
            goal = g;
            path.emplace(std::vector<Vec2>{{pr.x, pr.y}, {goal.x, goal.y}});
            s = 0.0;
          }
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "박스 맞춤: 지도 대비 (%+.1f, %+.1f) cm · %+.1f° (측정 %d 번)",
            (tracker->c.x - tracker->c0.x) * 100, (tracker->c.y - tracker->c0.y) * 100, deg(tracker->th), tracker->n_ok);
        } else if (tracker->last_why != "freeze") {
          // 버린 이유 기록. 박스가 안 보임 · 벽만 보임 · 지도 자리 · 방향과 다름이 측정 한 번 없이 계속 → 지도 목표로 그냥 가지 않음
          //   (실물: 박스가 돌아간 채 측정 0 번 · 지도 목표로 가다 충돌, 박스 없는 지도 정차점에 서서 '도착')
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "박스 맞춤 버림: %s", tracker->last_why.c_str());
          ++n_off;
          if (a_.box_reject_n > 0 && tracker->n_ok == 0 && n_off >= a_.box_reject_n) {
            finish(gh, false);
            RCLCPP_WARN(get_logger(), "박스가 지도 자리에 없음 (%s, 연속 %d 스캔, 측정 0 번) → 멈추고 실패. "
              "박스 자리 · 방향 · 지도 정합 · dice_map 정차점 확인", tracker->last_why.c_str(), n_off);
            return;
          }
        } else if (tracker->n_ok == 0 && a_.box_reject_n > 0 && start_dist > tracker->freeze + 0.05) {
          // 한 번도 못 잰 채 freeze 안까지 옴 (그 뒤는 몸 상자 · range_min 안이라 못 잼) → 박스 없는 자리에 서지 않음.
          //   출발부터 freeze 근처면 (Nav2 가 정차점까지 다 온 뒤 넘김) 잴 기회가 없었으니 그대로 감
          finish(gh, false);
          RCLCPP_WARN(get_logger(), "박스를 한 번도 못 잼 (출발 %.0f cm) → 멈추고 실패. dice_map 정차점 확인", start_dist * 100);
          return;
        }
        if (a_.dock_block_n > 0 && sp) {               // 직선 경로에 벽 → 연속 dock_block_n 스캔이면 멈추고 실패
          const Pose2 pb = pose(fr);
          const auto [nb, at] = rl::straight_blocked(pb, goal, *tracker, sp->first, dock_body_);
          n_block = nb ? n_block + 1 : 0;
          if (n_block >= a_.dock_block_n) {
            finish(gh, false);
            RCLCPP_WARN(get_logger(), "정밀 접근 직선 경로에 박스 아닌 점 %d 개 (%.0f cm 앞) → 멈추고 실패 (목표 %.0f cm)",
              nb, at * 100, std::hypot(goal.x - pb.x, goal.y - pb.y) * 100);
            return;
          }
        }
      }
      // 1 s 마다 경로 다시 (bt_navigator 와 같은 주기). 계획은 따로 스레드에서 → 제어 주기는 계속 명령을 냄
      //   (같은 스레드에서 기다리면 명령이 끊겨 swerve_controller cmd_vel 0.3 s 시간 초과로 섬)
      if (!tracker && !job && now_s() - t_plan > 1.0) {
        t_plan = now_s();
        job = std::make_shared<PlanJob>();
        const double L_now = path->length();
        if (replan_.joinable()) {replan_.join();}
        replan_ = std::thread([this, job, goal_ps, goal, L_now]() {
            const auto t1 = std::chrono::steady_clock::now();
            try {
              auto np = plan_frame(goal_ps, goal);
              if (!np && L_now < a_.straight_max) {np = straight(goal);}   // 짧은 구간 직선 대체 중이면 계속 직선
              job->path = std::move(np);
            } catch (const std::exception & e) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "경로 다시 계획 실패: %s", e.what());
            }
            job->dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
            job->done = true;
          });
      }
      if (job && job->done) {
        if (job->dt > 0.3) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "경로 계획 %.2f s 걸림 (제어는 계속)", job->dt);
        }
        if (job->path) {
          path = std::move(job->path);
          const Pose2 pp = pose(fr);
          s = path->project({pp.x, pp.y}).first;
        }
        job.reset();
      }
      const Pose2 p = pose(fr);
      s = path->project({p.x, p.y}, s).first;
      const auto sc = lidar();
      if (!sc) {                                        // 스캔 끊김 → 멈추고 기다림 (시간 초과는 그대로 셈)
        send({0, 0, 0});
        last_cmd = {0, 0, 0};
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "스캔 없음 (%.1f s 넘음) → 정지", a_.scan_stale_s);
        nxt = now_s() + dt;
        wait_next();
        if (now_s() - t0 > T) {
          end_ = End::Abort;
          return;
        }
        continue;
      }
      const rl::JointObs jo = joints();
      const auto o = rl::build_obs(cfg_, sw_, *path, p, s, goal, *sc, jo, last_cmd, hist);
      if (!near_checked) {                              // 붙어 선 자리에서 출발: 영역 끄기 → 벗어나면 켜기
        if (!near_xy) {near_xy = Vec2{p.x, p.y};}
        const double moved = std::hypot(p.x - near_xy->x, p.y - near_xy->y);
        const bool cleared = near_off_ && moved > a_.near_clear_m && stop_zone_clear();
        if (moved > a_.near_leave_m || cleared) {       // 출발 자리 벗어남 · 정지 구역 비었음 → 끝 (껐으면 켬)
          near_checked = true;
          if (near_off_) {
            stop();
            if (!set_polygons(true, near_polygons_)) {
              finish(gh, false);
              RCLCPP_ERROR(get_logger(), "출발 후 영역 복구 실패 → 정지하고 복구함");
              return;
            }
            near_off_ = false;
            RCLCPP_INFO(get_logger(), "출발 자리 %.0f cm 벗어남%s → 영역 켬", moved * 100, cleared ? " · 정지 구역 빔" : "");
          }
        } else if (!near_off_) {
          // 박스 면 (몸 중심 0.164–0.17 m) 은 scan_merger 몸 상자 경계라 스캔마다 보였다 말았다 함 → 벗어날 때까지 매 주기 봄
          //   벽이 가깝기만 한 자리 (출발 구역) 는 PolygonStop 이 안 막으니 안 끔
          const bool blocking = stop_blocking();
          if ((blocking || escaped_) && set_polygons(false, near_polygons_)) {
            near_off_ = true;
            RCLCPP_INFO(get_logger(), "출발 자리 %s → %.0f cm 벗어나거나 정지 구역 빌 때까지 영역 끔 (병진 ≤ %.2f m/s)",
              escaped_ ? "빠져나오기 경로" : "PolygonStop 막힘", a_.near_leave_m * 100, a_.near_vmax);
          }
        }
      }
      const double dist = std::hypot(goal.x - p.x, goal.y - p.y);
      const double dy = std::abs(rl::wrap(goal.yaw - p.yaw));
      // 도착 범위 안이면 0 명령으로 붙잡음 (바퀴 안 흔듦). 범위 2 배 밖으로 밀리면 다시 정책
      if (!a_.no_settle) {
        if (dist < tol_p_ && dy < tol_y_) {
          settled = true;
        } else if (dist > 2 * tol_p_ || dy > 2 * tol_y_) {
          settled = false;
        }
      }
      last_cmd = settled ? std::array<double, 3>{0, 0, 0} : act(o, dist);
      if ((a_.min_cmd_lin > 0 || a_.min_cmd_ang > 0) && std::hypot(last_cmd[0], last_cmd[1]) < a_.min_cmd_lin &&
        std::abs(last_cmd[2]) < a_.min_cmd_ang)
      {
        last_cmd = {0, 0, 0};                           // 아주 작은 명령 = 0 (바퀴 방향 안 바꿈)
      }
      send(last_cmd);
      rot_->odom += std::abs(jo.odom[2]) * dt;
      if (std::abs(last_cmd[2]) >= 0.05) {
        if (rot_->last_wz * last_cmd[2] < 0) {++rot_->rev;}
        rot_->last_wz = last_cmd[2];
      }
      const bool still = std::hypot(jo.odom[0], jo.odom[1]) < cfg_.goal_lin_vel && std::abs(jo.odom[2]) < cfg_.goal_ang_vel;
      hold = (dist < tol_p_ && dy < tol_y_ && still) ? hold + dt : 0.0;
      // 거의 다 옴: 범위 2 배 안에 near_ok_s 머물면 성공 (경계 밖에서 미세 조정하다 시간 초과 방지)
      near_t = (dist < 2 * tol_p_ && dy < 2 * tol_y_) ? near_t + dt : 0.0;
      if (a_.near_ok_s > 0 && near_t >= a_.near_ok_s && hold < cfg_.goal_hold_s) {
        finish(gh, true);
        RCLCPP_WARN(get_logger(), "범위 2 배 안 %.0f s → 성공으로 끝냄, 오차 %.1f cm · %.1f°", a_.near_ok_s, dist * 100, deg(dy));
        return;
      }
      auto fb = std::make_shared<Nav::Feedback>();
      fb->distance_remaining = static_cast<float>(path->length() - s);
      gh->publish_feedback(fb);
      if (hold >= cfg_.goal_hold_s) {
        finish(gh, true);
        std::string extra;
        if (tracker) {
          extra = fmt(", 박스 지도 대비 %.1f cm · %+.1f° (측정 %d 번", std::hypot(tracker->c.x - tracker->c0.x,
            tracker->c.y - tracker->c0.y) * 100, deg(tracker->th), tracker->n_ok) +
            (tracker->n_ok == 0 ? ", 마지막 이유 " + tracker->last_why + ")" : ")");
        }
        RCLCPP_INFO(get_logger(), "도착 %.1f s, 오차 %.1f cm · %.1f°%s", now_s() - t0, dist * 100, deg(dy), extra.c_str());
        return;
      }
      if (now_s() - t0 > T) {
        finish(gh, false);
        RCLCPP_WARN(get_logger(), "시간 초과");
        return;
      }
      std::optional<double> since;
      {
        std::lock_guard lk(in_mtx_);
        since = stop_since_;
      }
      if (a_.stop_abort_s > 0 && cm_polygons_.empty() && !near_off_ && since && now_s() - std::max(*since, t0) > a_.stop_abort_s) {
        // 영역을 끈 목표 (정밀 접근 · 출발 자리) 는 해당 없음. 사람이 막고 서 있어도 끝남 → planning 이 이동 단계를 다시 함
        std::optional<double> gb;
        {
          std::lock_guard lk(plan_mtx_);
          gb = goal_back_;
        }
        finish(gh, false);
        RCLCPP_WARN(get_logger(), "PolygonStop 이 %.0f s 넘게 막음 → 멈추고 실패 (목표 %.0f cm%s)", a_.stop_abort_s, dist * 100,
          gb ? fmt(", 목표 뒤 %.0f cm 물려 계획 중", *gb * 100).c_str() : "");
        return;
      }
      nxt += dt;
      wait_next();                                      // 시뮬 시간으로 다음 주기까지
    }
  }

  Args a_;
  rl::EnvCfg cfg_;
  rl::SwerveCfg sw_;
  rl::NpzPolicy policy_;
  double tol_p_ = 0.02, tol_y_ = 0.035;
  std::vector<std::string> cm_polygons_, near_polygons_;
  Vec2 dock_body_;

  rclcpp::CallbackGroup::SharedPtr cb_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<CmState>::SharedPtr sub_cm_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp_action::Client<Plan>::SharedPtr planner_;
  rclcpp_action::Server<Nav>::SharedPtr server_;
  rclcpp::Client<SetParameters>::SharedPtr cm_client_;

  std::mutex in_mtx_;                                   // 입력 (구독 콜백 ↔ 실행 스레드)
  rl::JointObs j_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  double scan_t_ = -1e9;
  CmState::ConstSharedPtr cm_state_;
  std::optional<double> stop_since_;

  std::mutex goal_mtx_;                                // 수락부터 결과 전송까지 목표 하나만 허용함
  bool busy_ = false;
  std::atomic<bool> stopping_{false}, plan_stop_{false}, zones_off_{false};
  std::mutex worker_mtx_;
  std::thread worker_, replan_;
  End end_ = End::Abort;
  rclcpp::TimerBase::SharedPtr restore_timer_;
  std::mutex plan_mtx_;                                 // 물림 거리 (실행 ↔ 다시 계획 스레드)
  std::optional<double> goal_back_;
  std::atomic<bool> escaped_{false};                    // 이번 목표에서 빠져나오기 경로를 씀 (= 치명 영역 안 출발)
  std::atomic<bool> near_off_{false};                   // 출발 자리 영역 끈 동안
  std::optional<std::chrono::steady_clock::time_point> last_send_;
  std::optional<Rot> rot_;
};

}  // namespace navigation

namespace
{
volatile std::sig_atomic_t exit_requested = 0;
void request_exit(int) {exit_requested = 1;}
}

int main(int argc, char ** argv)
{
  navigation::Args a;
  try {
    a = navigation::parse_args(argc, argv);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[local_planner_node] %s\n", e.what());
    return 2;
  }
  // ROS context를 먼저 끊으면 작업 중인 planner와 영역 복구 통신도 끊김
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_exit);
  std::signal(SIGTERM, request_exit);
  int rc = 0;
  try {
    auto node = std::make_shared<navigation::LocalPlannerNode>(a);
    rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 4);
    ex.add_node(node);
    std::atomic<bool> spin_done{false};
    std::exception_ptr spin_error;
    std::thread spin([&] {
      try {ex.spin();} catch (...) {spin_error = std::current_exception();}
      spin_done = true;
    });
    while (rclcpp::ok() && !exit_requested && !spin_done) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // 복구 서비스 응답은 executor 스레드에서 계속 처리함
    try {node->shutdown_worker();} catch (...) {
      ex.cancel();
      spin.join();
      throw;
    }
    ex.cancel();
    spin.join();
    if (spin_error) {std::rethrow_exception(spin_error);}
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[local_planner_node] %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
