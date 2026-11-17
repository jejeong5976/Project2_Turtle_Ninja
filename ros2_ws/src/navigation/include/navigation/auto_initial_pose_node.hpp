// auto_initial_pose_node.hpp — AMCL 초기 자세 자동 맞춤. 위치 메시지만 발행하며 주행/FAULT 상태는 바꾸지 않음.
// scripts/auto_initial_pose.py 의 C++ 판: 같은 노드 이름·파라미터·토픽·상태 JSON·로그 문구.
// 가볍게: /scan 콜백은 최신 메시지 포인터만 저장. 무거운 탐색 (match) 은 need_match · 정지 · retry_s 일 때만
//   작업 스레드 하나에서 (파이썬의 별도 프로세스 대신), 거리장은 같은 지도 판이면 다시 씀 (값 같음).
//   STANDBY 아니면 tick 은 상태 문자열만 냄.
// 시험 (test/test_auto_initial_pose_node.cpp) 이 current_inputs · status · publish_pose · submit_job 을 바꿔 씀.
#pragma once

#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "interfaces/msg/mission_status.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navigation/pose_match_core.hpp"
#include "navigation/py_json.hpp"
#include "navigation/resubscribe_watch.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace navigation
{

// status(..., current_score=x) 처럼 넘긴 값 (None 포함) 이 last_result 값을 덮음
struct AutoPoseStatusExtra
{
  bool has_current_score = false;
  std::optional<double> current_score;
};

class AutoInitialPose : public rclcpp::Node
{
public:
  using Pose = pose_match::Pose;
  using Points = pose_match::Points;
  using JobOutput = pose_match::JobOutput;

  struct Inputs
  {
    Points points;
    Pose current{0.0, 0.0, 0.0};
  };
  // 파이썬 ValueError 와 같은 입력 대기 사유 (상태 reason 문자열 = what())
  struct InputError : std::invalid_argument
  {
    using std::invalid_argument::invalid_argument;
  };
  struct Job
  {
    int64_t epoch = 0;
    int64_t map_revision = 0;
    Pose odom_pose{0.0, 0.0, 0.0};
  };
  struct LastResult
  {
    std::optional<double> score;
    std::optional<double> current_score;
    std::optional<double> elapsed_s;
  };
  using Extra = AutoPoseStatusExtra;
  static Extra with_current_score(std::optional<double> v) {return Extra{true, v};}

  static double yaw(const geometry_msgs::msg::Quaternion & q)
  {
    const double x0 = q.x, y0 = q.y, z0 = q.z, w0 = q.w;
    const double n = std::sqrt(x0 * x0 + y0 * y0 + z0 * z0 + w0 * w0);
    if (!(std::isfinite(x0) && std::isfinite(y0) && std::isfinite(z0) && std::isfinite(w0)) || n < 1e-6) {
      throw std::invalid_argument("잘못된 회전값");
    }
    const double x = x0 / n, y = y0 / n, z = z0 / n, w = w0 / n;
    return std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  }

  static Pose pose2(const geometry_msgs::msg::Pose & p)
  {
    const Pose out{p.position.x, p.position.y, yaw(p.orientation)};
    if (!(std::isfinite(out[0]) && std::isfinite(out[1]) && std::isfinite(out[2]))) {
      throw std::invalid_argument("잘못된 위치값");
    }
    return out;
  }

  static double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  static double monotonic()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static double radians(double deg) {return deg * (M_PI / 180.0);}

  explicit AutoInitialPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("auto_initial_pose", options), tf(get_clock()), tf_listener(tf)
  {
    // 탐색 기본값의 유일한 원본은 ROS 독립 MatchConfig임.
    const pose_match::MatchConfig d;
    cfg.search_x = declare_parameter<double>("search_x", d.search_x);
    cfg.search_y = declare_parameter<double>("search_y", d.search_y);
    cfg.search_yaw_deg = declare_parameter<double>("search_yaw_deg", d.search_yaw_deg);
    cfg.coarse_xy = declare_parameter<double>("coarse_xy", d.coarse_xy);
    cfg.coarse_yaw_deg = declare_parameter<double>("coarse_yaw_deg", d.coarse_yaw_deg);
    cfg.fine_xy = declare_parameter<double>("fine_xy", d.fine_xy);
    cfg.fine_yaw_deg = declare_parameter<double>("fine_yaw_deg", d.fine_yaw_deg);
    cfg.wall_distance = declare_parameter<double>("wall_distance", d.wall_distance);
    cfg.min_score = declare_parameter<double>("min_score", d.min_score);
    cfg.min_improvement = declare_parameter<double>("min_improvement", d.min_improvement);
    cfg.ambiguity_margin = declare_parameter<double>("ambiguity_margin", d.ambiguity_margin);
    cfg.distinct_xy = declare_parameter<double>("distinct_xy", d.distinct_xy);
    cfg.distinct_yaw_deg = declare_parameter<double>("distinct_yaw_deg", d.distinct_yaw_deg);
    cfg.max_points = declare_parameter<int64_t>("max_points", d.max_points);
    cfg.min_points = declare_parameter<int64_t>("min_points", d.min_points);
    cfg.refine_modes = declare_parameter<int64_t>("refine_modes", d.refine_modes);
    cfg.validate();
    seed = Pose{declare_parameter<double>("init_x", 0.0), declare_parameter<double>("init_y", 0.0),
      radians(declare_parameter<double>("init_yaw_deg", 0.0))};
    base_frame = declare_parameter<std::string>("base_frame", "base_footprint");
    odom_frame = declare_parameter<std::string>("odom_frame", "odom");
    retry_s = declare_parameter<double>("retry_s", 3.0);
    scan_max_age = declare_parameter<double>("scan_max_age", 0.5);
    odom_max_age = declare_parameter<double>("odom_max_age", 0.5);
    tf_max_age = declare_parameter<double>("tf_max_age", 1.0);
    still_v = declare_parameter<double>("still_v", 0.01);
    still_w = declare_parameter<double>("still_w", 0.02);
    // 기존 amcl_settle의 초기 공분산과 같음. 팔/주행 보정값이 아님.
    init_std_xy = declare_parameter<double>("init_std_xy", 0.05);
    init_std_yaw = radians(declare_parameter<double>("init_std_yaw_deg", 3.0));
    for (double v : {retry_s, scan_max_age, odom_max_age, tf_max_age, still_v, still_w, init_std_xy, init_std_yaw}) {
      if (!(std::isfinite(v) && v > 0)) {
        throw std::invalid_argument("시간/속도/초기 자세 파라미터 확인 필요");
      }
    }
    if (!(std::isfinite(seed[0]) && std::isfinite(seed[1]) && std::isfinite(seed[2]))) {
      throw std::invalid_argument("시간/속도/초기 자세 파라미터 확인 필요");
    }
    const auto transient = rclcpp::QoS(1).transient_local().reliable();
    pub = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    status_pub = create_publisher<std_msgs::msg::String>("/startup/pose_match_status", transient);
    subscribe_map();
    // 스캔 콜백은 포인터만 저장 (변환·점수는 tick 에서, STANDBY 일 때만)
    subscribe_scan();
    sub_odom = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_odom(*m);});
    sub_health = create_subscription<diagnostic_msgs::msg::DiagnosticStatus>("/localization_health", 10,
        [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr m) {on_health(*m);});
    // 미션이 STANDBY 일 때만 맞춤 (RUNNING 중 /initialpose 로 map→odom 이 움직이지 않게). 상태를 아직 못 받았으면 STANDBY 로 봄
    sub_mission = create_subscription<interfaces::msg::MissionStatus>("/mission/status", 10,
        [this](interfaces::msg::MissionStatus::ConstSharedPtr m) {on_mission_status(*m);});
    timer = create_timer(std::chrono::milliseconds(250), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "자동 초기 위치: 스캔/지도 탐색 · 정지 때 재시도 · /startup/pose_match_status");
    status("waiting_inputs", false, "map_scan_odom_tf");
  }

  ~AutoInitialPose() override
  {
    if (timer) {
      timer->cancel();
    }
    // 진행 중 계산은 끝나기를 기다림 (std::async future 소멸자, 한 번 탐색 수십 ms)
  }

  void on_map(const nav_msgs::msg::OccupancyGrid & msg)
  {
    try {
      const Pose p = pose2(msg.info.origin);
      const Pose origin{p[0], p[1], yaw(msg.info.origin.orientation)};
      const std::size_t h = msg.info.height, w = msg.info.width;
      if (h * w != msg.data.size()) {
        throw std::invalid_argument("cannot reshape array of size " + std::to_string(msg.data.size()) +
                " into shape (" + std::to_string(h) + "," + std::to_string(w) + ")");
      }
      if (h * w == 0 || msg.info.resolution <= 0) {
        throw std::invalid_argument("빈 지도/해상도");
      }
      std::string frame = msg.header.frame_id;
      frame.erase(0, frame.find_first_not_of('/') == std::string::npos ? frame.size() : frame.find_first_not_of('/'));
      if (frame.empty()) {
        frame = "map";
      }
      const double resolution = static_cast<double>(msg.info.resolution);
      if (map_data && map_data->resolution == resolution && map_data->origin == origin &&
        map_data->frame == frame && map_data->height == static_cast<int64_t>(h) &&
        map_data->width == static_cast<int64_t>(w) &&
        std::equal(msg.data.begin(), msg.data.end(), map_data->grid.begin()))
      {
        return;
      }
      auto data = std::make_shared<pose_match::MapData>();
      data->grid.assign(msg.data.begin(), msg.data.end());
      data->width = static_cast<int64_t>(w);
      data->height = static_cast<int64_t>(h);
      data->resolution = resolution;
      data->origin = origin;
      data->frame = frame;
      map_data = std::move(data);
      map_revision += 1;
      need_match = true;
      model.reset();
    } catch (const std::invalid_argument & exc) {
      map_data.reset();
      model.reset();
      map_revision += 1;
      need_match = true;
      RCLCPP_WARN(get_logger(), "지도 다시 기다림: %s", exc.what());
    }
  }

  void on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    scan = std::move(msg);
    scan_received = monotonic();
    scan_watch.seen(scan_received);
  }

  // 발행자는 도는데 구독만 못 받는 시작 (DDS 짝짓기 유실) → 조용하면 구독을 새로 만듦
  //   /map 10 s (받은 뒤엔 안 봄, 한 번만 옴) · /scan 5 s. 연달아면 2 배씩 최대 60 s
  void subscribe_map()
  {
    sub_map = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {on_map(*m);});
    map_watch.start(monotonic());
  }

  void subscribe_scan()
  {
    sub_scan = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {on_scan(std::move(m));});
    scan_watch.start(monotonic());
  }

  // 발행자가 보이는데 못 받으면 WARN (짝짓기 유실), 발행자가 아직 안 보이면 INFO. /map 은 발행자가 보일 때만
  void heal_subscriptions(double now)
  {
    if (!map_data && sub_map->get_publisher_count() == 0) {
      map_watch.start(now);   // map_server ACTIVE 전엔 안 셈
    } else if (!map_data && map_watch.due(now)) {
      const size_t pubs = sub_map->get_publisher_count();
      sub_map.reset();
      subscribe_map();
      RCLCPP_WARN(get_logger(), "/map 발행자 %zu 인데 못 받음 → 구독 다시 만듦 (%d 번째)", pubs, map_watch.count());
    }
    if (scan_watch.due(now)) {
      const size_t pubs = sub_scan->get_publisher_count();
      sub_scan.reset();
      subscribe_scan();
      if (pubs > 0) {
        RCLCPP_WARN(get_logger(), "/scan 발행자 %zu 인데 %s → 구독 다시 만듦 (%d 번째)", pubs,
          scan_watch.ever() ? "끊김" : "못 받음", scan_watch.count());
      } else {
        RCLCPP_INFO(get_logger(), "/scan 발행자 아직 안 보임 → 구독 새로 만듦 (%d 번째)", scan_watch.count());
      }
    }
  }

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    try {
      const Pose p = pose2(msg.pose.pose);
      const auto & t = msg.twist.twist;
      const bool finite = std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.angular.z);
      const bool stationary = finite && std::hypot(t.linear.x, t.linear.y) < still_v &&
        std::fabs(t.angular.z) < still_w;
      // 계산 시작/끝만 비교하면 움직였다 돌아온 경우를 놓치므로 중간 이동도 기록함.
      if (!stationary || (job && pose_match::pose_changed(job->odom_pose, p))) {
        motion_epoch += 1;
        last_motion_stamp = stamp_seconds(msg.header.stamp);
      }
      still = stationary;
      odom_pose = p;
      odom_received = monotonic();
      odom_stamp = stamp_seconds(msg.header.stamp);
    } catch (const std::invalid_argument &) {
      still = false;
      motion_epoch += 1;
      odom_received = 0.0;
    }
  }

  void on_health(const diagnostic_msgs::msg::DiagnosticStatus & msg)
  {
    for (const auto & item : msg.values) {
      if (item.key == "match") {
        double score = 0.0;
        if (pyjson::parse_python_float(item.value, score)) {
          health_low = std::isfinite(score) && score < cfg.min_score;
          if (health_low) {
            need_match = true;
          }
        }
        break;
      }
    }
  }

  void on_mission_status(const interfaces::msg::MissionStatus & msg)
  {
    mission_state = msg.state;
  }

  virtual Inputs current_inputs()
  {
    const double now = monotonic();
    const double ros_now = static_cast<double>(get_clock()->now().nanoseconds()) / 1e9;
    if (!map_data || !scan || !odom_pose) {
      throw InputError("map_scan_odom_wait");
    }
    if (now - scan_received > scan_max_age || now - odom_received > odom_max_age) {
      throw InputError("scan_odom_stale");
    }
    const double age = ros_now - stamp_seconds(scan->header.stamp);
    const double odom_age = ros_now - odom_stamp;
    if (!(-0.2 <= age && age <= scan_max_age && -0.2 <= odom_age && odom_age <= odom_max_age)) {
      throw InputError("scan_odom_stamp_stale");
    }
    const auto msg = scan;
    if (stamp_seconds(msg->header.stamp) < last_motion_stamp) {
      throw InputError("scan_before_stationary");
    }
    Points points = pose_match::scan_points(msg->ranges, static_cast<double>(msg->angle_min),
      static_cast<double>(msg->angle_increment),
      std::max(0.25, static_cast<double>(msg->range_min)), std::min(4.0, static_cast<double>(msg->range_max)),
      &trig_cache);
    std::string frame = msg->header.frame_id;
    const auto first = frame.find_first_not_of('/');
    frame.erase(0, first == std::string::npos ? frame.size() : first);
    if (frame.empty()) {
      throw InputError("scan_frame_empty");
    }
    if (frame != base_frame) {
      const auto tr = tf.lookupTransform(base_frame, frame, to_time_point(msg->header.stamp));
      const double a = yaw(tr.transform.rotation);
      const double c = std::cos(a), s = std::sin(a);
      const double tx = tr.transform.translation.x, ty = tr.transform.translation.y;
      for (std::size_t i = 0; i < points.size(); ++i) {
        const double px = points.x[i], py = points.y[i];
        points.x[i] = px * c + py * -s + tx;
        points.y[i] = px * s + py * c + ty;
      }
    }
    points = pose_match::prepare_points(points, cfg);
    const std::string & map_frame = map_data->frame;
    // 합성 map→base의 stamp만 보지 않고 AMCL 소유 map→odom도 신선한지 확인함.
    const auto mo = tf.lookupTransform(map_frame, odom_frame, tf2::TimePointZero);
    if (ros_now - stamp_seconds(mo.header.stamp) > tf_max_age) {
      throw InputError("map_odom_stale");
    }
    const auto tr = tf.lookupTransform(map_frame, base_frame, tf2::TimePointZero);
    if (ros_now - stamp_seconds(tr.header.stamp) > tf_max_age) {
      throw InputError("base_tf_stale");
    }
    const Pose current{tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation)};
    if (!(std::isfinite(current[0]) && std::isfinite(current[1]) && std::isfinite(current[2]))) {
      throw InputError("pose_nonfinite");
    }
    return Inputs{std::move(points), current};
  }

  virtual void status(const std::string & state, bool ready, const std::string & reason, const Extra & extra = {})
  {
    std::string & s = status_text;
    s.clear();
    s += "{\"state\": ";
    pyjson::append_string(s, state);
    s += ", \"ready\": ";
    pyjson::append_bool(s, ready);
    s += ", \"reason\": ";
    pyjson::append_string(s, reason);
    auto opt = [&s](const std::optional<double> & v) {
        if (v) {
          pyjson::append_float(s, *v);
        } else {
          s += "null";
        }
      };
    s += ", \"score\": ";
    opt(last_result.score);
    s += ", \"current_score\": ";
    opt(extra.has_current_score ? extra.current_score : last_result.current_score);
    s += ", \"elapsed_s\": ";
    opt(last_result.elapsed_s);
    s += ", \"attempt\": ";
    pyjson::append_int(s, attempt);
    s += ", \"published\": ";
    pyjson::append_int(s, published);
    s += "}";
    std_msgs::msg::String m;
    m.data = s;
    status_pub->publish(m);
    auto key = std::make_tuple(state, reason, attempt, published);
    if (!last_status_key || *last_status_key != key) {
      RCLCPP_INFO(get_logger(), "초기 위치: %s", s.c_str());
      last_status_key = std::move(key);
    }
  }

  virtual void publish_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    pub->publish(msg);
  }

  virtual std::future<JobOutput> submit_job(std::shared_ptr<const pose_match::MapData> map,
    std::shared_ptr<const pose_match::MapModel> reuse, Points points, Pose center, Pose current)
  {
    const pose_match::MatchConfig c = cfg;
    return std::async(std::launch::async,
             [map = std::move(map), reuse = std::move(reuse), points = std::move(points), center, current, c]() {
               return pose_match::run_match_job(*map, reuse, points, center, current, c);
             });
  }

  void tick()
  {
    const double now = monotonic();
    heal_subscriptions(now);
    if (!(mission_state.empty() || mission_state == "STANDBY")) {
      if (future.valid()) {
        motion_epoch += 1;   // 진행 중 계산 결과는 버림
      }
      status("paused_mission", true, "mission_" + mission_state);
      return;
    }
    Inputs in;
    try {
      in = current_inputs();
    } catch (const std::invalid_argument & exc) {
      // 진행 중 계산은 받아 두되 입력이 다시 신선해지기 전에는 발행하지 않음.
      if (future.valid()) {
        motion_epoch += 1;
      }
      status("waiting_inputs", false, exc.what());
      return;
    } catch (const tf2::TransformException & exc) {
      if (future.valid()) {
        motion_epoch += 1;
      }
      status("waiting_inputs", false, exc.what());
      return;
    }
    bool ready = false;
    std::optional<double> current_score;
    if (model && model_revision == map_revision) {
      current_score = model->score(in.points, in.current, cfg.wall_distance);
      ready = *current_score >= cfg.min_score;
    }
    if (future.valid()) {
      if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        status("matching", ready, "worker", with_current_score(current_score));
        return;
      }
      const Job j = *job;
      auto f = std::move(future);
      job.reset();
      future = std::future<JobOutput>();
      next_attempt = now + retry_s;
      JobOutput out;
      try {
        out = f.get();
      } catch (const std::exception & exc) {  // 작업 오류가 ROS 실행기를 끝내지 않고 다음 시도로 이어짐
        need_match = true;
        status("retrying", ready, std::string("worker_error: ") + exc.what(), with_current_score(current_score));
        return;
      }
      if (out.model) {
        cached_model = out.model;
        cached_revision = j.map_revision;
      }
      if (j.map_revision == map_revision) {
        model = out.model;
        model_revision = map_revision;
      }
      if (j.map_revision != map_revision || j.epoch != motion_epoch || !still ||
        pose_match::pose_changed(j.odom_pose, *odom_pose))
      {
        need_match = true;
        status("retrying", ready, "moved_or_map_changed", with_current_score(current_score));
        return;
      }
      // 계산 완료 뒤 받은 현재 스캔/현재 TF로 다시 비교함.
      const auto decision = pose_match::decide_publication(out.result, *out.model, in.points, in.current, cfg);
      last_result.score = decision.score;
      last_result.current_score = decision.current_score;
      last_result.elapsed_s = out.result.elapsed_s;
      ready = decision.current_score >= cfg.min_score;
      need_match = decision.retry;
      if (decision.publish) {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = map_data->frame;
        msg.header.stamp = get_clock()->now();
        const Pose & q = *out.result.pose;
        msg.pose.pose.position.x = q[0];
        msg.pose.pose.position.y = q[1];
        msg.pose.pose.orientation.z = std::sin(q[2] / 2);
        msg.pose.pose.orientation.w = std::cos(q[2] / 2);
        msg.pose.covariance[0] = msg.pose.covariance[7] = init_std_xy * init_std_xy;
        msg.pose.covariance[35] = init_std_yaw * init_std_yaw;
        publish_pose(msg);
        published += 1;
        // 발행 성공과 위치 수렴은 다름. 새 TF의 현재 점수를 다시 확인함.
        need_match = true;
        status("published", ready, "await_current_pose");
      } else {
        status(ready ? "aligned" : "retrying", ready, decision.reason);
      }
      return;
    }
    if (ready) {
      need_match = false;
      first_search = false;
      status("aligned", true, "current_scan", with_current_score(current_score));
      return;
    }
    if (model && !ready) {
      need_match = true;
    }
    if (!still) {
      status("deferred_motion", false, "wait_stationary", with_current_score(current_score));
      return;
    }
    if (!need_match || now < next_attempt) {
      status("retrying", false, "retry_delay", with_current_score(current_score));
      return;
    }
    const Pose center = first_search ? seed : in.current;
    first_search = false;
    attempt += 1;
    job = Job{motion_epoch, map_revision, *odom_pose};
    std::shared_ptr<const pose_match::MapModel> reuse;
    if (cached_model && cached_revision == map_revision) {
      reuse = cached_model;
    }
    try {
      future = submit_job(map_data, reuse, in.points, center, in.current);
    } catch (const std::runtime_error & exc) {
      job.reset();
      next_attempt = now + retry_s;
      status("retrying", false, std::string("submit_error: ") + exc.what(), with_current_score(current_score));
      return;
    }
    status("matching", false, "worker", with_current_score(current_score));
  }

  static tf2::TimePoint to_time_point(const builtin_interfaces::msg::Time & t)
  {
    return tf2::TimePoint(std::chrono::seconds(t.sec) + std::chrono::nanoseconds(t.nanosec));
  }

  // ---- 상태 (파이썬 속성과 같은 이름, 시험이 직접 읽고 씀) ----
  pose_match::MatchConfig cfg;
  Pose seed{0.0, 0.0, 0.0};
  std::string base_frame, odom_frame;
  double retry_s = 3.0, scan_max_age = 0.5, odom_max_age = 0.5, tf_max_age = 1.0;
  double still_v = 0.01, still_w = 0.02, init_std_xy = 0.05, init_std_yaw = 0.0;

  tf2_ros::Buffer tf;
  tf2_ros::TransformListener tf_listener;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;

  std::shared_ptr<const pose_match::MapData> map_data;
  int64_t map_revision = 0;
  std::shared_ptr<const pose_match::MapModel> model;
  int64_t model_revision = -1;
  std::shared_ptr<const pose_match::MapModel> cached_model;   // 작업 스레드가 다시 쓰는 거리장 (지도 판 cached_revision)
  int64_t cached_revision = -1;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  double scan_received = 0.0;
  std::optional<Pose> odom_pose;
  double odom_received = 0.0;
  double odom_stamp = 0.0;
  bool still = false;
  int64_t motion_epoch = 0;
  double last_motion_stamp = -std::numeric_limits<double>::infinity();
  bool health_low = false;
  bool need_match = true;
  bool first_search = true;
  std::future<JobOutput> future;
  std::optional<Job> job;
  int64_t attempt = 0;
  int64_t published = 0;
  double next_attempt = 0.0;
  std::optional<std::tuple<std::string, std::string, int64_t, int64_t>> last_status_key;
  LastResult last_result;
  std::string mission_state;
  ResubscribeWatch map_watch{10.0, 60.0}, scan_watch{5.0, 60.0};

private:
  pose_match::ScanTrigCache trig_cache;
  std::string status_text;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr sub_health;
  rclcpp::Subscription<interfaces::msg::MissionStatus>::SharedPtr sub_mission;
  rclcpp::TimerBase::SharedPtr timer;
};

}  // namespace navigation
