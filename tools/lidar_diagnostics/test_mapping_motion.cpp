#define MAPPING_MOTION_TEST
#include "run_mapping_motion.cpp"
#include <rosbag2_cpp/reader.hpp>
#include <functional>
#include <limits>
#include <set>

namespace {
using Cmd = geometry_msgs::msg::TwistStamped;
using Odom = nav_msgs::msg::Odometry;
using Scan = sensor_msgs::msg::LaserScan;
using Map = nav_msgs::msg::OccupancyGrid;
using Text = std_msgs::msg::String;
using TF = tf2_msgs::msg::TFMessage;
using State = lifecycle_msgs::srv::GetState;
const std::string prefix = "/mapping_trial_test";

void must_fail(const std::function<void()> &fn) {
  bool failed = false;
  try { fn(); } catch (const std::exception &) { failed = true; }
  require(failed, "Expected rejection");
}

void core_tests() {
  using namespace mapping_trial;
  const double pi = std::acos(-1.);
  const Pose origin{0, 2, 3, pi / 2};
  const auto d = relative({0, 2, 3.1, pi / 2}, origin);
  require(std::abs(d.forward - .1) < 1e-9 && std::abs(d.lateral) < 1e-9,
    "Body-frame displacement");
  bounds({0, 2, 3.1, pi / 2}, origin);
  must_fail([&] { bounds({0, 2, 3.16, pi / 2}, origin); });
  must_fail([&] { bounds({0, 2.04, 3, pi / 2}, origin); });
  must_fail([&] { bounds({0, 2, 2.98, pi / 2}, origin); });
  must_fail([&] { bounds({0, 2, 3, pi / 2 + .11}, origin); });
  bounds({0, 2, 3, pi / 2 + .20}, origin, Motion::rotation);
  must_fail([&] { bounds({0, 2.031, 3, pi / 2}, origin, Motion::rotation); });
  must_fail([&] { bounds({0, 2, 3, pi / 2 - .021}, origin, Motion::rotation); });
  must_fail([&] { bounds({0, 2, 3, pi / 2 + .251}, origin, Motion::rotation); });
  bounds({0, 0, 0, -pi + .1}, {0, 0, 0, pi - .1}, Motion::rotation);
  completed({0, 0, 0, .2}, {}, Motion::rotation);
  must_fail([] { completed({0, .1, 0, .05}, {}, Motion::rotation); });
  require(motion_from_flag("--execute-rotation-pulse") == Motion::rotation &&
    motion_from_flag("--execute-forward-pulse") == Motion::forward, "Explicit mode selection");
  must_fail([] { motion_from_flag("--execute-pulse"); });
  advancing({20000000, .0005, 0, 0}, {});
  must_fail([] { advancing({41000000, 0, 0, 0}, {}); });
  must_fail([] { advancing({20000000, .002, 0, 0}, {}); });
  must_fail([] { advancing({20000000, 0, 0, .004}, {}); });
  std::deque<Pose> poses;
  for (int i = 0; i < 25; ++i) poses.push_back({i * 20000000LL, 0, 0, 0});
  require(stationary(poses), "Stationary window");
  poses[12].x = .001;
  require(!stationary(poses), "Interior excursion must fail stop window");
  const std::string good = "model=nominal assumed_delay_ns=0 acquisition_time_valid=0 bins=1600 "
    "published=10 rejected=3 fault=0 fresh=1 reference_ns=100 reason=publishing";
  require(status(good).published == 10, "Status parsing");
  must_fail([&] { status(good + " fault=1"); });
  must_fail([&] { status("model=nominal"); });
  auto bad = good;
  bad.replace(bad.find("fresh=1"), 7, "fresh=0");
  must_fail([&] { status(bad); });
}

struct Fixture {
  rclcpp::Node::SharedPtr wheel = std::make_shared<rclcpp::Node>("swerve_controller", prefix);
  rclcpp::Node::SharedPtr input = std::make_shared<rclcpp::Node>("front_mapping_input", prefix);
  rclcpp::Node::SharedPtr slam = std::make_shared<rclcpp::Node>("slam_toolbox", prefix);
  rclcpp::Publisher<Odom>::SharedPtr odom = wheel->create_publisher<Odom>(prefix + "/odom", 100);
  rclcpp::Publisher<Scan>::SharedPtr scan = input->create_publisher<Scan>(prefix + "/lidar_front/scan_mapping", rclcpp::SensorDataQoS());
  rclcpp::Publisher<Text>::SharedPtr status = input->create_publisher<Text>(prefix + "/lidar_front/mapping_status", 10);
  rclcpp::Publisher<Map>::SharedPtr map = slam->create_publisher<Map>(prefix + "/map", rclcpp::QoS(1).transient_local());
  rclcpp::Publisher<TF>::SharedPtr tf = slam->create_publisher<TF>(prefix + "/tf", 100);
  rclcpp::Publisher<TF>::SharedPtr fixed = input->create_publisher<TF>(prefix + "/tf_static", rclcpp::QoS(1).transient_local());
  rclcpp::Subscription<Cmd>::SharedPtr commands;
  rclcpp::Service<State>::SharedPtr state;
  rclcpp::Publisher<Cmd>::SharedPtr competitor;
  std::vector<Cmd> received;
  double x = 0, heading = 0, velocity = 0, yaw_velocity = 0;
  double yaw_scale = 1, translation_drift = 0, last_command = 0, last_pump = steady_seconds();
  double odom_at = 0, scan_at = 0, status_at = 0, map_at = 0;
  int scans = 0;
  bool drop_odom = false, drop_scan = false, fault = false, active = true;

  Fixture() {
    commands = wheel->create_subscription<Cmd>(prefix + "/cmd_vel", 100,
      [this](Cmd::ConstSharedPtr m) {
        received.push_back(*m); velocity = m->twist.linear.x; yaw_velocity = m->twist.angular.z;
        last_command = steady_seconds();
      });
    state = slam->create_service<State>(prefix + "/slam_toolbox/get_state",
      [this](std::shared_ptr<State::Request>, std::shared_ptr<State::Response> r) {
        r->current_state.id = active ? 3 : 2;
      });
    TF f;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "base_footprint"; t.child_frame_id = "lidar_front_link";
    t.transform.translation.x = .110; t.transform.translation.y = .005;
    t.transform.translation.z = .0545; t.transform.rotation.z = 1; t.transform.rotation.w = 0;
    f.transforms.push_back(t); fixed->publish(f);
  }
  size_t moving() const {
    return std::count_if(received.begin(), received.end(), [](const Cmd &m) {
      return m.twist.linear.x != 0 || m.twist.angular.z != 0;
    });
  }
  void pump() {
    const double t = steady_seconds();
    if (t - last_command < .3) {
      x += (velocity + (yaw_velocity != 0 ? translation_drift : 0)) * (t - last_pump);
      heading += yaw_velocity * yaw_scale * (t - last_pump);
    }
    last_pump = t;
    const auto stamp = wheel->now();
    if (t - odom_at >= .020) {
      odom_at = t;
      if (!drop_odom) {
        Odom m; m.header.stamp = stamp; m.header.frame_id = "odom";
        m.child_frame_id = "base_footprint"; m.pose.pose.position.x = x;
        m.pose.pose.orientation.z = std::sin(heading / 2);
        m.pose.pose.orientation.w = std::cos(heading / 2); odom->publish(m);
      }
      TF f;
      geometry_msgs::msg::TransformStamped a;
      a.header.stamp = stamp; a.header.frame_id = "odom"; a.child_frame_id = "base_footprint";
      a.transform.translation.x = x; a.transform.rotation.z = std::sin(heading / 2);
      a.transform.rotation.w = std::cos(heading / 2); f.transforms.push_back(a);
      a.header.frame_id = "map"; a.child_frame_id = "odom"; a.transform.translation.x = 0;
      a.transform.rotation.z = 0; a.transform.rotation.w = 1;
      f.transforms.push_back(a); tf->publish(f);
    }
    if (!drop_scan && t - scan_at >= .1) {
      scan_at = t; ++scans;
      Scan m; m.header.stamp = stamp; m.header.frame_id = "lidar_front_link";
      m.angle_increment = 2 * std::acos(-1.) / 1600; m.angle_max = 1599 * m.angle_increment;
      m.range_min = .05; m.range_max = 12;
      m.ranges.assign(1600, std::numeric_limits<float>::quiet_NaN());
      for (int i = 0; i < 1600; i += 4) m.ranges[i] = 2;
      scan->publish(m);
    }
    if (t - status_at >= .2) {
      status_at = t;
      Text m; m.data = "model=nominal assumed_delay_ns=0 acquisition_time_valid=0 bins=1600 published=" +
        std::to_string(scans) + " rejected=3 fault=" + (fault ? "1" : "0") +
        " fresh=1 reference_ns=" + std::to_string(stamp.nanoseconds()) + " reason=publishing";
      status->publish(m);
    }
    if (t - map_at >= .5) {
      map_at = t;
      Map m; m.header.stamp = stamp; m.header.frame_id = "map";
      m.info.resolution = .05; m.info.width = 10; m.info.height = 10;
      m.info.origin.orientation.w = 1; m.data.assign(100, -1); m.data[0] = x > .05 || heading > .1 ? 100 : 0;
      map->publish(m);
    }
  }
};

void verify_bag(const fs::path &dir, const Fixture &fixture, bool complete,
    mapping_trial::Motion motion, bool late_failure) {
  const auto pulse = mapping_trial::pulse(motion);
  rosbag2_cpp::Reader reader; reader.open((dir / "bag").string());
  std::set<std::string> topics;
  std::vector<Cmd> commands;
  bool saw_nan = false;
  while (reader.has_next()) {
    auto record = reader.read_next(); topics.insert(record->topic_name);
    rclcpp::SerializedMessage serialized(*record->serialized_data);
    if (record->topic_name == prefix + "/cmd_vel") {
      Cmd m; rclcpp::Serialization<Cmd>().deserialize_message(&serialized, &m); commands.push_back(m);
    }
    if (record->topic_name == prefix + "/lidar_front/scan_mapping") {
      Scan m; rclcpp::Serialization<Scan>().deserialize_message(&serialized, &m);
      saw_nan = saw_nan || std::isnan(m.ranges[1]);
    }
  }
  require(topics.size() == (fixture.received.empty() ? 6 : 7) && saw_nan, "Streams and unknown scan bins preserved");
  require(commands.size() == fixture.received.size(), "Recorded commands differ from delivered test commands");
  bool zero_seen = false;
  for (size_t i = 0; i < commands.size(); ++i) {
    const auto &m = commands[i];
    require(m.twist.linear.x == fixture.received[i].twist.linear.x && m.twist.linear.y == 0 &&
      m.twist.linear.z == 0 && m.twist.angular.x == 0 && m.twist.angular.y == 0 &&
      m.twist.angular.z == fixture.received[i].twist.angular.z && m.header.frame_id == "base_footprint",
      "Unexpected command values");
    if (m.twist.linear.x == 0 && m.twist.angular.z == 0) zero_seen = true;
    else require(!zero_seen && m.twist.linear.x == pulse.forward && m.twist.angular.z == pulse.yaw,
      "Motion after zero or wrong velocity");
  }
  require(commands.size() - fixture.moving() == (fixture.received.empty() ? 0 : late_failure ? 10 : 5),
    "Expected five explicit zeros per normal/abort stop sequence");
  std::ifstream result(dir / "result.json");
  std::string json((std::istreambuf_iterator<char>(result)), std::istreambuf_iterator<char>());
  require(json.find(complete ? "\"complete\": true" : "\"complete\": false") != std::string::npos,
    "Result completion flag");
  require(json.find(std::string("\"motion\": \"") + pulse.name + "\"") != std::string::npos,
    "Result records selected mode");
  if (complete) {
    require(fixture.moving() == pulse.messages && commands.size() == pulse.messages + 5, "Finite pulse count");
    const double duration = (rclcpp::Time(commands[pulse.messages].header.stamp) - rclcpp::Time(commands[0].header.stamp)).seconds();
    require(duration >= pulse.messages * .1 && duration < pulse.deadline, "Pulse duration");
    if (motion == mapping_trial::Motion::rotation)
      require(fixture.heading > .19 && fixture.heading < .22 && fixture.x == 0, "Rotation displacement and no linear command");
    else require(fixture.x > .09 && fixture.x < .11 && fixture.heading == 0, "Forward displacement and no yaw command");
    require(json.find("\"map_cells_changed\": true") != std::string::npos, "Changed map detected");
  } else require(late_failure ? fixture.moving() == pulse.messages : fixture.moving() < pulse.messages,
    "Fault must interrupt the pulse unless detected at completion");
}

// Offline-only topic relabeling for the reviewer. No root-topic ROS publishers.
void export_review_bag(const fs::path &dir) {
  rosbag2_cpp::Reader reader; reader.open((dir / "bag").string());
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions options;
  options.uri = (dir / "review_bag").string(); options.storage_id = "sqlite3";
  writer.open(options);
  for (auto meta : reader.get_all_topics_and_types()) {
    require(meta.name.rfind(prefix + "/", 0) == 0, "Test bag namespace");
    meta.name.erase(0, prefix.size()); writer.create_topic(meta);
  }
  while (reader.has_next()) {
    auto record = reader.read_next(); record->topic_name.erase(0, prefix.size());
    writer.write(record);
  }
  writer.close();
}

void scenario(const std::string &name, const fs::path &root, mapping_trial::Motion motion) {
  const auto dir = root / (std::string(mapping_trial::pulse(motion).name) + "_" + name); fs::create_directory(dir);
  Fixture fixture;
  if (name == "wrong_yaw") fixture.yaw_scale = -1;
  if (name == "yaw_limit") fixture.yaw_scale = 1.4;
  if (name == "translation_limit") fixture.translation_drift = .035;
  if (name == "insufficient_yaw") fixture.yaw_scale = .1;
  if (name == "preflight_competitor")
    fixture.competitor = fixture.wheel->create_publisher<Cmd>(prefix + "/cmd_vel", 10);
  auto trial = std::make_shared<MappingMotion>(dir, motion, prefix);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(fixture.wheel); executor.add_node(fixture.input); executor.add_node(fixture.slam);
  executor.add_node(trial);
  std::string failure;
  bool injected = false;
  try {
    while (!trial->done()) {
      fixture.pump(); executor.spin_some();
      if (!injected && fixture.moving() >= 5 && name != "normal") {
        injected = true;
        if (name == "stale_odom") fixture.drop_odom = true;
        if (name == "stale_scan") fixture.drop_scan = true;
        if (name == "input_fault") fixture.fault = true;
        if (name == "inactive_mapper") fixture.active = false;
        if (name == "competing_publisher")
          fixture.competitor = fixture.wheel->create_publisher<Cmd>(prefix + "/cmd_vel", 10);
        if (name == "schedule_stall") {
          const auto until = steady_seconds() + .18;
          while (steady_seconds() < until) {
            fixture.pump(); executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(2));
          }
        }
      }
      trial->tick(); std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  } catch (const std::exception &e) {
    failure = e.what(); trial->abort_zeros();
  }
  // DDS delivery can finish after publish returns. Drain without feeding stale data.
  const auto drain_until = steady_seconds() + 1.;
  do {
    executor.spin_some();
    if (!trial->attempted() || fixture.received.size() - fixture.moving() == (name == "insufficient_yaw" ? 10 : 5)) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (steady_seconds() < drain_until);
  const bool complete = name == "normal";
  require(complete ? failure.empty() : !failure.empty(), "Unexpected scenario result: " + name + ": " + failure);
  const std::map<std::string, std::string> expected{
    {"stale_odom", "fresh odometry"}, {"stale_scan", "fresh mapping scans"},
    {"input_fault", "Mapping status fault"}, {"inactive_mapper", "Mapper left active"},
    {"competing_publisher", "sole matched command ownership"}, {"schedule_stall", "Command schedule stalled"},
    {"preflight_competitor", "Preflight timeout waiting for sole matched command ownership"},
    {"wrong_yaw", "Odom rotation limit"}, {"yaw_limit", "Odom rotation limit"},
    {"translation_limit", "Odom rotation limit"}, {"insufficient_yaw", "Less than +0.100 rad"}};
  if (!complete) require(failure.find(expected.at(name)) != std::string::npos, "Wrong failure: " + failure);
  if (name == "preflight_competitor") require(fixture.received.empty(), "Preflight must publish no commands");
  trial->close(complete);
  verify_bag(dir, fixture, complete, motion, name == "insufficient_yaw");
  if (complete && motion == mapping_trial::Motion::rotation) export_review_bag(dir);
  std::cout << "PASS " << dir.filename() << " " << failure << '\n' << std::flush;
}
} // namespace

int main(int argc, char **argv) {
  try {
    require(std::getenv("ROS_DOMAIN_ID") && std::string(std::getenv("ROS_DOMAIN_ID")) == "193", "Tests require domain 193");
    require(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE") && std::string(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE")) == "LOCALHOST", "Tests require LOCALHOST");
    require(!std::getenv("ROS_STATIC_PEERS") || std::string(std::getenv("ROS_STATIC_PEERS")).empty(), "Test static peers must be empty");
    core_tests(); rclcpp::init(argc, argv);
    std::string pattern = "/tmp/mapping-motion-test.XXXXXX";
    char *made = mkdtemp(pattern.data()); require(made, "Test directory creation");
    const fs::path root = made; std::cout << "TEST DATA: " << root << '\n';
    for (auto motion : {mapping_trial::Motion::forward, mapping_trial::Motion::rotation})
      for (const auto *name : {"normal", "stale_odom", "stale_scan", "input_fault", "inactive_mapper", "competing_publisher", "schedule_stall", "preflight_competitor"})
        scenario(name, root, motion);
    for (const auto *name : {"wrong_yaw", "yaw_limit", "translation_limit", "insufficient_yaw"})
      scenario(name, root, mapping_trial::Motion::rotation);
    rclcpp::shutdown(); return 0;
  } catch (const std::exception &e) {
    std::cerr << "TEST FAILED: " << e.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
