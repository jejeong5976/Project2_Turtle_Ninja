// model.cpp — 설정 파일 읽기 · 단위 바꾸기, 설명은 model.hpp

#include "robot_cli/model.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <regex>

#include <yaml-cpp/yaml.h>

namespace robot_cli
{

namespace
{
std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

bool read_pose(const YAML::Node & n, Pose2D & out)
{
  if (!n || !n.IsSequence() || n.size() != 3) {return false;}
  out = {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
  return true;
}
}  // namespace

double joint_to_tick(const JointCal & c, double q) {return q / c.scale / kTickRad + c.zero_tick;}
double tick_to_joint(const JointCal & c, double tick) {return (tick - c.zero_tick) * kTickRad * c.scale;}
double gripper_gap(double q) {return 2.0 * (0.0091 + q);}

double wrap_angle(double a) {return std::atan2(std::sin(a), std::cos(a));}

Pose2D arena_to_map(const Pose2D & T, const Pose2D & p)
{
  const double c = std::cos(T.yaw), s = std::sin(T.yaw);
  return {T.x + c * p.x - s * p.y, T.y + s * p.x + c * p.y, wrap_angle(T.yaw + p.yaw)};
}

Pose2D inverse_pose(const Pose2D & a)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {-(c * a.x + s * a.y), -(-s * a.x + c * a.y), wrap_angle(-a.yaw)};
}

bool load_hardware_map(const std::string & path, std::map<std::string, JointCal> & out, std::string & err)
{
  try {
    const YAML::Node motors = YAML::LoadFile(path)["motors"];
    if (!motors || !motors.IsMap()) {err = path + ": motors 없음"; return false;}
    out.clear();
    for (const auto & kv : motors) {
      JointCal c;
      c.zero_tick = kv.second["zero_tick"] ? kv.second["zero_tick"].as<int>() : 2048;
      c.scale = kv.second["scale"] ? kv.second["scale"].as<double>() : 1.0;
      out[kv.first.as<std::string>()] = c;
    }
    return true;
  } catch (const std::exception & e) {
    err = path + ": " + e.what();
    return false;
  }
}

bool load_stops(const std::string & path, std::vector<NamedStop> & out, std::string & err)
{
  try {
    const YAML::Node y = YAML::LoadFile(path);
    Pose2D start, barcode, T;
    if (!read_pose(y["start"], start)) {err = path + ": start 없음"; return false;}
    if (!read_pose(y["barcode_view"], barcode)) {err = path + ": barcode_view 없음"; return false;}
    const auto mfa = y["map_from_arena"];
    if (mfa && mfa.IsScalar() && mfa.as<std::string>() == "start") {
      T = inverse_pose(start);
    } else if (!read_pose(mfa, T)) {
      err = path + ": map_from_arena 는 start 또는 [x, y, yaw]";
      return false;
    }
    out.clear();
    out.push_back({"start", arena_to_map(T, start)});
    out.push_back({"barcode", arena_to_map(T, barcode)});
    struct Group {const char * key; const char * prefix; int n;};
    for (const Group g : {Group{"boxes", "B", 6}, Group{"places", "L", 3}}) {
      for (int i = 1; i <= g.n; ++i) {
        const std::string name = g.prefix + std::to_string(i);
        Pose2D st;
        if (!read_pose(y[g.key][name]["stop"], st)) {err = path + ": " + g.key + "." + name + ".stop 없음"; return false;}
        out.push_back({name, arena_to_map(T, st)});
      }
    }
    return true;
  } catch (const std::exception & e) {
    err = path + ": " + e.what();
    return false;
  }
}

bool load_slots(const std::string & path, std::vector<std::string> & out, std::string & err)
{
  try {
    const YAML::Node cells = YAML::LoadFile(path)["rack_cells"]["cells"];
    if (!cells || !cells.IsMap()) {err = path + ": rack_cells.cells 없음"; return false;}
    out.clear();
    for (const auto & kv : cells) {out.push_back(kv.first.as<std::string>());}
    return true;
  } catch (const std::exception & e) {
    err = path + ": " + e.what();
    return false;
  }
}

bool load_arm_ref(const std::string & control_yaml, const std::string & arm_paths_yaml, ArmRef & out, std::string & err)
{
  auto path_q = [](const YAML::Node & path, bool first) {
      std::vector<double> q;
      if (!path || !path["q"] || !path["q"].IsSequence() || path["q"].size() < 5) {return q;}
      const std::size_t n = path["q"].size();
      const std::size_t from = first ? 0 : n - 5;
      for (std::size_t i = from; i < from + 5; ++i) {q.push_back(path["q"][i].as<double>());}
      return q;
    };
  try {
    out = ArmRef{};
    const YAML::Node c = YAML::LoadFile(control_yaml)["control_master"]["ros__parameters"];
    if (!c) {err = control_yaml + ": control_master.ros__parameters 없음"; return false;}
    if (c["poses"] && c["poses"]["home"]) {out.poses.push_back({"받침", c["poses"]["home"].as<std::vector<double>>()});}
    out.fold_tol = c["fold_tolerance"] ? c["fold_tolerance"].as<std::vector<double>>() : std::vector<double>{0.05};
    if (c["path_start_tolerance"]) {out.near_tol = c["path_start_tolerance"].as<double>();}
    out.lazy_fold = c["lazy_fold"] && c["lazy_fold"].as<bool>();
    if (c["gripper_open"]) {out.grip_open = c["gripper_open"].as<double>();}
    if (c["gripper_close"]) {out.grip_close = c["gripper_close"].as<double>();}
    const YAML::Node a = YAML::LoadFile(arm_paths_yaml)["control_master"]["ros__parameters"];
    if (!a) {err = arm_paths_yaml + ": control_master.ros__parameters 없음"; return false;}
    const YAML::Node paths = a["arm_paths"];
    // P0 = fold 경로 첫 점 (lazy_fold 의 STORE · PLACE 끝), LOOK 류 = 그 경로 끝 점
    const auto p0 = path_q(paths["fold"], true);
    if (!p0.empty()) {out.poses.push_back({"P0", p0});}
    if (a["staging_pose"]) {out.poses.push_back({"기준점", a["staging_pose"].as<std::vector<double>>()});}
    const auto look = path_q(paths["look"], false);
    if (!look.empty()) {out.poses.push_back({"LOOK", look});}
    const auto lb = path_q(paths["look_barcode"], false);
    if (!lb.empty()) {out.poses.push_back({"LOOK_BARCODE", lb});}
    for (const auto & p : out.poses) {
      if (p.q.size() != 5) {err = "자세 " + p.name + " 관절 수가 5 가 아님"; return false;}
    }
    if (out.poses.empty() || out.poses.front().name != "받침") {err = control_yaml + ": poses.home 없음"; return false;}
    return true;
  } catch (const std::exception & e) {
    err = std::string("팔 자세 설정: ") + e.what();
    return false;
  }
}

const std::vector<std::string> kArmJoint5 = {"joint1", "joint2", "joint3", "joint4", "joint5"};

void load_battery_limits(const std::string & path, double & warn, double & stop)
{
  try {
    const YAML::Node y = YAML::LoadFile(path);
    if (y["battery_warn_v"]) {warn = y["battery_warn_v"].as<double>();}
    if (y["battery_stop_v"]) {stop = y["battery_stop_v"].as<double>();}
  } catch (const std::exception &) {}
}

bool load_drive_limits(const std::string & path, double & max_linear, double & max_angular)
{
  try {
    const YAML::Node p = YAML::LoadFile(path)["swerve_controller"]["ros__parameters"];
    if (!p || !p["max_linear"] || !p["max_angular"]) {return false;}
    const double lin = p["max_linear"].as<double>(), ang = p["max_angular"].as<double>();
    // 둘 다 맞는 값일 때만 함께 바꿈 (음수 · NaN · 엉뚱하게 큰 값이면 기본값 그대로 + 경고)
    if (!std::isfinite(lin) || !std::isfinite(ang) || lin <= 0.0 || ang <= 0.0 || lin > 1.0 || ang > 3.0) {return false;}
    max_linear = lin;
    max_angular = ang;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_battery_log(const std::string & msg, std::string & board, double & mean, double & min)
{
  static const std::regex stats(R"(^(\S+) 보드 안 .*배터리 ([0-9]+\.[0-9]) V \(최저 ([0-9]+\.[0-9])\))");
  static const std::regex low(R"(^(\S+) 배터리 낮음: 평균 ([0-9.]+) V · 최저 ([0-9.]+) V)");
  static const std::regex stop(R"(^(\S+) 배터리 ([0-9.]+) V \(정지 기준)");
  if (msg.find("배터리") == std::string::npos) {return false;}
  std::smatch m;
  try {
    if (std::regex_search(msg, m, stats) || std::regex_search(msg, m, low)) {
      board = m[1];
      mean = std::stod(m[2]);
      min = std::stod(m[3]);
      return true;
    }
    if (std::regex_search(msg, m, stop)) {
      board = m[1];
      mean = min = std::stod(m[2]);
      return true;
    }
  } catch (const std::exception &) {}
  return false;
}

bool gripper_holding(const ArmRef & ref, double q)
{
  return std::isfinite(q) && std::isfinite(ref.grip_open) && std::isfinite(ref.grip_close) &&
         q > ref.grip_close + 0.003 && q < ref.grip_open - 0.004;
}

namespace
{
// 관절 5 개 차이의 최댓값 (측정이 빠지면 -1)
double max_diff(const std::vector<double> & q, const std::map<std::string, double> & joints, std::size_t * worst)
{
  double m = 0.0;
  for (std::size_t i = 0; i < kArmJoint5.size() && i < q.size(); ++i) {
    const auto it = joints.find(kArmJoint5[i]);
    if (it == joints.end() || !std::isfinite(it->second)) {return -1.0;}
    const double d = std::fabs(it->second - q[i]);
    if (d > m) {
      m = d;
      if (worst) {*worst = i;}
    }
  }
  return m;
}
}  // namespace

const NamedPose * nearest_pose(const ArmRef & ref, const std::map<std::string, double> & joints, double * diff)
{
  const NamedPose * best = nullptr;
  double best_d = std::numeric_limits<double>::infinity();
  for (const auto & p : ref.poses) {
    const double d = max_diff(p.q, joints, nullptr);
    if (d < 0.0) {return nullptr;}
    if (d < best_d) {best_d = d; best = &p;}
  }
  if (diff) {*diff = best_d;}
  return best;
}

bool folded(const ArmRef & ref, const std::map<std::string, double> & joints, std::string * worst, double * worst_diff)
{
  if (ref.poses.empty() || ref.poses.front().q.size() != 5) {return false;}
  const auto & home = ref.poses.front().q;
  bool ok = true;
  double wd = -1.0;
  for (std::size_t i = 0; i < 5; ++i) {
    const auto it = joints.find(kArmJoint5[i]);
    if (it == joints.end() || !std::isfinite(it->second)) {
      if (worst) {*worst = kArmJoint5[i] + " 측정 없음";}
      return false;
    }
    const double tol = ref.fold_tol.empty() ? 0.05 : ref.fold_tol.size() == 1 ? ref.fold_tol[0] : ref.fold_tol[std::min(i, ref.fold_tol.size() - 1)];
    const double d = it->second - home[i];
    ok = ok && std::fabs(d) <= tol;
    if (std::fabs(d) - tol > wd) {
      wd = std::fabs(d) - tol;
      if (worst) {*worst = kArmJoint5[i];}
      if (worst_diff) {*worst_diff = d;}
    }
  }
  return ok;
}

const NamedStop * find_stop(const std::vector<NamedStop> & stops, const std::string & name)
{
  for (const auto & s : stops) {
    if (lower(s.name) == lower(name)) {return &s;}
  }
  return nullptr;
}

const NamedStop * nearest_stop(const std::vector<NamedStop> & stops, double x, double y, double * dist)
{
  const NamedStop * best = nullptr;
  double best_d = std::numeric_limits<double>::infinity();
  for (const auto & s : stops) {
    const double d = std::hypot(s.pose.x - x, s.pose.y - y);
    if (d < best_d) {best_d = d; best = &s;}
  }
  if (dist) {*dist = best_d;}
  return best;
}

}  // namespace robot_cli
