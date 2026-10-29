#pragma once

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace workbench {
inline void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}
inline double number(const std::string &s) {
  size_t end = 0;
  const double v = std::stod(s, &end);
  require(end == s.size() && std::isfinite(v), "Expected a finite number: " + s);
  return v;
}
inline std::string decimal(double v) {
  std::ostringstream out; out << std::setprecision(12) << v; return out.str();
}
struct Options {
  std::string hardware = "mock", drive_port, front_port, rear_port, output, script;
  int domain = 215, bins = 1600, neighbors = 0;
  double speed = .025, turn_rate = .10, max_linear = .04, max_angular = .15;
  double map_resolution = .05, filter_radius = .05, protect_radius = .30;
  double max_age = .25, max_skew = .075, max_gap = .04;
  double endpoint_error = .025, origin_shift = .005;
  double max_distance = .30, max_turn = 30., settle = .5, hold = 2., startup = 60.;
  double session_seconds = 1800., min_range = .05, max_range = 12.;
  bool execute = false, dry_run = false, rviz = false, help = false;

  void validate() const {
    require(hardware == "mock" || hardware == "opencr", "--hardware must be mock or opencr");
    require(domain >= 0 && domain <= 232, "--domain must be 0..232");
    require(bins >= 400 && bins <= 6400, "--scan-bins must be 400..6400 (output bins, not native resolution)");
    require(speed > 0 && speed <= max_linear && max_linear <= .10,
      "Require 0 < speed <= max-linear <= unchanged controller cap 0.10 m/s");
    require(turn_rate > 0 && turn_rate <= max_angular && max_angular <= .5,
      "Require 0 < turn-rate <= max-angular <= unchanged controller cap 0.5 rad/s");
    require(max_linear > 0 && max_angular > 0, "Motion limits must be positive");
    require(map_resolution >= .01 && map_resolution <= .10, "--map-resolution must be 0.01..0.10 m");
    require(neighbors >= 0 && neighbors <= 20 && filter_radius >= .005 && filter_radius <= .20 &&
      protect_radius >= 0 && protect_radius <= 2., "Invalid spatial filter settings");
    require(max_age >= .15 && max_age <= .5 && max_skew >= 0 && max_skew <= .10 && max_skew < max_age &&
      max_gap > 0 && max_gap <= .04, "Invalid freshness/skew/odometry-gap settings");
    require(endpoint_error > 0 && endpoint_error <= .025 && origin_shift > 0 && origin_shift <= .005,
      "Projection error bounds may be tightened but not enlarged beyond the baseline");
    require(min_range >= .05 && min_range < max_range && max_range <= 12., "Range must stay within 0.05..12 m");
    require(max_distance > 0 && max_distance <= 2. && max_turn > 0 && max_turn <= 180., "Invalid bounded segment limits");
    require(settle >= .3 && settle <= 5 && hold >= .5 && hold <= 30 && startup >= 5 && startup <= 180 &&
      session_seconds >= 10 && session_seconds <= 14400, "Invalid duration settings");
    if (hardware == "opencr") {
      require(script.empty(), "Scripts are mock-only; hardware motion requires interactive commands");
      require(execute || dry_run, "Real startup requires --execute; it configures hardware and enables wheel torque");
      for (const auto &p : {drive_port, front_port, rear_port})
        require(!p.empty() && std::filesystem::path(p).is_absolute(), "Real hardware requires three explicit absolute device paths");
      require(drive_port != front_port && drive_port != rear_port && front_port != rear_port, "Device paths must be distinct");
    } else {
      require(drive_port.empty() && front_port.empty() && rear_port.empty(), "Mock mode accepts no serial paths");
    }
  }
};
inline Options parse_options(const std::vector<std::string> &args) {
  Options o;
  std::map<std::string, double *> doubles{
    {"--speed", &o.speed}, {"--turn-rate", &o.turn_rate}, {"--max-linear", &o.max_linear},
    {"--max-angular", &o.max_angular}, {"--map-resolution", &o.map_resolution},
    {"--filter-radius", &o.filter_radius}, {"--protect-radius", &o.protect_radius},
    {"--max-age", &o.max_age}, {"--max-skew", &o.max_skew}, {"--max-odom-gap", &o.max_gap},
    {"--max-endpoint-error", &o.endpoint_error}, {"--max-origin-shift", &o.origin_shift},
    {"--max-distance", &o.max_distance}, {"--max-turn", &o.max_turn}, {"--settle", &o.settle},
    {"--hold", &o.hold}, {"--startup-timeout", &o.startup}, {"--session-seconds", &o.session_seconds},
    {"--min-range", &o.min_range}, {"--max-range", &o.max_range}};
  std::map<std::string, std::string *> strings{{"--hardware", &o.hardware}, {"--drive-port", &o.drive_port},
    {"--front-port", &o.front_port}, {"--rear-port", &o.rear_port}, {"--output", &o.output}, {"--script", &o.script}};
  for (size_t i = 0; i < args.size(); ++i) {
    const auto &key = args[i];
    if (key == "--help") {o.help = true; continue;}
    if (key == "--execute") {o.execute = true; continue;}
    if (key == "--dry-run") {o.dry_run = true; continue;}
    if (key == "--rviz") {o.rviz = true; continue;}
    require(i + 1 < args.size(), "Missing value for " + key);
    const auto value = args[++i];
    if (doubles.count(key)) *doubles.at(key) = number(value);
    else if (strings.count(key)) *strings.at(key) = value;
    else if (key == "--domain" || key == "--scan-bins" || key == "--filter-neighbors") {
      const auto v = number(value);
      require(v == std::floor(v) && v >= 0 && v <= 100000, "Expected bounded integer for " + key);
      if (key == "--domain") o.domain = static_cast<int>(v);
      else if (key == "--scan-bins") o.bins = static_cast<int>(v);
      else o.neighbors = static_cast<int>(v);
    } else throw std::runtime_error("Unknown option: " + key);
  }
  if (!o.help) o.validate();
  return o;
}
inline const char *help_text() {
  return R"(lidar_workbench [options]
Default: hardware-free mock wheels + synthetic dual LiDAR + real slam_toolbox.
  --hardware mock|opencr       --execute (required for real torque-enabled startup)
  --drive-port PATH            --front-port PATH          --rear-port PATH
  --output NEW_DIRECTORY       --domain 215               --dry-run       --rviz
  --speed 0.025                --turn-rate 0.10            (m/s, rad/s)
  --max-linear 0.04            --max-angular 0.15         (input admission limits)
  --scan-bins 1600             --map-resolution 0.05      (output bins, map metres)
  --min-range 0.05             --max-range 12
  --filter-neighbors 0         --filter-radius 0.05       --protect-radius 0.30
  --max-age 0.25               --max-skew 0.075           --max-odom-gap 0.04
  --max-endpoint-error 0.025    --max-origin-shift 0.005
  --max-distance 0.30          --max-turn 30              (metres, degrees per segment)
  --settle 0.5                 --hold 2                   --startup-timeout 60
  --session-seconds 1800       --script FILE (mock only)
Commands: forward M, backward M, left M, right M, ccw DEG, cw DEG;
          speed M/S, turn-rate RAD/S, status, stop, save NAME, quit.
Motion is bounded and accepted only at READY. No queued hardware movements.
Output bins do not create extra native returns. Timing remains nominal/delay=0.
Higher CLI speed limits are experimental; unchanged controller caps still apply.
Filtering affects merged diagnostic observations, never the front SLAM scan.
)";
}
struct Segment { double x = 0, y = 0, yaw = 0, target = 0, speed = 0; bool turning = false; };
inline Segment parse_segment(const std::string &line, const Options &o) {
  std::istringstream input(line); std::string name, value, extra;
  require(bool(input >> name >> value) && !(input >> extra), "Use DIRECTION positive-distance/angle");
  const double n = number(value); require(n > 0, "Segment target must be positive");
  Segment s;
  if (name == "forward") s.x = 1;
  else if (name == "backward") s.x = -1;
  else if (name == "left") s.y = 1;
  else if (name == "right") s.y = -1;
  else if (name == "ccw" || name == "cw") {s.turning = true; s.yaw = name == "ccw" ? 1 : -1;}
  else throw std::runtime_error("Unknown motion command");
  require(n <= (s.turning ? o.max_turn : o.max_distance), "Requested segment exceeds configured bound");
  s.target = s.turning ? n * std::acos(-1.) / 180. : n;
  s.speed = s.turning ? o.turn_rate : o.speed; return s;
}
}  // namespace workbench
