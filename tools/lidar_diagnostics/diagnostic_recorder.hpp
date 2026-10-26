// Project2 diagnostic instrumentation; does not change upstream scan conversion.
#ifndef PROJECT2_DIAGNOSTIC_RECORDER_HPP
#define PROJECT2_DIAGNOSTIC_RECORDER_HPP
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>
#include "coin_d4_driver/lidar_sdk/lidar_information.hpp"

namespace robotis { namespace coin_d4 {
class DiagnosticRecorder {
public:
  DiagnosticRecorder(const std::string& path, int limit) : limit_(limit) {
    if (path.empty() || limit < 1 || limit > 1000) {
      throw std::invalid_argument("diagnostic_csv must be nonempty; diagnostic_scans must be 1..1000");
    }
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (fd < 0) throw std::runtime_error("Cannot create diagnostic CSV (must not exist): " +
      path + ": " + std::strerror(errno));
    file_ = ::fdopen(fd, "w");
    if (!file_) { ::close(fd); throw std::runtime_error("fdopen failed for diagnostic CSV"); }
    captures_.reserve(static_cast<size_t>(limit));
  }
  ~DiagnosticRecorder() {
    // Preserve a partial capture on normal Ctrl+C shutdown. Never throw from destruction.
    if (file_) {
      try { write(); } catch (const std::exception& e) {
        std::fprintf(stderr, "Diagnostic CSV write failed: %s\n", e.what());
      }
      std::fclose(file_);
    }
  }
  DiagnosticRecorder(const DiagnosticRecorder&) = delete;
  DiagnosticRecorder& operator=(const DiagnosticRecorder&) = delete;
  bool wants_scan() const { return !written_ && captures_.size() < static_cast<size_t>(limit_); }
  static int64_t steady_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void capture(std::vector<node_info> raw, uint64_t start, uint64_t end,
      int64_t snapshot_ns, const LaserScan& converted, bool reverse) {
    if (!wants_scan()) return;
    Capture c;
    c.raw = std::move(raw); c.start = start; c.end = end;
    c.snapshot_ns = snapshot_ns; c.reverse = reverse;
    const auto& cfg = converted.config;
    c.config = {cfg.min_angle, cfg.max_angle, cfg.angle_increment, cfg.scan_time,
      cfg.time_increment, cfg.min_range, cfg.max_range};
    // Copy only fields initialized by the upstream converter (range_check is unused/uninitialized).
    for (const auto& point : converted.points) c.ranges.push_back(point.range);
    captures_.push_back(std::move(c));
  }
  // Called immediately after the normal publisher stamps the scan, before publishing it.
  void publication(int64_t ros_ns) {
    if (!captures_.empty() && captures_.back().publish_ns == -1) {
      captures_.back().publish_ns = ros_ns;
    }
  }
  // Called after ROS publication. Returns true only on completion of the full requested capture.
  bool flush_if_complete() {
    if (!written_ && captures_.size() == static_cast<size_t>(limit_)) {
      write(); return true;
    }
    return false;
  }
private:
  struct Capture {
    std::vector<node_info> raw;
    std::vector<float> ranges;
    struct Config {
      float min_angle, max_angle, angle_increment, scan_time, time_increment, min_range, max_range;
    } config{};
    uint64_t start = 0, end = 0;
    int64_t snapshot_ns = 0, publish_ns = -1;
    bool reverse = false;
  };
  static double angle(const node_info& n) {
    return (n.angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0;
  }
  void write() {
    if (written_) return;
    std::ostringstream out;
    out.imbue(std::locale::classic()); out << std::setprecision(17);
    out << "scan_id,raw_count,published_count,raw_index,sync_flag,raw_angle_deg,raw_distance_mm,"
      "raw_intensity,exposure_flag,conversion_angle_source_index,conversion_angle_deg,"
      "published_index,published_angle_rad,published_range_m,reverse,"
      "angle_min,angle_max,angle_increment,scan_time,time_increment,range_min,range_max,"
      "internal_start_ns,internal_end_ns,snapshot_steady_ns,publish_ros_ns\n";
    for (size_t s = 0; s < captures_.size(); ++s) {
      const auto& c = captures_[s];
      for (size_t i = 0; i < c.raw.size(); ++i) {
        const auto& n = c.raw[i];
        // Match the existing loop exactly; no attempt to correct its mapping.
        const bool emitted = i > 0 && c.ranges.size() == c.raw.size() - 1;
        const int angle_source = emitted ? static_cast<int>(c.raw.size() - i) : -1;
        const int index = emitted ? static_cast<int>(c.reverse ? i - 1 : c.raw.size() - 1 - i) : -1;
        out << s << ',' << c.raw.size() << ',' << c.ranges.size() << ',' << i << ','
          << static_cast<unsigned>(n.sync_flag) << ',' << angle(n) << ',' << n.distance_q2 << ','
          << n.sync_quality << ',' << static_cast<unsigned>(n.exp_m) << ',' << angle_source << ',';
        if (emitted) out << angle(c.raw[angle_source]);
        out << ',' << index << ',';
        if (emitted) out << c.config.min_angle + index * c.config.angle_increment;
        out << ',';
        if (emitted) out << c.ranges[c.raw.size() - 1 - i];
        out << ',' << c.reverse << ',' << c.config.min_angle << ',' << c.config.max_angle << ','
          << c.config.angle_increment << ',' << c.config.scan_time << ',' << c.config.time_increment << ','
          << c.config.min_range << ',' << c.config.max_range << ',' << c.start << ',' << c.end << ','
          << c.snapshot_ns << ',' << c.publish_ns << '\n';
      }
    }
    const std::string data = out.str();
    // Mark before I/O so a failed write is reported rather than duplicated on destruction.
    written_ = true;
    if (std::fwrite(data.data(), 1, data.size(), file_) != data.size() || std::fflush(file_) != 0) {
      throw std::runtime_error("Failed to write diagnostic CSV");
    }
  }
  int limit_;
  FILE* file_ = nullptr;
  bool written_ = false;
  std::vector<Capture> captures_;
};
}}  // namespace robotis::coin_d4
#endif
