#ifndef PROJECT2_TIMING_RECORDER_HPP
#define PROJECT2_TIMING_RECORDER_HPP
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <initializer_list>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <time.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace lidar_timing {
inline int64_t mono_ns() {
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC,&ts)!=0) throw std::runtime_error("CLOCK_MONOTONIC failed");
  return static_cast<int64_t>(ts.tv_sec)*1000000000LL+ts.tv_nsec;
}
using Fields = std::initializer_list<std::pair<const char*,int64_t>>;
class Recorder {
public:
  Recorder(const std::string& path, int scans, std::size_t byte_limit=64*1024*1024)
    : limit_(scans), byte_limit_(byte_limit) {
    if (path.empty() || scans<1 || scans>1000) throw std::invalid_argument("timing_jsonl required; timing_scans must be 1..1000");
    const int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
    if (fd<0) throw std::runtime_error("Cannot create new timing file: "+path+": "+std::strerror(errno));
    file_=fdopen(fd,"w");
    if (!file_) { close(fd); throw std::runtime_error("fdopen failed"); }
    event("metadata",{{"schema",1},{"requested_scans",scans},{"byte_limit",static_cast<int64_t>(byte_limit)}});
  }
  ~Recorder() {
    try { finish(false); } catch (const std::exception& e) { std::fprintf(stderr,"TIMING WRITE FAILED: %s\n",e.what()); }
    if (file_) std::fclose(file_);
  }
  Recorder(const Recorder&)=delete;
  Recorder& operator=(const Recorder&)=delete;
  void event(const char* kind, Fields fields) {
    std::lock_guard<std::mutex> guard(mutex_);
    append(kind,fields);
  }
  bool recording() {
    std::lock_guard<std::mutex> guard(mutex_);
    return !sealed_ && !overflow_;
  }
  void invalidate_clock() {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!sealed_) {
      clock_bad_=true;
      append("simulation_time_rejected",{});
    }
  }
  void pair(const char* kind,int64_t scan,int64_t before,int64_t ros,int64_t after) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (sealed_) return;
    // ROS clock offset is bracketed, not sampled atomically. Ignore reordered pairs
    // for detection; all records remain available for offline chronological analysis.
    if (after<before) clock_bad_=true;
    const int64_t low=ros-after, high=ros-before;
    if (have_pair_ && before>=last_after_ &&
        (low>last_high_+5000000LL || high<last_low_-5000000LL)) clock_bad_=true;
    if (!have_pair_ || after>last_after_) {
      last_low_=low; last_high_=high; last_after_=after; have_pair_=true;
    }
    append(kind,{{"scan_id",scan},{"mono_before_ns",before},{"ros_ns",ros},
      {"mono_after_ns",after},{"clock_bad",clock_bad_}});
  }
  bool published() {
    bool ready=false;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (sealed_) return false;
      ++published_;
      ready=published_>=limit_ || overflow_;
    }
    if (ready) finish(true);
    return ready;
  }
  void finish(bool requested_completion) {
    std::vector<std::string> records;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (sealed_) return;
      sealed_=true;
      records.swap(records_);
      std::ostringstream footer;
      footer << "{\"kind\":\"end\",\"published_scans\":" << published_
        << ",\"complete\":" << (requested_completion && published_>=limit_ && !overflow_ && !clock_bad_ ? 1:0)
        << ",\"overflow\":" << overflow_ << ",\"clock_bad\":" << clock_bad_ << "}\n";
      records.push_back(footer.str());
    }
    for (const auto& row:records) {
      if (std::fwrite(row.data(),1,row.size(),file_)!=row.size()) throw std::runtime_error("timing write failed");
    }
    if (std::fflush(file_)!=0) throw std::runtime_error("timing flush failed");
  }
private:
  void append(const char* kind,Fields fields) {
    if (sealed_ || overflow_) return;
    std::ostringstream stream;
    stream << "{\"kind\":\"" << kind << '"';
    for (const auto& field:fields) stream << ",\"" << field.first << "\":" << field.second;
    stream << "}\n";
    auto row=stream.str();
    if (records_.size()>=250000 || row.size()>byte_limit_-bytes_) { overflow_=true; return; }
    bytes_+=row.size(); records_.push_back(std::move(row));
  }
  std::mutex mutex_;
  FILE* file_=nullptr;
  int limit_,published_=0;
  std::size_t byte_limit_,bytes_=0;
  bool sealed_=false,overflow_=false,clock_bad_=false,have_pair_=false;
  int64_t last_low_=0,last_high_=0,last_after_=0;
  std::vector<std::string> records_;
};
} // namespace lidar_timing
#endif
