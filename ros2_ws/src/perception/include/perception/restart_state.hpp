// restart_state.hpp — perception respawn 때 켜기 상태 · 실행 중 바꾼 파라미터 되살리기 (ROS 없음, 시험 대상)
//
// 왜: 켜기 서비스 (~/wide · wrist · wrist_tags · box · barcode/enable) 를 부르는 쪽 (mission · planning 실행기 · control_master 태그) 은
//   "켰다고 앎 (known)" 이면 다시 안 보냄. 멈춤 감시로 perception 이 다시 뜨면 yaml 기본값 (wrist · box · wide 켬, 태그 · 바코드 끔) 으로
//   시작 → 부르는 쪽이 아는 값과 달라짐. 그래서 perception 이 스스로 마지막 상태를 파일에 두고, 같은 launch 안 respawn 이면 되살림.
// 규칙
//   - 파일 = ~/.ros/perception_state.txt. 켜기 서비스 · ros2 param set 때마다 통째로 다시 씀 (tmp → rename)
//   - 되살림 = 파일의 launch 표시 (부모 pid + 부모 시작 시각 /proc/<ppid>/stat 22 번째) 가 지금과 같을 때만
//       = 같은 ros2 launch 가 respawn 한 것. 새로 launch 하면 표시가 달라 yaml 그대로 (이전 실행 상태가 안 섞임)
//   - 파라미터 값은 글 그대로 보관 (형 변환은 노드가 함: perception_node.cpp)
// 형식 (한 줄 한 항목, 이름 · 값에 빈칸 없음 — 값 글은 노드가 16 진으로 바꿔 넣음)
//   launch <ppid> <ppid_start>
//   enable <이름> <0|1>
//   param <이름> <값 글>
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <sstream>
#include <string>

#include <unistd.h>

namespace perception
{

struct LaunchMark
{
  long ppid = -1;
  unsigned long long ppid_start = 0;   // 부모 시작 시각 (부팅 뒤 clock tick) — pid 재사용 구분
  bool operator==(const LaunchMark & o) const {return ppid == o.ppid && ppid_start == o.ppid_start;}
  bool valid() const {return ppid > 1;}
};

// /proc/<pid>/stat 의 starttime (22 번째 필드, comm 의 괄호 뒤부터 셈). 못 읽으면 0
inline unsigned long long proc_start_ticks(long pid)
{
  std::ifstream f("/proc/" + std::to_string(pid) + "/stat");
  std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  const auto b = s.rfind(')');
  if (b == std::string::npos) {return 0;}
  std::istringstream in(s.substr(b + 1));
  std::string x;
  for (int field = 3; in >> x; ++field) {
    if (field == 22) {return std::strtoull(x.c_str(), nullptr, 10);}
  }
  return 0;
}

inline LaunchMark current_launch_mark()
{
  LaunchMark m;
  m.ppid = static_cast<long>(::getppid());
  m.ppid_start = proc_start_ticks(m.ppid);
  return m;
}

struct SavedState
{
  LaunchMark launch;
  std::map<std::string, bool> enables;          // wide · wrist · wrist_tags · box · barcode
  std::map<std::string, std::string> params;    // 실행 중 바꾼 파라미터 (노드가 만든 값 글)
};

inline bool token_ok(const std::string & s)
{
  if (s.empty()) {return false;}
  for (char c : s) {
    if (c == ' ' || c == '\n' || c == '\t' || c == '\r') {return false;}
  }
  return true;
}

inline std::string encode_state(const SavedState & s)
{
  std::ostringstream o;
  o << "# perception 켜기 상태 · 실행 중 바꾼 파라미터 — 같은 launch 의 respawn 때만 되살림 (restart_state.hpp)\n";
  o << "launch " << s.launch.ppid << " " << s.launch.ppid_start << "\n";
  for (const auto & kv : s.enables) {
    if (token_ok(kv.first)) {o << "enable " << kv.first << " " << (kv.second ? 1 : 0) << "\n";}
  }
  for (const auto & kv : s.params) {
    if (token_ok(kv.first) && token_ok(kv.second)) {o << "param " << kv.first << " " << kv.second << "\n";}
  }
  return o.str();
}

inline std::optional<SavedState> decode_state(const std::string & text)
{
  SavedState s;
  bool have_launch = false;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream l(line);
    std::string kind, name, value;
    l >> kind;
    if (kind == "launch") {
      if (l >> s.launch.ppid >> s.launch.ppid_start) {have_launch = true;}
    } else if (kind == "enable" && (l >> name >> value)) {
      s.enables[name] = value == "1";
    } else if (kind == "param" && (l >> name >> value)) {
      s.params[name] = value;
    }
  }
  if (!have_launch) {return std::nullopt;}
  return s;
}

// tmp 에 쓰고 rename (반쯤 쓴 파일을 다음 프로세스가 읽지 않게). tmp 는 pid 마다 따로. 한 프로세스 안에서는 부르는 쪽이 한 번에 하나만
inline bool save_state_file(const std::string & path, const SavedState & s)
{
  const std::string tmp = path + ".tmp." + std::to_string(static_cast<long>(::getpid()));
  {
    std::ofstream f(tmp, std::ios::trunc);
    if (!f) {return false;}
    f << encode_state(s);
    if (!f.good()) {return false;}
  }
  return std::rename(tmp.c_str(), path.c_str()) == 0;
}

inline std::optional<SavedState> load_state_file(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {return std::nullopt;}
  std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  return decode_state(text);
}

// 되살릴지: 같은 launch (respawn) 일 때만
inline bool should_restore(const std::optional<SavedState> & s, const LaunchMark & now)
{
  return s && now.valid() && s->launch == now;
}

// 16 진 글 (파라미터 값 글에 빈칸 · 줄바꿈이 있어도 한 낱말로)
inline std::string hex_encode(const std::string & s)
{
  static const char * d = "0123456789abcdef";
  std::string o;
  o.reserve(s.size() * 2);
  for (unsigned char c : s) {
    o += d[c >> 4];
    o += d[c & 15];
  }
  return o.empty() ? "-" : o;
}

inline std::optional<std::string> hex_decode(const std::string & h)
{
  if (h == "-") {return std::string();}
  if (h.size() % 2) {return std::nullopt;}
  auto v = [](char c) -> int {
      if (c >= '0' && c <= '9') {return c - '0';}
      if (c >= 'a' && c <= 'f') {return c - 'a' + 10;}
      return -1;
    };
  std::string o;
  for (std::size_t i = 0; i < h.size(); i += 2) {
    const int a = v(h[i]), b = v(h[i + 1]);
    if (a < 0 || b < 0) {return std::nullopt;}
    o += static_cast<char>(a * 16 + b);
  }
  return o;
}

}  // namespace perception
