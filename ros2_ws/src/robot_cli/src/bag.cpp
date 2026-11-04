// bag.cpp — ros2 bag record 띄우기 · 끝내기, 설명은 bag.hpp

#include "robot_cli/bag.hpp"

#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <thread>

extern char ** environ;

namespace robot_cli
{

const std::vector<std::string> kBagTopics = {
  "/joint_states", "/joint_states_corrected", "/cmd_vel", "/odom", "/scan", "/lidar_front/scan", "/lidar_rear/scan",
  "/tf", "/tf_static", "/arm_task/_action/status", "/arm_task/_action/feedback", "/control/motion_mode",
  "/control/motion_mode_ack", "/diagnostics"};

std::string next_free(const std::string & dir, const std::string & prefix, const std::string & ext)
{
  for (int i = 1; i < 10000; ++i) {
    char b[32];
    std::snprintf(b, sizeof(b), "%s_%03d", prefix.c_str(), i);
    const std::string p = dir + "/" + b;
    std::error_code ec;
    if (!std::filesystem::exists(p + ext, ec)) {return p;}
  }
  return dir + "/" + prefix + "_overflow";
}

std::string reserve_free(const std::string & dir, const std::string & prefix, const std::string & ext)
{
  for (int i = 1; i < 10000; ++i) {
    char b[32];
    std::snprintf(b, sizeof(b), "%s_%03d", prefix.c_str(), i);
    const std::string p = dir + "/" + b + ext;
    const int fd = ::open(p.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
    if (fd >= 0) {
      ::close(fd);
      return p;
    }
    if (errno != EEXIST) {return "";}
  }
  return "";
}

bool BagRecorder::start(const std::string & dir, std::string & text)
{
  if (running()) {text = "이미 기록 중: " + path_; return false;}
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {text = "폴더 " + dir + " 못 만듦 — " + ec.message(); return false;}
  const std::string out = next_free(dir, "bag", "");
  const std::string log = out + "_record.log";
  std::vector<std::string> args = {"ros2", "bag", "record", "-o", out, "--topics"};   // lyrical: 토픽은 --topics 뒤 (위치 인자 안 받음)
  args.insert(args.end(), kBagTopics.begin(), kBagTopics.end());
  std::vector<char *> argv;
  for (auto & a : args) {argv.push_back(a.data());}
  argv.push_back(nullptr);

  posix_spawn_file_actions_t fa;
  posix_spawnattr_t at;
  posix_spawn_file_actions_init(&fa);
  posix_spawnattr_init(&at);
  posix_spawn_file_actions_addopen(&fa, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
  posix_spawn_file_actions_addopen(&fa, STDOUT_FILENO, log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  posix_spawn_file_actions_adddup2(&fa, STDOUT_FILENO, STDERR_FILENO);
  posix_spawnattr_setpgroup(&at, 0);   // 자기 그룹: 터미널 Ctrl-C 가 바로 안 감 → stop() 이 정상 종료
  posix_spawnattr_setflags(&at, POSIX_SPAWN_SETPGROUP);
  pid_t pid = -1;
  const int r = posix_spawnp(&pid, "ros2", &fa, &at, argv.data(), environ);
  posix_spawn_file_actions_destroy(&fa);
  posix_spawnattr_destroy(&at);
  if (r != 0) {text = "ros2 bag record 를 못 띄움 (ros2 가 PATH 에 있나?)"; return false;}
  pid_ = pid;
  path_ = out;
  text = out;
  return true;
}

namespace
{
double steady_s()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

bool BagRecorder::running()
{
  if (pid_ <= 0) {return false;}
  int st = 0;
  const pid_t r = waitpid(pid_, &st, WNOHANG);
  if (r == pid_ || r < 0) {   // 끝남 (혼자 · 끝내기) — 토픽 오류 등은 *_record.log
    pid_ = -1;
    stop_since_ = -1.0;
    return false;
  }
  return true;
}

bool BagRecorder::request_stop()
{
  if (!running()) {return false;}
  if (stop_since_ < 0.0) {
    kill(-pid_, SIGINT);   // 그룹 전체 (ros2 → 기록 프로세스): 정상 종료 = metadata 씀
    stop_since_ = steady_s();
    sent_term_ = sent_kill_ = false;
  }
  return true;
}

bool BagRecorder::poll()
{
  if (!running()) {return true;}
  if (stop_since_ < 0.0) {return false;}
  const double a = steady_s() - stop_since_;
  if (a > 10.0 && !sent_term_) {kill(-pid_, SIGTERM); sent_term_ = true;}
  if (a > 13.0 && !sent_kill_) {kill(-pid_, SIGKILL); sent_kill_ = true;}
  return false;
}

bool BagRecorder::stop()
{
  if (!request_stop()) {return false;}
  while (!poll()) {std::this_thread::sleep_for(std::chrono::milliseconds(50));}
  return true;
}

}  // namespace robot_cli
