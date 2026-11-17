// hang_watch.cpp — 멈춤 감시 · 모든 스레드 스택 기록 (설명은 include/perception/hang_watch.hpp)
#include "perception/hang_watch.hpp"

#include <cxxabi.h>
#include <dirent.h>
#include <dlfcn.h>
#include <execinfo.h>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>

extern char ** environ;

namespace perception
{

namespace
{

constexpr int kMaxFrames = 64;

std::atomic<pid_t> g_target{0};
std::atomic<int> g_n{-1};
void * g_frames[kMaxFrames];
int g_sig = 0;
std::once_flag g_once;

pid_t tid_now() {return static_cast<pid_t>(::syscall(SYS_gettid));}

// 신호 처리기: 대상 스레드면 backtrace 만 (async-signal-safe 에 가깝게: libgcc 는 prepare 때 미리 읽음)
void bt_handler(int, siginfo_t *, void *)
{
  const int saved = errno;
  if (tid_now() == g_target.load(std::memory_order_acquire)) {
    const int n = ::backtrace(g_frames, kMaxFrames);
    g_n.store(n, std::memory_order_release);
  }
  errno = saved;
}

std::string read_file(const std::string & path, std::size_t max = 1 << 20)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {return "";}
  std::string s;
  s.resize(max);
  f.read(&s[0], static_cast<std::streamsize>(max));
  s.resize(static_cast<std::size_t>(f.gcount()));
  return s;
}

std::string trim(std::string s)
{
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ' || s.back() == '\0')) {s.pop_back();}
  return s;
}

std::vector<pid_t> list_tids()
{
  std::vector<pid_t> out;
  if (DIR * d = ::opendir("/proc/self/task")) {
    while (dirent * e = ::readdir(d)) {
      if (e->d_name[0] >= '0' && e->d_name[0] <= '9') {out.push_back(static_cast<pid_t>(std::atoi(e->d_name)));}
    }
    ::closedir(d);
  }
  std::sort(out.begin(), out.end());
  return out;
}

struct TaskStat
{
  std::string comm;
  char state = '?';
  uint64_t ticks = 0;   // utime + stime
  bool ok = false;
};

// /proc/self/task/<tid>/stat: "tid (comm) S ppid … utime(14) stime(15) …" — comm 에 괄호 · 빈칸이 있을 수 있어 마지막 ')' 뒤부터
TaskStat read_stat(pid_t tid)
{
  TaskStat t;
  const std::string s = read_file("/proc/self/task/" + std::to_string(tid) + "/stat", 4096);
  const auto a = s.find('('), b = s.rfind(')');
  if (a == std::string::npos || b == std::string::npos || b < a) {return t;}
  t.comm = s.substr(a + 1, b - a - 1);
  std::istringstream in(s.substr(b + 1));
  std::string f;
  std::vector<std::string> v;
  while (in >> f) {v.push_back(f);}
  // v[0] = state (필드 3), utime = 필드 14 → v[11], stime = 필드 15 → v[12]
  if (v.size() < 13) {return t;}
  t.state = v[0].empty() ? '?' : v[0][0];
  t.ticks = std::strtoull(v[11].c_str(), nullptr, 10) + std::strtoull(v[12].c_str(), nullptr, 10);
  t.ok = true;
  return t;
}

std::string demangle(const char * s)
{
  if (!s) {return "?";}
  int st = 0;
  char * d = abi::__cxa_demangle(s, nullptr, nullptr, &st);
  std::string out = (st == 0 && d) ? d : s;
  std::free(d);
  if (out.size() > 300) {out = out.substr(0, 300) + "…";}
  return out;
}

void write_frames(std::FILE * f, void * const * fr, int n)
{
  for (int i = 0; i < n; ++i) {
    Dl_info info{};
    const auto addr = reinterpret_cast<uintptr_t>(fr[i]);
    if (::dladdr(fr[i], &info) && info.dli_fname) {
      const auto base = reinterpret_cast<uintptr_t>(info.dli_fbase);
      std::string sym = "?";
      if (info.dli_sname) {
        sym = demangle(info.dli_sname) + "+0x" +
          [&] {char b[32]; std::snprintf(b, sizeof(b), "%lx", static_cast<unsigned long>(addr - reinterpret_cast<uintptr_t>(info.dli_saddr))); return std::string(b);}();
      }
      std::fprintf(f, "    #%02d 0x%016lx %s+0x%lx  %s\n", i, static_cast<unsigned long>(addr), info.dli_fname,
        static_cast<unsigned long>(addr - base), sym.c_str());
    } else {
      std::fprintf(f, "    #%02d 0x%016lx ?\n", i, static_cast<unsigned long>(addr));
    }
  }
}

std::string now_text()
{
  const auto t = std::chrono::system_clock::now();
  const std::time_t tt = std::chrono::system_clock::to_time_t(t);
  std::tm tm{};
  ::localtime_r(&tt, &tm);
  char b[64];
  std::strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", &tm);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count() % 1000;
  char o[80];
  std::snprintf(o, sizeof(o), "%s.%03lld", b, static_cast<long long>(ms));
  return o;
}

std::string find_in_path(const std::string & exe)
{
  const char * p = std::getenv("PATH");
  std::string path = p ? p : "/usr/bin:/bin";
  std::stringstream ss(path);
  std::string dir;
  while (std::getline(ss, dir, ':')) {
    if (dir.empty()) {continue;}
    const std::string full = dir + "/" + exe;
    if (::access(full.c_str(), X_OK) == 0) {return full;}
  }
  return "";
}

// gdb -p 로 모든 스레드 bt → fd 에 덧붙임. sh 가 gdb 뒤 (시간 초과로 죽어도) kill -CONT 로 이 프로세스를 다시 돌림
void run_gdb(int fd, const HangDumpOptions & o)
{
  const std::string gdb = find_in_path("gdb");
  if (gdb.empty()) {
    ::dprintf(fd, "\n== gdb 없음 (sudo apt install gdb 면 다음엔 기호 · 인자까지)\n");
    return;
  }
  const pid_t me = ::getpid();
  // ptrace_scope 1: 자식이 부모를 붙으려면 허락 필요. 이 프로세스는 곧 끝남
  ::prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0);
  char budget[32];
  std::snprintf(budget, sizeof(budget), "%.0f", std::max(1.0, o.gdb_budget_s));
  const std::string cmd = std::string("timeout -k 2 ") + budget + " '" + gdb + "' -p " + std::to_string(me) +
    " -batch -nx -iex 'set debuginfod enabled off' -ex 'set pagination off' -ex 'set print thread-events off'"
    " -ex 'info threads' -ex 'thread apply all bt 40' </dev/null; kill -CONT " + std::to_string(me);
  ::dprintf(fd, "\n== gdb (%s, 최대 %s s)\n", gdb.c_str(), budget);
  // 환경: debuginfod 끔 (인터넷에서 기호 받다 멈추지 않게)
  std::vector<std::string> env_s;
  for (char ** e = environ; e && *e; ++e) {
    if (std::strncmp(*e, "DEBUGINFOD_URLS=", 16) != 0) {env_s.emplace_back(*e);}
  }
  env_s.emplace_back("DEBUGINFOD_URLS=");
  std::vector<char *> envp;
  for (auto & s : env_s) {envp.push_back(&s[0]);}
  envp.push_back(nullptr);
  std::string a0 = "sh", a1 = "-c", a2 = cmd;
  char * argv[] = {&a0[0], &a1[0], &a2[0], nullptr};
  posix_spawn_file_actions_t fa;
  ::posix_spawn_file_actions_init(&fa);
  ::posix_spawn_file_actions_adddup2(&fa, fd, 1);
  ::posix_spawn_file_actions_adddup2(&fa, fd, 2);
  pid_t child = -1;
  const int rc = ::posix_spawn(&child, "/bin/sh", &fa, nullptr, argv, envp.data());
  ::posix_spawn_file_actions_destroy(&fa);
  if (rc != 0) {
    ::dprintf(fd, "gdb 못 띄움: %s\n", std::strerror(rc));
    return;
  }
  int st = 0;
  while (::waitpid(child, &st, 0) < 0 && errno == EINTR) {}
  ::dprintf(fd, "\n== gdb 끝 (sh 상태 %d)\n", WIFEXITED(st) ? WEXITSTATUS(st) : -1);
}

}  // namespace

std::string ros_home_dir()
{
  if (const char * r = std::getenv("ROS_HOME"); r && *r) {return r;}
  if (const char * h = std::getenv("HOME"); h && *h) {return std::string(h) + "/.ros";}
  return "/tmp";
}

std::string hang_dump_path(const std::string & dir, const std::string & prefix)
{
  const std::time_t tt = std::time(nullptr);
  std::tm tm{};
  ::localtime_r(&tt, &tm);
  char b[32];
  std::strftime(b, sizeof(b), "%Y%m%d_%H%M%S", &tm);
  return dir + "/" + prefix + "_" + b + ".txt";
}

void prepare_thread_dump()
{
  std::call_once(g_once, [] {
      void * warm[4];
      (void)::backtrace(warm, 4);   // libgcc_s 를 지금 읽음 (처리기 안에서 dlopen 안 하게)
      g_sig = SIGRTMIN + 7;
      struct sigaction sa{};
      sa.sa_sigaction = bt_handler;
      sa.sa_flags = SA_SIGINFO | SA_RESTART;
      sigemptyset(&sa.sa_mask);
      ::sigaction(g_sig, &sa, nullptr);
    });
}

bool dump_all_threads(const std::string & path, const std::string & reason, const HangDumpOptions & o)
{
  prepare_thread_dump();
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_APPEND | O_CLOEXEC, 0644);
  if (fd < 0) {return false;}
  std::FILE * f = ::fdopen(::dup(fd), "a");
  if (!f) {
    ::close(fd);
    return false;
  }
  const pid_t me = ::getpid(), self = tid_now();
  char exe[512] = {0};
  (void)!::readlink("/proc/self/exe", exe, sizeof(exe) - 1);
  std::fprintf(f, "# perception 멈춤 기록 %s\n# 이유: %s\n# pid %d · 실행 파일 %s · 기록한 스레드 %d\n", now_text().c_str(),
    reason.c_str(), me, exe, self);
  std::fprintf(f, "# 읽는 법: CPU %% 가 큰 (R) 스레드가 멈춘 곳. 아래 신호 스택의 '모듈+0x오프셋' 은\n"
    "#   addr2line -f -C -e <모듈> 0x<오프셋-1>   (실행 파일 안 static 함수 이름도 나옴), gdb 절에는 이름 · 인자\n");

  // 1) 스레드별 CPU: cpu_sample_s 동안 utime+stime 늘어난 만큼
  const auto tids = list_tids();
  std::map<pid_t, TaskStat> s0;
  for (auto t : tids) {s0[t] = read_stat(t);}
  std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.05, o.cpu_sample_s)));
  const double hz = static_cast<double>(::sysconf(_SC_CLK_TCK));
  std::fprintf(f, "\n== 스레드 %zu 개 (CPU %% = %.1f s 동안)\n", tids.size(), o.cpu_sample_s);
  std::map<pid_t, double> cpu;
  for (auto t : tids) {
    const auto a = s0[t], b = read_stat(t);
    const double pc = (a.ok && b.ok && hz > 0) ? 100.0 * static_cast<double>(b.ticks - a.ticks) / (hz * o.cpu_sample_s) : -1.0;
    cpu[t] = pc;
    const std::string base = "/proc/self/task/" + std::to_string(t);
    std::fprintf(f, "  tid %-7d %-16s %c  CPU %5.0f %%  wchan %-22s syscall %s\n", t, b.comm.c_str(), b.state, pc,
      trim(read_file(base + "/wchan", 256)).c_str(), trim(read_file(base + "/syscall", 256)).c_str());
  }
  std::fflush(f);

  // 2) 스레드마다 신호로 스택 (CPU 큰 스레드부터)
  std::vector<pid_t> order = tids;
  std::stable_sort(order.begin(), order.end(), [&](pid_t x, pid_t y) {return cpu[x] > cpu[y];});
  std::fprintf(f, "\n== 신호 스택 (CPU 큰 순)\n");
  for (auto t : order) {
    std::fprintf(f, "\n-- tid %d %s CPU %.0f %%%s\n", t, s0[t].comm.c_str(), cpu[t], t == self ? " (감시 스레드 = 기록 중)" : "");
    if (t == self) {
      void * fr[kMaxFrames];
      write_frames(f, fr, ::backtrace(fr, kMaxFrames));
      continue;
    }
    g_n.store(-1, std::memory_order_release);
    g_target.store(t, std::memory_order_release);
    if (::syscall(SYS_tgkill, me, t, g_sig) != 0) {
      std::fprintf(f, "    (신호 못 보냄: %s)\n", std::strerror(errno));
      continue;
    }
    int n = -1;
    for (int k = 0; k < 300 && (n = g_n.load(std::memory_order_acquire)) < 0; ++k) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    g_target.store(0, std::memory_order_release);
    if (n < 0) {
      std::fprintf(f, "    (0.3 s 응답 없음: 신호를 막은 스레드 · 커널 안에서 못 깸 → gdb 절 참고)\n");
      continue;
    }
    void * fr[kMaxFrames];
    std::memcpy(fr, g_frames, sizeof(void *) * static_cast<std::size_t>(n));
    write_frames(f, fr, n);
  }

  // 3) maps (오프셋 → 파일 확인용)
  std::fprintf(f, "\n== /proc/self/maps (실행 권한 줄만)\n");
  {
    std::istringstream maps(read_file("/proc/self/maps", 4 << 20));
    std::string line;
    while (std::getline(maps, line)) {
      const auto sp = line.find(' ');
      if (sp != std::string::npos && line.size() > sp + 3 && line[sp + 3] == 'x') {std::fprintf(f, "%s\n", line.c_str());}
    }
  }
  std::fflush(f);
  std::fclose(f);

  // 4) gdb (있으면)
  if (o.use_gdb) {run_gdb(fd, o);}
  ::dprintf(fd, "\n# 끝 %s\n", now_text().c_str());
  ::fsync(fd);
  ::close(fd);
  return true;
}

// ---------------------------------------------------------------- HangWatch

int64_t HangWatch::mono_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

HangWatch::HangWatch(std::vector<std::string> names, Options o)
: names_(std::move(names)), o_(std::move(o)), beat_ns_(names_.size())
{
  const int64_t now = mono_ns();
  for (auto & b : beat_ns_) {b.store(now);}
  prepare_thread_dump();
}

HangWatch::~HangWatch() {stop();}

void HangWatch::beat(std::size_t i)
{
  if (i < beat_ns_.size()) {beat_ns_[i].store(mono_ns(), std::memory_order_relaxed);}
}

void HangWatch::start()
{
  if (th_.joinable() || o_.limit_s <= 0.0 || names_.empty()) {return;}
  const int64_t now = mono_ns();
  for (auto & b : beat_ns_) {b.store(now);}
  stop_ = false;
  th_ = std::thread([this]() {run();});
}

void HangWatch::stop()
{
  stop_ = true;
  if (th_.joinable() && th_.get_id() != std::this_thread::get_id()) {th_.join();}
}

std::size_t HangWatch::oldest(double & age_s, std::string * all) const
{
  const int64_t now = mono_ns();
  std::size_t worst = 0;
  for (std::size_t i = 1; i < beat_ns_.size(); ++i) {
    if (beat_ns_[i].load() < beat_ns_[worst].load()) {worst = i;}
  }
  age_s = beat_ns_.empty() ? 0.0 : static_cast<double>(now - beat_ns_[worst].load()) * 1e-9;
  if (all) {
    all->clear();
    for (std::size_t i = 0; i < beat_ns_.size(); ++i) {
      char b[96];
      std::snprintf(b, sizeof(b), "%s%s %.1f s", i ? " · " : "", names_[i].c_str(), static_cast<double>(now - beat_ns_[i].load()) * 1e-9);
      *all += b;
    }
  }
  return worst;
}

void HangWatch::run()
{
  // 이 스레드는 SIGALRM (deadman) · 스택 신호를 받을 수 있어야 함
  sigset_t un;
  sigemptyset(&un);
  sigaddset(&un, SIGALRM);
  ::pthread_sigmask(SIG_UNBLOCK, &un, nullptr);
  while (!stop_) {
    std::this_thread::sleep_for(std::chrono::duration<double>(o_.poll_s));
    if (stop_) {break;}
    double age = 0.0;
    std::string all;
    const std::size_t w = oldest(age, &all);
    if (age <= o_.limit_s) {continue;}
    fired_ = true;
    if (o_.deadman_s > 0.0) {   // 스택 뜨기가 멈춰도 끝나게 (SIGALRM 기본 동작 = 끝냄 → launch respawn)
      ::signal(SIGALRM, SIG_DFL);
      ::alarm(static_cast<unsigned>(std::max(1.0, o_.deadman_s)));
    }
    std::size_t stale = 0;
    for (std::size_t i = 0; i < beat_ns_.size(); ++i) {
      if (static_cast<double>(mono_ns() - beat_ns_[i].load()) * 1e-9 > o_.limit_s) {++stale;}
    }
    const std::string what = names_[w] + (stale == beat_ns_.size() && stale > 1 ? " (모든 그룹 = executor 전체 멈춤)" : "") + " — " + all;
    const std::string dir = o_.dir.empty() ? ros_home_dir() : o_.dir;
    ::mkdir(dir.c_str(), 0755);
    const std::string path = hang_dump_path(dir, o_.prefix);
    {   // 어디에 쓰려 했는지 먼저 (다음 프로세스가 시작 때 알림)
      std::ofstream last(dir + "/" + o_.prefix + "_last.txt", std::ios::trunc);
      last << path << "\n" << what << "\n";
    }
    std::fprintf(stderr, "[ERROR] [perception]: 멈춤 감시: 하트비트 %.1f s 없음 (한계 %.1f s): %s → 모든 스레드 스택 기록 중 %s\n",
      age, o_.limit_s, what.c_str(), path.c_str());
    std::fflush(stderr);
    const bool ok = dump_all_threads(path, "하트비트 " + what, o_.dump);
    const std::string written = ok ? path : "";
    if (o_.on_hang) {
      o_.on_hang(what, age, written);
    } else {
      std::fprintf(stderr, "[ERROR] [perception]: 멈춤 기록 %s → 종료 (launch respawn)\n", ok ? path.c_str() : "(못 씀)");
      std::fflush(stderr);
      ::_exit(70);
    }
    if (o_.deadman_s > 0.0) {::alarm(0);}
    return;   // 한 번만
  }
}

}  // namespace perception
