// test_hang_watch.cpp — 멈춤 감시 (hang_watch.hpp) · respawn 상태 되살리기 (restart_state.hpp) 시험. ROS 없음.
//   하트비트가 서는 칸 (끝없이 도는 가짜 콜백) → 한 번 on_hang + 기록 파일에 그 스레드 CPU · 스택 (함수 이름)
//   계속 뛰면 안 울림 · 모든 칸이 서면 executor 전체 멈춤으로 씀 · gdb 있으면 gdb 절도
#include <gtest/gtest.h>

#include <stdlib.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "perception/hang_watch.hpp"
#include "perception/restart_state.hpp"

using perception::HangWatch;

std::atomic<bool> g_release{false};
std::atomic<uint64_t> g_spins{0};

// 실물 멈춤 흉내: 콜백 하나가 사용자 공간에서 끝없이 돎 (-rdynamic → 기록에 이 이름이 나와야 함)
__attribute__((noinline)) void spin_forever_in_callback()
{
  while (!g_release.load(std::memory_order_relaxed)) {
    g_spins.fetch_add(1, std::memory_order_relaxed);
  }
}

namespace
{

std::string temp_dir()
{
  char t[] = "/tmp/hang_watch_test_XXXXXX";
  const char * d = ::mkdtemp(t);
  return d ? d : "/tmp";
}

std::string slurp(const std::string & p)
{
  std::ifstream f(p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

struct Fired
{
  std::mutex m;
  bool done = false;
  std::string what, path;
  double age = 0.0;
};

bool wait_for(const std::function<bool()> & ok, double s)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(s);
  while (std::chrono::steady_clock::now() < end) {
    if (ok()) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return ok();
}

}  // namespace

TEST(HangWatch, BlockedCallbackIsDumpedOnce)
{
  const std::string dir = temp_dir();
  Fired fired;
  int calls = 0;
  HangWatch::Options o;
  o.limit_s = 0.6;
  o.poll_s = 0.05;
  o.dir = dir;
  o.dump.use_gdb = false;
  o.dump.cpu_sample_s = 0.2;
  o.deadman_s = 0.0;   // 시험 프로세스를 끝내지 않게
  o.on_hang = [&](const std::string & what, double age, const std::string & path) {
      std::lock_guard<std::mutex> lk(fired.m);
      ++calls;
      fired.done = true;
      fired.what = what;
      fired.path = path;
      fired.age = age;
    };
  HangWatch w({"wide", "wrist_tags"}, o);
  std::atomic<bool> run{true};
  g_release = false;
  w.start();
  std::thread healthy([&] {while (run) {w.beat(0); std::this_thread::sleep_for(std::chrono::milliseconds(50));}});
  std::thread stuck([&] {w.beat(1); spin_forever_in_callback();});
  ASSERT_TRUE(wait_for([&] {std::lock_guard<std::mutex> lk(fired.m); return fired.done;}, 10.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));   // 한 번만인지
  g_release = true;
  run = false;
  stuck.join();
  healthy.join();
  w.stop();

  std::lock_guard<std::mutex> lk(fired.m);
  EXPECT_EQ(calls, 1);
  EXPECT_TRUE(w.fired());
  EXPECT_EQ(fired.what.rfind("wrist_tags", 0), 0u) << fired.what;
  EXPECT_GT(fired.age, 0.6);
  EXPECT_LT(fired.age, 2.0);
  ASSERT_FALSE(fired.path.empty());
  const std::string txt = slurp(fired.path);
  EXPECT_NE(txt.find("== 스레드"), std::string::npos);
  EXPECT_NE(txt.find("== 신호 스택"), std::string::npos);
  EXPECT_NE(txt.find("spin_forever_in_callback"), std::string::npos) << txt.substr(0, 4000);
  EXPECT_NE(txt.find("== /proc/self/maps"), std::string::npos);
  // 신호 스택은 CPU 큰 순 → 첫 스레드 = 도는 스레드
  const auto first = txt.find("-- tid", txt.find("== 신호 스택"));
  const auto second = txt.find("-- tid", first + 1);
  ASSERT_NE(first, std::string::npos);
  EXPECT_NE(txt.substr(first, second - first).find("spin_forever_in_callback"), std::string::npos) << txt.substr(first, 2000);
  // 다음 프로세스가 알릴 표시
  const std::string last = slurp(dir + "/perception_hang_last.txt");
  EXPECT_EQ(last.rfind(fired.path, 0), 0u) << last;
}

TEST(HangWatch, KeepsQuietWhileAllSlotsBeat)
{
  bool fired = false;
  HangWatch::Options o;
  o.limit_s = 0.5;
  o.poll_s = 0.05;
  o.dir = temp_dir();
  o.deadman_s = 0.0;
  o.on_hang = [&](const std::string &, double, const std::string &) {fired = true;};
  HangWatch w({"a", "b", "c"}, o);
  w.start();
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
  while (std::chrono::steady_clock::now() < end) {
    for (std::size_t i = 0; i < 3; ++i) {w.beat(i);}
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  w.stop();
  EXPECT_FALSE(fired);
  EXPECT_FALSE(w.fired());
  double age = 0.0;
  std::string all;
  w.oldest(age, &all);
  EXPECT_LT(age, 0.5);
  EXPECT_NE(all.find("b "), std::string::npos) << all;
}

TEST(HangWatch, AllSlotsStaleMeansWholeExecutor)
{
  std::atomic<bool> fired{false};
  std::string what;
  std::mutex m;
  HangWatch::Options o;
  o.limit_s = 0.3;
  o.poll_s = 0.05;
  o.dir = temp_dir();
  o.dump.use_gdb = false;
  o.dump.cpu_sample_s = 0.05;
  o.deadman_s = 0.0;
  o.on_hang = [&](const std::string & w, double, const std::string &) {
      std::lock_guard<std::mutex> lk(m);
      what = w;
      fired = true;
    };
  HangWatch w({"wide", "wrist", "admin"}, o);
  w.start();
  ASSERT_TRUE(wait_for([&] {return fired.load();}, 5.0));
  w.stop();
  std::lock_guard<std::mutex> lk(m);
  EXPECT_NE(what.find("executor 전체"), std::string::npos) << what;
}

TEST(HangWatch, DisabledWhenLimitZero)
{
  bool fired = false;
  HangWatch::Options o;
  o.limit_s = 0.0;
  o.on_hang = [&](const std::string &, double, const std::string &) {fired = true;};
  HangWatch w({"a"}, o);
  w.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  w.stop();
  EXPECT_FALSE(fired);
}

// gdb 가 있으면 gdb 절 (ptrace_scope 1 이어도 PR_SET_PTRACER_ANY 로 붙음) · 프로세스가 다시 돎 (kill -CONT)
TEST(HangDump, GdbSectionWhenAvailable)
{
  if (::system("command -v gdb >/dev/null 2>&1") != 0) {GTEST_SKIP() << "gdb 없음";}
  g_release = false;
  std::thread stuck([] {spin_forever_in_callback();});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const std::string path = temp_dir() + "/gdb_dump.txt";
  perception::HangDumpOptions o;
  o.use_gdb = true;
  o.gdb_budget_s = 30.0;
  o.cpu_sample_s = 0.1;
  ASSERT_TRUE(perception::dump_all_threads(path, "시험", o));
  const uint64_t before = g_spins.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_GT(g_spins.load(), before) << "gdb 뒤 프로세스가 다시 안 돎";
  g_release = true;
  stuck.join();
  const std::string txt = slurp(path);
  EXPECT_NE(txt.find("== gdb"), std::string::npos);
  EXPECT_NE(txt.find("== gdb 끝"), std::string::npos);
  if (txt.find("ptrace: Operation not permitted") == std::string::npos) {
    EXPECT_NE(txt.find("Thread "), std::string::npos) << txt.substr(txt.find("== gdb"), 3000);
  }
}

// ---------------------------------------------------------------- restart_state

TEST(RestartState, RoundTripAndSameLaunchOnly)
{
  perception::SavedState s;
  s.launch = perception::current_launch_mark();
  ASSERT_TRUE(s.launch.valid());
  EXPECT_GT(s.launch.ppid_start, 0u);
  s.enables["wide"] = false;
  s.enables["wrist_tags"] = true;
  s.enables["barcode"] = true;
  s.params["wrist.held.enable"] = perception::hex_encode("b:1");
  s.params["box.guess"] = perception::hex_encode("D:0.2935,0,0");
  const std::string path = temp_dir() + "/perception_state.txt";
  ASSERT_TRUE(perception::save_state_file(path, s));
  const auto r = perception::load_state_file(path);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->enables.size(), 3u);
  EXPECT_FALSE(r->enables.at("wide"));
  EXPECT_TRUE(r->enables.at("wrist_tags"));
  EXPECT_EQ(*perception::hex_decode(r->params.at("box.guess")), "D:0.2935,0,0");
  EXPECT_TRUE(perception::should_restore(r, perception::current_launch_mark()));   // 같은 부모 = respawn
  perception::LaunchMark other = s.launch;
  other.ppid_start += 1;   // 같은 pid 를 다시 쓴 다른 launch
  EXPECT_FALSE(perception::should_restore(r, other));
  other = s.launch;
  other.ppid += 1;
  EXPECT_FALSE(perception::should_restore(r, other));
  EXPECT_FALSE(perception::should_restore(std::nullopt, s.launch));
}

TEST(RestartState, BadFilesAreIgnored)
{
  EXPECT_FALSE(perception::decode_state("").has_value());
  EXPECT_FALSE(perception::decode_state("enable wide 1\n").has_value());   // launch 줄 없음
  EXPECT_FALSE(perception::load_state_file("/nonexistent/perception_state.txt").has_value());
  const auto s = perception::decode_state("launch 10 20\nenable wide 1\ngarbage\nparam a 6231\n");
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->launch.ppid, 10);
  EXPECT_TRUE(s->enables.at("wide"));
  EXPECT_EQ(*perception::hex_decode(s->params.at("a")), "b1");
}

TEST(RestartState, HexKeepsSpacesAndEmpty)
{
  for (const std::string & v : {std::string("s:a b\nc"), std::string(""), std::string("S:ff,00")}) {
    const auto e = perception::hex_encode(v);
    EXPECT_TRUE(perception::token_ok(e));
    EXPECT_EQ(*perception::hex_decode(e), v);
  }
  EXPECT_FALSE(perception::hex_decode("abc").has_value());
  EXPECT_FALSE(perception::hex_decode("zz").has_value());
}
