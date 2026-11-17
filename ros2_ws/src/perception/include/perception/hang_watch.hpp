// hang_watch.hpp — 멈춤 감시: 이름 붙은 하트비트 칸 + 감시 스레드 + 멈췄을 때 모든 스레드 스택을 파일로 (ROS 없음, 시험 대상)
//
// 이유: 스레드 하나가 끝없이 돌고 나머지가 대기하면 서비스 · 통계가 다 멈추는데 프로세스는 살아 있어 respawn 이 안 됨
//   → 멈춤을 감지해 스택을 남기고 프로세스를 끝냄
// 역할
//   - beat(i): 칸 i 가 살아 있음 (콜백 그룹마다 하트비트 타이머가 부름 → 그 그룹 콜백이 안 끝나면 타이머도 못 돎)
//   - 감시 스레드: 어느 칸이든 limit_s 넘게 안 뛰면 한 번만
//       1) ~/.ros/perception_hang_last.txt 에 기록 파일 이름 (스택 뜨기가 멈춰도 어디에 쓰려 했는지 남음)
//       2) dump_all_threads: 스레드마다 이름 · 상태 · CPU % (0.3 s 잼) · wchan · syscall · 신호로 뜬 스택 (모듈+오프셋 · 기호)
//          + /proc/self/maps + gdb 가 있으면 gdb -p 로 thread apply all bt (기호 · 인자, 프로세스 밖에서 봄)
//       3) on_hang (perception_node: stderr ERROR + _exit → launch respawn)
//   - 스택 뜨기가 아예 멈춰도 deadman_s 뒤 SIGALRM 기본 동작으로 프로세스가 끝남 (launch respawn)
// 받음: 칸 이름들, 설정. 보냄: 기록 파일 (~/.ros/perception_hang_YYYYmmdd_HHMMSS.txt), on_hang 호출
// note
//   - ptrace_scope 1 (Jetson): 자식 gdb 가 부모를 못 붙음 → 멈춘 뒤에만 prctl(PR_SET_PTRACER_ANY) (곧 끝나는 프로세스)
//   - gdb 가 붙으면 이 프로세스 전체가 잠깐 섬 → sh 가 gdb 끝난 뒤 (시간 초과로 죽여도) kill -CONT 로 다시 돌림
//   - 신호 스택: SIGRTMIN+7 을 스레드마다 tgkill, 처리기는 backtrace() 만 (시작 때 한 번 불러 libgcc 를 미리 읽음).
//       신호를 막은 스레드 · 커널 안에서 못 깨는 스레드는 "응답 없음" (gdb 쪽에는 나옴)
//   - 기호: dladdr = 내보낸 이름만. 실행 파일 안 static 함수는 모듈+오프셋 → addr2line -f -C -e <모듈> <오프셋-1>
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace perception
{

struct HangDumpOptions
{
  bool use_gdb = true;          // gdb 가 PATH 에 있으면 씀 (없으면 신호 스택만)
  double gdb_budget_s = 8.0;    // gdb 최대 시간, 넘으면 timeout 이 죽이고 sh 가 kill -CONT
  double cpu_sample_s = 0.3;    // 스레드별 CPU % 잴 시간
};

// ROS 로그 폴더: $ROS_LOG_DIR 가 아니라 $ROS_HOME (없으면 ~/.ros, HOME 도 없으면 /tmp)
std::string ros_home_dir();
// <dir>/<prefix>_YYYYmmdd_HHMMSS.txt
std::string hang_dump_path(const std::string & dir, const std::string & prefix);
// 지금 프로세스의 모든 스레드 상태 · 스택 → path (덮어씀). reason 은 머리글. 파일을 못 열면 false
bool dump_all_threads(const std::string & path, const std::string & reason, const HangDumpOptions & o = {});
// 신호 스택 처리기 설치 + backtrace 미리 부름 (dump_all_threads 가 알아서 부르지만, 시작 때 부르면 멈춘 뒤 할 일이 줄어듦)
void prepare_thread_dump();

class HangWatch
{
public:
  // what = 멈춘 칸 설명 ("wrist 6.1 s (wide 0.2 · …)"), age_s = 가장 오래된 칸 나이, dump_path = 기록 파일 ("" = 못 씀)
  using OnHang = std::function<void (const std::string & what, double age_s, const std::string & dump_path)>;

  struct Options
  {
    double limit_s = 5.0;            // 이보다 오래 안 뛴 칸이 있으면 멈춤 (<= 0 = 감시 안 함)
    double poll_s = 0.25;            // 감시 주기
    std::string dir;                 // 기록 폴더 ("" = ros_home_dir())
    std::string prefix = "perception_hang";
    HangDumpOptions dump;
    double deadman_s = 40.0;         // 멈춤을 본 뒤 이 시간 안에 on_hang 이 프로세스를 못 끝내면 SIGALRM 으로 끝냄 (0 = 안 함, 시험용)
    OnHang on_hang;                  // 비면 stderr 한 줄 + _exit(70)
  };

  HangWatch(std::vector<std::string> names, Options o);
  ~HangWatch();
  HangWatch(const HangWatch &) = delete;
  HangWatch & operator=(const HangWatch &) = delete;

  void beat(std::size_t i);
  void start();                      // 모든 칸을 지금으로 맞추고 감시 스레드 시작
  void stop();                       // 감시 스레드 끝 (on_hang 이 돌아온 뒤에도 부를 수 있음)
  void request_stop() {stop_ = true;}   // 기다리지 않고 멈춤 표시만 (rclcpp 끝내기 콜백에서: 끝내는 동안 하트비트가 서도 안 울림)
  bool fired() const {return fired_.load();}
  // 가장 오래 안 뛴 칸 번호 · 나이 [s], 모든 칸 나이 글 ("wide 0.2 s · wrist 6.1 s")
  std::size_t oldest(double & age_s, std::string * all = nullptr) const;
  const std::string & name(std::size_t i) const {return names_.at(i);}
  std::size_t size() const {return names_.size();}

  static int64_t mono_ns();

private:
  void run();

  std::vector<std::string> names_;
  Options o_;
  std::vector<std::atomic<int64_t>> beat_ns_;
  std::atomic<bool> stop_{false}, fired_{false};
  std::thread th_;
};

}  // namespace perception
