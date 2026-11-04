// command.hpp — 명령 한 줄 → Command (ROS 없이, test/test_command.cpp)
//   화면 안 입력줄과 한 줄 명령 (nt h · nt g B1 …) 이 같은 문법을 씀. 문법 · 짧은 별칭은 kHelp
#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_cli
{

enum class Cmd
{
  HELP, QUIT, STATUS, ARM, NAV, CANCEL, ESTOP, DRIVE, STOP, TELEOP, CALL, STOPS, LOC,
  PAGE, MARK, MARKDIR, CYCLE, TEACH,
  GO,       // 접고 이동: 팔이 받침에 접혀 있지 않으면 home 먼저, 끝나면 nav (퍽을 쥐었으면 거절)
  CHECK,    // 시작 전 점검
  NOTE,     // 메모 (name = 글)
  BAG       // ros2 bag 기록 (on = start)
};

// 수동 주행 한계 기본값 — 실제 값은 controllers.yaml swerve_controller max_linear · max_angular (Names 로 받음)
constexpr double kMaxDriveSecs = 10.0;
constexpr double kClockTolS = 0.25;   // 로봇과 시계 차이 한계 (swerve_controller 는 stamp 가 0.3 s 넘게 다른 /cmd_vel 을 버림)

struct Command
{
  Cmd cmd{Cmd::HELP};
  // ARM — task 는 interfaces/action/ArmTask 번호, slot 은 S1~S12 또는 HOME 의 "force"
  uint8_t task{0};
  std::string slot;
  double x{0.0}, y{0.0}, z{0.0};      // PICK · PLACE target, base_link [m]. z = 0 → control_master 박스 위 기본 높이
  double yaw_deg{NAN};                // NaN = 방향 모름 (자세 0,0,0,0)
  // NAV — stop 이름이 있으면 그것, 없으면 map 기준 x y yaw
  std::string stop;
  double yaw_nav_deg{0.0};
  // CANCEL: arm | nav | all
  std::string which{"all"};
  // ESTOP on | off
  bool on{true};
  // DRIVE
  double vx{0.0}, vy{0.0}, wz{0.0}, secs{1.0};
  bool force{false};
  // CALL — setbool: -1 = std_srvs/Trigger, 0 · 1 = std_srvs/SetBool data
  std::string service;
  int setbool{-1};
  // PAGE · STATUS — 화면 쪽 0~7
  int page{0};
  // MARK 이름 · MARKDIR 폴더
  std::string name;
  // CYCLE — slot 칸, count 횟수 (0 = cycle stop), store_first = STORE 먼저
  int count{0};
  bool store_first{false};
};

struct Names
{
  std::vector<std::string> stops;     // dice_map 정차 자세 이름
  std::vector<std::string> slots;     // rack_cells 칸 이름
  double max_linear{0.20}, max_angular{0.5};   // drive 한계 [m/s] · [rad/s]
};

// Tab 자동 완성: 입력줄 → 후보 (마지막 낱말을 채울 것들, 명령 · 칸 · 정차 이름)
std::vector<std::string> complete_candidates(const std::string & line, const Names & names);

std::vector<std::string> split_words(const std::string & line);
// 실패하면 false + err (사람이 읽는 이유)
bool parse_command(const std::vector<std::string> & tok, const Names & names, Command & out, std::string & err);
const char * task_name(uint8_t task);

extern const char * const kHelp;

}  // namespace robot_cli
