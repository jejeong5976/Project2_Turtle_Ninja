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
  // ARM — task 는 interfaces/action/ArmTask 번호, slot 은 S1~S12 · HOME 의 "force" · LOOK 의 "rack" (빈 손 적재함 스캔)
  uint8_t task{0};
  std::string slot;
  double x{0.0}, y{0.0}, z{0.0};      // PICK · PLACE target, base_link [m]. z = 0 → control_master 박스 위 기본 높이
  double yaw_deg{NAN};                // NaN = 방향 모름 (자세 0,0,0,0)
  // PICK det · PICK tag: 손목 인지 검출로 집기 — det_tag 비면 퍽이 하나뿐일 때 그것, 아니면 그 퍽 태그 ID
  bool from_det{false};
  std::string det_tag;
  // PLACE 인자 없음 (pl): 마지막으로 성공한 PICK 목표에 다시 놓기 — 목표는 보낼 때 LastPick 으로 채움
  bool from_last{false};
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

// PICK · PLACE 목표 (base_link) 를 보낸 그대로: 위치 + 방향 (모르면 0,0,0,0). yaw_deg 는 표시용
struct ArmTarget
{
  double x{0}, y{0}, z{0};
  double qx{0}, qy{0}, qz{0}, qw{0};
  double yaw_deg{NAN};
};

// 인자 없는 place 가 놓을 자리 = 이 CLI 가 보낸 마지막 성공 PICK 의 목표 그대로 (RobotLink 가 팔 결과마다 알림)
//   그 뒤 PLACE · STORE 가 성공했으면 (퍽을 이미 놓음) 없음. 다른 작업 · 실패는 상관없음.
//   다른 노드가 보낸 팔 작업은 모름 (status 토픽에 작업 종류가 없음) · 한 줄 명령은 프로세스마다 새로 시작
class LastPick
{
public:
  void on_result(uint8_t task, const ArmTarget & t, bool ok);
  // 놓을 목표. 없으면 false + why (한 줄)
  bool place_target(ArmTarget & out, std::string & why) const;

private:
  bool have_{false};
  ArmTarget t_;
  std::string used_by_;   // PICK 뒤 성공한 PLACE · STORE (have_ 가 꺼진 까닭)
};

// Tab 자동 완성: 입력줄 → 후보 (마지막 낱말을 채울 것들, 명령 · 칸 · 정차 이름)
std::vector<std::string> complete_candidates(const std::string & line, const Names & names);

std::vector<std::string> split_words(const std::string & line);
// 실패하면 false + err (사람이 읽는 이유)
bool parse_command(const std::vector<std::string> & tok, const Names & names, Command & out, std::string & err);
const char * task_name(uint8_t task);

extern const char * const kHelp;

}  // namespace robot_cli
