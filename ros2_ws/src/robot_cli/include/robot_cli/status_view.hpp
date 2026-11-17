// status_view.hpp — Snapshot → 상태 화면 글줄 (ROS 없이, test/test_status_view.cpp)
//   ncurses 화면과 `robot_cli status [쪽]` 가 같은 글을 씀
//   쪽: 0 전체 요약 · 1 위치·SLAM · 2 팔 · 3 주행·Nav2 · 4 인지 · 5 미션 · 6 진단 · 7 실험 (pages.cpp)
//   0 쪽 = 덩어리마다 한 줄 + 상태 배지 ([정상] 초록 · [주의] 노랑 · [문제] 빨강 · [꺼짐] 회색), 숫자는 1~7 쪽 표에
#pragma once

#include <map>
#include <string>
#include <vector>

#include "robot_cli/command.hpp"
#include "robot_cli/model.hpp"
#include "robot_cli/snapshot.hpp"

namespace robot_cli
{

enum class Level {NORMAL, HEAD, GOOD, WARN, BAD, OFF};   // OK · ERROR 는 ncurses 매크로라 피함. OFF = 회색 (꺼짐 · 없음)

struct Line
{
  std::string text;
  Level level{Level::NORMAL};
  // 배지 줄: text 의 [mark_from, mark_from + mark_len) 바이트만 level 색, 나머지는 보통 글
  std::size_t mark_from{0}, mark_len{0};
};

constexpr double kStaleS = 2.0;           // 이보다 오래된 값은 경고 색
constexpr double kModeFreshS = 0.5;       // swerve_controller 가 DRIVE 를 믿는 나이 (MotionMode.msg)

constexpr int kMaxWidth = 100;          // 쪽 글 한 줄 최대 칸 (넘으면 " · " 에서 다음 줄로)
constexpr int kPages = 8;
extern const char * const kPageNames[kPages];   // 메뉴 글: "전체 요약" · "위치·SLAM" …

// 화면 쪽 상태 (7 실험 쪽에 보임) — TUI 가 채움
struct UiState
{
  std::string mark_dir;                       // mark 를 쓰는 폴더
  std::vector<std::string> marks;             // 최근 mark 글 (새 것이 마지막)
  std::string last_mark;                      // 마지막 mark 이름
  std::map<std::string, double> last_mark_q;  // 그 관절값 (/joint_states) — 지금과 차이 보기
  std::string cycle;                          // 반복 시험 상태 글 (안 하면 "")
  std::vector<std::string> cycle_log;         // 반복 시험 기록 (새 것이 마지막)
  std::string session_file;                   // 이 화면의 명령 · 결과 기록 파일
  std::string bag;                            // ros2 bag 기록 상태 글 (안 하면 "")
  std::string last_note;                      // 마지막 note
  std::string go;                             // 접고 이동 상태 글 (안 하면 "")
};

// 팔 관절 이름 (표 순서)
extern const std::vector<std::string> kArmJoints;   // joint1~5, gripper_left_joint
// 있나 없나 보여줄 노드
extern const std::vector<std::string> kWatchNodes;

// 화면 칸 수 (한글 · 한자 = 2 칸, 나머지 1 칸) — 열 맞춤용
int text_width(const std::string & s);
// 0 쪽 (전체 요약)
std::vector<Line> status_lines(const Snapshot & s, const Config & cfg);
// page 0~7. 0 = status_lines
std::vector<Line> page_lines(int page, const Snapshot & s, const Config & cfg, const UiState & ui);
// drive · teleop 를 막는 이유 ("" = 됨). RobotLink 가 시작할 때와 보내는 틱마다 (잃으면 멈춤) 쓰고 3 쪽이 미리 보여 줌
struct DriveFacts
{
  double now{0};
  bool estop{false};
  double odom_t{-1}, clock_off{0}, mode_t{-1};
  uint8_t mode{0};
  bool nav2_up{false};
  bool nodes_known{false};   // 노드 목록에 로봇 (/controller_manager) 이 보임 — 아니면 Nav2 가 떠 있는지 모름
};
DriveFacts drive_facts(const Snapshot & s);
std::string drive_block_reason(const DriveFacts & f, bool force);
std::string drive_block_reason(const Snapshot & s, bool force);

// 시작 전 점검 (check): 항목 · 결과 (GOOD 통과 · WARN 주의 · BAD 실패 · OFF 건너뜀) · 내용
struct CheckItem
{
  std::string name;
  Level level{Level::OFF};
  std::string detail;
};
std::vector<CheckItem> preflight(const Snapshot & s, const Config & cfg, const std::string & domain);
bool preflight_ok(const std::vector<CheckItem> & items);   // 실패 (BAD) 가 없으면 true
std::vector<Line> check_lines(const std::vector<CheckItem> & items);

// 관절 한계 가까움 (URDF limit, kLimitWarnRad) — 관절마다 한 줄, worst = 가장 나쁜 수준 (NORMAL = 없음)
std::vector<std::string> limit_warnings(const Snapshot & s, const Config & cfg, Level & worst);

// go (접고 이동): 막는 이유 ("" = 됨 — E-STOP · 퍽 쥠 · 팔 작업 중), 이동 전에 home 이 필요한가 (DRIVE 가 아니거나 받침에 안 접힘)
std::string go_block_reason(const Snapshot & s, const Config & cfg);
bool go_needs_fold(const Snapshot & s, const Config & cfg);

// 쪽 번호 동작: 번호 → 명령 (ask 가 있으면 그것을 물어 답을 {} 자리에, confirm 이면 y 일 때만)
struct PageAction
{
  std::string label, command, ask;
  bool confirm{false};
};
std::vector<PageAction> page_actions(int page, const Config & cfg);
// 팔 작업 phase 시간 글 "release 8.1 · grasp 3.2 …" (끝 시각 t_end 까지)
std::string phase_text(const TaskTiming & h, double t_end);
// stops 명령 · loc 명령 글
std::vector<std::string> stops_text(const std::vector<NamedStop> & stops);
std::vector<std::string> loc_hint();
std::string goal_status_name(int8_t status);
std::string light_name(uint8_t state);
std::string mode_name(uint8_t mode);

}  // namespace robot_cli
