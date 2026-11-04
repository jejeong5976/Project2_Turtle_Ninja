// view_parts.hpp — 상태 글 조각 (status_view.cpp 에 있음). 0 쪽 요약과 1~7 쪽 (pages.cpp) 이 같이 씀
#pragma once

#include <string>
#include <vector>

#include "robot_cli/status_view.hpp"

namespace robot_cli
{
namespace parts
{

std::string fmt(const char * f, ...);
double deg(double rad);
double nz(double v);                                       // −0.000 → 0 (표 글)
std::string age(const Snapshot & s, double t);             // "0.3 s" · "없음"
Level age_level(const Snapshot & s, double t, Level ok = Level::NORMAL);
bool active(int8_t st);                                    // ACCEPTED · EXECUTING · CANCELING
bool any_active(const std::vector<GoalView> & goals);
std::string goals_text(const std::vector<GoalView> & goals);
bool has_node(const Snapshot & s, const std::string & n);
std::string pad(const std::string & s, int w);             // 오른쪽을 빈칸으로 채워 w 칸

using Lines = std::vector<Line>;
constexpr int kLabelW = 10;                                // 왼쪽 이름 칸 (이름은 9 칸까지 → 늘 한 칸 띄움)
constexpr int kBadgeW = 12;                                // 배지 칸
constexpr double kMapStaleS = 1.0;                         // TF map 위치가 이보다 오래되면 오래됨
constexpr double kPollStaleS = 3.0;                        // controller_manager 목록 (1 Hz 묻기) 답이 이보다 오래되면 모름

void head(Lines & L, const std::string & t);               // (앞에 글이 있으면 빈 줄) + 머리줄
void row(Lines & L, const std::string & label, const std::string & text, Level lv = Level::NORMAL);
void block(Lines & L, const std::string & label, const std::string & badge, Level lv, const std::string & text);

// 팔 자세 (이름 자세 · 받침 접힘), 팔 작업 기록
struct PoseView
{
  const NamedPose * pose{nullptr};   // 가장 가까운 이름 자세 (측정이 빠지면 없음)
  double diff{0.0};                  // 그 차이 [rad]
  bool near{false};                  // 그 자세에 있음 (≤ near_tol)
  bool fold{false};                  // 받침 접힘
  std::string worst;                 // 접힘에서 가장 벗어난 관절
  double worst_diff{0.0};
};
PoseView pose_view(const Snapshot & s, const Config & cfg);
const TaskTiming * running_task(const Snapshot & s);
const TaskTiming * last_done_task(const Snapshot & s);
std::string task_label(const TaskTiming & h);              // "STORE S12" · "다른 쪽 작업"
std::string result_word(int8_t status);                    // 성공 · 취소 · 실패

// 덩어리 배지 줄 (0 쪽, 각 쪽 맨 위)
void wheels_block(Lines & L, const Snapshot & s, const Config & cfg);
void arm_block(Lines & L, const Snapshot & s, const Config & cfg);
void limit_block(Lines & L, const Snapshot & s, const Config & cfg);       // 한계 가까울 때만
void battery_block(Lines & L, const Snapshot & s, const Config & cfg);
std::string grip_text(const Snapshot & s, const Config & cfg);
void manual_block(Lines & L, const Snapshot & s, bool always);
void loc_block(Lines & L, const Snapshot & s, const Config & cfg);
void nav_block(Lines & L, const Snapshot & s);
void percep_block(Lines & L, const Snapshot & s);
void mission_block(Lines & L, const Snapshot & s);
void hardware_block(Lines & L, const Snapshot & s);

// 표 (단위는 머리줄에만)
void joint_table(Lines & L, const Snapshot & s, const Config & cfg, bool wide);
void pose_table(Lines & L, const Snapshot & s);
void arm_timings(Lines & L, const Snapshot & s, std::size_t n);
void component_table(Lines & L, const Snapshot & s);
void diagnostics(Lines & L, const Snapshot & s);

// kMaxWidth 를 넘는 줄은 " · " 에서 나눠 다음 줄로 (이름 칸만큼 들여씀)
Lines wrap(const Lines & in, int max);

}  // namespace parts
}  // namespace robot_cli
