// lying_backoff.hpp — 누운 퍽이 차체에 너무 가까우면 그 자리에서 안 집고 곧게 뒤로 빠진 뒤 다시 LOOK · 계획
//
// 역할
//   - lying_backoff_choice: 이 정차에서 집을 퍽 중 누운 퍽 x < near_x_m 또는 선 퍽 x < up_near_x_m (중심 x, base_link, 손목 깊이)
//       측정을 못 믿는 퍽 (bad) 은 선 퍽도 near_x_m 기준 (LOOK 화면 아래쪽 = 차체 가까운 쪽은 깊이가 비거나 틀림)
//       → 뒤로 갈 거리 = target_x_m − (걸린 퍽 중 가장 가까운 x), max_m 로 자름. min_m 보다 짧으면 안 감
//   - straight_back_cmd: 시작 자세 (odom) 기준 곧게 뒤로 goal_m — 몸 속도 명령 (뒤 · 옆 어긋남 · 방향 바로잡기)
// 입력 · 출력
//   - 값만 (ROS 없음). 노드 (mission_manager) 가 실행기 손목 검출 · odom TF 를 넣고 cmd_vel_nav 로 냄
// note
//   - 누운 퍽이 x 0.16–0.17 m (차체 앞 끝 0.141) 면 끝 쥐기에서 그리퍼가 차체 · 박스에 걸림,
//     누운 퍽 쥐기 표의 가장 가까운 칸은 x 0.26 → 뒤로 빠져 다시 보는 쪽이 빠름

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace mission
{

struct LyingBackoffConfig
{
  double near_x_m = 0.20;      // 누운 퍽 중심 x 가 이보다 작으면 뒤로 (0 이하 = 안 봄)
  double up_near_x_m = 0.0;    // 선 퍽 (방향 모름 포함) 중심 x 가 이보다 작으면 뒤로 (0 이하 = 안 봄)
  double target_x_m = 0.25;    // 뒤로 간 뒤 그 퍽 x 목표 (쥐기 표 시작 0.26 근처)
  double max_m = 0.12;         // 뒤로 최대
  double min_m = 0.01;         // 이보다 짧으면 안 감
};

struct LyingCandidate          // 이 정차에서 집을 퍽 하나 (계획 순서)
{
  std::string id;
  double x = 0.0;              // 중심 x [m] (base_link)
  bool lying = false;          // 방향 있는 검출에서 누움 (false = 선 퍽 · 방향 모름)
  bool bad = false;            // 지금 측정을 못 믿음 (PICK 이 빈손 · 높이 틀림으로 실패했거나 앞 LOOK 자세에서 안 보임) → 선 퍽도 near_x_m 기준
};

struct LyingBackoffChoice
{
  std::string id;              // 걸린 퍽 중 가장 가까운 것
  double x = 0.0;
  bool lying = false;
  double back_m = 0.0;         // 뒤로 갈 거리
  bool bad = false;            // 측정을 못 믿어서 걸림 (LyingCandidate::bad)
};

// 누운 퍽 x < near_x_m · 선 퍽 x < up_near_x_m 중 가장 가까운 것 기준 뒤로 거리. 없으면 nullopt
std::optional<LyingBackoffChoice> lying_backoff_choice(const std::vector<LyingCandidate> & picks, const LyingBackoffConfig & cfg);

// ---- 곧게 뒤로 (odom 2D)
struct StraightBackConfig
{
  double v_max = 0.10;         // [m/s]
  double v_min = 0.03;         // 끝 가까이에서도 이만큼은 (바퀴 마찰)
  double k_lin = 1.5;          // 남은 거리 → 속도
  double k_lat = 1.5;          // 옆 어긋남 → 옆 속도 (스워브)
  double vy_max = 0.03;
  double k_yaw = 1.5;          // 방향 어긋남 → 회전
  double wz_max = 0.20;
  double tol_m = 0.004;        // 남은 거리 이만큼이면 끝
};

struct StraightBackCmd
{
  double vx = 0.0, vy = 0.0, wz = 0.0;   // 몸 속도 (base_link)
  double traveled = 0.0;                 // 뒤로 간 거리 (시작 방향 기준, 뒤 = +)
  double lateral = 0.0;                  // 옆 어긋남 (왼쪽 +)
  double dyaw = 0.0;                     // 방향 어긋남 [rad]
  double remaining = 0.0;
  bool done = false;
};

// 시작 (x0, y0, yaw0) 에서 goal_m 뒤로. 지금 (x, y, yaw) → 명령. 남은 거리 ≤ tol 이면 done (명령 0)
StraightBackCmd straight_back_cmd(double x0, double y0, double yaw0, double goal_m, double x, double y, double yaw,
  const StraightBackConfig & cfg);

// ---- 20 Hz 틱에서 할 일
enum class LyingPhase { WaitDrive, Disable, Move };   // DRIVE 기다림 → 영역 끄기 (답 기다림) → 곧게 뒤로
enum class LyingTick { None, Cleanup, Move };

// active = 뒤로 빼기 진행 중, interrupted = 미션이 RUNNING 아님 · 지금 단계가 이 뒤로 빼기 아님 · FAULT 대기
//   진행 중 끊기면 단계 상관없이 Cleanup (명령 0 · 영역 켬). 영역은 WaitDrive 끝에서 끄므로 Disable 에서 끊겨도 켜야 함
inline LyingTick lying_tick_action(bool active, LyingPhase phase, bool interrupted)
{
  if (!active) {return LyingTick::None;}
  if (interrupted) {return LyingTick::Cleanup;}
  return phase == LyingPhase::Move ? LyingTick::Move : LyingTick::None;
}

}  // namespace mission
