// arm_path.hpp — 실물에서 검사 · 실행한 팔 경로를 동작 목록으로 (순수 C++, test/test_arm_path.cpp)
// 경로 = 경유점 목록 (config/arm_paths.yaml, tools/arm 의 생성기가 계획기 · 안전 검사 결과로 만듦)
//   경유점 하나 = 관절 5 개 명령값 [rad] + 받침 구간인가 + 그 점으로 가는 최고 속도 + 도착 뒤 그리퍼 동작 + 멈춤
// 시간표 = 실물 실행기(arm_pose_runner)와 같은 규칙
//   - 경로는 경유점을 곧게 잇는 꺾은선 그대로, 길이 = 가장 많이 움직이는 관절의 각도 [°]
//   - 구간 최고 속도: 받침 구간 stand_speed, 그 밖 speed, 경유점에 speed 가 있으면 그보다 작게
//   - 꺾이는 곳 속도 × cos²(꺾인 각 / 2), 가속 · 감속 accel, 처음 · 끝 0
//   - dt 마다 점 하나 (JTC 가 점 사이를 선형 보간)
// 동작 나누기: 그리퍼 동작이나 멈춤이 있는 경유점에서 끊음 → 팔 동작(여러 점) · 그리퍼 동작 차례로
//   grip_on_way 인 시간 맞춘 점은 안 끊고 그 팔 동작의 side_grip_t (도착 시각) 로

#pragma once

#include <array>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "manipulation/arm.hpp"

namespace manipulation
{

struct PathNode
{
  std::string name;
  std::vector<double> q;     // 관절 5 개 명령값 [rad] (처짐 앞먹임 포함)
  bool stand{false};         // 받침 근처 세우기 · 복귀 구간 (느리게)
  double speed_dps{0.0};     // 이 점으로 가는 최고 속도 [°/s], 0 = 기본
  std::string grip;          // 도착 뒤 그리퍼: "" · open · release · close
  bool stop{false};          // 도착해서 멈춤 (동작을 여기서 끊음)
  double hold_s{0.0};        // 도착해서 이만큼 그 자세로 기다림 [s] (stop 과 같이 씀)
  double dt_s{0.0};          // > 0: 앞 점에서 이 점까지 정확히 이 시간 [s] (속도 · 가속 한계 없이 — 표를 녹화 간격 그대로 재생)
  bool grip_on_way{false};   // grip 을 팔을 멈추지 않고 (그 점 도착 시각에 그리퍼만, 팔은 계속) — 손에 퍽이 없을 때 · 세우기 여는 점
  bool grip_early{false};    // grip 을 이 점으로 출발할 때 (앞 점 도착 시각) 팔을 멈추지 않고 — PICK 퍽 위로 가며 미리 열기
};

struct ArmPath
{
  std::string name;
  std::vector<PathNode> nodes;
  bool ends_home{false};     // 마지막 점 = 받침(home) → 끝나면 접힘 실측 확인 뒤 주행 허용
};

struct PathTiming
{
  double speed_dps{40.0};
  double stand_speed_dps{20.0};
  double accel_dps2{150.0};
  double dt_s{0.05};
  double grip_settle_s{0.0};   // 그리퍼를 여닫기 전 같은 자세로 기다림 [s] (JTC 도착 판정 뒤에도 수 ° 남음)
};

struct GripperValues
{
  double open{0.0164};
  double release{0.0093};
  double close{-0.0016};
  double loose{0.0060};   // 퍽이 손가락 사이에서 돌 수 있게 살짝 풂 (경로 grip "loose"). 퍽에 닿은 자리 약 0.0046
};

// start 에서 nodes 를 차례로 멈추지 않고 지나가는 시간표 (start 는 넣지 않음). 길이가 0 이면 마지막 점 하나를 0.3 s 에
//   nodes 가 모두 dt_s > 0 이면 경유점마다 그 시간 간격 그대로 (녹화 재생 — 속도 상한으로 다시 짜면 세우기 성공률이 떨어짐)
//   출력 = dt 간격 점 + 경유점마다 그 도착 시각의 점 (JTC 가 점 사이를 직선으로 이어도 검사한 꺾은선 그대로)
std::vector<TrajPoint> time_path(
  const std::vector<double> & start, const std::vector<PathNode> & nodes, const PathTiming & timing);

// 경로 → 동작 목록. 첫 경유점 = 출발 자세 (지금 팔이 거기 있어야 함 — path_start_ok 로 확인)
//   last_pose: 마지막 팔 동작의 pose 이름 (ends_home 이면 "home" → ArmRunner 가 접힘 실측 확인)
//   q_now: 지금 실측 관절값. 주면 첫 구간을 첫 경유점 대신 여기서 출발해 계산 (허용 오차만큼 어긋난 시작도 같은 속도 · 가속 한계로)
std::vector<ArmAction> path_actions(
  const ArmPath & path, const PathTiming & timing, const GripperValues & grip,
  const std::vector<double> & q_now = {});

// 옮기기 점 (regrasp_transit_dps): 다시 쥐기 · 옮겨 놓기 · 먼 퍽 다시 쥐기에서 빈 공간 구간 끝점
//   (re_place_above · reloc_put_above · far_put_above · re_place_clear · reloc_put_clear · far_put_clear · re_above · far_re_above ·
//   re_lift · reloc_lift · far_re_lift). 퍽에 닿는 · 놓는 · 쥐는 점과 닿은 바로 뒤 올리기 (*_up) 는 아님
bool transit_node(const std::string & name);
// nodes 중 옮기기 점의 속도를 dps 로 (시간 맞춘 점 dt_s > 0 · 받침 점 · dps ≤ 0 은 그대로). 자세 · 그리퍼 · 멈춤은 안 바꿈. 바꾼 점 수
int set_transit_speed(std::vector<PathNode> & nodes, double dps);
// 동작 목록의 팔 동작 시간 합 [s] (그리퍼 · 논리 대기 (점 없음) 는 뺌) — 옮기기 속도 전후 비교
double arm_motion_time(const std::vector<ArmAction> & plan);

// 지금 관절값이 경로 첫 점에서 관절마다 tol 안인가 (NaN · 관절 수 다름 = 아님)
bool path_start_ok(const ArmPath & path, const std::vector<double> & q_now, double tol, std::string & why);

// 나중에 접기 (lazy_fold) 도우미. 바꿨으면 true
//   cut_fold_tail: 끝이 fold 경로와 같으면 P0 에서 끝냄 (ends_home = false)
//   skip_unfold: 앞이 unfold 경로와 같으면 P0 에서 시작
//   p0_distinct: P0 와 받침이 관절 하나라도 tol 넘게 다른가 (같으면 둘을 구분 못 함)
bool cut_fold_tail(ArmPath & path, const ArmPath & fold);
bool skip_unfold(ArmPath & path, const ArmPath & unfold);
bool p0_distinct(const ArmPath & fold, const ArmPath & unfold, double tol);

// 빈손 쥐기 뒤 온 길로 되돌아가기 (control_master 의 HOME · 실패 뒤 자동 접기): nodes[0..from] 을 거꾸로 지나되
//   같은 자세로 돌아온 고리 (내려가 쥐고 올라온 구간 · 옮겨 놓고 올라온 구간) 는 뺌 → 결과의 이웃 두 점은 모두 실제로 지나온 구간 (거꾸로)
//   구간 속도 · 받침 표시는 지나올 때 그 구간 그대로 (시간 맞춘 구간 = 표 재생은 그 평균 속도 = 각도 / dt_s), 그리퍼 · 멈춤 · 시간 맞춤은 뺌 (손은 빈 채 닫혀 있음)
//   결과 = [nodes[from], …, 끝점] — 끝점 = is_exit 가 true 인 첫 점 (검사된 복귀 경로가 있는 자세: P0 · LOOK 등)
//   그 전에 이름이 free_names 에 없는 점이 나오거나 끝점이 없으면 빈 목록 (모르는 구간은 거꾸로 안 감)
std::vector<PathNode> retrace_nodes(const std::vector<PathNode> & nodes, std::size_t from,
  const std::vector<std::string> & free_names, const std::function<bool(const PathNode &)> & is_exit,
  double same_tol = 1e-6);

// 다시 쥐기 (re_grasp) 빈손 되돌아가기 (control_master regrasp_empty_retreat): retrace_nodes 가 안 될 때
//   (지나온 길에 re_observe · far_lift 등) nodes[from] 이 re_up 이고 바로 앞이 re_grasp · re_down (= re_up 자세) · re_above 면
//   [re_up, re_above, exit] (그리퍼 · 멈춤 · 시간 맞춤 없이, re_up → re_above 는 지나온 re_above → re_down 속도). 아니면 빈 목록
//   re_above = 박스 위 다시 쥘 퍽 바로 위 수직 손, exit = 관찰 자세 (LOOK) — 박스 PICK 의 LOOK → 퍽 위 와 같은 구간 (거꾸로)
std::vector<PathNode> regrasp_retreat_nodes(const std::vector<PathNode> & nodes, std::size_t from, const PathNode & exit);

// 받침 세우기 쥐기 빈손 되돌아가기 (control_master regrasp_empty_retreat): 지나온 길에 표 재생 (lp*) · 옮겨 놓기 점이 있어
//   retrace_nodes 가 안 될 때. 받침 세우기 경로 (park_ 점이 있음) 에서만: nodes[from] 이 up 이고 앞이 grasp · down (= up 자세) · above 이거나
//   reloc_up · reloc_grasp · reloc_down · reloc_above (다시 계획한 옮기기 쥐기) 면 regrasp_retreat_nodes 와 같은 모양 [up, above, exit].
//   above = 누운 퍽 바로 위, exit = 관찰 자세 (LOOK) — 보통 PICK 의 LOOK → 퍽 위 와 같은 구간 (거꾸로). 아니면 빈 목록
std::vector<PathNode> park_retreat_nodes(const std::vector<PathNode> & nodes, std::size_t from, const PathNode & exit);

// 도중 정지 뒤 HOME (control_master home_retreat): 경로 작업이 도중에 멈추면 (FAULT · 시간 초과 · 취소) 실제로 보낸 팔 궤적 (명령값) 을
//   거꾸로 가서 검사된 복귀 경로가 있는 자세 (P0 · LOOK · 받침 근처) 에 닿은 뒤 그 경로로 접음. 새 직선은 없음 (지나온 구간만, 고리는 뺌)
// trace_append: 지나온 점 하나 더함. 앞 두 점과 일직선 (가운데 점이 양 끝을 잇는 구간에서 관절마다 line_tol 안) 이면 가운데를 지움 (궤적의 dt 점 줄이기),
//   앞 점과 거의 같으면 (1e-4 rad) 안 더함. 관절 수가 다르거나 NaN 이면 안 더함. max_points 넘으면 가장 오래된 점부터 지움
void trace_append(std::vector<std::vector<double>> & trace, const std::vector<double> & q, double line_tol = 1e-3,
  std::size_t max_points = 4000, std::size_t keep_before = 0);
// 점 q 에서 구간 a–b 까지 거리 (관절 공간, 관절마다 차이 중 가장 큰 값) [rad]
double segment_distance(const std::vector<double> & q, const std::vector<double> & a, const std::vector<double> & b);
// 되돌아가기 경로: 지금 자세가 있는 지나온 구간 (snap_tol 안, 여러 개면 모두) 의 양 끝에서 출발해, 지나온 구간 (어느 쪽으로든) ·
//   같은 자세 (loop_tol 안, 고리 건너뛰기) 로만 이어 가장 짧은 관절 이동으로 is_exit 가 true 인 점까지. blocked (퍽에 닿은 그리퍼 점: 닫기 · 놓기)
//   근처 점은 안 지남 → 넣고 빼는 중 멈추면 넣은 깊이로 안 돌아감. 결과 = [q_now, …, 끝점] (그리퍼 · 시간 맞춤 없음, 속도 speed_dps, 마지막 점 stop).
//   지나온 길에서 snap_tol 넘게 벗어남 · 닿는 길 없음이면 빈 목록 + why. exit_index: 끝점의 trace 번호
std::vector<PathNode> retreat_from_trace(const std::vector<std::vector<double>> & trace, const std::vector<double> & q_now,
  double snap_tol, double loop_tol, const std::function<bool(const std::vector<double> &)> & is_exit, double speed_dps,
  std::string & why, std::size_t * exit_index = nullptr, const std::vector<std::vector<double>> * blocked = nullptr);
// 도중 정지 HOME 되돌아가기를 쓸지: planning 의 HOME 요청 · home_retreat 켬 · 받침 다시 쥐기 빈손 아님일 때만. "" = 씀, 아니면 이유
//   실패 뒤 자동 접기 · P0 자동 접기 (home_request false) 는 안 씀 → 거절 (팔이 혼자 움직이지 않게)
std::string home_retreat_block(bool home_request, bool enabled, bool park_left);
// 되돌아간 뒤 접어도 되나: 명령값 끝점이 아니라 실측 관절이 검사된 복귀 자세 (is_exit) 안. NaN · 관절 수 0 = 아님
bool home_retreat_landed(const std::vector<double> & q_meas, const std::function<bool(const std::vector<double> &)> & is_exit);
// HOME 옮김 (control_master home_j1/j3/j4/j5_offset_rad): q[j] += d[j] [rad] (관절 1..5, 0 인 관절은 안 건드림).
//   관절 5 개 미만이면 그대로
void add_joint_offsets(std::vector<double> & q, const std::array<double, 5> & d);

// LOOK_RACK 관찰 자세에서 칸 경로 잇기: 칸 경로 (store_<칸>_d30 · store_<칸>) 첫 점은 공통 기준점 (lift)
//   이라 관찰 자세 (look_rack 끝) 에서는 출발 검사가 거절. 대신 방금 간 look_rack 경로 (look, 실행한 그대로) 를 거꾸로 (retrace_nodes)
//   ① store.nodes[j ≥ 1] 과 같은 점 (same_tol 안, 같은 칸 = lift_real · S3 는 r8 까지 같이 씀) 이 나오면 거기서 store.nodes[j..] 로
//   ② 아니면 look.nodes[1] 까지 거꾸로 → look_start (설정 look_rack 첫 점) 가 store 첫 점과 같을 때만 그 점 → store.nodes[1..]
//   새 직선 없음: 지나온 구간 (거꾸로, 그 구간 속도) + 칸 경로 그대로 (낮게 적재함 위를 쓸지 않음). 결과 첫 점 = q_now
//   q_now 가 look 끝에서 tol 넘게 다름 · 점 2 개 미만 · 관절 수 다름 · NaN · 이을 점 없음 → 빈 목록 + why
//   store_from: 결과 끝 (store.nodes.size() − *store_from) 점 = store.nodes[*store_from..] 그대로 (앞은 이은 점)
std::vector<PathNode> look_rack_back_join(const ArmPath & store, const ArmPath & look, const PathNode & look_start,
  const std::vector<double> & q_now, double tol, std::string & why, double same_tol = 1e-3, std::size_t * store_from = nullptr);

// 보기 자세에서 다음 작업 잇기 (control_master join_from_look): returning = 지금 자세에서 시작하는 검사된 복귀 경로
//   (home_from_look · home_from_look_barcode · LOOK_AT 자세 → look_barcode 끝 → home_from_look_barcode, 끝 = 받침).
//   p0_ok 이고 returning 끝이 fold · path 앞이 unfold 면 [returning 의 P0 까지, path 의 P0 다음 …] (받침까지 접었다 펴지 않음),
//   아니면 [returning 전부, path 둘째 점 …] (path 첫 점 = 받침 = returning 끝). 새 직선 없음. 반환 true = P0 에서 이음
bool join_via_return(ArmPath & path, const ArmPath & returning, const ArmPath & fold, const ArmPath & unfold, bool p0_ok);

// STORE 칸 경로 고르기: held = 새 칸 경로 (storeheld_paths, _d30 등) 이름 있음 · over_look = storeheld_over_look ·
//   look_ok = 그 칸 LOOK_RACK 관찰 자세라 store_look_<칸> 출발 검사 통과 · held_joinable = 새 칸 경로를 지금 자세에서 이을 수 있음
//   (출발 검사 · 집은 자리 잇기 · 관찰 자세 거꾸로 잇기). 관찰 자세에서 새 칸 경로를 못 이으면 store_look_<칸> 으로
enum class StorePick { kLook, kHeld, kPlain };
StorePick store_path_pick(bool held, bool over_look, bool look_ok, bool held_joinable);

// LOOK_RACK 생략: 다음 STORE 가 새 칸 경로 (받침에서 다시 쥔 퍽 _d30 · _d30_low, 기울여 쥔 퍽 _d40_tilt*) 를
//   지금 자세 (PICK 끝 · 받침 다시 쥐기 끝) 에서 바로 이을 수 있으면 관찰 자세에 가지 않음 (그 경로는 관찰 결과를 안 씀 —
//   관찰 자세에 가면 되짚어 와서 적재함까지 두 번 감). 새 경로 없음 · storeheld_over_look false · 지금 자세에서 못 잇는 경우는 관찰 자세로
//   held_name = storeheld_store_name 결과 ("" = 새 경로 안 씀), joinable_now = 그 경로를 지금 자세에서 이을 수 있음 (store_joinable)
bool look_rack_skip_for_store(const std::string & held_name, bool over_look, bool joinable_now);

// 받침에서 다시 쥔 퍽의 STORE: 지금 자세 (받침 위 park_re_lift) 에서 칸 경로의 박스 앞 점 (lift_real … 박스에서 들어 올리는 기록) 을
//   건너뛰고 table 의 "경로이름=점이름" 점으로 관절 직선으로 바로 감. path 첫 점은 이미 지금 자세 (join_from_pick_end). 그 점이 없거나 표에 없으면 그대로 (false)
//   표 값은 URDF 충돌 형상으로 고른 점 (park_join.py: 팔 · 손 · 쥔 퍽 30 mm ↔ 몸 · 적재함 · 받침 · 다 찬 칸 퍽 간격). skipped = 뺀 점 수
bool park_store_join(ArmPath & path, const std::string & name, const std::vector<std::string> & table, std::string & to, std::size_t & skipped);
// park_store_join 기본 표 (control_master park_store_join 기본값 · config/control.yaml 같은 값). park_re_lift 에서 관절 직선으로
//   팔 · 손 · 쥔 퍽 ↔ 몸 · 적재함 · 받침 · 다른 11 칸 퍽 12 mm 이상인 칸 근처 (손끝이 칸 중심 0.10 m 안) 점 중 놓기까지 가장 빠른 점
inline const std::vector<std::string> kParkStoreJoinDefault{
  "store_S1_d30=r74", "store_S2_d30=r50", "store_S3_d30=r42", "store_S4_d30=r18", "store_S5_d30=r10", "store_S6_d30=r42",
  "store_S7_d30=r34", "store_S8_d30=r46", "store_S9_d30=r26", "store_S10_d30=r14", "store_S11_d30=r30", "store_S12_d30=r26"};

}  // namespace manipulation
