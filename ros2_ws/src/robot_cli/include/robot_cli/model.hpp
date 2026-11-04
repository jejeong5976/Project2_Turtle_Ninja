// model.hpp — 설정 파일 읽기 · 단위 바꾸기 (ROS 없이, test/test_model.cpp)
//   hardware_map.yaml  관절별 zero_tick · scale → 엔코더 눈금 ↔ 관절값 (hardware opencr_link motor_to_joint 와 같은 식)
//   dice_map.yaml      이름 있는 정차 자세 start · barcode · B1~B6 · L1~L3 → map 프레임 (mission dice_map.cpp 와 같은 변환)
//   rack_cells.yaml    적재함 칸 이름 S1~S12
//   control.yaml · arm_paths.yaml  이름 있는 팔 자세 (받침 · P0 · 기준점 · LOOK) · 접힘 허용 오차 · lazy_fold (control_master 와 같은 값)
#pragma once

#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace robot_cli
{

constexpr double kTickRad = 2.0 * M_PI / 4096.0;

struct JointCal
{
  int zero_tick{2048};
  double scale{1.0};        // 그리퍼는 m/rad (0.01149)
};

struct Pose2D
{
  double x{0.0}, y{0.0}, yaw{0.0};
};

struct NamedStop
{
  std::string name;
  Pose2D pose;              // map 프레임
};

// 이름 있는 팔 자세 joint1~5 [rad] — 경로 점은 처짐 앞먹임이 든 명령값 (control_master 도 /joint_states 와 그대로 비교)
struct NamedPose
{
  std::string name;
  std::vector<double> q;
};

struct ArmRef
{
  std::vector<NamedPose> poses;     // 받침 · P0 · 기준점 · LOOK · LOOK_BARCODE (있는 것만)
  std::vector<double> fold_tol;     // 접힘 판정 받침 ± [rad], 하나면 모든 관절
  double near_tol{0.15};            // 이 자세에 있다 = 모든 관절 차이 ≤ (control_master path_start_tolerance)
  bool lazy_fold{false};            // STORE · PLACE 가 P0 에서 끝남 (주행 전 HOME)
  double grip_open{NAN}, grip_close{NAN};   // 그리퍼 열림 · 닫힘 목표 [m] (control.yaml gripper_open · gripper_close)
};

constexpr double kLimitWarnRad = 0.03;    // 관절 한계까지 이보다 가까우면 주의 (≈ 1.7°, 받침 자세 joint2 는 2.8° 떨어짐)

// 설정 전부 (main 이 읽고 RobotLink · 화면이 씀)
struct Config
{
  std::map<std::string, JointCal> cal;
  std::vector<NamedStop> stops;
  std::vector<std::string> slots;
  ArmRef arm;
  double battery_warn{10.8}, battery_stop{10.2};   // hardware_map.yaml battery_warn_v · battery_stop_v
  double max_linear{0.20}, max_angular{0.5};       // 수동 주행 한계 = controllers.yaml swerve_controller max_linear · max_angular
};

// 관절값 (rad, 그리퍼 m) ↔ 눈금: tick = q / scale / (2π/4096) + zero_tick
double joint_to_tick(const JointCal & c, double q);
double tick_to_joint(const JointCal & c, double tick);
// 그리퍼 손가락 안쪽 면 사이 간격 [m] = 2 × (9.1 mm + q) (manipulation grasp_select inner0, q = gripper_left_joint)
double gripper_gap(double q);

double wrap_angle(double a);
Pose2D arena_to_map(const Pose2D & T, const Pose2D & p);
Pose2D inverse_pose(const Pose2D & a);

bool load_hardware_map(const std::string & path, std::map<std::string, JointCal> & out, std::string & err);
bool load_stops(const std::string & path, std::vector<NamedStop> & out, std::string & err);
bool load_slots(const std::string & path, std::vector<std::string> & out, std::string & err);
// hardware_map.yaml 의 battery_warn_v · battery_stop_v (없으면 그대로 둠)
void load_battery_limits(const std::string & path, double & warn, double & stop);
// controllers.yaml 의 swerve_controller max_linear · max_angular (없으면 그대로 둠, 읽으면 true)
bool load_drive_limits(const std::string & path, double & max_linear, double & max_angular);
bool load_arm_ref(const std::string & control_yaml, const std::string & arm_paths_yaml, ArmRef & out, std::string & err);
// OpenCR 하드웨어 로그 한 줄에서 배터리 전압 읽기 ("bus_arm 보드 안 (10 s): … · 배터리 12.3 V (최저 12.1)" ·
//   "bus_arm 배터리 낮음: 평균 10.7 V · 최저 10.5 V …" · "bus_arm 배터리 10.1 V (정지 기준 …"). 아니면 false
bool parse_battery_log(const std::string & msg, std::string & board, double & mean, double & min);
// 그리퍼가 반쯤 닫힘 = 퍽을 쥔 것으로 봄 (control_master home_plan 과 같은 식: 닫힘 + 3 mm < q < 열림 − 4 mm)
bool gripper_holding(const ArmRef & ref, double q);

// 팔 관절 이름 joint1~5 (자세 비교 순서)
extern const std::vector<std::string> kArmJoint5;
// 가장 가까운 이름 자세 (관절 차이의 최댓값 기준), diff 에 그 값 [rad]. 관절 측정이 빠졌거나 자세가 없으면 nullptr
const NamedPose * nearest_pose(const ArmRef & ref, const std::map<std::string, double> & joints, double * diff);
// 받침 접힘 판정 (control_master FoldCheck 와 같은 식: joint1~5 가 받침 ± fold_tol). worst = 가장 벗어난 관절 · 차이
bool folded(const ArmRef & ref, const std::map<std::string, double> & joints, std::string * worst = nullptr, double * worst_diff = nullptr);

// 이름 찾기 (대소문자 무시). 없으면 nullptr
const NamedStop * find_stop(const std::vector<NamedStop> & stops, const std::string & name);
// 가장 가까운 정차 자세, dist 에 거리 [m]. 목록이 비면 nullptr
const NamedStop * nearest_stop(const std::vector<NamedStop> & stops, double x, double y, double * dist);

}  // namespace robot_cli
