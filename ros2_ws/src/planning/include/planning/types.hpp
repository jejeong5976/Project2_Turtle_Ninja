// types.hpp — planning 공통 자료형, 실행기 · 미션 공용
//
// 역할
//   - Vec3 · Quat · Pose2D 기본 자료형
//   - Step: 실행기가 순서대로 실행하는 단계 하나
//       StepType = Navigate · Arm · Custom
//       Target = None · Fixed · Puck · Custom
//       kind · ref 는 미션이 뜻을 정하는 칸, 실행기는 해석 안 함
//   - ArmTask 상수 kArm*: interfaces/action/ArmTask 와 같은 값
//   - 공통 도우미 함수
// 입력
//   - 없음, 자료형 정의만
// 출력
//   - 없음
// 실행
//   - planning_core 라이브러리. 실행기 · mission 이 include
// note
//   - ROS 를 모르는 순수 C++
//   - 좌표는 map 프레임 x · y m, yaw rad
//   - STORE · RETRIEVE · LOOK · LOOK_BARCODE · LOOK_RACK 은 control_master 구현 전에는 거절됨

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace planning
{

// 3D 점 · 방향 벡터 [m]
struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// 쿼터니언 (x, y, z, w)
struct Quat
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

// 지도 위 2D 자세
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// 단계 종류
//   - 미션이 만든 단계 목록을 실행기(Executor)가 순서대로 실행함
//   - Custom 은 실행기가 모르는 미션 단계 (바코드 읽기 · 실행 중 계획 등) → 미션이 넘긴 함수로 실행
enum class StepType : uint8_t
{
  Navigate,      // Nav2 navigate_to_pose
  Arm,           // control_master /arm_task
  Custom,        // 미션 정의 단계 (kind 로 구분)
};

// 단계의 목표가 어디서 오는가
enum class Target : uint8_t
{
  None,          // 목표 없음 (HOME · LOOK · STORE · RETRIEVE · Custom)
  Fixed,         // 단계에 적힌 pose 그대로 (Navigate)
  Puck,          // 퍽 기록부의 puck_id 위치 (실행 시점에 계산: Navigate = 퍽 앞, PICK = 최근 검출)
  Custom,        // 미션이 계산 (실행 시점에 미션 함수가 지도 점을 줌, 뜻은 미션 쪽에서 정함)
};

struct Step
{
  StepType type = StepType::Navigate;
  Target target = Target::None;
  Pose2D pose;                  // Target::Fixed 일 때
  std::string puck_id;          // 대상 퍽 (PICK · STORE · RETRIEVE · PLACE, Target::Puck)
  uint8_t arm_task = 0;         // StepType::Arm 일 때 (ArmTask 상수와 같은 값)
  std::string slot;             // STORE · RETRIEVE · LOOK_RACK 일 때 적재함 칸
  int kind = 0;                 // 미션이 정하는 종류 (Custom 단계 · Custom 목표). 실행기는 해석 안 함
  std::array<int, 3> ref{{-1, -1, -1}};   // 미션이 정하는 번호 (뜻은 미션 쪽에서 정함). 실행기는 해석 안 함
  double timeout_s = 0.0;       // 0 = 설정 기본값 사용
  bool dock = false;            // Navigate: 정밀 접근 정차 (dock_action 켜져 있으면 중간 정차까지 Nav2 → 정밀 접근 액션)
  std::string name;             // 상태 표시용 이름
};

// ArmTask 상수 (interfaces/action/ArmTask 와 같은 값)
constexpr uint8_t kArmHome = 0;
constexpr uint8_t kArmPick = 1;
constexpr uint8_t kArmPlace = 2;
// 아래 셋은 interfaces ArmTask 에 2026-11-01 추가 (control_master 구현 전에는 거절됨)
constexpr uint8_t kArmStore = 3;
constexpr uint8_t kArmRetrieve = 4;
constexpr uint8_t kArmLook = 5;
constexpr uint8_t kArmLookBarcode = 6;   // 바코드 정면 보기 (손목, 2026-11-01)
constexpr uint8_t kArmLookRack = 8;      // 퍽 쥔 채 적재함 칸 관찰 → 다음 STORE 에 칸 측정값 (2026-11-03)

// 손목 관찰 자세 (LOOK) 인지. 성공하면 팔이 펴진 채 남음
inline bool is_look_task(uint8_t t) {return t == kArmLook;}

// ---------------------------------------------------------------- 공통 도우미

// error 가 있으면 이유 기록 (실패 반환하는 함수들이 같이 씀)
inline void set_error(std::string * error, const std::string & msg)
{
  if (error) {*error = msg;}
}

// 방향을 아는 쿼터니언인가 ((0,0,0,0) = 모름. 크기가 1 근처일 때만 유효)
inline bool valid_quaternion(double x, double y, double z, double w)
{
  const double n = x * x + y * y + z * z + w * w;
  return std::isfinite(n) && std::fabs(n - 1.0) < 0.1;
}

// 쿼터니언 → yaw [rad] (z 축 회전만 씀)
inline double yaw_from_quaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace planning
