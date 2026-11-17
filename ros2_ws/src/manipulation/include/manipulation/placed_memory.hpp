// placed_memory.hpp — 놓은 퍽 기억 (remember_placed)
//   손목 카메라 마지막 장만 보면 방금 놓은 퍽 위에 내려놓을 자리를 고를 수 있음 → control_master 가 성공한 PLACE (· 실패한 먼 PLACE 의 내려놓은 자리) 를 odom 좌표로 기억 → 지금 base_link 로 옮겨 이웃으로 씀.
//   지움: 차체가 놓을 때 자리에서 max_move 넘게 움직임 · max_age 넘음 · 새 미션 (LOOK_BARCODE) · 그 자리 PICK 성공.
#pragma once

#include <Eigen/Dense>

#include <string>
#include <vector>

#include "manipulation/grasp_select.hpp"

namespace manipulation
{

struct PlacedPuck
{
  Eigen::Vector2d p{Eigen::Vector2d::Zero()};       // 퍽 중심 (odom xy) [m]
  Eigen::Vector2d robot{Eigen::Vector2d::Zero()};   // 놓을 때 base_link 원점 (odom xy) [m]
  double t{0.0};                                    // 놓은 시각 [s]
  double yaw{0.0};                                  // 면 법선 (odom) [rad] = 놓을 때 닫힘축 (손가락이 면을 쥠)
  bool yaw_known{false};
  bool square{true};                                // 모르면 네모
  double radius{0.0};                               // > 0 = 이 반지름 원 (누운 퍽 30 mm, NeighborPuck 과 같음)
  bool measured{false};                             // 손목 검출로 자리를 다시 잼
  std::string what;                                 // 로그용 (PLACE · 먼 PLACE 내려놓기)
};

// 지울 것 지움: 놓을 때 자리에서 차체가 max_move [m] 넘게 움직임 · max_age_s 넘음 (≤ 0 = 시간 제한 없음). 지운 수
int expire_placed(std::vector<PlacedPuck> & v, const Eigen::Vector2d & robot_now, double now_s, double max_move, double max_age_s);

// p (odom) 에서 r 안 기억을 지움 (그 퍽을 다시 집음). 지운 수
int forget_placed_near(std::vector<PlacedPuck> & v, const Eigen::Vector2d & p, double r);

// 기억 → 지금 base_link 이웃 (b = R · p + T, 면 방향 + dyaw). J1 에서 reach [m] 안만. 여유 없음 (카메라 이웃과 같은 꼴)
std::vector<NeighborPuck> placed_neighbors(const std::vector<PlacedPuck> & v, const Eigen::Matrix2d & R, const Eigen::Vector2d & T,
  double dyaw, const Eigen::Vector2d & j1, double reach);

// 놓기 미끄러짐 여유: 이웃을 반지름 (반폭 square_half + margin) 원으로 (내려놓을 자리 고르기에만)
std::vector<NeighborPuck> inflate_neighbors(std::vector<NeighborPuck> nb, double margin, double square_half = 0.0174);

// 먼 PLACE 내려놓을 자리 후보 (이 순서대로 시도): 목표 p0 3 cm 앞부터 J1 쪽으로 1 cm 씩 × 옆 0 · +4 · −4 · +8 · −8 cm,
//   박스 밖 (x < min_x · |y| > max_abs_y) · 목표와 6 cm 안은 뺌
std::vector<Eigen::Vector2d> far_putdown_points(const Eigen::Vector2d & j1, const Eigen::Vector2d & p0, double min_x, double max_abs_y);

// 내려놓을 자리 s 가 기억한 퍽 (여유 넣은 원, inflate_neighbors) 과 겹치지 않음: spot_gap ≥ 0
bool putdown_clear(const Eigen::Vector2d & s, double half_t, const std::vector<NeighborPuck> & placed_inflated);

// 먼 PLACE 가 내려놓은 자리 S 에 퍽이 지금 남아 있나 (remember_placed_fail_released_only): 팔 동작을 보낼 때마다 바로 앞 팔 동작 이름 (prev_pose) 으로 갱신.
//   far_put (놓기 열기) 뒤 = 남음, far_re_grasp (다시 쥐기 닫기) 뒤 = 빈손이면 남음 · 쥐었으면 안 남음. 그 밖 = left 그대로
//   → 실패 · 취소한 먼 PLACE 가 놓기 전 · 다시 쥔 뒤 끝나면 S 를 기억하지 않음 (없는 퍽이 그 자리 PLACE 를 막지 않게)
bool far_put_left_after(const std::string & prev_pose, bool left, bool regrip_empty);

}  // namespace manipulation
