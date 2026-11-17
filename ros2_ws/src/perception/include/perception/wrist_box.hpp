// wrist_box.hpp — 손목 D435 깊이로 장소 · 시작 박스 윗면의 중심 · 방향을 구함
//
// 역할
//   - 깊이 화소 → 로봇 바닥 프레임 점 (z = 위, 바닥 = 0)
//   - 높이로 나눔: 윗면 = top_z ± band, 바닥 쪽 = top_z − drop 보다 낮음, 퍽 · 팔 = 윗면보다 높음 (안 씀)
//   - 테두리 점 = 윗면 화소 중 dilate 화소 안에 바닥 쪽 화소가 있는 것
//       화면 끝에서 잘린 경계 (이웃 없음) · 퍽이 가린 경계 (이웃이 더 높음) 는 안 들어감
//   - 크기 아는 정사각형 (한 변 size) 을 테두리 점에 맞춤: 가우스-뉴턴 + Huber 가중, 점마다 가까운 변을 다시 고르며 반복
//   - 앞 · 뒤 변 중 하나, 왼 · 오른 변 중 하나가 min_side_pts 이상 보여야 성공 (뒤 변 · 한쪽 옆 변은 안 보여도 됨)
// 입력
//   - fit_place_box: 깊이 [m] (컬러에 맞춘 것, 0 · NaN = 없음), 내부 파라미터, 바닥 프레임 ← 카메라 (R, t), 설정, 처음 추정
// 출력
//   - BoxFit: 성공 여부 · 이유, 중심 x · y [m], yaw [rad] (처음 추정 yaw 에서 ±45° 안), 변마다 점 수, 맞춤 잔차
// 실행
//   - perception_core 라이브러리, WristPipeline 이 씀. 시험 test_wrist_box
// note
//   - 박스는 정사각형이라 yaw 는 90° 주기 → 처음 추정 yaw 근처로 접음
//   - 뒤 변을 못 보고 앞 변이 차체에 가리면 앞뒤 위치가 틀어짐 → 그때 찾은 앞 변이 front_min_x 보다 가까우면 실패
//     (LOOK 이 박스 전체를 보면 뒤 변으로 앞뒤가 정해져 이 검사는 안 함)
//   - 정확도는 카메라 외부 파라미터 (팔 순기구학 · 핸드-아이) 가 정함. 깊이 잡음만이면 중심 2–3 mm 안 (시뮬)

#pragma once

#include <array>
#include <string>

#include <opencv2/core.hpp>

#include "perception/wrist_pose.hpp"

namespace perception::wrist
{

struct BoxParams
{
  double size{0.205};          // 윗면 한 변 [m]
  double top_z{0.07};          // 윗면 높이 [m]
  double band{0.012};          // 윗면으로 볼 높이 폭 ± [m] (처짐 · 깊이 잡음)
  double drop{0.03};           // 이만큼 낮으면 바닥 쪽 [m]
  double min_x{0.15};          // 이보다 가까운 점은 차체 (앞 끝 0.145, 바퀴 포함) → 안 씀 [m]
  double max_range{0.8};       // 이보다 먼 깊이는 안 씀 [m]
  int stride{2};               // 화소 건너뛰기
  int dilate{2};               // 테두리 이웃 거리 [화소, stride 적용 뒤]
  int min_side_pts{40};        // 성공에 필요한 변 점 수
  double side_tol{0.006};      // 변 점으로 셀 거리 [m]
  double huber{0.003};         // 맞춤 Huber 경계 [m]
  double front_min_x{0.175};   // 뒤 변을 못 봤을 때 앞 변이 이보다 가까우면 실패 [m]
  int iters{20};
  // 넓게 찾기: 처음 추정에서 맞춘 답이 변 부족 · 뒤집힘이면 테두리 점 범위 격자 (search_step) 에서 다시 → 가장 잘 맞는 답
  //   답 중심은 처음 추정에서 search 안만. 0 = 끔. 처음 추정 답이 맞으면 (변 · 안쪽 바닥 점 검사 통과) 그 답 그대로
  double search{0.50};
  double search_step{0.05};
  int max_inside{20};            // 윗면 안쪽 (테두리 1.5 cm 안) 바닥 점이 이보다 많으면 박스 아님 (−1 = 검사 안 함, search 0 이면 안 함)
  double search_min_frac{0.90};  // 넓게 찾은 답: 테두리 점 중 변 점 비율이 이보다 낮으면 버림 (틀린 답은 0.84–0.94, 맞는 답 1.0, 시뮬)
  double body_half_y{0.16};      // 넓게 찾은 답이 차체 (x < min_x, |y| < 이 값) 와 겹치면 버림
  int search_max_starts{150};    // 넓게 찾기 시작 수 상한 (처음 추정에 가까운 순)
  double search_budget_ms{15.0}; // 넓게 찾기 시간 상한 → 넘으면 그때까지 가장 좋은 답
};

struct BoxFit
{
  bool ok{false};
  std::string why;                 // 실패 이유 (성공이면 "")
  double x{0.0}, y{0.0}, yaw{0.0};  // 바닥 프레임
  std::array<int, 4> side_pts{};   // 앞 (−x) · 뒤 (+x) · 왼 (+y) · 오른 (−y), 박스 좌표 기준
  int edge_pts{0};
  double rms{0.0};                 // 맞춤 잔차 RMS [m] (Huber 안 점)
  int inside_low{0};               // 윗면 안쪽 바닥 점 수 (뒤집힌 답이면 많음)
  int search_starts{0};            // 넓게 찾기 시작 수 (0 = 안 함)
  bool search_capped{false};       // 시작 수 · 시간 상한에 걸림
  double search_ms{0.0};           // 넓게 찾기 걸린 시간 [ms]
};

// 깊이 → 박스 윗면 정사각형. R, t = 바닥 프레임 ← 카메라 광학 프레임, guess = (x, y, yaw) 바닥 프레임
//   edge_mask 를 주면 테두리 점 화소를 255 로 칠함 (확인용, 깊이와 같은 크기)
BoxFit fit_place_box(
  const cv::Mat1f & depth, const Intrinsics & K, const Mat3 & R, const Vec3 & t,
  const BoxParams & p, const Vec3 & guess, cv::Mat1b * edge_mask = nullptr);

// a 를 ref 에서 (−45°, 45°] 안으로 90° 단위로 옮김
double fold_quarter(double a, double ref);

}  // namespace perception::wrist
