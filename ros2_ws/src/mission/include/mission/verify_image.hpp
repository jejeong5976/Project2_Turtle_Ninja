// verify_image.hpp — 재검증 판정 그림: VERIFY LOOK 때 손목 영상에 본 퍽 · 칸 · 판정을 그림
//
// 역할
//   - map 점 → 카메라 영상 점 (핀홀, 왜곡 무시)
//   - 장소 하나 판정 (PlaceVerify) 을 영상 위에 그림
//       박스 윗면 = 주황 반투명 면 + 테두리 (재검증 박스 측정 있을 때, 박스 윗면 높이)
//       칸      = 허용 거리 원 (맞음 초록 점선 · 문제 빨강 반투명 면, 빈 칸 회색 점선) + 중심 + '칸 번호:놓은 퍽 ID'
//       본 퍽   = perception 검출 상자 (bbox, 마지막 장, 맞음 초록 · 문제 빨강 꺾쇠 테, bbox 없으면 투영 중심 ×)
//                 + 계획 칸 → 퍽 오차 화살표 (박스 윗면 높이) + 라벨 칩 "ID 색/모양 이유" (겹치지 않게 놓고 지시선)
//       뺀 검출 = 회색 점선 상자 + "ignored (edge)" (박스 윗면 가장자리 가까이 · 밖)
//       위 띠   = 장소 · 그룹 칩 · 맞은 수 / 눈 수 · 본 수 · SUCCESS/FAIL 배지, 칸 기준 출처 · 허용 · Y 모양 · 뺀 수
//       아래 띠 = 안 보인 ID · 빈 칸 · 개수 다름 (있을 때만)
// 입력
//   - BGR 영상 (손목 D435 컬러), 카메라 내부값, map → 카메라 광학 프레임 변환, 퍽 중심 높이 · 반 높이 (map z)
// 출력
//   - 같은 영상에 덧그림 → 노드가 /planning/verify_image 로 발행 · run 폴더에 verify_L<n>.jpg
// 실행
//   - mission_image 라이브러리 (OpenCV), mission_manager 가 부름. 시험 test_transfer (VerifyImage)
// note
//   - 글자는 OpenCV 기본 글꼴이라 영문만
//   - 재검증 관측 · 박스 측정은 손목 카메라 돈 만큼 (drift) 되돌린 값 → 그림 변환도 apply_camera_drift 로 같은 기준
//   - 검출 상자는 그 퍽 ID 가 마지막으로 읽힌 장 것 → 그림 장보다 앞 (팔 움직임) 이면 drop_stale_bboxes 로 빼고 × 로

#pragma once

#include <array>
#include <optional>

#include <opencv2/core.hpp>

#include "mission/dice_verify.hpp"

namespace mission
{

struct VerifyCamera
{
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  std::array<double, 9> R{{1, 0, 0, 0, 1, 0, 0, 0, 1}};   // map → 카메라 광학 프레임 회전 (행 우선)
  std::array<double, 3> t{{0, 0, 0}};                     // p_cam = R · p_map + t
};

// map 점 → 영상 점. 카메라 뒤 (z ≤ 1 cm) · 내부값 없음 (fx ≤ 0) 이면 nullopt (영상 밖이어도 점은 돌려줌)
std::optional<cv::Point2d> project_map_point(const VerifyCamera & cam, double x, double y, double z);

// 손목 카메라 돌아감 (camera_drift_node, 광학 프레임 기준 쿼터니언 x y z w) 반영.
//   검출 · 박스 측정 = 실제 카메라 점 p 를 명목 TF · (D · p) 로 옮김 → 그림은 p = Dᵀ · (R · p_map + t)
void apply_camera_drift(VerifyCamera & cam, const std::array<double, 4> & q_xyzw);

// 그림 장 (since_s = LOOK 끝, 팔 멈춤) 보다 앞 장의 검출 상자를 지움 → draw_verify 가 위치 투영 × 로 그림
void drop_stale_bboxes(PlaceVerify & pv, double since_s);

// bgr 에 판정을 덧그림. puck_z = 퍽 중심 높이 (map z), puck_half_m = 퍽 높이 반 (박스 윗면 = puck_z − puck_half_m)
void draw_verify(cv::Mat & bgr, const PlaceVerify & pv, const VerifyCamera & cam, double puck_z, double puck_half_m = 0.03);

}  // namespace mission
