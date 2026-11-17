// wrist_pose.hpp — 손목 D435 퍽 자세 기하 계산, yolo/wrist/scripts/wrist_pose.py 의 C++ 판
//
// 역할: puck_pose 순서
//   1. 마스크 안 유효 깊이 0.10–1.20 m 화소 → 점군, 중앙값에서 6 cm 밖 점은 버림
//   2. 점 평균 = 중심, PCA = 긴축. 위쪽을 모를 때 쓰는 값
//   3. 태그: 위쪽을 알면 ID 만 붙이고, 모르면 태그 법선 · 위쪽으로 긴축을 정함
//   4. 위쪽을 알면 gravity_refine
//        높이 h = p·up, 발자국 = p − h·up
//        선 퍽 판정: 윗면 − 아래 5 % 높이 > 4.5 cm 또는 발자국 긴 길이 < 4.8 cm
//        긴축: 선 퍽 = up, 누운 퍽 = 발자국 PCA 긴 방향
//        윗면 점: 윗면 아래 8 mm 띠에서 시작해 평면 맞춤 ±4 mm 를 3 번
//        중심 = 윗면 점 수평 평균에서 퍽 크기 절반만큼 내린 점
//        사각기둥 면 방향: 선 퍽 = 윗면 점의 최소 면적 사각형 한 변, 누운 퍽 = 위쪽
//   5. mask_pose 를 주면: 마스크 안 깊이가 모자라거나 한쪽에 몰렸을 때 중심 (누운 퍽은 긴축도) 을 마스크 모양으로 (mask_top_refine)
// 입력
//   - 퍽 마스크 1 개, 컬러에 정렬된 깊이 m, 컬러 내부 파라미터
//   - AprilTag 검출 결과
//   - 카메라 좌표의 월드 위쪽 up_cam, 모르면 nullptr
// 출력
//   - PuckPose: 중심 · 긴축 · 선/누움 · 면 방향 · 윗면 중심 · 화면 잘림 여부
//   - 좌표는 카메라 광학 프레임 x 오른쪽 · y 아래 · z 앞, m
// 실행
//   - perception_core 라이브러리, WristPipeline 이 부름. 시험 test_wrist_pose
// note
//   - 순수 C++, OpenCV · Eigen 만 씀
//   - 판정 상수는 wrist_pose.py 와 같은 값
//   - 누운 사각기둥의 윗면 띠 폭이 2.2 cm 보다 좁으면 모서리로 기댄 것 → 면 방향 모름
//   - 마스크가 화면 가장자리에 닿으면 truncated → 자세를 믿지 말 것

#pragma once

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace perception::wrist
{

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

// ---- 퍽 · 판정 상수 (wrist_pose.py 와 같은 값)
constexpr double PUCK_D = 0.030;            // 원기둥 지름 · 사각기둥 한 변 (m)
constexpr double PUCK_H = 0.060;            // 긴축 길이 (m)
constexpr double DEPTH_MIN = 0.10;          // 이 밖의 깊이는 버림 (m)
constexpr double DEPTH_MAX = 1.20;
constexpr double OUTLIER_R = 0.06;          // 중앙값에서 이 거리 밖 점 버림 (퍽 대각 약 7 cm)
constexpr int MIN_POSE_POINTS = 30;         // 이보다 적으면 자세 안 냄
constexpr double UPRIGHT_MIN_H = 0.045;     // 높이 범위가 이보다 크면 선 퍽
constexpr double SUPPORT_UPRIGHT_H = 0.047;  // 윗면 − 놓인 면 이보다 크면 선 퍽 (누운 34.8 · 선 60 mm 가운데)
// 놓인 면 후보 띠 (윗면에서 아래로 [m]): 누운 퍽 (원기둥 30 · 네모 34.8 mm) · 선 퍽 (60 mm). support_height 가 박스 밖 바닥을 안 고르게
constexpr double SUPPORT_LIE_LO = 0.022, SUPPORT_LIE_HI = 0.043, SUPPORT_STAND_LO = 0.051, SUPPORT_STAND_HI = 0.070;
constexpr int RING_IN = 4, RING_OUT = 14, RING_MIN_POINTS = 40;
constexpr double FLAT_MIN_LEN = 0.048;      // 발자국 긴 길이가 이보다 짧으면 선 퍽 (누운 퍽 60 mm, 선 퍽 ≤ 42 mm)
constexpr double TOP_WIN = 0.004;           // 윗면 높이 반복 창 (± m)
constexpr double TOP_BAND = 0.008;          // 윗면 점 씨앗 = 윗면 높이에서 이만큼 아래까지
constexpr double PLANE_TOL = 0.004;         // 윗면 평면에서 이 거리 안 점 = 윗면 점
constexpr double PLANE_MAX_TILT = 30.0;     // 맞춘 윗면 법선이 위쪽과 이보다 벌어지면 씨앗 그대로 (deg)
constexpr double LYING_FACE_MIN_W = 0.022;  // 누운 퍽 윗면 띠 폭이 이보다 넓으면 면이 위 (면 약 30 mm, 모서리 약 16 mm)
constexpr double FACE_SIDE_MIN = 0.020;     // 윗면 사각형 한 변 허용 범위 (m)
constexpr double FACE_SIDE_MAX = 0.045;
constexpr double FACE_MAX_ASPECT = 1.4;
constexpr double SQUARE_SIDE = 0.0348;      // 네모 퍽 단면 한 변 실측 = 누운 높이 (m) — 마스크 자세의 옆면 모델용

struct Intrinsics
{
  double fx{0.0}, fy{0.0}, cx{0.0}, cy{0.0};
};

struct TagPose
{
  int id{-1};
  std::array<cv::Point2f, 4> corners{};     // 화소 (aruco 순서)
  Mat3 R{Mat3::Identity()};                 // 태그 → 카메라 회전 (열 z = 면 법선, 카메라 쪽)
  Vec3 t{Vec3::Zero()};                     // 태그 중심 (m, 한 변 kPuckTagSideM 가정. 지금은 안 씀, 거리는 깊이 점군)
  std::vector<std::pair<Mat3, Vec3>> alts;  // IPPE 두 해 (작은 태그는 법선 뒤집힌 해 → 깊이로 다시 고름)
};

struct PuckPose
{
  Vec3 center{Vec3::Zero()};                // 퍽 중심 (m)
  Vec3 axis{Vec3::UnitZ()};                 // 긴축 단위벡터 (부호 = orient_axis: 선 퍽 위쪽, 그 밖 화면 오른쪽)
  int n_points{0};
  Vec3 extent{Vec3::Zero()};                // PCA 축별 퍼짐 (2σ, 큰 순)
  std::array<double, 3> spread{};           // PCA 축별 5–95 % 범위 (m, 큰 순) — 크기 거르기용 (wrist_detector.point_spread)
  std::optional<TagPose> tag;
  std::string axis_source{"pca"};           // pca / tag_normal / tag_up / gravity_up / gravity_flat / mask_flat
  std::array<int, 4> bbox{};                // 마스크 상자 x0, y0, x1, y1 (끝 포함)
  bool truncated{false};                    // 마스크가 화면 가장자리에 닿음 → 자세 못 믿음
  std::optional<bool> upright;              // 위쪽 알 때만 판정
  std::optional<Vec3> face_dir;             // 사각기둥 면 법선 방향 (90° 주기). 선 퍽 = 윗면 사각형 한 변, 누운 퍽 = 위쪽
  std::array<double, 2> face_size{};        // 선 퍽 = 윗면 사각형 두 변, 누운 퍽 = [띠 폭, 0] (m) — 확인용
  std::optional<Vec3> top_center;           // 윗면 중심 (위쪽 알 때만)
  bool tag_centre{false};                   // tag_top_refine 이 윗면 중심 · 면 방향을 태그로 잡음 (마스크가 잘려도 중심은 믿음)
  bool mask_centre{false};                  // mask_top_refine 이 중심 (누운 퍽은 긴축도) 을 마스크 모양으로 잡음 (깊이 모자람)
  std::array<double, 2> depth_cover{};      // 깊이 덮임 [쓴 점 / 마스크 화소, 윗면 긴 변 덮은 비율] — mask_top_refine 이 잼 (로그용)
  Vec3 depth_center{Vec3::Zero()};          // mask_centre 일 때 바꾸기 전 깊이 중심 (로그 · 시험용)
};

// 마스크 자세: LOOK 화면 아래쪽은 그리퍼 · 차체가 D435 적외선을 가려 정렬 깊이가 거의 없음
//   누운 퍽 마스크의 7 % 만 (한쪽 끝) 깊이 → 깊이 중심이 긴축으로 26 mm 치우쳐 끝 30 mm 쥐기가 퍽 밖을 쥠
//   깊이가 모자라거나 (min_fill) 한쪽에 몰리면 (min_span) 중심 · 누운 긴축을 마스크 모양으로. 윗면 높이 · 선/누움은 깊이 그대로
//   거르기는 안 늘림 (못 믿을 때 더 나은 값으로 바꿀 뿐). 깊이가 넉넉하면 안 바꿈
struct MaskPoseParams
{
  bool enable{true};
  double min_fill{0.3};    // 쓴 깊이 점 (튀는 점 뺀 뒤) 이 마스크 화소의 이 비율보다 적으면 (실물: 잘 보이는 선 퍽 0.82 · 0.87, 누운 퍽 0.07.
                           //   옆면 깊이가 하나도 없는 비스듬한 선 퍽 합성 0.53 — 그래도 안 바뀌게 낮게)
  double min_span{0.7};    // 윗면 점이 윗면 긴 변의 이 비율도 못 덮으면 = 한쪽 끝만 보임 (같은 장면: 0.89 · 0.89, 누운 퍽 0.12)
};

// ---- numpy 와 같은 통계 (percentile 은 numpy 기본 linear 보간)
double percentile(std::vector<double> v, double q);
double median(std::vector<double> v);

// 마스크(0 아님 = 퍽) 안 유효 깊이 화소 → 카메라 좌표 점 (행 우선 순서 = np.nonzero 와 같음)
std::vector<Vec3> deproject(const cv::Mat1b & mask, const cv::Mat1f & depth_m, const Intrinsics & K);
// deproject 와 같지만 roi 안 화소만 (roi 밖 마스크가 0 일 때 결과 같음)
std::vector<Vec3> deproject_in(const cv::Mat1b & mask, const cv::Mat1f & depth_m, const Intrinsics & K, const cv::Rect & roi);

// 점 높이 → 윗면 높이 (상위 5 % 에서 시작, ±TOP_WIN 안 점 중앙값 5 번 반복)
double top_height(const std::vector<double> & h);

// 회색 영상 → AprilTag 36h11 (ID 0–14 만, IPPE_SQUARE 두 해). dist 비우면 왜곡 0
std::vector<TagPose> detect_tags(const cv::Mat & gray, const Intrinsics & K, const cv::Mat & dist = cv::Mat());

// detect_tags 와 같지만 rois 안만 검출 (퍽 분할 상자 둘레). 겹치는 영역은 합침, 모서리는 원본 좌표, 같은 ID 는 처음 것
std::vector<TagPose> detect_tags_in(
  const cv::Mat & gray, const std::vector<cv::Rect> & rois, const Intrinsics & K, const cv::Mat & dist = cv::Mat());

// 영상 밖은 자르고, 겹치는 사각형은 감싸는 사각형 하나로 합침
std::vector<cv::Rect> merge_rois(std::vector<cv::Rect> rois, const cv::Size & img);

// 모서리 4 개 (aruco 순서, 원본 화소) → TagPose (IPPE_SQUARE 두 해). PnP 실패면 없음. detect_tags · 태그 다시 찾기 공용
std::optional<TagPose> tag_pose_from_corners(
  int id, const std::array<cv::Point2f, 4> & corners, const Intrinsics & K, const cv::Mat & dist = cv::Mat());

// 태그 안쪽 깊이 평면 법선과 더 가까운 IPPE 해를 고름 (점 < 12 면 그대로)
TagPose refine_tag_with_depth(TagPose t, const cv::Mat1f & depth_m, const Intrinsics & K);

// 윗면(아랫면) 태그인지: 점군이 법선 방향으로 더 길게 퍼졌으면 윗면
bool tag_is_top(const std::vector<Vec3> & pts, const Vec3 & n, const Vec3 & up);

// 긴축 부호: 선 퍽(위쪽과 45° 안) = 위쪽, 그 밖 = 화면 오른쪽(+x), 거의 세로면 화면 아래(+y)
Vec3 orient_axis(const Vec3 & axis, const Vec3 * up);

// 윗면 점: 높이 띠 씨앗 → 평면 맞춤 → 평면 ±PLANE_TOL 안 점, 3 번
std::vector<Vec3> top_face_points(const std::vector<Vec3> & pts, const std::vector<double> & h, double top_h, const Vec3 & up);

// 누운 퍽 윗면 띠 → (면 방향 = 위쪽 또는 없음, [띠 폭, 0])
std::pair<std::optional<Vec3>, std::array<double, 2>> lying_face(
  const std::vector<Vec3> & band, const Vec3 & axis, const Vec3 & up);

// 선 퍽 윗면 점 → (면 방향 또는 없음, 사각형 두 변 m)
std::pair<std::optional<Vec3>, std::array<double, 2>> upright_face(const std::vector<Vec3> & top, const Vec3 & up);

// 선 퍽 윗면에 태그가 보이면 (puck_pose 가 고른 p.tag): 태그 네 모서리 화소 광선을 윗면 높이 평면과 만나게 해
//   윗면 중심 · 퍽 중심 · 면 방향 (태그 변) 을 다시 잡음. 깊이 점 평균은 비스듬히 볼 때 약 10 mm 치우침 (실물, 태그와 비교)
//   태그 한 변이 kPuckTagSideM 에서 4 mm 넘게 다르거나 윗면 중심에서 25 mm 넘게 떨어지면 그대로 (다른 퍽 태그 · 높이 틀림)
//   잡으면 p.tag_centre = true → 마스크가 화면 가장자리에 닿아도 (truncated) 거르지 않음 (reject_reason)
bool tag_top_refine(PuckPose & p, const Intrinsics & K, const Vec3 & up);

// 누운 네모 퍽의 윗면 (위를 향한 옆면) 에 태그가 보이면 (p.tag): 태그 네 모서리 광선을 윗면 높이 평면과 만나게 해 윗면 중심 · 퍽 중심 ·
//   긴축 (태그 두 변 중 지금 긴축에 가까운 쪽) 을 다시 잡음. 깊이 점 평균은 비스듬히 볼 때 · 마스크 끝이 덜 잡힐 때 긴축 방향으로
//   수 mm 흔들림 → 받침 세우기에서 쥐는 자리가 그만큼 틀어짐 (시뮬: 긴축 ±2 mm 면 다 섬, ±4 mm 면 거의 안 섬)
//   태그 한 변이 kPuckTagSideM 에서 4 mm 넘게 다르면 (윗면이 아닌 옆면 태그) · 윗면 중심에서 25 mm 넘게 떨어지면 그대로
bool tag_lying_refine(PuckPose & p, const Intrinsics & K, const Vec3 & up);

// 위쪽 알 때 선/누움 · 긴축 · 중심 · 면 방향 보정 (p 를 고침)
//   support_h (놓인 면 높이, 위쪽 방향 내적) 가 있으면 선/누움 = 윗면이 놓인 면에서 SUPPORT_UPRIGHT_H 넘게 높은가 (누운 퍽 34.8 · 선 퍽 60 mm)
//   없으면 점 높이 범위 · 발자국 길이로. 실물: LOOK 에서 누운 퍽을 장마다 섰다 누웠다 함 (윗면 높이는 누운 퍽)
void gravity_refine(PuckPose & p, const std::vector<Vec3> & pts, const Vec3 & up, std::optional<double> support_h = std::nullopt,
  std::vector<Vec3> * top_out = nullptr);
//   top_out: 중심을 정한 윗면 점 (top_face_points) 을 넘겨줌 (mask_top_refine 덮임용, 다시 안 셈)

// 마스크 둘레 고리 (마스크에서 RING_IN ~ RING_OUT 화소) 깊이 점 → 놓인 면 높이 (위쪽 내적). 윗면 − 10 mm 보다 낮은 점의 중앙값
//   (옆 퍽 · 박스 끝 바닥은 고리 일부). 점이 RING_MIN_POINTS 보다 적으면 없음
std::optional<double> support_height(const cv::Mat1b & mask, const cv::Mat1f & depth_m, const Intrinsics & K, const Vec3 & up,
  double top_h);

// 마스크 둘레 고리 (RING_IN ~ RING_OUT 화소) 깊이 점의 높이 (위쪽 내적) 10 % 값 — 둘레에서 가장 낮은 면 쪽. 점이 RING_MIN_POINTS 보다 적으면 없음
std::optional<double> ring_low_height(const cv::Mat1b & mask, const cv::Mat1f & depth_m, const Intrinsics & K, const Vec3 & up);

// 옆면 모델 한 축: 윗면 평면에서 본 마스크 양 끝 가운데 g (카메라 바로 아래 점 = 0 기준, m) → 윗면 중심 c
//   마스크 = 윗면 + 카메라 쪽 옆면. 그 평면에서 바닥 모서리는 카메라 바로 아래 점 쪽으로 k (= 카메라 ↔ 윗면 / 카메라 ↔ 바닥 높이) 배 줄어 보임
//   → 끝 = [min(c − a, k(c − a)), max(c + a, k(c + a))] (a = 윗면 반 크기). 가운데가 c 에 단조라 바로 풂
//   양 끝을 같이 쓰므로 마스크가 고르게 부풀거나 줄어도 (분할 경계) 가운데는 그대로
double side_corrected_centre(double g, double a, double k);

// 마스크 윤곽 → 윗면 (평면 p·up = top_h, 카메라 좌표) 중심 · 긴 변 방향
struct MaskTop
{
  Vec3 top_center{Vec3::Zero()};            // 윗면 중심 (평면 위)
  Vec3 long_dir{Vec3::UnitX()};             // 윤곽 최소 사각형 긴 변 방향 (평면 안, 단위벡터)
  Vec3 raw_center{Vec3::Zero()};            // 옆면 보정 전 (윤곽 최소 사각형 중심) — 확인용
  double k{1.0};                            // 바닥 모서리 줄어든 배율
};
//   가장 큰 바깥 윤곽 화소 광선을 평면과 만나게 함 → 최소 면적 사각형 두 축마다 side_corrected_centre
//   height = 퍽 세로 길이 (선 60 · 누운 34.8 mm), half_long · half_short = 윗면 반 크기. 윤곽 점 < 8 · 평면이 카메라 위면 없음
std::optional<MaskTop> mask_top_face(const cv::Mat1b & mask, const cv::Rect & roi, const Intrinsics & K, const Vec3 & up,
  double top_h, double height, double half_long, double half_short);

// gravity_refine 뒤 (마스크가 화면 끝에 안 닿았을 때, 윗면 높이 평면 = p.top_center): 덮임 p.depth_cover 를 재고
//     [쓴 깊이 점 / 마스크 화소, 윗면 점이 마스크로 잡은 윗면 긴 변 (누운 60 · 선 34.8 mm) 을 덮은 비율 (5–95 % 범위 ÷ 길이 90 %)]
//   어느 하나가 mp 기준보다 모자라면 중심을 mask_top_face 윗면 중심에서 퍽 크기 절반 아래로 (gravity_refine 과 같은 약속),
//   누운 퍽은 긴축도 마스크 긴 변으로. 윗면 높이 · 선/누움 · 면 방향은 그대로. 바꾸면 p.mask_centre = true · p.depth_center = 바꾸기 전 중심
//   top = gravity_refine 이 쓴 윗면 점 (top_out)
bool mask_top_refine(PuckPose & p, const cv::Mat1b & mask, const cv::Rect & roi, const std::vector<Vec3> & pts,
  const std::vector<Vec3> & top, const Intrinsics & K, const Vec3 & up, const MaskPoseParams & mp);

// puck_pose 단계별 시간 [ms] (더해 감, 계측용)
struct PoseTiming
{
  double points = 0;   // 마스크 → 점 · 튀는 점 제거
  double shape = 0;    // 주축 · 퍼짐 · 상자 · 태그 고르기
  double ring = 0;     // 둘레 고리 (놓인 면 높이 · min_rise)
  double refine = 0;   // gravity_refine + 태그 보정
};

// 마스크 1개 → 퍽 자세. 점 부족이면 없음. up_cam 은 카메라 좌표 월드 위쪽 (없으면 nullptr)
//   min_rise_m > 0 이고 위쪽을 알면: 윗면이 둘레의 낮은 면 (ring_low_height) 보다 이만큼도 안 높으면 퍽이 아님 (상자에 인쇄된 무늬 등) → 없음
std::optional<PuckPose> puck_pose(
  const cv::Mat1b & mask, const cv::Mat1f & depth_m, const Intrinsics & K,
  const std::vector<TagPose> & tags, const Vec3 * up_cam, double min_rise_m = 0.0, bool tag_centre = true,
  PoseTiming * timing = nullptr, const MaskPoseParams * mask_pose = nullptr);
//   tag_centre false: 중심 · 면 방향을 모양 (마스크 · 깊이) 으로만 — 태그가 퍽에 비뚤게 붙어도 안 따라감 (태그는 ID · 기울기 축에만).
//     단 마스크가 화면 끝에 잘리면 (truncated) 모양 중심을 못 믿으므로 태그 중심 그대로
//   mask_pose (위쪽 알 때만, nullptr = 안 씀): gravity_refine 뒤 · 태그 중심 보정 전에 mask_top_refine. 태그가 중심을 잡으면 태그가 이김

}  // namespace perception::wrist
