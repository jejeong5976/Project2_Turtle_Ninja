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
constexpr double FLAT_MIN_LEN = 0.048;      // 발자국 긴 길이가 이보다 짧으면 선 퍽 (누운 퍽 60 mm, 선 퍽 ≤ 42 mm)
constexpr double TOP_WIN = 0.004;           // 윗면 높이 반복 창 (± m)
constexpr double TOP_BAND = 0.008;          // 윗면 점 씨앗 = 윗면 높이에서 이만큼 아래까지
constexpr double PLANE_TOL = 0.004;         // 윗면 평면에서 이 거리 안 점 = 윗면 점
constexpr double PLANE_MAX_TILT = 30.0;     // 맞춘 윗면 법선이 위쪽과 이보다 벌어지면 씨앗 그대로 (deg)
constexpr double LYING_FACE_MIN_W = 0.022;  // 누운 퍽 윗면 띠 폭이 이보다 넓으면 면이 위 (면 약 30 mm, 모서리 약 16 mm)
constexpr double FACE_SIDE_MIN = 0.020;     // 윗면 사각형 한 변 허용 범위 (m)
constexpr double FACE_SIDE_MAX = 0.045;
constexpr double FACE_MAX_ASPECT = 1.4;

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
  std::string axis_source{"pca"};           // pca / tag_normal / tag_up / gravity_up / gravity_flat
  std::array<int, 4> bbox{};                // 마스크 상자 x0, y0, x1, y1 (끝 포함)
  bool truncated{false};                    // 마스크가 화면 가장자리에 닿음 → 자세 못 믿음
  std::optional<bool> upright;              // 위쪽 알 때만 판정
  std::optional<Vec3> face_dir;             // 사각기둥 면 법선 방향 (90° 주기). 선 퍽 = 윗면 사각형 한 변, 누운 퍽 = 위쪽
  std::array<double, 2> face_size{};        // 선 퍽 = 윗면 사각형 두 변, 누운 퍽 = [띠 폭, 0] (m) — 확인용
  std::optional<Vec3> top_center;           // 윗면 중심 (위쪽 알 때만)
};

// ---- numpy 와 같은 통계 (percentile 은 numpy 기본 linear 보간)
double percentile(std::vector<double> v, double q);
double median(std::vector<double> v);

// 마스크(0 아님 = 퍽) 안 유효 깊이 화소 → 카메라 좌표 점 (행 우선 순서 = np.nonzero 와 같음)
std::vector<Vec3> deproject(const cv::Mat1b & mask, const cv::Mat1f & depth_m, const Intrinsics & K);

// 점 높이 → 윗면 높이 (상위 5 % 에서 시작, ±TOP_WIN 안 점 중앙값 5 번 반복)
double top_height(const std::vector<double> & h);

// 회색 영상 → AprilTag 36h11 (ID 0–14 만, IPPE_SQUARE 두 해). dist 비우면 왜곡 0
std::vector<TagPose> detect_tags(const cv::Mat & gray, const Intrinsics & K, const cv::Mat & dist = cv::Mat());

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
//   윗면 중심 · 퍽 중심 · 면 방향 (태그 변) 을 다시 잡음. 깊이 점 평균은 비스듬히 볼 때 약 10 mm 치우침 (11-04 실물, 태그와 비교)
//   태그 한 변이 kPuckTagSideM 에서 4 mm 넘게 다르거나 윗면 중심에서 25 mm 넘게 떨어지면 그대로 (다른 퍽 태그 · 높이 틀림)
bool tag_top_refine(PuckPose & p, const Intrinsics & K, const Vec3 & up);

// 위쪽 알 때 선/누움 · 긴축 · 중심 · 면 방향 보정 (p 를 고침)
void gravity_refine(PuckPose & p, const std::vector<Vec3> & pts, const Vec3 & up);

// 마스크 1개 → 퍽 자세. 점 부족이면 없음. up_cam 은 카메라 좌표 월드 위쪽 (없으면 nullptr)
std::optional<PuckPose> puck_pose(
  const cv::Mat1b & mask, const cv::Mat1f & depth_m, const Intrinsics & K,
  const std::vector<TagPose> & tags, const Vec3 * up_cam);

}  // namespace perception::wrist
