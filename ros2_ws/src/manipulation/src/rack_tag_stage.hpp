// rack_tag_stage.hpp — 손목 태그 (perception /perception/wrist/tags) 로 적재함 칸 · 칸 안 퍽 · 쥔 퍽 위치를 잼, control_master 안에서 돎
// =====================================================================
// 입력: /perception/wrist/tags (interfaces/WristTagArray, 태그 ID · 왜곡 푼 모서리 · 촬영 시각), camera_info (쥔 퍽 재투영 검사 초점 거리),
//       TF base_link ← 손목 카메라 (처짐 보정한 실제 팔 자세, arm_state_publisher), wrist_link ← 카메라 (고정)
// 출력: /rack_tags/detections (interfaces/DetectionArray, frame base_link, kind "rack_tag", label = 칸 S1~S12, id = 태그 ID)
//         pose = 칸 바닥 위 태그 중심, z 축 위, x 축 = 태그 x 축. 받은 장마다 (없으면 빈 배열)
//       /held_puck/tag · /held_puck/tag_fresh (kind "held_tag", frame wrist_link)
//       ~/rack_tags/report (std_srvs/Trigger): 칸별 잰 위치 (중앙값) − config/rack_cells.yaml, 적재함 전체 이동 맞춤, 칸별 옮길 양
//       ~/rack_tags/reset (std_srvs/Trigger): 모은 값 지움
//       ~/rack_tags/dump (std_srvs/Trigger): 멈춘 장의 원본 (태그 ID · 왜곡 푼 모서리 4 개 · 카메라 자세) 을 CSV 로 → PC 에서 카메라 보정 · 분석
//       perception 태그 단계 켜기 · 끄기 (rack_tags.enable_service, std_srvs/SetBool): control_master 가 want() 로 정한 대로
// 태그: tag36h11, 칸 태그 ID = rack_tags.cell_ids (config/rack_cells.yaml rack_tags, 칸 순서 = rack_cell_names), 칸 바닥 가운데
//   잰 자리가 다른 칸 표 위치에 더 가깝고 제 칸에서 mismatch_m 넘게 떨어지면 ID 를 잘못 붙인 것으로 보고 안 씀 (경고에 그 칸 이름)
// 팔이 멈춰 있을 때 (앞 장과 카메라 자리 차이가 still_m · still_deg 안) 만 모음 (움직이는 중은 영상 · 관절 시각이 어긋나 수 mm 틀림)
// 잰 값 − 표 값에는 태그를 칸 가운데에 붙인 오차도 들어감 → 경로가 잘 맞을 때 한 번 재 두고 그 뒤와 비교하는 게 가장 정확
// 칸 안 퍽: 퍽 윗면 태그 (puck_ids, 0~11) 를 퍽 윗면 높이 평면에 내려 잼 → 빈 칸 태그를 먼저 쟀으면 report 에 퍽 − 칸
//   표본마다 카메라 자세를 같이 둠 → 같은 자세 (pose_tol 안) 에서 잰 칸 · 퍽끼리만 빼서 카메라 · 처짐 보정의 치우침이 빠짐
//   경로 옮길 양 = 칸 − 퍽 (STORE 가 퍽을 놓은 자리 기준). 같은 자세의 빈 칸 값이 없으면 차이를 안 냄
//   빠지는 건 카메라 위치 치우침. 카메라 기울기 치우침은 바닥 · 퍽 윗면 높이 차 (60 mm) 만큼 남음 (2° 면 약 2 mm)
//   퍽이 컵 벽에 닿으면 (틈 약 ±2 mm) 그 이상은 못 잼, 턱에 걸려 기울면 높이가 달라 크기 검사에서 빠짐
// 쥔 퍽: 퍽 윗면 태그가 카메라에서 held_max_dist_m 안 (손목 카메라는 J5 뒤 wrist_link 에 붙어 손과 같이 움직임) 이면 쥔 퍽으로 봄
//   단 빈손 스캔 중 (set_empty_scan) 은 손이 비었으므로 안 봄 → 가까운 퍽 태그도 칸 안 퍽으로
//   → 태그 크기 (puck_tag_size_m) 로 PnP 해서 태그 중심 · 면 방향 (z 축) 을 wrist_link 좌표로 /held_puck/tag
//   본 검출에 없으면 perception 이 원본 영상 aruco 로 다시 찾은 태그 (rescue) 를 씀
// 퍽 윗면 태그 PnP (넣은 뒤 보기 store_seat_check): 본 검출 퍽 태그마다 거리와 상관없이
//   PnP 자세를 카메라 광학 프레임으로 /rack_tags/puck_pnp (rack_tags.puck_pnp_topic, kind "puck_pnp", label = 가까운 칸,
//   confidence 0.5 = 두 해가 비슷해 덜 기운 해를 고름) → control_master 기본 그룹이 STORE 뒤 칸 위 관찰 자세에서 모아 drift 를 되돌린 뒤
//   촬영 시각 TF 로 base_link → 칸 기준 위치 · 높이 · 기울기 · 면 (rack_puck 은 평면 높이를 정해 놓고 내려서 높이 · 기울기를 못 잼)
// note
//   - 영상 · 검출 · 왜곡 풀기는 perception wrist_tag_stage (이 파일은 태그 → 위치만)
//   - 자기 콜백 그룹 (촬영 시각 TF 를 tf_timeout_s 까지 기다림 → 팔 계획 · 차례 콜백을 막지 않게)

#pragma once

#include <array>
#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "interfaces/msg/detection_array.hpp"
#include "interfaces/msg/wrist_tag_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.hpp"

#include "manipulation/rack_tags.hpp"

namespace manipulation
{

class RackTagStage
{
public:
  // 칸 이름 · 표 위치는 control_master 가 이미 선언한 rack_cell_names · rack_cell_x · rack_cell_y 를 읽음 → 그 뒤에 만듦
  RackTagStage(rclcpp::Node & node, tf2_ros::Buffer & tf, rclcpp::CallbackGroup::SharedPtr group);
  // 태그 단계를 켜 둘지 (control_master 주기에서 부름). 바뀌면 perception 에 SetBool, 실패 · 무응답이면 1 s 마다 다시
  void want(bool on, double now_s);
  // 빈손 적재함 스캔 중인지 (control_master 주기에서 부름). 켜면 가까운 퍽 태그도 쥔 퍽으로 안 보고 칸 안 퍽 (rack_puck) 으로 잼
  //   스캔 자세에서 칸 안 퍽 윗면은 카메라에서 0.133–0.144 m 라 held_max_dist_m (0.15) 안 → 안 켜면 칸 안 퍽이 늘 쥔 퍽으로 빠짐
  void set_empty_scan(bool on) {empty_scan_.store(on);}

  // 받침 다시 보기: 최근 퍽 윗면 태그 원본 (왜곡 푼 모서리 · 그때 base ← 카메라, drift 안 되돌림) — since 뒤 촬영만, 오래된 것부터
  //   control_master 주기 (다른 스레드) 에서 부름. 평면 높이 · drift 는 부르는 쪽이 정함
  struct PuckTagSeen
  {
    double stamp{0.0};
    int id{-1};
    std::string cam;
    Eigen::Isometry3d T{Eigen::Isometry3d::Identity()};
    std::array<Eigen::Vector2d, 4> cn{};
    bool rescue{false};
  };
  std::vector<PuckTagSeen> puck_tags_since(double since) const;
  TagPlaneConfig puck_plane() const {return cfg_puck_;}   // 만든 뒤 안 바뀜

private:
  void on_tags(const interfaces::msg::WristTagArray & msg);
  bool held_tag(const std::vector<cv::Point2f> & n, double size, Eigen::Vector3d & c, Eigen::Matrix3d & R, double & rms) const;
  // 퍽 윗면 태그 PnP 한 장 → out (카메라 광학 프레임). n = 왜곡 푼 정규 모서리, px = 화소 모서리, T = base ← 카메라, focal [px] (0 = 모름)
  void add_pnp(interfaces::msg::DetectionArray & out, int id, const std::vector<cv::Point2f> & n, const std::vector<cv::Point2f> & px,
    const Eigen::Isometry3d & T, double focal) const;
  void publish_held(const builtin_interfaces::msg::Time & stamp, const std::string & cam, int id, const Eigen::Vector3d & c,
    const Eigen::Matrix3d & Rc, bool fresh);
  static interfaces::msg::Detection detection(const std_msgs::msg::Header & h, const char * kind, const std::string & label, int id,
    const TagOnPlane & hit, double z, const std::vector<cv::Point2f> & corners, bool still);
  void on_puck(int id, const TagOnPlane & hit, const std::vector<cv::Point2f> & corners, bool still, const Eigen::Isometry3d & T,
    interfaces::msg::DetectionArray & out);
  size_t nearest_cell(const Eigen::Vector2d & p) const;

  struct Sample
  {
    Eigen::Vector2d c;          // 잰 중심 (base_link xy)
    Eigen::Vector3d cam_p;      // 그때 카메라 자세
    Eigen::Quaterniond cam_q;
    double side{0.0};           // 평면에 내린 태그 한 변 [m] (태그 크기와 다르면 평면 높이 · 카메라 TF 가 틀림)
  };
  bool same_pose(const Sample & a, const Sample & b) const;
  static Eigen::Vector2d med(const std::deque<Sample> & g);
  bool puck_minus_cell(size_t i, Eigen::Vector2d & d, int & pairs) const;
  Eigen::Vector2d cell_median(size_t i) const {return med(got_[i]);}
  std::string report() const;

  rclcpp::Node & node_;
  tf2_ros::Buffer & tf_;
  TagPlaneConfig cfg_;
  std::string camera_frame_, base_frame_;
  std::vector<std::string> names_;
  std::map<int, size_t> index_;   // 태그 ID → 칸 번호
  std::vector<Eigen::Vector2d> cells_;
  Eigen::Vector2d pivot_;
  double max_rate_hz_{5.0}, still_m_{0.001}, still_rad_{0.005}, min_span_m_{0.06}, tf_timeout_s_{0.05}, mismatch_m_{0.02};
  int samples_{60}, min_samples_{5};
  double held_max_dist_m_{0.15};
  std::atomic<bool> empty_scan_{false};   // 빈손 스캔 중 (쥔 퍽 판정 안 함)
  std::string wrist_frame_;
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr held_pub_, held_fresh_pub_, pnp_pub_;

  std::vector<std::deque<Sample>> got_;        // 칸마다 멈춘 장에서 잰 중심 · 카메라 자세 (최근 samples_ 개)
  std::vector<std::deque<Sample>> puck_got_;   // 칸마다 그 안 퍽 윗면 태그 중심 · 카메라 자세
  double pose_tol_m_{0.003}, pose_tol_rad_{0.0175}, wall_gap_m_{0.0025};
  std::set<int> puck_ids_;
  TagPlaneConfig cfg_puck_;
  std::map<int, size_t> wrong_;                    // 다른 칸 자리에서 본 태그 ID → 그 칸
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  bool have_last_{false};
  rclcpp::Time last_;
  Eigen::Isometry3d prev_cam_{Eigen::Isometry3d::Identity()};

  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr pub_;
  rclcpp::Subscription<interfaces::msg::WristTagArray>::SharedPtr tags_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr report_srv_, reset_srv_, dump_srv_;
  struct Raw
  {
    double t;
    int tag;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    std::array<Eigen::Vector2d, 4> cn;   // 왜곡 푼 정규 좌표 모서리
  };
  std::vector<Raw> raw_;
  std::string dump_path_;

  mutable std::mutex seen_mtx_;            // recent_puck_tags_ (on_tags ↔ control_master 주기)
  std::deque<PuckTagSeen> recent_puck_tags_;   // 최근 2 s 퍽 윗면 태그 원본

  // perception 태그 단계 켜기 · 끄기 (want 는 control_master 주기, 응답은 이 그룹 → mtx_)
  std::mutex mtx_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_cli_;
  std::string enable_service_;
  bool want_{false}, known_{false}, on_{false}, pending_{false};
  double sent_s_{0.0}, last_try_s_{-1e9}, ready_check_s_{-1e9};
  int gen_{0};
};

}  // namespace manipulation
