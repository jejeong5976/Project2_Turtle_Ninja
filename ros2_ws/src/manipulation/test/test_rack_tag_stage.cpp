// test_rack_tag_stage.cpp — 손목 태그 → 적재함 칸 · 쥔 퍽 (rack_tag_stage) 시험, ROS 노드 하나 안에서
//   카메라를 S1 칸 위 0.30 m 에서 수직으로 내려다보게 static TF → 칸 바닥 태그 모서리 (정규 좌표) 를 만들어 보냄
//   보는 것: /rack_tags/detections 칸 이름 · 중심 (0.5 mm 안) · 멈춤 표시, 쥔 퍽 태그 → /held_puck/tag (wrist_link), 켜기 서비스 호출,
//     퍽 윗면 태그 PnP → /rack_tags/puck_pnp (카메라 광학 프레임, 넣은 뒤 보기: 위치 · 높이 · 기울기 · 면, 두 해 모호하면 덜 기운 해)

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "../src/rack_tag_stage.hpp"

using manipulation::RackTagStage;
using namespace std::chrono_literals;

namespace
{
constexpr double kCamZ = 0.30, kFloor = 0.1325, kTag = 0.0246;

geometry_msgs::msg::TransformStamped tf_msg(const std::string & parent, const std::string & child, double x, double y, double z,
  double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  t.transform.rotation.x = qx;
  t.transform.rotation.y = qy;
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;
  return t;
}

// 카메라 (base_link 의 (cx, cy, kCamZ), 광축 = −z, 광학 x = base x, 광학 y = −base y) 에서 본 base 점 → 정규 좌표
cv::Point2f to_norm(double cx, double cy, double x, double y, double z)
{
  const double X = x - cx, Y = -(y - cy), Z = kCamZ - z;
  return {static_cast<float>(X / Z), static_cast<float>(Y / Z)};
}

interfaces::msg::WristTag tag(int id, const std::vector<cv::Point2f> & n, bool rescue = false)
{
  interfaces::msg::WristTag t;
  t.id = id;
  for (int j = 0; j < 4; ++j) {
    t.corners_norm[2 * j] = n[j].x;
    t.corners_norm[2 * j + 1] = n[j].y;
    t.corners_px[2 * j] = 320.0f + 600.0f * n[j].x;
    t.corners_px[2 * j + 1] = 240.0f + 600.0f * n[j].y;
  }
  t.rescue = rescue;
  return t;
}

class RackTagStageTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions o;
    o.parameter_overrides({{"rack_tags.cell_ids", std::vector<int64_t>{20, 21}}, {"rack_tags.floor_z", kFloor},
      {"rack_tags.tag_size_m", kTag}, {"rack_tags.enable_service", std::string("/rack_stage_test/enable")},
      {"rack_tags.tags_topic", std::string("/rack_stage_test/tags")}});
    node_ = std::make_shared<rclcpp::Node>("rack_stage_test", o);
    node_->declare_parameter<std::vector<std::string>>("rack_cell_names", {"S1", "S2"});
    node_->declare_parameter<std::vector<double>>("rack_cell_x", {0.0, 0.06});
    node_->declare_parameter<std::vector<double>>("rack_cell_y", {0.10, 0.10});
    tf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    listen_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
    stat_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    // 광학 프레임: x = base x, y = −base y, z = −base z  (x 축 둘레 180° → q = (1, 0, 0, 0)). wrist_link = 카메라와 같은 자리 (부모 하나만)
    stat_->sendTransform({tf_msg("base_link", "cam", 0.0, 0.10, kCamZ, 1, 0, 0, 0), tf_msg("cam", "wrist_link", 0, 0, 0, 0, 0, 0, 1)});
    srv_ = node_->create_service<std_srvs::srv::SetBool>("/rack_stage_test/enable",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> q, std::shared_ptr<std_srvs::srv::SetBool::Response> r) {
        calls_.push_back(q->data);
        r->success = true;
      });
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    stage_ = std::make_unique<RackTagStage>(*node_, *tf_, group_);
    pub_ = node_->create_publisher<interfaces::msg::WristTagArray>("/rack_stage_test/tags", 10);
    sub_ = node_->create_subscription<interfaces::msg::DetectionArray>("/rack_tags/detections", 10,
      [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {racks_.push_back(*m);});
    held_sub_ = node_->create_subscription<interfaces::msg::DetectionArray>("/held_puck/tag", 10,
      [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {helds_.push_back(*m);});
    exec_.add_node(node_);
  }
  void TearDown() override
  {
    exec_.remove_node(node_);
    stage_.reset();
  }
  template<class F>
  bool spin_until(F done, double timeout_s = 3.0)
  {
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::duration<double>(timeout_s)) {
      exec_.spin_some(10ms);
      if (done()) {return true;}
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }
  void send(const std::vector<interfaces::msg::WristTag> & tags, double t_s)
  {
    interfaces::msg::WristTagArray m;
    m.header.stamp = rclcpp::Time(static_cast<int64_t>(t_s * 1e9), RCL_ROS_TIME);
    m.header.frame_id = "cam";
    m.tags = tags;
    pub_->publish(m);
  }
  // 칸 바닥 (cx, cy) 의 태그 모서리 (aruco 순서: 화면 좌상 · 우상 · 우하 · 좌하)
  std::vector<cv::Point2f> floor_tag(double x, double y) const
  {
    const double h = kTag / 2;
    return {to_norm(0.0, 0.10, x - h, y + h, kFloor), to_norm(0.0, 0.10, x + h, y + h, kFloor),
      to_norm(0.0, 0.10, x + h, y - h, kFloor), to_norm(0.0, 0.10, x - h, y - h, kFloor)};
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listen_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> stat_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::unique_ptr<RackTagStage> stage_;
  rclcpp::Publisher<interfaces::msg::WristTagArray>::SharedPtr pub_;
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr sub_, held_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
  std::vector<interfaces::msg::DetectionArray> racks_, helds_;
  std::vector<bool> calls_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};
}  // namespace

// 칸 바닥 태그 S1 (+2 mm, −1 mm 어긋남) → 칸 S1, 중심 0.5 mm 안, 두 번째 장 (0.2 s 뒤, 카메라 그대로) 은 멈춤 (confidence 1)
TEST_F(RackTagStageTest, FloorTagToCell)
{
  ASSERT_TRUE(spin_until([&] {return tf_->canTransform("base_link", "cam", tf2::TimePointZero);}));
  const double x = 0.002, y = 0.099;
  send({tag(20, floor_tag(x, y)), tag(99, floor_tag(0.05, 0.05))}, 100.0);   // 99 = 모르는 태그 (안 씀)
  ASSERT_TRUE(spin_until([&] {return racks_.size() == 1;}));
  send({tag(20, floor_tag(x, y))}, 100.2);
  ASSERT_TRUE(spin_until([&] {return racks_.size() == 2;}));
  for (const auto & m : racks_) {
    ASSERT_EQ(m.detections.size(), 1u);
    const auto & d = m.detections[0];
    EXPECT_EQ(d.kind, "rack_tag");
    EXPECT_EQ(d.label, "S1");
    EXPECT_EQ(d.id, "20");
    EXPECT_NEAR(d.pose.position.x, x, 5e-4);
    EXPECT_NEAR(d.pose.position.y, y, 5e-4);
    EXPECT_NEAR(d.pose.position.z, kFloor, 1e-9);
    EXPECT_EQ(m.header.frame_id, "base_link");
  }
  EXPECT_FLOAT_EQ(racks_[0].detections[0].confidence, 0.5f);   // 첫 장은 앞 장이 없어 멈춤을 모름
  EXPECT_FLOAT_EQ(racks_[1].detections[0].confidence, 1.0f);
}

// 칸 S1 태그 ID (20) 가 S2 자리에서 보이면 ID 잘못 붙임 → 안 씀
TEST_F(RackTagStageTest, WrongCellRejected)
{
  ASSERT_TRUE(spin_until([&] {return tf_->canTransform("base_link", "cam", tf2::TimePointZero);}));
  send({tag(20, floor_tag(0.06, 0.10))}, 100.0);
  ASSERT_TRUE(spin_until([&] {return racks_.size() == 1;}));
  EXPECT_TRUE(racks_[0].detections.empty());
}

// 카메라 0.10 m 앞 퍽 태그 (15.3 mm) = 쥔 퍽 → /held_puck/tag (wrist_link), 본 검출에 있으면 rescue 는 안 씀
//   본 검출에 없으면 rescue 태그를 씀
TEST_F(RackTagStageTest, HeldTagAndRescue)
{
  ASSERT_TRUE(spin_until([&] {return tf_->canTransform("wrist_link", "cam", tf2::TimePointZero);}));
  const double h = 0.0153 / 2, z = 0.10;
  const std::vector<cv::Point2f> n{{float(-h / z), float(-h / z)}, {float(h / z), float(-h / z)}, {float(h / z), float(h / z)},
    {float(-h / z), float(h / z)}};
  send({tag(3, n), tag(4, n, true)}, 100.0);
  ASSERT_TRUE(spin_until([&] {return helds_.size() == 1;}));
  ASSERT_EQ(helds_[0].detections.size(), 1u);
  EXPECT_EQ(helds_[0].detections[0].id, "3");
  EXPECT_EQ(helds_[0].header.frame_id, "wrist_link");
  EXPECT_NEAR(helds_[0].detections[0].pose.position.z, z, 1e-3);
  send({tag(5, n, true)}, 100.2);
  ASSERT_TRUE(spin_until([&] {return helds_.size() == 2;}));
  EXPECT_EQ(helds_[1].detections[0].id, "5");
}

// 켜기 서비스: want(true) → true 한 번, 같은 값이면 더 안 부름, want(false) → false
TEST_F(RackTagStageTest, EnableServiceFollowsWant)
{
  double t = 0.0;
  ASSERT_TRUE(spin_until([&] {stage_->want(true, t += 0.05); return calls_.size() == 1;}));
  EXPECT_TRUE(calls_[0]);
  for (int i = 0; i < 20; ++i) {stage_->want(true, t += 0.05); exec_.spin_some(5ms);}
  EXPECT_EQ(calls_.size(), 1u);
  ASSERT_TRUE(spin_until([&] {stage_->want(false, t += 0.05); return calls_.size() == 2;}));
  EXPECT_FALSE(calls_[1]);
}

// ---------------------------------------------------------------------------- 퍽 윗면 태그 PnP (/rack_tags/puck_pnp, 넣은 뒤 보기)
namespace
{
constexpr double kPuckTag = 0.0153, kTop = kFloor + 0.06;

// base 자세 (중심 c, 축 R: z = 윗면 법선) 의 퍽 태그 모서리 → 정규 좌표 (IPPE_SQUARE 순서, 카메라 (0, 0.10, kCamZ) 수직 아래)
std::vector<cv::Point2f> puck_tag(const Eigen::Vector3d & c, const Eigen::Matrix3d & R)
{
  const double h = kPuckTag / 2;
  std::vector<cv::Point2f> n;
  for (const auto & o : {Eigen::Vector2d(-h, h), Eigen::Vector2d(h, h), Eigen::Vector2d(h, -h), Eigen::Vector2d(-h, -h)}) {
    const Eigen::Vector3d p = c + R.col(0) * o.x() + R.col(1) * o.y();
    n.push_back(to_norm(0.0, 0.10, p.x(), p.y(), p.z()));
  }
  return n;
}

// 면 yaw (z 둘레) 뒤 기울기 (축 (cos a, sin a, 0) 둘레 tilt)
Eigen::Matrix3d puck_rot(double yaw_deg, double tilt_deg, double axis_deg)
{
  const double a = axis_deg * M_PI / 180.0;
  return (Eigen::AngleAxisd(tilt_deg * M_PI / 180.0, Eigen::Vector3d(std::cos(a), std::sin(a), 0.0)) *
    Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();
}

// 카메라 광학 프레임 PnP 자세 → base (fixture static TF: 광학 x = base x, y = −base y, z = −base z, 원점 (0, 0.10, kCamZ))
void pnp_to_base(const interfaces::msg::Detection & d, Eigen::Vector3d & p, Eigen::Matrix3d & R)
{
  const Eigen::Matrix3d Rbc = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
  p = Rbc * Eigen::Vector3d(d.pose.position.x, d.pose.position.y, d.pose.position.z) + Eigen::Vector3d(0.0, 0.10, kCamZ);
  R = Rbc * Eigen::Quaterniond(d.pose.orientation.w, d.pose.orientation.x, d.pose.orientation.y, d.pose.orientation.z).toRotationMatrix();
}

double tilt_deg(const Eigen::Matrix3d & R) {return std::acos(std::clamp(R(2, 2), -1.0, 1.0)) * 180.0 / M_PI;}
}  // namespace

class RackTagStagePnpTest : public RackTagStageTest
{
protected:
  void SetUp() override
  {
    RackTagStageTest::SetUp();
    pnp_sub_ = node_->create_subscription<interfaces::msg::DetectionArray>("/rack_tags/puck_pnp", 10,
      [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {pnps_.push_back(*m);});
  }
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr pnp_sub_;
  std::vector<interfaces::msg::DetectionArray> pnps_;
};

// 칸 S1 안 퍽 7: (+3, −2) mm 어긋남 · 윗면 8 mm 뜸 · 면 25° · 20° 기욺 → 카메라 광학 프레임 PnP 한 개 (kind puck_pnp, label S1, frame cam)
//   base 로 바꾸면 중심 0.3 mm · 기울기 0.5° · 면 0.5° 안 (모서리 잡음 없음). camera_info 없음 → 초점 거리는 모서리로 (모호 판정만)
TEST_F(RackTagStagePnpTest, PuckTopPoseInCameraFrame)
{
  ASSERT_TRUE(spin_until([&] {return tf_->canTransform("base_link", "cam", tf2::TimePointZero);}));
  const Eigen::Vector3d c(0.003, 0.098, kTop + 0.008);
  const Eigen::Matrix3d R = puck_rot(25.0, 20.0, 60.0);
  send({tag(7, puck_tag(c, R))}, 100.0);
  ASSERT_TRUE(spin_until([&] {return pnps_.size() == 1;}));
  const auto & m = pnps_[0];
  EXPECT_EQ(m.header.frame_id, "cam");
  ASSERT_EQ(m.detections.size(), 1u);
  const auto & d = m.detections[0];
  EXPECT_EQ(d.kind, "puck_pnp");
  EXPECT_EQ(d.id, "7");
  EXPECT_EQ(d.label, "S1");
  EXPECT_FLOAT_EQ(d.confidence, 1.0f);
  Eigen::Vector3d p;
  Eigen::Matrix3d Rb;
  pnp_to_base(d, p, Rb);
  EXPECT_NEAR(p.x(), c.x(), 3e-4);
  EXPECT_NEAR(p.y(), c.y(), 3e-4);
  EXPECT_NEAR(p.z(), c.z(), 3e-4);
  EXPECT_NEAR(tilt_deg(Rb), 20.0, 0.5);
  EXPECT_NEAR(Eigen::AngleAxisd(R.transpose() * Rb).angle() * 180.0 / M_PI, 0.0, 0.5);   // 면 · 기울기 방향까지 같은 자세
}

// 바로 앉은 퍽 (기울기 0, 칸 S2 쪽 비스듬히 봄) → 기울기 1° 안, label S2. 칸 바닥 태그 · rescue 퍽 태그 · 모르는 태그는 PnP 안 냄
TEST_F(RackTagStagePnpTest, SeatedPuckAndNoPnpForOtherTags)
{
  ASSERT_TRUE(spin_until([&] {return tf_->canTransform("base_link", "cam", tf2::TimePointZero);}));
  send({tag(20, floor_tag(0.0, 0.10)), tag(99, floor_tag(0.03, 0.05)), tag(4, puck_tag({0.0, 0.10, kTop}, puck_rot(0, 0, 0)), true)}, 100.0);
  ASSERT_TRUE(spin_until([&] {return racks_.size() == 1;}));
  spin_until([&] {return false;}, 0.3);
  EXPECT_TRUE(pnps_.empty());   // 퍽 태그 PnP 가 없으면 토픽을 안 냄
  const Eigen::Vector3d c(0.061, 0.101, kTop);
  send({tag(8, puck_tag(c, puck_rot(10.0, 0.0, 0.0))), tag(20, floor_tag(0.0, 0.10))}, 100.2);
  ASSERT_TRUE(spin_until([&] {return pnps_.size() == 1;}));
  ASSERT_EQ(pnps_[0].detections.size(), 1u);
  const auto & d = pnps_[0].detections[0];
  EXPECT_EQ(d.id, "8");
  EXPECT_EQ(d.label, "S2");
  Eigen::Vector3d p;
  Eigen::Matrix3d Rb;
  pnp_to_base(d, p, Rb);
  EXPECT_NEAR((p - c).norm(), 0.0, 5e-4);
  EXPECT_LT(tilt_deg(Rb), 1.0);
}

// 모호한 두 해 (재투영 오차가 비슷): 거의 수직으로 본 작은 태그를 시선 쪽으로 8° 기울이고 모서리를 0.15 px 흔듦 → 거울 해는 약 2.6° (시선 5.3° × 2 − 8)
//   → 모호한 장은 덜 기운 해 (confidence 0.5), 그 장들 기울기 가운데값이 실제보다 3° 넘게 작음 (장 하나는 15 mm 태그라 수 ° 흔들림 → 가운데값으로 봄)
//   초점 거리는 camera_info 없이 모서리로 (못 구하면 모호 판정이 안 돼 0.5 가 안 나옴)
TEST_F(RackTagStagePnpTest, AmbiguousPicksLessTilted)
{
  ASSERT_TRUE(spin_until([&] {return tf_->canTransform("base_link", "cam", tf2::TimePointZero);}));
  const Eigen::Vector3d c(0.010, 0.100, kTop);   // 카메라 바로 아래에서 10 mm → 시선이 수직에서 −x 쪽 5.3°
  const double true_tilt = 8.0;
  const Eigen::Matrix3d R = puck_rot(0.0, -true_tilt, 90.0);   // 윗면 법선을 −x (시선 쪽) 으로 8°
  std::vector<double> amb_tilt;
  for (int k = 0; k < 12; ++k) {
    auto cn = puck_tag(c, R);
    for (int j = 0; j < 4; ++j) {   // ±0.15 px (초점 600) 결정적 흔들림
      cn[j].x += static_cast<float>(0.15 / 600.0 * std::sin(1.7 * k + 2.1 * j));
      cn[j].y += static_cast<float>(0.15 / 600.0 * std::cos(1.3 * k + 0.7 * j));
    }
    send({tag(9, cn)}, 100.0 + 0.2 * k);
    ASSERT_TRUE(spin_until([&] {return static_cast<int>(pnps_.size()) == k + 1;}));
    const auto & d = pnps_.back().detections.at(0);
    Eigen::Vector3d p;
    Eigen::Matrix3d Rb;
    pnp_to_base(d, p, Rb);
    EXPECT_NEAR((p - c).norm(), 0.0, 1e-3);
    if (d.confidence < 0.75f) {amb_tilt.push_back(tilt_deg(Rb));}
  }
  ASSERT_GE(amb_tilt.size(), 6u);
  EXPECT_LT(manipulation::median(amb_tilt), true_tilt - 3.0);
}

// perception 을 다시 켬 (켜기 서비스가 없어졌다 다시 생김): 켰다고 알고 있어도 다시 true 를 보냄 (새 perception 은 태그 단계가 꺼진 채 시작)
TEST_F(RackTagStageTest, EnableResentAfterPerceptionRestart)
{
  double t = 0.0;
  ASSERT_TRUE(spin_until([&] {stage_->want(true, t += 0.05); return calls_.size() == 1;}));
  srv_.reset();   // perception 꺼짐
  spin_until([&] {stage_->want(true, t += 0.05); return false;}, 1.5);
  EXPECT_EQ(calls_.size(), 1u);
  srv_ = node_->create_service<std_srvs::srv::SetBool>("/rack_stage_test/enable",   // 다시 켜짐
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> q, std::shared_ptr<std_srvs::srv::SetBool::Response> r) {
      calls_.push_back(q->data);
      r->success = true;
    });
  ASSERT_TRUE(spin_until([&] {stage_->want(true, t += 0.05); return calls_.size() == 2;}, 5.0));
  EXPECT_TRUE(calls_[1]);
  for (int i = 0; i < 40; ++i) {stage_->want(true, t += 0.05); exec_.spin_some(5ms);}
  EXPECT_EQ(calls_.size(), 2u);   // 살아 있으면 더 안 보냄
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
