// test_auto_initial_pose_node.cpp — test_auto_initial_pose_runtime.py 를 옮김.
// 위치 보정 노드의 신선도·이동 취소·자동 재시도 회귀. 장비/모터 없이 실행함 (ROS_DOMAIN_ID 187, localhost).
#include <gtest/gtest.h>

#include <cstdlib>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "navigation/auto_initial_pose_node.hpp"
#include "startup_fixture.hpp"

using navigation::AutoInitialPose;
using navigation::pose_match::JobOutput;
using navigation::pose_match::MapData;
using navigation::pose_match::Pose;

namespace
{
class TestNode : public AutoInitialPose
{
public:
  std::vector<std::tuple<std::string, bool, std::string>> statuses;
  int publishes = 0;
  int submits = 0;
  std::vector<std::promise<JobOutput>> pending;   // 끝나지 않은 작업 (Mock pool.submit)
  bool mock_inputs = true;
  Inputs inputs;
  std::optional<std::string> input_error;

  Inputs current_inputs() override
  {
    if (!mock_inputs) {
      return AutoInitialPose::current_inputs();
    }
    if (input_error) {
      throw InputError(*input_error);
    }
    return inputs;
  }
  void status(const std::string & state, bool ready, const std::string & reason, const Extra & = {}) override
  {
    statuses.emplace_back(state, ready, reason);
  }
  void publish_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &) override {publishes += 1;}
  std::future<JobOutput> submit_job(std::shared_ptr<const MapData>, std::shared_ptr<const navigation::pose_match::MapModel>,
    Points, Pose, Pose) override
  {
    submits += 1;
    pending.emplace_back();
    return pending.back().get_future();
  }
  const std::tuple<std::string, bool, std::string> & last() const {return statuses.back();}
};

class RuntimeTests : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 0);
    rclcpp::InitOptions opts;
    opts.set_domain_id(187);
    rclcpp::init(0, nullptr, opts);
    model = startup_test::fixture();
    points = startup_test::raycast(*model, truth);
    result = navigation::pose_match::match(*model, points, current, current);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
    model.reset();
  }

  void SetUp() override
  {
    node = std::make_shared<TestNode>();
    auto data = std::make_shared<MapData>();
    data->grid = model->grid;
    data->width = model->width;
    data->height = model->height;
    data->resolution = model->resolution;
    data->origin = model->origin;
    data->frame = "map";
    node->map_data = data;
    node->map_revision = 1;
    node->odom_pose = Pose{0, 0, 0};
    node->still = true;
    node->inputs = AutoInitialPose::Inputs{points, current};
  }
  void TearDown() override {node.reset();}

  void complete_job(int64_t epoch = 0, int64_t map_revision = 1)
  {
    node->job = AutoInitialPose::Job{epoch, map_revision, Pose{0, 0, 0}};
    std::promise<JobOutput> p;
    p.set_value(JobOutput{result, model});
    node->future = p.get_future();
  }

  static inline std::shared_ptr<const navigation::pose_match::MapModel> model;
  static inline const Pose truth{.15, -.15, .08};
  static inline const Pose current{.45, -.15, .08};
  static inline navigation::pose_match::Points points;
  static inline navigation::pose_match::MatchResult result;
  std::shared_ptr<TestNode> node;
};
}  // namespace

TEST_F(RuntimeTests, StationaryImprovementPublishesButNotReadyYet)
{
  complete_job();
  node->tick();
  EXPECT_EQ(node->publishes, 1);
  EXPECT_EQ(std::get<0>(node->last()), "published");
  EXPECT_FALSE(std::get<1>(node->last()));
  node->inputs = AutoInitialPose::Inputs{points, truth};
  node->tick();
  EXPECT_EQ(std::get<0>(node->last()), "aligned");
  EXPECT_TRUE(std::get<1>(node->last()));
}

TEST_F(RuntimeTests, MotionDuringWorkerDiscardsEvenAfterReturning)
{
  complete_job();
  node->motion_epoch = 1;
  node->tick();
  EXPECT_EQ(node->publishes, 0);
  EXPECT_EQ(std::get<2>(node->last()), "moved_or_map_changed");
  EXPECT_TRUE(node->need_match);
}

TEST_F(RuntimeTests, MapChangeDiscards)
{
  complete_job(0, 0);
  node->tick();
  EXPECT_EQ(node->publishes, 0);
  EXPECT_EQ(std::get<2>(node->last()), "moved_or_map_changed");
}

TEST_F(RuntimeTests, StaleInputsCancelThenRetry)
{
  complete_job();
  node->input_error = "scan_odom_stale";
  node->tick();
  EXPECT_EQ(node->publishes, 0);
  EXPECT_GT(node->motion_epoch, 0);
  EXPECT_EQ(std::get<2>(node->last()), "scan_odom_stale");
  node->input_error.reset();
  node->tick();
  EXPECT_EQ(node->publishes, 0);
  EXPECT_TRUE(node->need_match);
}

TEST_F(RuntimeTests, MovingDefersLowHealthWithoutWorkerSubmission)
{
  node->still = false;
  node->health_low = node->need_match = true;
  node->tick();
  EXPECT_EQ(node->submits, 0);
  EXPECT_EQ(node->publishes, 0);
  EXPECT_EQ(std::get<0>(node->last()), "deferred_motion");
  node->still = true;
  node->tick();
  EXPECT_EQ(node->submits, 1);
}

TEST_F(RuntimeTests, CurrentTfAlreadyCorrectDoesNotRepublish)
{
  complete_job();
  node->inputs = AutoInitialPose::Inputs{points, truth};
  node->tick();
  EXPECT_EQ(node->publishes, 0);
  EXPECT_EQ(std::get<0>(node->last()), "aligned");
  EXPECT_TRUE(std::get<1>(node->last()));
}

TEST_F(RuntimeTests, OdomVelocityMarksMotionEpoch)
{
  nav_msgs::msg::Odometry m;
  m.pose.pose.orientation.w = 1.;
  m.header.stamp = node->get_clock()->now();
  m.twist.twist.linear.x = .1;
  node->on_odom(m);
  EXPECT_FALSE(node->still);
  EXPECT_EQ(node->motion_epoch, 1);
  m.twist.twist.linear.x = 0.;
  node->on_odom(m);
  EXPECT_TRUE(node->still);
  EXPECT_EQ(node->motion_epoch, 1);
}

TEST_F(RuntimeTests, ScanBeforeMotionNotUsed)
{
  // 실제 current_inputs를 직접 불러 지난 scan 판정이 TF 참조보다 앞서는지 확인함.
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.stamp = node->get_clock()->now();
  node->scan = scan;
  node->scan_received = node->odom_received = AutoInitialPose::monotonic();
  node->odom_stamp = AutoInitialPose::stamp_seconds(scan->header.stamp);
  node->last_motion_stamp = node->odom_stamp + .01;
  node->mock_inputs = false;
  try {
    node->current_inputs();
    FAIL() << "scan_before_stationary 예외 없음";
  } catch (const AutoInitialPose::InputError & e) {
    EXPECT_STREQ(e.what(), "scan_before_stationary");
  }
}

// C++ 판 추가: 미션이 STANDBY 가 아니면 입력·계산 없이 상태만 냄 (진행 중 계산은 버림)
TEST_F(RuntimeTests, NonStandbyMissionPausesWithoutInputs)
{
  complete_job();
  node->mission_state = "RUNNING";
  node->input_error = "should_not_be_called";
  node->tick();
  EXPECT_EQ(node->last(), std::make_tuple(std::string("paused_mission"), true, std::string("mission_RUNNING")));
  EXPECT_EQ(node->motion_epoch, 1);
  EXPECT_EQ(node->publishes, 0);
}

// C++ 판 추가: 상태 JSON 글자 (파이썬 json.dumps(data, ensure_ascii=False, allow_nan=False))
TEST_F(RuntimeTests, StatusJsonText)
{
  class Capture : public AutoInitialPose
  {
  public:
    std::string text;
    Capture()
    {
      sub = create_subscription<std_msgs::msg::String>("/startup/pose_match_status",
          rclcpp::QoS(1).transient_local().reliable(), [this](std_msgs::msg::String::ConstSharedPtr m) {text = m->data;});
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  };
  auto n = std::make_shared<Capture>();
  n->last_result.score = 0.9583333333333334;
  n->last_result.current_score = 0.5;
  n->last_result.elapsed_s = 0.25;
  n->attempt = 2;
  n->published = 1;
  n->status("matching", false, "worker", AutoInitialPose::with_current_score(std::nullopt));
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(n);
  for (int i = 0; i < 100 && n->text.find("\"matching\"") == std::string::npos; ++i) {
    ex.spin_some(std::chrono::milliseconds(20));
  }
  // 기대값 = python3 json.dumps({...}, ensure_ascii=False, allow_nan=False)
  EXPECT_EQ(n->text, "{\"state\": \"matching\", \"ready\": false, \"reason\": \"worker\", \"score\": 0.9583333333333334, "
    "\"current_score\": null, \"elapsed_s\": 0.25, \"attempt\": 2, \"published\": 1}");
}
