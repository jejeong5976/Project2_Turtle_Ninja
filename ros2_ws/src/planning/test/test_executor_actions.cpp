// test_executor_actions.cpp — 실행기 액션 처리 시험: 가짜 Nav2 · /arm_task 서버를 같은 프로세스에 띄움
//   - 새 목표가 받아들여지기 전 stop() → 끝난 이전 목표 핸들로 취소하지 않음 (예외로 노드가 죽지 않음)
//   - 팔 성공 결과가 온 뒤 step() 전에 stop() → 재고에 한 번만 반영
//   - 팔 시간 초과 뒤 최종 결과가 끝내 안 옴 (server_wait_s · 서버 없어짐) → 재시도 대신 FAULT
//   - 취소 결과 기다리는 중 서버가 없어졌다 다시 뜸 → 더 안 기다림 (resume 안 막힘)
//   - 손목 카메라 drift: 재검증용 원시 관측만 되돌린 위치, 기록부는 명목 TF. 다른 프레임 drift 는 무시
//   - perception 적재함 판독: 처음 끔 → LOOK_RACK 보낼 때 켬 → 칸 측정 기다리기 끝나면 끔
//   실행: colcon test --packages-select planning (ROS_DOMAIN_ID · ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST 권장)
//   액션 · 발행 이름은 시험마다 고유 이름공간으로 바꿈 → 같은 도메인의 실물 /arm_task · Nav2 에 안 닿음

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "planning/executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

using namespace planning;
using Nav = nav2_msgs::action::NavigateToPose;
using ArmTask = interfaces::action::ArmTask;

namespace
{

// 받아들이고 끝내지 않는 서버 (끝내기 · 취소 처리는 시험이 직접)
template<typename A>
struct FakeServer
{
  using GoalHandle = rclcpp_action::ServerGoalHandle<A>;
  typename rclcpp_action::Server<A>::SharedPtr server;
  std::vector<std::shared_ptr<GoalHandle>> goals;
  std::vector<rclcpp_action::GoalUUID> cancel_requests;

  FakeServer(rclcpp::Node::SharedPtr node, const std::string & name)
  {
    server = rclcpp_action::create_server<A>(node, name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename A::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle> gh) {
        cancel_requests.push_back(gh->get_goal_id());
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> gh) {goals.push_back(gh);});
  }
  // 취소 요청 받은 목표는 취소로 끝냄
  void finish_canceling()
  {
    for (auto & g : goals) {
      if (g->is_active() && g->is_canceling()) {g->canceled(std::make_shared<typename A::Result>());}
    }
  }
};

class ExecutorActions : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = std::filesystem::temp_directory_path() / ("planning_exec_test_" + std::to_string(::getpid()));
    std::filesystem::create_directories(dir_);
  }

  void TearDown() override
  {
    if (client_) {
      client_ex_->remove_node(client_);
      server_ex_->remove_node(server_);
    }
    exec_.reset();
    client_.reset();
    nav_srv_.reset();
    arm_srv_.reset();
    rack_srv_.reset();
    wrist_srv_.reset();
    server_.reset();
    std::filesystem::remove_all(dir_);
  }

  // gripper_puck 이 있으면 그리퍼에 그 퍽을 쥔 재고 기록으로 시작
  void start(const std::string & gripper_puck = "", double server_wait_s = 30.0)
  {
    const std::string cargo_file = (dir_ / "cargo.yaml").string();
    if (!gripper_puck.empty()) {
      Cargo c({"S1", "S2", "S3", "S8", "S11", "S6", "S12", "S7", "S10", "S5", "S9", "S4"});   // 실행기 기본 cargo_slots 와 같은 순서 (파일 칸 순서가 다르면 복원 실패)
      CargoItem it;
      it.puck_id = gripper_puck;
      ASSERT_TRUE(c.on_pick(it));
      ASSERT_TRUE(c.save(cargo_file));
    }
    // 클라이언트 노드 이름공간 = ns_ (상대 이름 navigate_to_pose · cmd_vel_safe), 절대 이름은 재매핑
    ns_ = "/exec_test_" + std::to_string(::getpid()) + "_" + std::to_string(++run_);
    rclcpp::NodeOptions o;
    o.use_global_arguments(false);
    o.arguments({"--ros-args", "-r", "/arm_task:=" + ns_ + "/arm_task",
      "-r", "/mission/cargo:=" + ns_ + "/cargo", "-r", "/planning/markers:=" + ns_ + "/markers",
      "-r", "/wrist_cam/drift:=" + ns_ + "/drift", "-r", "/perception/wrist/detections:=" + ns_ + "/detections"});
    o.parameter_overrides({
      {"require_scan", false}, {"require_detections", false}, {"require_localization", false},
      {"require_motion_mode", false}, {"lifecycle_services", std::vector<std::string>{}},
      {"cargo_file", cargo_file}, {"cargo_restore", !gripper_puck.empty()}, {"server_wait_s", server_wait_s},
      {"rack_enable_service", ns_ + "/rack_enable"}, {"rack_wait_s", 0.3},
      {"wrist_enable_service", ns_ + "/wrist_enable"}});
    client_ = std::make_shared<rclcpp::Node>("exec_test_client", ns_, o);
    server_ = std::make_shared<rclcpp::Node>("exec_test_server", ns_);
    exec_ = std::make_unique<Executor>(*client_);
    nav_srv_ = std::make_unique<FakeServer<Nav>>(server_, ns_ + "/navigate_to_pose");
    arm_srv_ = std::make_unique<FakeServer<ArmTask>>(server_, ns_ + "/arm_task");
    rack_srv_ = server_->create_service<std_srvs::srv::SetBool>(ns_ + "/rack_enable",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> q, std::shared_ptr<std_srvs::srv::SetBool::Response> r) {
        rack_calls_.push_back(q->data);
        r->success = true;
      });
    wrist_srv_ = server_->create_service<std_srvs::srv::SetBool>(ns_ + "/wrist_enable",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> q, std::shared_ptr<std_srvs::srv::SetBool::Response> r) {
        wrist_calls_.push_back(q->data);
        r->success = true;
      });
    client_ex_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    server_ex_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    client_ex_->add_node(client_);
    server_ex_->add_node(server_);
  }

  double now() {return client_->get_clock()->now().seconds();}

  bool not_ready(const std::string & why)
  {
    const auto r = exec_->ready_reasons(now());
    return std::find(r.begin(), r.end(), why) != r.end();
  }

  // 서버 쪽까지 돌림 (server = false 면 클라이언트만 → 서버는 새 목표를 아직 못 받음)
  void spin(bool server = true)
  {
    client_ex_->spin_some();
    if (server) {server_ex_->spin_some();}
  }

  // cond 가 참이 될 때까지 (매번 tick 실행 후 돌림), 10 s 넘으면 false
  bool spin_until(const std::function<bool()> & cond, const std::function<void()> & tick = [] {})
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < end) {
      tick();
      if (cond()) {return true;}
      spin();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }

  // 팔 목표가 받아들여지고 (피드백 수신) 서버가 성공을 낸 뒤 결과가 클라이언트에 옴 (step() 은 안 부름)
  void arm_succeed_without_step(std::size_t goal_i)
  {
    ASSERT_TRUE(spin_until([&] {return arm_srv_->goals.size() > goal_i;}));
    auto gh = arm_srv_->goals[goal_i];
    auto fb = std::make_shared<ArmTask::Feedback>();
    fb->phase = "moving";
    // 피드백은 가끔만: 액션 클라이언트는 spin 한 번에 하나씩 처리 → 계속 내면 수락 응답이 밀림
    int n = 0;
    ASSERT_TRUE(spin_until([&] {return exec_->arm_phase() == "moving";},
      [&] {if (n++ % 20 == 0) {gh->publish_feedback(fb);}}));
    auto res = std::make_shared<ArmTask::Result>();
    res->success = true;
    gh->succeed(res);
    ASSERT_TRUE(spin_until([&] {return exec_->arm_phase().empty();}));   // 결과 도착 (active 끝)
  }

  static Step look()
  {
    Step s;
    s.type = StepType::Arm;
    s.arm_task = kArmLook;
    s.name = "arm:LOOK";
    return s;
  }

  static Step store(const std::string & puck, const std::string & slot)
  {
    Step s;
    s.type = StepType::Arm;
    s.arm_task = kArmStore;
    s.puck_id = puck;
    s.slot = slot;
    s.name = "arm:STORE " + slot;
    return s;
  }

  static Step go(double x)
  {
    Step s;
    s.type = StepType::Navigate;
    s.target = Target::Fixed;
    s.pose.x = x;
    s.name = "nav " + std::to_string(x);
    return s;
  }

  const std::optional<CargoItem> & slot(const std::string & name) const
  {
    for (const auto & s : exec_->cargo().slots()) {
      if (s.first == name) {return s.second;}
    }
    static const std::optional<CargoItem> none;
    return none;
  }

  std::filesystem::path dir_;
  std::string ns_;
  static inline int run_ = 0;
  rclcpp::Node::SharedPtr client_, server_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> client_ex_, server_ex_;
  std::unique_ptr<Executor> exec_;
  std::unique_ptr<FakeServer<Nav>> nav_srv_;
  std::unique_ptr<FakeServer<ArmTask>> arm_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr rack_srv_;
  std::vector<bool> rack_calls_;   // 적재함 판독 켜기 요청 순서
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr wrist_srv_;
  std::vector<bool> wrist_calls_;  // 손목 검출 켜기 요청 순서
};

}  // namespace

// Nav2 목표 A 성공 → B 보냄 (서버가 아직 안 받음) → stop(): A 핸들로 취소하면 UnknownGoalHandleError
TEST_F(ExecutorActions, NavStopBeforeAcceptCancelsNewGoalOnly)
{
  start();
  exec_->set_steps({go(1.0), go(2.0)});
  ASSERT_TRUE(spin_until([&] {return !nav_srv_->goals.empty();}, [&] {exec_->step(now());}));
  nav_srv_->goals[0]->succeed(std::make_shared<Nav::Result>());
  ExecEvent ev;
  ASSERT_TRUE(spin_until([&] {return ev.kind == ExecEvent::Kind::StepOk;}, [&] {ev = exec_->step(now());}));
  ASSERT_EQ(exec_->index(), 1u);

  exec_->step(now());   // B 보냄
  spin(false);
  ASSERT_EQ(nav_srv_->goals.size(), 1u);
  EXPECT_NO_THROW(exec_->stop());
  EXPECT_FALSE(exec_->cancel_pending_reason().empty());

  // B 가 받아들여지면 취소됨 (A 는 취소 요청 없음)
  ASSERT_TRUE(spin_until([&] {return exec_->cancel_pending_reason().empty();}, [&] {nav_srv_->finish_canceling();}));
  ASSERT_EQ(nav_srv_->goals.size(), 2u);
  ASSERT_EQ(nav_srv_->cancel_requests.size(), 1u);
  EXPECT_EQ(nav_srv_->cancel_requests[0], nav_srv_->goals[1]->get_goal_id());
}

// 팔도 같음: STORE A 성공 → LOOK B 보냄 (안 받음) → stop()   (B 를 STORE 로 하면 재고 확인에서 걸려 안 보냄 — 쥔 퍽 없음)
TEST_F(ExecutorActions, ArmStopBeforeAcceptCancelsNewGoalOnly)
{
  start("P1");
  exec_->set_steps({store("P1", "S1"), look()});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_succeed_without_step(0);
  ASSERT_EQ(exec_->step(now()).kind, ExecEvent::Kind::StepOk);

  exec_->step(now());   // B 보냄
  spin(false);
  ASSERT_EQ(arm_srv_->goals.size(), 1u);
  EXPECT_NO_THROW(exec_->stop());
  EXPECT_FALSE(exec_->cancel_pending_reason().empty());

  ASSERT_TRUE(spin_until([&] {return exec_->cancel_pending_reason().empty();}, [&] {arm_srv_->finish_canceling();}));
  ASSERT_EQ(arm_srv_->goals.size(), 2u);
  ASSERT_EQ(arm_srv_->cancel_requests.size(), 1u);
  EXPECT_EQ(arm_srv_->cancel_requests[0], arm_srv_->goals[1]->get_goal_id());
}

// 팔 성공 결과 도착 → step() 전에 stop() (FAULT) → 다음 update() 가 재고에 반영
TEST_F(ExecutorActions, ArmSuccessBeforeStopIsApplied)
{
  start("P1");
  exec_->set_steps({store("P1", "S1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_succeed_without_step(0);

  exec_->stop();
  exec_->update(now());
  EXPECT_FALSE(exec_->cargo().holding());
  ASSERT_TRUE(slot("S1").has_value());
  EXPECT_EQ(slot("S1")->puck_id, "P1");
}

// step() 이 이미 반영한 성공은 stop() 뒤 다시 반영하지 않음
TEST_F(ExecutorActions, ArmSuccessAppliedOnce)
{
  start("P1");
  exec_->set_steps({store("P1", "S1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_succeed_without_step(0);
  ASSERT_EQ(exec_->step(now()).kind, ExecEvent::Kind::StepOk);
  ASSERT_TRUE(slot("S1").has_value());

  ASSERT_TRUE(exec_->cargo_retrieve("S1", nullptr));   // 다시 그리퍼로 → 한 번 더 반영하면 S1 이 다시 참
  exec_->stop();
  exec_->update(now());
  EXPECT_FALSE(slot("S1").has_value());
  EXPECT_TRUE(exec_->cargo().holding());
}

// 시간 초과 → 취소했는데 서버가 취소 결과를 안 냄 → server_wait_s 뒤 FAULT (재시도 · 실패로 넘기면 같은 작업을 또 함)
//   FAULT 뒤 그 작업이 결국 성공하면 update() 가 재고에 한 번 반영
TEST_F(ExecutorActions, ArmTimeoutWithoutResultFaults)
{
  start("P1", 0.5);
  Step st = store("P1", "S1");
  st.timeout_s = 0.3;
  exec_->set_steps({st});
  ExecEvent ev;
  ASSERT_TRUE(spin_until([&] {return ev.kind != ExecEvent::Kind::None;}, [&] {ev = exec_->step(now());}));
  EXPECT_EQ(ev.kind, ExecEvent::Kind::Fault);
  EXPECT_EQ(ev.code, "INVENTORY_UNCERTAIN");   // 재고를 바꾸는 작업은 재고 불확실 (팀 재고 정책), 그 밖 (LOOK 등) 은 ARM_NO_RESULT
  ASSERT_EQ(arm_srv_->cancel_requests.size(), 1u);

  exec_->stop();
  EXPECT_FALSE(exec_->cancel_pending_reason().empty());
  auto res = std::make_shared<ArmTask::Result>();
  res->success = true;
  arm_srv_->goals[0]->succeed(res);
  ASSERT_TRUE(spin_until([&] {return exec_->cancel_pending_reason().empty();}));
  exec_->update(now());
  EXPECT_FALSE(exec_->cargo().holding());
  ASSERT_TRUE(slot("S1").has_value());
}

// 시간 초과 → 취소 결과 기다리는 중 /arm_task 서버가 없어짐 → FAULT
//   그 뒤 update() 가 취소 대기를 지움 → 서버가 다시 떠도 resume 이 안 막힘
TEST_F(ExecutorActions, ArmTimeoutServerGoneFaultsThenForgets)
{
  start("P1");
  Step st = store("P1", "S1");
  st.timeout_s = 0.3;
  exec_->set_steps({st});
  ExecEvent ev;
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->cancel_requests.empty();}, [&] {ev = exec_->step(now());}));
  ASSERT_EQ(ev.kind, ExecEvent::Kind::None);

  // 목표 핸들은 서버보다 오래 둠 (서버가 살아 있을 때 핸들이 없어지면 ABORTED · CANCELED 결과를 냄 = 결과 없는 소멸이 아님)
  const auto keep = arm_srv_->goals;
  arm_srv_.reset();
  ASSERT_TRUE(spin_until([&] {return ev.kind != ExecEvent::Kind::None;}, [&] {
      exec_->update(now());
      ev = exec_->step(now());
    }));
  EXPECT_EQ(ev.kind, ExecEvent::Kind::Fault);
  EXPECT_EQ(ev.code, "INVENTORY_UNCERTAIN");

  exec_->stop();
  ASSERT_TRUE(not_ready("ARM_SERVER"));
  exec_->update(now());
  arm_srv_ = std::make_unique<FakeServer<ArmTask>>(server_, ns_ + "/arm_task");
  ASSERT_TRUE(spin_until([&] {return !not_ready("ARM_SERVER");}));
  EXPECT_TRUE(exec_->cancel_pending_reason().empty());
}

// Nav2 도 같음: stop() 으로 취소 → 결과 전에 서버가 없어졌다 다시 뜸 → 더 안 기다림
TEST_F(ExecutorActions, NavCancelForgottenWhenServerGone)
{
  start();
  exec_->set_steps({go(1.0)});
  ASSERT_TRUE(spin_until([&] {return !nav_srv_->goals.empty();}, [&] {exec_->step(now());}));
  exec_->stop();
  ASSERT_FALSE(exec_->cancel_pending_reason().empty());

  const auto keep = nav_srv_->goals;   // 서버보다 오래 둠 (위와 같음)
  nav_srv_.reset();
  ASSERT_TRUE(spin_until([&] {return not_ready("NAV2_SERVER");}));
  exec_->update(now());
  nav_srv_ = std::make_unique<FakeServer<Nav>>(server_, ns_ + "/navigate_to_pose");
  ASSERT_TRUE(spin_until([&] {return !not_ready("NAV2_SERVER");}));
  EXPECT_TRUE(exec_->cancel_pending_reason().empty());
}

// drift 3° (카메라 y 축 둘레) + 카메라 정면 0.30 m 검출 → 재검증 관측 x = 0.30 · sin 3°, 기록부 x = 0 (map → odom → 카메라 고정 단위 TF)
TEST_F(ExecutorActions, DriftAppliedToVerifyObservationsOnly)
{
  start();
  const std::string cam = "exec_test_cam_" + std::to_string(::getpid());
  tf2_ros::StaticTransformBroadcaster stb(server_);
  geometry_msgs::msg::TransformStamped t;   // map → odom → 카메라, 모두 단위 (카메라 좌표 = map 좌표)
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  t.transform.rotation.w = 1.0;
  auto t2 = t;
  t2.header.frame_id = "odom";
  t2.child_frame_id = cam;
  stb.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{t, t2});
  auto drift_pub = server_->create_publisher<geometry_msgs::msg::QuaternionStamped>(
    ns_ + "/drift", rclcpp::QoS(1).reliable().transient_local());
  auto det_pub = server_->create_publisher<interfaces::msg::DetectionArray>(ns_ + "/detections", 10);
  auto send_drift = [&](const std::string & frame) {
      tf2::Quaternion q;
      q.setRPY(0.0, 3.0 * M_PI / 180.0, 0.0);
      geometry_msgs::msg::QuaternionStamped m;
      m.header.frame_id = frame;
      m.quaternion.x = q.x();
      m.quaternion.y = q.y();
      m.quaternion.z = q.z();
      m.quaternion.w = q.w();
      drift_pub->publish(m);
    };
  auto send_det = [&](const std::string & id) {
      interfaces::msg::DetectionArray m;
      m.header.frame_id = cam;
      m.header.stamp = client_->now();
      interfaces::msg::Detection d;
      d.header = m.header;
      d.kind = "puck";
      d.id = id;
      d.label = "red";
      d.confidence = 0.9;
      d.pose.position.z = 0.30;
      m.detections.push_back(d);
      det_pub->publish(m);
    };
  auto obs_of = [&](const std::string & id) -> const ObsRecord * {
      const auto & o = exec_->recent_obs();
      for (auto it = o.rbegin(); it != o.rend(); ++it) {if (it->id == id) {return &*it;}}
      return nullptr;
    };
  // 다른 프레임 drift → 무시 (재검증 관측도 명목)
  send_drift("other_camera_frame");
  ASSERT_TRUE(spin_until([&] {send_det("1"); return obs_of("1") != nullptr;}));
  EXPECT_NEAR(obs_of("1")->x, 0.0, 1e-6);
  // 같은 프레임 drift → 재검증 관측만 되돌림
  send_drift(cam);
  ASSERT_TRUE(spin_until([&] {
      send_det("2");
      const auto * o = obs_of("2");
      return o && std::fabs(o->x) > 1e-4;
    }));
  EXPECT_NEAR(obs_of("2")->x, 0.30 * std::sin(3.0 * M_PI / 180.0), 1e-4);
  bool found = false;
  for (const auto & r : exec_->registry().records()) {
    if (r.id == "2") {
      found = true;
      EXPECT_NEAR(r.x, 0.0, 1e-6);   // 기록부 (PICK 목표 · 수집 계획) 는 명목 TF 그대로
    }
  }
  EXPECT_TRUE(found);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}

// 적재함 판독은 LOOK_RACK 동안만: 처음 끔 → 팔 목표 보낼 때 켬 → 측정 기다리기 (못 받음) 끝나면 끔
TEST_F(ExecutorActions, RackReaderOnlyDuringLookRack)
{
  start("P1");
  Step s;
  s.type = StepType::Arm;
  s.arm_task = kArmLookRack;
  s.puck_id = "P1";
  s.slot = "S1";
  s.name = "arm:LOOK_RACK S1";
  exec_->set_steps({s});
  // 단계 시작 전 = 끔 (perception 이 켜져 있어도 맞춤)
  ASSERT_TRUE(spin_until([&] {return rack_calls_.size() == 1;}, [&] {exec_->update(now());}));
  EXPECT_FALSE(rack_calls_[0]);
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->update(now()); exec_->step(now());}));
  ASSERT_TRUE(spin_until([&] {return rack_calls_.size() == 2;}, [&] {exec_->update(now());}));
  EXPECT_TRUE(rack_calls_[1]);
  arm_succeed_without_step(0);
  ExecEvent ev;
  ASSERT_TRUE(spin_until([&] {return ev.kind == ExecEvent::Kind::StepOk;},
    [&] {exec_->update(now()); ev = exec_->step(now());}));
  ASSERT_TRUE(spin_until([&] {return rack_calls_.size() == 3;}, [&] {exec_->update(now());}));
  EXPECT_FALSE(rack_calls_[2]);
  // 이미 끔 → 더 안 부름
  for (int i = 0; i < 50; ++i) {exec_->update(now()); spin(); std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  EXPECT_EQ(rack_calls_.size(), 3u);
}

// 손목 검출은 주행 목표 진행 중에만 끔: 처음 켬 (지금 상태로 맞춤) → Nav2 목표 보내면 끔 → 도착하면 켬 → 그대로면 더 안 부름
TEST_F(ExecutorActions, WristOffOnlyWhileNavigating)
{
  start();
  exec_->set_steps({go(1.0)});
  ASSERT_TRUE(spin_until([&] {return wrist_calls_.size() == 1;}, [&] {exec_->update(now());}));
  EXPECT_TRUE(wrist_calls_[0]);
  ASSERT_TRUE(spin_until([&] {return !nav_srv_->goals.empty();}, [&] {exec_->update(now()); exec_->step(now());}));
  ASSERT_TRUE(spin_until([&] {return wrist_calls_.size() == 2;}, [&] {exec_->update(now());}));
  EXPECT_FALSE(wrist_calls_[1]);
  nav_srv_->goals[0]->succeed(std::make_shared<Nav::Result>());
  ExecEvent ev;
  ASSERT_TRUE(spin_until([&] {return ev.kind == ExecEvent::Kind::StepOk;},
    [&] {exec_->update(now()); ev = exec_->step(now());}));
  ASSERT_TRUE(spin_until([&] {return wrist_calls_.size() == 3;}, [&] {exec_->update(now());}));
  EXPECT_TRUE(wrist_calls_[2]);
  for (int i = 0; i < 50; ++i) {exec_->update(now()); spin(); std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  EXPECT_EQ(wrist_calls_.size(), 3u);
}
