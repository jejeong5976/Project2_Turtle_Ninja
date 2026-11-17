// test_executor_actions.cpp — 실행기 액션 처리 시험: 가짜 Nav2 · /arm_task 서버를 같은 프로세스에 띄움
//   - 새 목표가 받아들여지기 전 stop() → 끝난 이전 목표 핸들로 취소하지 않음 (예외로 노드가 죽지 않음)
//   - 팔 성공 결과가 온 뒤 step() 전에 stop() → 재고에 한 번만 반영
//   - 팔 시간 초과 뒤 최종 결과가 끝내 안 옴 (server_wait_s · 서버 없어짐) → 재시도 대신 FAULT
//   - 취소 결과 기다리는 중 서버가 없어졌다 다시 뜸 → 더 안 기다림 (resume 안 막힘)
//   - 손목 카메라 drift: 재검증용 원시 관측만 되돌린 위치, 기록부는 명목 TF. 다른 프레임 drift 는 무시
//   - perception 적재함 판독: 처음 끔 → LOOK_RACK 보낼 때 켬 → 칸 측정 기다리기 끝나면 끔
//   - PICK 빈손 확인 (결과 글 EMPTY_GRASP): 재고 불확실 FAULT 대신 StepFailed (need_home), 그 뒤 HOME 이 이동보다 먼저
//   - RETRIEVE 성공 결과 뒤 그리퍼 실측 빈손 → 재고 안 바꾸고 자동 정합 StepFailed (retrieve_hand_check)
//   - 경로 계획 실패 (path_retry_*): 바로 실패 + 안 움직임 · 경로 코드면 기다렸다 같은 목표 다시 (단계 재시도 안 씀), 다 쓰면 보통 실패 처리
//   - perception 멈춤 · respawn: 세션 글이 바뀌면 켜기 다시 보냄 · hook, 검출 끊김 중 LOOK 안 보냄 → 회복 뒤 보냄,
//       끊김이 detection_fault_s 안이면 FAULT 아님 · 넘으면 DETECTION_LOST
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
#include "interfaces/msg/detection_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
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
  bool reject = false;   // true 면 새 목표 거절 (control 이 스스로 접는 중 등)
  int rejected = 0;

  FakeServer(rclcpp::Node::SharedPtr node, const std::string & name)
  {
    server = rclcpp_action::create_server<A>(node, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename A::Goal>) {
        if (reject) {
          ++rejected;
          return rclcpp_action::GoalResponse::REJECT;
        }
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
    wrist_srv_.reset();
    box_srv_.reset();
    server_.reset();
    std::filesystem::remove_all(dir_);
  }

  // gripper_puck 이 있으면 그리퍼에 그 퍽을 쥔 재고 기록으로 시작. extra = 시험별 파라미터 더하기
  void start(const std::string & gripper_puck = "", double server_wait_s = 30.0,
    const std::vector<rclcpp::Parameter> & extra = {}, const std::vector<std::string> & remaps = {})
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
      "-r", "/wrist_cam/drift:=" + ns_ + "/drift", "-r", "/perception/wrist/detections:=" + ns_ + "/detections",
      "-r", "/control/motion_mode:=" + ns_ + "/motion_mode"});
    if (!remaps.empty()) {   // 시험별 더 (예: /odom)
      auto args = o.arguments();
      for (const auto & r : remaps) {
        args.push_back("-r");
        args.push_back(r);
      }
      o.arguments(args);
    }
    std::vector<rclcpp::Parameter> params{
      {"require_scan", false}, {"require_detections", false}, {"require_localization", false},
      {"require_motion_mode", false}, {"lifecycle_services", std::vector<std::string>{}},
      {"cargo_file", cargo_file}, {"cargo_restore", !gripper_puck.empty()}, {"server_wait_s", server_wait_s},

      {"wrist_enable_service", ns_ + "/wrist_enable"}, {"box_enable_service", ns_ + "/box_enable"}};
    for (const auto & e : extra) {   // 같은 이름은 시험 값으로 (overrides 에 둘 있으면 앞 것이 남음)
      params.erase(std::remove_if(params.begin(), params.end(), [&](const rclcpp::Parameter & p) {return p.get_name() == e.get_name();}),
        params.end());
    }
    params.insert(params.end(), extra.begin(), extra.end());
    o.parameter_overrides(params);
    client_ = std::make_shared<rclcpp::Node>("exec_test_client", ns_, o);
    server_ = std::make_shared<rclcpp::Node>("exec_test_server", ns_);
    exec_ = std::make_unique<Executor>(*client_);
    nav_srv_ = std::make_unique<FakeServer<Nav>>(server_, ns_ + "/navigate_to_pose");
    arm_srv_ = std::make_unique<FakeServer<ArmTask>>(server_, ns_ + "/arm_task");
    wrist_srv_ = server_->create_service<std_srvs::srv::SetBool>(ns_ + "/wrist_enable",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> q, std::shared_ptr<std_srvs::srv::SetBool::Response> r) {
        wrist_calls_.push_back(q->data);
        r->success = true;
      });
    box_srv_ = server_->create_service<std_srvs::srv::SetBool>(ns_ + "/box_enable",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> q, std::shared_ptr<std_srvs::srv::SetBool::Response> r) {
        box_calls_.push_back(q->data);
        r->success = true;
      });
    client_ex_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    server_ex_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    client_ex_->add_node(client_);
    server_ex_->add_node(server_);
  }

  double now() {return client_->get_clock()->now().seconds();}
  // 다음 start() 가 쓸 이름공간 (remap 을 start 전에 정할 때)
  std::string ns_for_next() const {return "/exec_test_" + std::to_string(::getpid()) + "_" + std::to_string(run_ + 1);}

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
  void arm_succeed_without_step(std::size_t goal_i) {arm_finish_without_step(goal_i, true);}

  // 위와 같은데 실패 (abort + 결과 글) 도 냄
  void arm_finish_without_step(std::size_t goal_i, bool success, const std::string & message = "")
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
    res->success = success;
    res->message = message;
    if (success) {gh->succeed(res);} else {gh->abort(res);}
    ASSERT_TRUE(spin_until([&] {return exec_->arm_phase().empty();}));   // 결과 도착 (active 끝)
  }

  // PICK 목표 = 미션 resolver 가 주는 base_link 점 (검출 · TF 없이 보낼 수 있게)
  void use_base_target()
  {
    exec_->set_target_resolver([](const Step &, geometry_msgs::msg::PoseStamped & out, std::string &) {
        out.header.frame_id = "base_link";
        out.pose.position.x = 0.25;
        return true;
      });
  }

  static Step pick(const std::string & puck)
  {
    Step s;
    s.type = StepType::Arm;
    s.arm_task = kArmPick;
    s.target = Target::Custom;
    s.puck_id = puck;
    s.name = "arm:PICK:" + puck;
    return s;
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

  // 가짜 Nav2 (경로 계획 실패 시험): 앞 n_fail 개 목표는 받은 뒤 delay_s 에 abort (결과 error_code = code), 그다음은 성공
  struct NavPlay
  {
    int n_fail = 0;
    int code = 0;
    double delay_s = 0.0;
    std::size_t done = 0;
    std::vector<std::chrono::steady_clock::time_point> got;   // 목표 받은 시각 (시험 고리가 본 때)
  };
  void play(NavPlay & p)
  {
    while (p.got.size() < nav_srv_->goals.size()) {p.got.push_back(std::chrono::steady_clock::now());}
    while (p.done < nav_srv_->goals.size()) {
      if (std::chrono::steady_clock::now() - p.got[p.done] < std::chrono::duration<double>(p.delay_s)) {return;}
      auto res = std::make_shared<Nav::Result>();
      if (static_cast<int>(p.done) < p.n_fail) {
        res->error_code = static_cast<uint16_t>(p.code);
        nav_srv_->goals[p.done]->abort(res);
      } else {
        nav_srv_->goals[p.done]->succeed(res);
      }
      ++p.done;
    }
  }
  // 이동 단계를 StepOk · StepFailed · Fault 까지 돌림 (10 s 안). 받은 사건 종류 (None 빼고) 순서
  std::vector<ExecEvent::Kind> run_play(NavPlay & p)
  {
    std::vector<ExecEvent::Kind> kinds;
    spin_until([&] {return !kinds.empty();}, [&] {
        play(p);
        const ExecEvent ev = exec_->step(now());
        if (ev.kind != ExecEvent::Kind::None) {kinds.push_back(ev.kind);}
      });
    return kinds;
  }
  static double gap_s(const NavPlay & p, std::size_t a, std::size_t b)
  {
    return std::chrono::duration<double>(p.got[b] - p.got[a]).count();
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
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr wrist_srv_;
  std::vector<bool> wrist_calls_;  // 손목 검출 켜기 요청 순서
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr box_srv_;
  std::vector<bool> box_calls_;    // 손목 박스 측정 켜기 요청 순서
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

// 재현: 박스 LOOK 에서 원기둥으로 본 퍽 11 → PICK 중 (받침 · 손 안) 손목이 green box 로 더 많이 읽음
//   → 적재 기록 (그리퍼 CargoItem) 은 원기둥, 기록부도 원기둥. 집은 뒤 (in_gripper) box 검출도 모양을 안 바꿈, 위치는 바뀜
TEST_F(ExecutorActions, PickKeepsLookShapeAgainstHeldMisread)
{
  start();
  use_base_target();
  const std::string cam = "exec_test_cam_" + std::to_string(::getpid());
  tf2_ros::StaticTransformBroadcaster stb(server_);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  t.transform.rotation.w = 1.0;
  auto t2 = t;
  t2.header.frame_id = "odom";
  t2.child_frame_id = cam;
  stb.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{t, t2});
  auto det_pub = server_->create_publisher<interfaces::msg::DetectionArray>(ns_ + "/detections", 10);
  int sent = 0;
  auto send_det = [&](const std::string & shape, double x) {
      interfaces::msg::DetectionArray m;
      m.header.frame_id = cam;
      m.header.stamp = client_->now();
      interfaces::msg::Detection d;
      d.header = m.header;
      d.kind = "puck";
      d.id = "11";
      d.label = "green";
      d.shape = shape;
      d.confidence = 0.9;
      d.pose.position.x = x;
      d.pose.position.z = 0.30;
      m.detections.push_back(d);
      det_pub->publish(m);
      ++sent;
    };
  auto obs_n = [&](const std::string & shape) {
      int n = 0;
      for (const auto & o : exec_->recent_obs()) {n += o.id == "11" && o.shape == shape;}
      return n;
    };
  // LOOK: 원기둥 3 장
  ASSERT_TRUE(spin_until([&] {return obs_n("cylinder") >= 3;}, [&] {if (obs_n("cylinder") < 3) {send_det("cylinder", 0.0);}}));
  ASSERT_EQ(exec_->registry().find("11")->shape, "cylinder");

  exec_->set_steps({pick("11")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  // PICK 중: green box 10 장 (표를 세면 이 장들이 기록부 · 적재 기록을 덮어씀)
  ASSERT_TRUE(spin_until([&] {return obs_n("box") >= 10;}, [&] {if (obs_n("box") < 10) {send_det("box", 0.0);}}));
  EXPECT_EQ(exec_->registry().find("11")->shape, "cylinder");
  arm_succeed_without_step(0);
  ASSERT_EQ(exec_->step(now()).kind, ExecEvent::Kind::StepOk);
  ASSERT_TRUE(exec_->cargo().holding());
  EXPECT_EQ(exec_->cargo().gripper()->shape, "cylinder");
  EXPECT_EQ(exec_->cargo().gripper()->color, "green");
  EXPECT_EQ(exec_->registry().find("11")->state, "in_gripper");

  // 쥔 뒤 (in_gripper): box 검출 → 위치만 바뀌고 모양 그대로
  ASSERT_TRUE(spin_until([&] {return std::fabs(exec_->registry().find("11")->x - 0.05) < 1e-6;}, [&] {send_det("box", 0.05);}));
  EXPECT_EQ(exec_->registry().find("11")->shape, "cylinder");
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

// 누움 판정: 방향 있는 검출로만, 방향 없는 검출이 와도 이어 받음 · base 위치 (puck_local)
TEST_F(ExecutorActions, PuckLocalKeepsLyingAcrossUnorientedDetections)
{
  start();
  const std::string cam = "exec_test_lcam_" + std::to_string(::getpid());
  tf2_ros::StaticTransformBroadcaster stb(server_);
  geometry_msgs::msg::TransformStamped t;   // map → odom → 카메라 · odom → base_link, 모두 단위
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  t.transform.rotation.w = 1.0;
  auto t2 = t;
  t2.header.frame_id = "odom";
  t2.child_frame_id = cam;
  auto t3 = t2;
  t3.child_frame_id = "base_link";
  stb.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{t, t2, t3});
  auto det_pub = server_->create_publisher<interfaces::msg::DetectionArray>(ns_ + "/detections", 10);
  // 단계마다 x 를 달리 해서 (0.168 누움 · 0.170 방향 없음 · 0.172 선) 늦게 온 앞 메시지와 구분
  auto send = [&](double x, double qx, double qw) {
      interfaces::msg::DetectionArray m;
      m.header.frame_id = cam;
      m.header.stamp = client_->now();
      interfaces::msg::Detection d;
      d.header = m.header;
      d.kind = "puck";
      d.id = "5";
      d.label = "red";
      d.shape = "box";
      d.confidence = 0.9;
      d.pose.position.x = x;
      d.pose.position.y = 0.06;
      d.pose.position.z = 0.05;
      d.pose.orientation.x = qx;
      d.pose.orientation.w = qw;
      m.detections.push_back(d);
      det_pub->publish(m);
    };
  auto at_x = [&](double x) {const auto p = exec_->puck_local("5"); return p && std::fabs(p->x - x) < 1e-6;};
  auto drain = [&] {   // 앞에 보낸 것이 다 오게
      const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
      spin_until([&] {return std::chrono::steady_clock::now() > until;});
    };
  EXPECT_FALSE(exec_->puck_local("5").has_value());
  const double s45 = std::sin(M_PI / 4.0);
  ASSERT_TRUE(spin_until([&] {send(0.168, s45, s45); return at_x(0.168);}));
  drain();
  auto p = exec_->puck_local("5");
  EXPECT_TRUE(p->lying);
  EXPECT_GT(p->oriented_s, 0.0);
  EXPECT_NEAR(p->y, 0.06, 1e-6);
  const double o1 = p->oriented_s;
  // 방향 없음 (0,0,0,0) → 누움 · 방향 시각 그대로, 위치만 새로
  ASSERT_TRUE(spin_until([&] {send(0.170, 0.0, 0.0); return at_x(0.170);}));
  drain();
  p = exec_->puck_local("5");
  EXPECT_TRUE(p->lying);
  EXPECT_DOUBLE_EQ(p->oriented_s, o1);
  // 선 (단위 쿼터니언) → 누움 아님
  ASSERT_TRUE(spin_until([&] {send(0.172, 0.0, 1.0); return at_x(0.172);}));
  drain();
  EXPECT_FALSE(exec_->puck_local("5")->lying);
  EXPECT_GT(exec_->puck_local("5")->oriented_s, o1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}

// LOOK_RACK: 성공하면 바로 다음 단계 (칸 위치는 control_master 가 손목 태그로 잼, planning 은 기다리지 않음)
TEST_F(ExecutorActions, LookRackSucceedsWithoutWaiting)
{
  start("P1");
  Step s;
  s.type = StepType::Arm;
  s.arm_task = kArmLookRack;
  s.puck_id = "P1";
  s.slot = "S1";
  s.name = "arm:LOOK_RACK S1";
  exec_->set_steps({s});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->update(now()); exec_->step(now());}));
  arm_succeed_without_step(0);
  ExecEvent ev;
  ASSERT_TRUE(spin_until([&] {return ev.kind == ExecEvent::Kind::StepOk;},
    [&] {exec_->update(now()); ev = exec_->step(now());}));
}

// 손목 검출 · 박스 측정은 주행 목표 진행 중에만 끔: 처음 켬 (지금 상태로 맞춤) → Nav2 목표 보내면 끔 → 도착하면 켬 → 그대로면 더 안 부름
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
  EXPECT_EQ(box_calls_, (std::vector<bool>{true, false, true}));
}

// PICK 빈손 확인 (결과 글 EMPTY_GRASP, ArmRunner 앞말 붙어도) → FAULT 아님, StepFailed + need_home, 재고 · 표시 그대로
//   재시도 · 다시 보기는 안 끼움 (미션 dice_recover 가 그 퍽 skipped). 미션처럼 HOME 을 끼우면 HOME 이 이동보다 먼저
TEST_F(ExecutorActions, EmptyGraspPickSkippedThenHomeBeforeDrive)
{
  start();
  use_base_target();
  exec_->set_steps({pick("P1"), go(1.0)});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_finish_without_step(0, false, "팔 작업 중단: EMPTY_GRASP 빈손: 그리퍼가 -0.0010 m 까지 닫힘");
  const ExecEvent ev = exec_->step(now());
  ASSERT_EQ(ev.kind, ExecEvent::Kind::StepFailed);
  EXPECT_EQ(ev.step.puck_id, "P1");
  EXPECT_TRUE(ev.need_home);
  EXPECT_NE(ev.detail.find("EMPTY_GRASP"), std::string::npos);
  EXPECT_EQ(exec_->index(), 0u);
  EXPECT_EQ(exec_->steps().size(), 2u);   // 재시도 HOME · 다시 보기 LOOK 안 끼움
  EXPECT_EQ(arm_srv_->goals.size(), 1u);
  EXPECT_FALSE(exec_->cargo().holding());
  EXPECT_FALSE(exec_->cargo().doubt().has_value());
  EXPECT_FALSE(not_ready("INVENTORY_UNCERTAIN"));
  EXPECT_FALSE(not_ready("GRIPPER_RECORD"));

  // 미션 (dice_recover 가 PICK 지움 + on_step_failed 가 HOME) 처럼
  exec_->erase_steps(0, 1);
  exec_->insert_home("팔 작업 실패 뒤");
  ASSERT_TRUE(spin_until([&] {return arm_srv_->goals.size() == 2u;}, [&] {exec_->step(now());}));
  EXPECT_EQ(arm_srv_->goals[1]->get_goal()->task, kArmHome);
  EXPECT_TRUE(nav_srv_->goals.empty());   // 접기 전에는 이동 목표 없음
  arm_succeed_without_step(1);
  ASSERT_EQ(exec_->step(now()).kind, ExecEvent::Kind::StepOk);
  ASSERT_TRUE(spin_until([&] {return !nav_srv_->goals.empty();}, [&] {exec_->step(now());}));
  EXPECT_FALSE(exec_->cargo().doubt().has_value());
}

// 빈손 뒤 HOME 이 control 스스로 접기와 겹쳐 거절됨 → action_retry_s 뒤 다시 보냄 → 받으면 HOME → 이동
TEST_F(ExecutorActions, EmptyGraspHomeRejectedWhileAutoFoldingIsRetried)
{
  start("", 30.0, {rclcpp::Parameter("action_retry_s", 0.1)});
  use_base_target();
  exec_->set_steps({pick("P1"), go(1.0)});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_finish_without_step(0, false, "EMPTY_GRASP 빈손");
  ASSERT_EQ(exec_->step(now()).kind, ExecEvent::Kind::StepFailed);
  exec_->erase_steps(0, 1);
  exec_->insert_home("팔 작업 실패 뒤");

  arm_srv_->reject = true;   // 스스로 접는 중
  ExecEvent ev;
  ASSERT_TRUE(spin_until([&] {return arm_srv_->rejected >= 2;}, [&] {ev = exec_->step(now());}));
  EXPECT_EQ(ev.kind, ExecEvent::Kind::None);   // 거절은 실패 아님 (action_retry_max_s 안)
  arm_srv_->reject = false;  // 접기 끝
  ASSERT_TRUE(spin_until([&] {return arm_srv_->goals.size() == 2u;}, [&] {exec_->step(now());}));
  EXPECT_EQ(arm_srv_->goals[1]->get_goal()->task, kArmHome);
  arm_succeed_without_step(1);
  ASSERT_EQ(exec_->step(now()).kind, ExecEvent::Kind::StepOk);
  ASSERT_TRUE(spin_until([&] {return !nav_srv_->goals.empty();}, [&] {exec_->step(now());}));
}

// 표시 없는 PICK 실패 (세우기 멈춤 등) → 재고 불확실 FAULT
TEST_F(ExecutorActions, PickFailWithoutTokenStillUncertain)
{
  start();
  use_base_target();
  exec_->set_steps({pick("P1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_finish_without_step(0, false, "팔 작업 중단: 세우기 멈춤: 손끝이 쥐는 자리에서 멂");
  const ExecEvent ev = exec_->step(now());
  EXPECT_EQ(ev.kind, ExecEvent::Kind::Fault);
  EXPECT_EQ(ev.code, "INVENTORY_UNCERTAIN");
  EXPECT_TRUE(exec_->cargo().doubt().has_value());
  EXPECT_TRUE(not_ready("INVENTORY_UNCERTAIN"));
}

// empty_grasp_skip false → EMPTY_GRASP 여도 FAULT
TEST_F(ExecutorActions, EmptyGraspSkipOffKeepsFault)
{
  start("", 30.0, {rclcpp::Parameter("empty_grasp_skip", false)});
  use_base_target();
  exec_->set_steps({pick("P1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_finish_without_step(0, false, "EMPTY_GRASP 빈손");
  const ExecEvent ev = exec_->step(now());
  EXPECT_EQ(ev.kind, ExecEvent::Kind::Fault);
  EXPECT_EQ(ev.code, "INVENTORY_UNCERTAIN");
  EXPECT_TRUE(exec_->cargo().doubt().has_value());
}

// 빈손 결과 도착 → step() 전에 stop() (다른 FAULT) → 불확실 표시 안 함 (재고 그대로)
TEST_F(ExecutorActions, EmptyGraspBeforeStopLeavesNoDoubt)
{
  start();
  use_base_target();
  exec_->set_steps({pick("P1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_finish_without_step(0, false, "EMPTY_GRASP 빈손");
  exec_->stop();
  exec_->update(now());
  EXPECT_FALSE(exec_->cargo().doubt().has_value());
  EXPECT_FALSE(exec_->cargo().holding());
  EXPECT_FALSE(not_ready("INVENTORY_UNCERTAIN"));
}

// 같은 경우 표시 없는 실패면 불확실 표시
TEST_F(ExecutorActions, PickFailBeforeStopMarksDoubt)
{
  start();
  use_base_target();
  exec_->set_steps({pick("P1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_finish_without_step(0, false, "팔 작업 중단: 하드웨어 이상");
  exec_->stop();
  EXPECT_TRUE(exec_->cargo().doubt().has_value());
}

// STORE 넣기 걸림 → control 이 퍽을 박스에 되돌림 (결과 글 STORE_RETURNED_TO_BOX x y) → FAULT 아님, StepFailed + need_home,
//   그리퍼 기록 비움 · 칸 그대로 · 불확실 표시 없음 (미션 dice_recover 가 다시 계획 · 두 번이면 건너뜀)
TEST_F(ExecutorActions, StoreReturnedToBoxIsStepFailedWithEmptyGripper)
{
  start("P1");
  exec_->set_steps({store("P1", "S1"), go(1.0)});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_finish_without_step(0, false, "STORE_RETURNED_TO_BOX 0.2149 0.0000 · STORE 보정: 칸 넣기 걸림 → 퍽을 박스에 되돌려 놓음");
  const ExecEvent ev = exec_->step(now());
  ASSERT_EQ(ev.kind, ExecEvent::Kind::StepFailed);
  EXPECT_EQ(ev.step.puck_id, "P1");
  EXPECT_TRUE(ev.need_home);
  EXPECT_NE(ev.detail.find("STORE_RETURNED_TO_BOX"), std::string::npos);
  EXPECT_EQ(exec_->steps().size(), 2u);   // 재시도 안 끼움
  EXPECT_FALSE(exec_->cargo().holding());
  EXPECT_FALSE(slot("S1").has_value());
  EXPECT_FALSE(exec_->cargo().doubt().has_value());
  EXPECT_FALSE(not_ready("INVENTORY_UNCERTAIN"));
  EXPECT_FALSE(not_ready("GRIPPER_RECORD"));
}

// store_return_replan false → 토큰이 있어도 재고 불확실 FAULT (그리퍼 기록 그대로)
TEST_F(ExecutorActions, StoreReturnedSwitchOffKeepsFault)
{
  start("P1", 30.0, {rclcpp::Parameter("store_return_replan", false)});
  exec_->set_steps({store("P1", "S1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_finish_without_step(0, false, "STORE_RETURNED_TO_BOX 0.2 0.0");
  const ExecEvent ev = exec_->step(now());
  EXPECT_EQ(ev.kind, ExecEvent::Kind::Fault);
  EXPECT_EQ(ev.code, "INVENTORY_UNCERTAIN");
  EXPECT_TRUE(exec_->cargo().holding());
  EXPECT_TRUE(exec_->cargo().doubt().has_value());
}

// 토큰 없는 STORE 실패 (칸 안 · 쥔 채) → 재고 불확실 FAULT
TEST_F(ExecutorActions, StoreFailWithoutTokenStillUncertain)
{
  start("P1");
  exec_->set_steps({store("P1", "S1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_finish_without_step(0, false, "팔 동작 실패: 'release' 동작이 성공하지 못함");
  const ExecEvent ev = exec_->step(now());
  EXPECT_EQ(ev.kind, ExecEvent::Kind::Fault);
  EXPECT_EQ(ev.code, "INVENTORY_UNCERTAIN");
  EXPECT_TRUE(exec_->cargo().holding());
}

// 박스 되돌림 결과 도착 → step() 전에 stop() (다른 FAULT) → 그리퍼 비움 · 불확실 표시 없음
TEST_F(ExecutorActions, StoreReturnedBeforeStopClearsGripper)
{
  start("P1");
  exec_->set_steps({store("P1", "S1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_finish_without_step(0, false, "STORE_RETURNED_TO_BOX 0.2149 0.0000 · STORE 보정");
  exec_->stop();
  exec_->update(now());
  EXPECT_FALSE(exec_->cargo().holding());
  EXPECT_FALSE(exec_->cargo().doubt().has_value());
  EXPECT_FALSE(slot("S1").has_value());
}

// ---- RETRIEVE 성공 결과 뒤 그리퍼 실측 확인 (retrieve_hand_check)
namespace
{
Step retrieve(const std::string & puck, const std::string & slot)
{
  Step s;
  s.type = StepType::Arm;
  s.arm_task = kArmRetrieve;
  s.puck_id = puck;
  s.slot = slot;
  s.name = "arm:RETRIEVE " + slot;
  return s;
}
}  // namespace

// control 은 RETRIEVE 빈손을 확인 안 하고 성공을 냄 → 그리퍼 실측 빈손이면 재고 안 바꾸고 자동 정합 StepFailed
//   그 뒤 HOME 목표를 보낼 때 (keep_unapplied_arm_ok) · stop() 뒤에도 성공이 재고에 반영되지 않음
TEST_F(ExecutorActions, RetrieveSuccessWithEmptyHandIsReconcile)
{
  start("P1");
  std::string msg;
  ASSERT_TRUE(exec_->record_store("S1", msg)) << msg;
  exec_->set_hand_reader([](double, std::string & why) {why = "폭 -1.6 mm"; return HandGuess::Empty;});
  exec_->set_steps({retrieve("P1", "S1"), go(1.0)});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_succeed_without_step(0);
  const ExecEvent ev = exec_->step(now());
  ASSERT_EQ(ev.kind, ExecEvent::Kind::StepFailed);
  EXPECT_TRUE(ev.reconcile);
  EXPECT_TRUE(ev.need_home);
  EXPECT_NE(ev.detail.find("빈손"), std::string::npos);
  EXPECT_FALSE(exec_->cargo().holding());
  ASSERT_TRUE(slot("S1").has_value());
  EXPECT_EQ(slot("S1")->puck_id, "P1");
  EXPECT_TRUE(exec_->cargo().doubt().has_value());   // 미션이 적재함 스캔으로 정합

  exec_->stop();
  exec_->update(now());
  EXPECT_FALSE(exec_->cargo().holding());
  EXPECT_EQ(slot("S1")->puck_id, "P1");
}

// control 이 RETRIEVE 끝 빈손을 확인해 실패 (EMPTY_GRASP) → 손 상태 (hand reader) 를 안 봐도 FAULT 아닌 자동 정합 (재고 그대로 · 불확실 표시)
TEST_F(ExecutorActions, RetrieveEmptyGraspFailureIsReconcile)
{
  start("P1");
  std::string msg;
  ASSERT_TRUE(exec_->record_store("S1", msg)) << msg;
  exec_->set_hand_reader([](double, std::string & why) {why = "모름"; return HandGuess::Unknown;});
  exec_->set_steps({retrieve("P1", "S1"), go(1.0)});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_finish_without_step(0, false, "EMPTY_GRASP: 팔 작업 중단: 빈손: RETRIEVE S1 끝 그리퍼가 -0.0016 m 까지 닫힘");
  const ExecEvent ev = exec_->step(now());
  ASSERT_EQ(ev.kind, ExecEvent::Kind::StepFailed);
  EXPECT_TRUE(ev.reconcile);
  EXPECT_TRUE(ev.need_home);
  EXPECT_FALSE(exec_->cargo().holding());
  ASSERT_TRUE(slot("S1").has_value());
  EXPECT_EQ(slot("S1")->puck_id, "P1");
  EXPECT_TRUE(exec_->cargo().doubt().has_value());
}

// 그리퍼 실측 쥠 → 성공 그대로 (그리퍼에 퍽)
TEST_F(ExecutorActions, RetrieveSuccessWithHoldingHandIsOk)
{
  start("P1");
  std::string msg;
  ASSERT_TRUE(exec_->record_store("S1", msg)) << msg;
  exec_->set_hand_reader([](double, std::string & why) {why = "폭 33.5 mm"; return HandGuess::Holding;});
  exec_->set_steps({retrieve("P1", "S1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_succeed_without_step(0);
  EXPECT_EQ(exec_->step(now()).kind, ExecEvent::Kind::StepOk);
  EXPECT_TRUE(exec_->cargo().holding());
  EXPECT_FALSE(slot("S1").has_value());
  EXPECT_FALSE(exec_->cargo().doubt().has_value());
}

// 손 상태 모름 → hand_wait_s 기다림 (그동안 Running) → 그 안에 빈손으로 정해지면 실패 / 끝내 모름이면 성공 그대로 (아래)
TEST_F(ExecutorActions, RetrieveHandUnknownWaitsThenDecides)
{
  start("P1", 30.0, {rclcpp::Parameter("hand_wait_s", 0.3)});
  std::string msg;
  ASSERT_TRUE(exec_->record_store("S1", msg)) << msg;
  int calls = 0;
  exec_->set_hand_reader([&](double, std::string &) {return ++calls < 3 ? HandGuess::Unknown : HandGuess::Empty;});
  exec_->set_steps({retrieve("P1", "S1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_succeed_without_step(0);
  EXPECT_EQ(exec_->step(now()).kind, ExecEvent::Kind::None);   // 기다림
  ExecEvent ev;
  ASSERT_TRUE(spin_until([&] {return ev.kind != ExecEvent::Kind::None;}, [&] {ev = exec_->step(now());}));
  EXPECT_EQ(ev.kind, ExecEvent::Kind::StepFailed);
  EXPECT_FALSE(exec_->cargo().holding());
}

TEST_F(ExecutorActions, RetrieveHandStaysUnknownKeepsSuccess)
{
  start("P2", 30.0, {rclcpp::Parameter("hand_wait_s", 0.2)});
  std::string msg;
  ExecEvent ev;
  ASSERT_TRUE(exec_->record_store("S1", msg)) << msg;
  exec_->set_hand_reader([](double, std::string &) {return HandGuess::Unknown;});
  exec_->set_steps({retrieve("P2", "S1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_succeed_without_step(0);
  ASSERT_TRUE(spin_until([&] {return ev.kind != ExecEvent::Kind::None;}, [&] {ev = exec_->step(now());}));
  EXPECT_EQ(ev.kind, ExecEvent::Kind::StepOk);
  EXPECT_TRUE(exec_->cargo().holding());
}

// retrieve_hand_check false → 성공 결과를 믿음
TEST_F(ExecutorActions, RetrieveHandCheckOffTrustsSuccess)
{
  start("P1", 30.0, {rclcpp::Parameter("retrieve_hand_check", false)});
  std::string msg;
  ASSERT_TRUE(exec_->record_store("S1", msg)) << msg;
  exec_->set_hand_reader([](double, std::string &) {return HandGuess::Empty;});
  exec_->set_steps({retrieve("P1", "S1")});
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {exec_->step(now());}));
  arm_succeed_without_step(0);
  EXPECT_EQ(exec_->step(now()).kind, ExecEvent::Kind::StepOk);
  EXPECT_TRUE(exec_->cargo().holding());
}

// ---- 경로 계획 실패 기다렸다 다시 (path_retry_*)

// 판정: bt_navigator 경로 코드 · rl_nav 이유 코드 없음 (바로 실패 + 안 움직임) · 그 밖은 아님
TEST(NavPathFailure, Classify)
{
  EXPECT_TRUE(nav_path_failure(208, 30.0, 1.0, 5.0, 0.05));    // NO_VALID_PATH: 시간 · 움직임 상관없음
  EXPECT_TRUE(nav_path_failure(205, 0.2, 0.0, 5.0, 0.05));     // START_OCCUPIED
  EXPECT_FALSE(nav_path_failure(201, 0.2, 0.0, 5.0, 0.05));    // INVALID_PLANNER (설정 잘못)
  EXPECT_FALSE(nav_path_failure(9003, 0.2, 0.0, 5.0, 0.05));   // NavigateToPose TIMEOUT
  EXPECT_FALSE(nav_path_failure(106, 0.2, 0.0, 5.0, 0.05));    // FollowPath 쪽
  EXPECT_TRUE(nav_path_failure(0, 0.4, 0.01, 5.0, 0.05));      // rl_nav '경로 못 받음'
  EXPECT_FALSE(nav_path_failure(0, 9.0, 0.0, 5.0, 0.05));      // 늦은 실패 (막힘 멈춤 8 s 등)
  EXPECT_FALSE(nav_path_failure(0, 0.4, 0.30, 5.0, 0.05));     // 움직인 뒤 실패
}

// 바로 실패 2 번 (rl_nav 경로 못 받음) → 0.3 · 0.5 s 기다렸다 같은 목표 다시 → 성공. 단계 재시도 0 이어도 StepFailed 없음
TEST_F(ExecutorActions, PathFailWaitsAndReplansWithoutRetries)
{
  start("", 30.0, {rclcpp::Parameter("path_retry_total_s", 3.0),
    rclcpp::Parameter("path_retry_wait_s", std::vector<double>{0.3, 0.5}), rclcpp::Parameter("nav_retries", 0)});
  exec_->set_steps({go(1.0)});
  NavPlay p;
  p.n_fail = 2;
  const auto kinds = run_play(p);
  ASSERT_EQ(kinds.size(), 1u);
  EXPECT_EQ(kinds[0], ExecEvent::Kind::StepOk);
  ASSERT_EQ(nav_srv_->goals.size(), 3u);
  EXPECT_GE(gap_s(p, 0, 1), 0.2);   // 0.3 s 기다림 (시험 고리가 목표를 본 시각이라 여유)
  EXPECT_GE(gap_s(p, 1, 2), 0.4);   // 0.5 s
  const auto g0 = nav_srv_->goals[0]->get_goal()->pose.pose.position;
  const auto g2 = nav_srv_->goals[2]->get_goal()->pose.pose.position;
  EXPECT_DOUBLE_EQ(g0.x, g2.x);   // 같은 목표
}

// 늘 바로 실패 → path_retry_total_s (1 s) 까지 기다렸다 다시, 다 쓰면 보통 실패 처리 (단계 재시도 1 번 → StepFailed 한 번)
TEST_F(ExecutorActions, PathFailBudgetSpentThenOldHandling)
{
  start("", 30.0, {rclcpp::Parameter("path_retry_total_s", 1.0),
    rclcpp::Parameter("path_retry_wait_s", std::vector<double>{0.3}), rclcpp::Parameter("nav_retries", 1)});
  exec_->set_steps({go(1.0)});
  NavPlay p;
  p.n_fail = 100;
  const auto t0 = std::chrono::steady_clock::now();
  const auto kinds = run_play(p);
  const double took = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  ASSERT_EQ(kinds.size(), 1u);
  EXPECT_EQ(kinds[0], ExecEvent::Kind::StepFailed);
  EXPECT_GE(took, 0.9);
  EXPECT_LT(took, 3.0);
  EXPECT_GE(nav_srv_->goals.size(), 4u);   // 처음 + 기다린 뒤 2–3 번 + 단계 재시도 1 번
  EXPECT_LE(nav_srv_->goals.size(), 6u);
}

// 끔 (기본 path_retry_total_s 0) → 바로 StepFailed
TEST_F(ExecutorActions, PathRetryOffByDefault)
{
  start("", 30.0, {rclcpp::Parameter("nav_retries", 0)});
  exec_->set_steps({go(1.0)});
  NavPlay p;
  p.n_fail = 100;
  const auto kinds = run_play(p);
  ASSERT_EQ(kinds.size(), 1u);
  EXPECT_EQ(kinds[0], ExecEvent::Kind::StepFailed);
  EXPECT_EQ(nav_srv_->goals.size(), 1u);
}

// 경로 실패가 아닌 실패 (늦은 실패 · 다른 코드 · 거절) → 기다리지 않음 (바로 StepFailed)
static std::vector<rclcpp::Parameter> path_retry_on()
{
  return {rclcpp::Parameter("path_retry_total_s", 3.0), rclcpp::Parameter("path_retry_wait_s", std::vector<double>{0.3}),
    rclcpp::Parameter("path_fail_quick_s", 0.2), rclcpp::Parameter("nav_retries", 0)};
}

// 보낸 뒤 0.5 s 에 실패 (path_fail_quick_s 0.2 넘음, 이유 코드 없음 = 막힘 멈춤 같은 것)
TEST_F(ExecutorActions, SlowNavFailureDoesNotWait)
{
  start("", 30.0, path_retry_on());
  exec_->set_steps({go(1.0)});
  NavPlay p;
  p.n_fail = 100;
  p.delay_s = 0.5;
  const auto kinds = run_play(p);
  ASSERT_EQ(kinds.size(), 1u);
  EXPECT_EQ(kinds[0], ExecEvent::Kind::StepFailed);
  EXPECT_EQ(nav_srv_->goals.size(), 1u);
}

// 바로 실패지만 다른 이유 코드 (NavigateToPose TIMEOUT 9003)
TEST_F(ExecutorActions, OtherErrorCodeDoesNotWait)
{
  start("", 30.0, path_retry_on());
  exec_->set_steps({go(1.0)});
  NavPlay p;
  p.n_fail = 100;
  p.code = 9003;
  const auto kinds = run_play(p);
  ASSERT_EQ(kinds.size(), 1u);
  EXPECT_EQ(kinds[0], ExecEvent::Kind::StepFailed);
  EXPECT_EQ(nav_srv_->goals.size(), 1u);
}

// 목표 거절
TEST_F(ExecutorActions, RejectedNavGoalDoesNotWait)
{
  start("", 30.0, path_retry_on());
  nav_srv_->reject = true;
  exec_->set_steps({go(1.0)});
  NavPlay p;
  const auto kinds = run_play(p);
  ASSERT_EQ(kinds.size(), 1u);
  EXPECT_EQ(kinds[0], ExecEvent::Kind::StepFailed);
  EXPECT_EQ(nav_srv_->rejected, 1);
}

// 늦은 실패라도 bt_navigator 경로 코드 (208 NO_VALID_PATH) 면 기다렸다 다시
TEST_F(ExecutorActions, PathErrorCodeWaitsEvenWhenSlow)
{
  start("", 30.0, {rclcpp::Parameter("path_retry_total_s", 3.0),
    rclcpp::Parameter("path_retry_wait_s", std::vector<double>{0.3}), rclcpp::Parameter("path_fail_quick_s", 0.2),
    rclcpp::Parameter("nav_retries", 0)});
  exec_->set_steps({go(1.0)});
  NavPlay p;
  p.n_fail = 1;
  p.code = 208;
  p.delay_s = 0.4;
  const auto kinds = run_play(p);
  ASSERT_EQ(kinds.size(), 1u);
  EXPECT_EQ(kinds[0], ExecEvent::Kind::StepOk);
  EXPECT_EQ(nav_srv_->goals.size(), 2u);
}

// 이동 단계마다 따로 셈: 첫 단계가 기다림을 거의 다 써도 (성공) 다음 단계는 처음부터 (같은 이름 단계여도)
TEST_F(ExecutorActions, PathRetryBudgetPerStep)
{
  start("", 30.0, {rclcpp::Parameter("path_retry_total_s", 0.8),
    rclcpp::Parameter("path_retry_wait_s", std::vector<double>{0.3}), rclcpp::Parameter("nav_retries", 0)});
  exec_->set_steps({go(1.0), go(1.0)});
  NavPlay p;
  p.n_fail = 2;   // 첫 단계: 실패 2 번 (0.6 s 기다림) → 성공
  auto kinds = run_play(p);
  ASSERT_EQ(kinds.size(), 1u);
  ASSERT_EQ(kinds[0], ExecEvent::Kind::StepOk);
  ASSERT_EQ(nav_srv_->goals.size(), 3u);
  p.n_fail = 5;   // 둘째 단계: 실패 2 번 더 (0.6 s, 첫 단계 셈이 남았으면 0.8 넘어 StepFailed) → 성공
  kinds = run_play(p);
  ASSERT_EQ(kinds.size(), 1u);
  EXPECT_EQ(kinds[0], ExecEvent::Kind::StepOk);
  EXPECT_EQ(nav_srv_->goals.size(), 6u);
}

// 기다리는 중 끼운 HOME (DRIVE 복구 등) 이 성공해도 기다림 · 셈은 그대로 (이동 단계 성공에서만 비움)
TEST_F(ExecutorActions, PathRetryWaitSurvivesInsertedHome)
{
  start("", 30.0, {rclcpp::Parameter("path_retry_total_s", 3.0),
    rclcpp::Parameter("path_retry_wait_s", std::vector<double>{0.8}), rclcpp::Parameter("nav_retries", 0)});
  exec_->set_steps({go(1.0)});
  NavPlay p;
  p.n_fail = 1;
  ASSERT_TRUE(spin_until([&] {return p.done == 1u;}, [&] {play(p); exec_->step(now());}));
  const auto t_fail = std::chrono::steady_clock::now();
  // 실패 결과가 step() 에 닿을 때까지 (기다림 시작)
  spin_until([&] {return std::chrono::steady_clock::now() - t_fail > std::chrono::milliseconds(200);},
    [&] {play(p); exec_->step(now());});
  exec_->insert_home("시험: 기다리는 중 HOME");
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {play(p); exec_->step(now());}));
  arm_succeed_without_step(0);
  ASSERT_EQ(exec_->step(now()).kind, ExecEvent::Kind::StepOk);   // HOME
  ASSERT_TRUE(spin_until([&] {return nav_srv_->goals.size() == 2u;}, [&] {play(p); exec_->step(now());}));
  EXPECT_GE(std::chrono::duration<double>(std::chrono::steady_clock::now() - t_fail).count(), 0.6);   // 0.8 s 기다림 그대로
  const auto kinds = run_play(p);
  ASSERT_EQ(kinds.size(), 1u);
  EXPECT_EQ(kinds[0], ExecEvent::Kind::StepOk);
}

// 사람이 직접 한 STORE 기록 (/mission/store_done/<칸>) = STORE 성공과 같은 전환 → GRIPPER_RECORD 풀림
TEST_F(ExecutorActions, RecordManualStoreClearsGripperRecord)
{
  start("P1");
  ASSERT_TRUE(exec_->cargo().holding());
  EXPECT_TRUE(not_ready("GRIPPER_RECORD"));
  std::string msg;
  EXPECT_FALSE(exec_->record_store("S99", msg));   // 없는 칸 → 기록 그대로
  EXPECT_TRUE(exec_->cargo().holding());
  ASSERT_TRUE(exec_->record_store("S2", msg)) << msg;
  EXPECT_FALSE(exec_->cargo().holding());
  ASSERT_TRUE(slot("S2").has_value());
  EXPECT_EQ(slot("S2")->puck_id, "P1");
  EXPECT_FALSE(not_ready("GRIPPER_RECORD"));
  EXPECT_FALSE(exec_->record_store("S1", msg));   // 그리퍼 빔
  EXPECT_FALSE(slot("S1").has_value());
}

// ---------------------------------------------------------------- 손이 안 닿는 PICK 거절
namespace
{
// control 처럼 거절 이유를 /control/motion_mode reason 에 실음 (시험 이름공간)
rclcpp::Publisher<interfaces::msg::MotionMode>::SharedPtr mode_pub(rclcpp::Node::SharedPtr node, const std::string & ns)
{
  return node->create_publisher<interfaces::msg::MotionMode>(ns + "/motion_mode", rclcpp::QoS(1).reliable().transient_local());
}
interfaces::msg::MotionMode mode_with(const std::string & reason)
{
  interfaces::msg::MotionMode m;
  m.mode = interfaces::msg::MotionMode::STOP;
  m.ready = true;
  m.reason = reason;
  return m;
}
}  // namespace

// control 이 "PICK_UNREACHABLE puck=P1" 로 거절 → 거절 재전송 (action_retry_max_s 15 s)
//   없이 시도마다 바로 실패. 단계 재시도 (pick_retries 2) 는 그대로 → 보낸 목표 3 개 뒤 StepFailed (미션이 그 퍽 건너뜀 · 재계획), 재고 확실
TEST_F(ExecutorActions, UnreachablePickFailsWithoutResendLoop)
{
  start("", 30.0, {rclcpp::Parameter("action_retry_s", 0.1)});
  use_base_target();
  auto pub = mode_pub(server_, ns_);
  exec_->set_steps({pick("P1")});
  arm_srv_->reject = true;
  const auto m = mode_with("거절: PICK_UNREACHABLE puck=P1: 높이 30 mm: 손 IK 안 됨 (퍽 0.376, 0.136) — 퍽이 팔에서 너무 멀거나 옆");
  const auto t0 = std::chrono::steady_clock::now();
  ExecEvent ev;
  ASSERT_TRUE(spin_until([&] {return ev.kind != ExecEvent::Kind::None;}, [&] {pub->publish(m); ev = exec_->step(now());}));
  const double took = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_EQ(ev.kind, ExecEvent::Kind::StepFailed);
  EXPECT_EQ(ev.step.puck_id, "P1");
  EXPECT_FALSE(ev.need_home);   // 팔 안 움직임
  EXPECT_NE(ev.detail.find("PICK_UNREACHABLE"), std::string::npos) << ev.detail;
  EXPECT_EQ(arm_srv_->rejected, 3);   // 처음 + 단계 재시도 2 (pick_retries 그대로)
  EXPECT_LT(took, 5.0);               // 재전송이면 15 s × 3
  EXPECT_FALSE(exec_->cargo().doubt().has_value());
  EXPECT_FALSE(not_ready("INVENTORY_UNCERTAIN"));
}

// 다른 퍽의 안 닿음 이유 · 표시 없는 거절 (켤 때 재생기 준비 · 스스로 접는 중 등) 은 action_retry_s 마다 다시 보냄
TEST_F(ExecutorActions, OtherRejectionsStillResent)
{
  for (const std::string reason : {"거절: PICK_UNREACHABLE puck=P2: 손 IK 안 됨", "거절: 팔 작업 실패 뒤 자동으로 접는 중 (planning 이 다시 보냄)"}) {
    start("", 30.0, {rclcpp::Parameter("action_retry_s", 0.05)});
    use_base_target();
    auto pub = mode_pub(server_, ns_);
    exec_->set_steps({pick("P1")});
    arm_srv_->reject = true;
    const auto m = mode_with(reason);
    ExecEvent ev;
    ASSERT_TRUE(spin_until([&] {return arm_srv_->rejected >= 5 || ev.kind != ExecEvent::Kind::None;}, [&] {pub->publish(m); ev = exec_->step(now());}))
      << reason;
    EXPECT_EQ(ev.kind, ExecEvent::Kind::None) << reason;   // 실패 아님 (action_retry_max_s 안)
    EXPECT_GE(arm_srv_->rejected, 5) << reason;
    TearDown();
    SetUp();
  }
}

// HOME 이 계속 거절되면 FAULT HOME_FAILED 글에 control 의 마지막 거절 이유 (받침 확인 안내 등) 를 붙임
TEST_F(ExecutorActions, HomeRejectedFaultCarriesControlReason)
{
  start("", 30.0, {rclcpp::Parameter("action_retry_s", 0.05), rclcpp::Parameter("action_retry_max_s", 0.3)});
  auto pub = mode_pub(server_, ns_);
  exec_->set_steps({look()});
  exec_->insert_home("시험");
  arm_srv_->reject = true;
  const auto m = mode_with("거절: 받침 안 다시 쥐기 빈손 뒤: 받침에 퍽이 남았을 수 있음 — 사람이 받침에서 퍽을 빼고 HOME force");
  ExecEvent ev;
  ASSERT_TRUE(spin_until([&] {return ev.kind != ExecEvent::Kind::None;}, [&] {pub->publish(m); ev = exec_->step(now());}));
  EXPECT_EQ(ev.kind, ExecEvent::Kind::Fault);
  EXPECT_EQ(ev.code, "HOME_FAILED");
  EXPECT_NE(ev.detail.find("받침에 퍽이 남았을 수 있음"), std::string::npos) << ev.detail;
}

// ============================================================================ perception 멈춤 · respawn

namespace
{
// 검출 끊김 시험용: odom (supervise 의 ODOM_LOST 막기) · 손목 검출 (빈 배열 = 살아 있음) 발행
struct PercFeed
{
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr det;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr session;
  PercFeed(rclcpp::Node::SharedPtr n, const std::string & ns)
  {
    odom = n->create_publisher<nav_msgs::msg::Odometry>(ns + "/odom", 10);
    det = n->create_publisher<interfaces::msg::DetectionArray>(ns + "/detections", 10);
    session = n->create_publisher<std_msgs::msg::String>(ns + "/session", rclcpp::QoS(1).transient_local().reliable());
  }
  void tick(bool detections)
  {
    nav_msgs::msg::Odometry o;
    o.header.frame_id = "odom";
    odom->publish(o);
    if (detections) {
      interfaces::msg::DetectionArray d;
      d.header.frame_id = "cam";
      det->publish(d);
    }
  }
  // 실행기가 odom 을 받을 때까지 (supervise 가 ODOM_LOST 를 안 내게)
  template<typename Spin>
  void warm(Spin && spin_once)
  {
    for (int i = 0; i < 60; ++i) {
      tick(false);
      spin_once();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  void say(const std::string & s)
  {
    std_msgs::msg::String m;
    m.data = s;
    session->publish(m);
  }
};
}  // namespace

// 세션 글이 바뀜 = perception respawn → 상태가 그대로여도 손목 검출 · 박스 측정 켜기를 다시 보냄 + hook (처음 받은 글은 respawn 아님)
TEST_F(ExecutorActions, PerceptionRestartResendsToggles)
{
  start("", 30.0, {rclcpp::Parameter("perception_session_topic", ns_for_next() + "/session")});
  PercFeed feed(server_, ns_);
  std::vector<std::pair<std::string, std::string>> hooks;
  exec_->set_perception_restart_hook([&](const std::string & a, const std::string & b) {hooks.emplace_back(a, b);});
  feed.say("pid=1");
  ASSERT_TRUE(spin_until([&] {return wrist_calls_.size() == 1 && box_calls_.size() == 1;}, [&] {exec_->update(now());}));
  for (int i = 0; i < 40; ++i) {exec_->update(now()); spin(); std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  EXPECT_EQ(wrist_calls_.size(), 1u);   // 그대로면 안 부름
  EXPECT_EQ(exec_->perception_restarts(), 0);
  EXPECT_TRUE(hooks.empty());
  feed.say("pid=2");
  ASSERT_TRUE(spin_until([&] {return wrist_calls_.size() == 2 && box_calls_.size() == 2;}, [&] {exec_->update(now());}));
  EXPECT_TRUE(wrist_calls_[1]);
  EXPECT_TRUE(box_calls_[1]);
  EXPECT_EQ(exec_->perception_restarts(), 1);
  ASSERT_EQ(hooks.size(), 1u);
  EXPECT_EQ(hooks[0].first, "pid=1");
  EXPECT_EQ(hooks[0].second, "pid=2");
}

// 검출 끊김 중 새 LOOK 은 안 보냄 → 검출이 돌아오고 (회복 1 s) 나서 보냄, 끊김 구간이 남음. FAULT 아님 (detection_fault_s 안)
TEST_F(ExecutorActions, LookWaitsForDetectionsThenGoes)
{
  start("", 30.0, {rclcpp::Parameter("require_detections", true), rclcpp::Parameter("detection_frames", std::vector<std::string>{"cam"}),
      rclcpp::Parameter("detection_timeout_s", 0.3), rclcpp::Parameter("wrist_on_grace_s", 0.1), rclcpp::Parameter("detection_fault_s", 20.0),
      rclcpp::Parameter("monitor_fault_s", 1.0)},
    {"/odom:=" + ns_for_next() + "/odom"});
  PercFeed feed(server_, ns_);
  feed.warm([&] {spin();});
  // 끊김이 먼저 시작 (켠 직후 grace 안에 보낸 LOOK 은 막지 않음 — 그 LOOK 결과는 미션 gap_guard 가 기다렸다 다시)
  ASSERT_TRUE(spin_until([&] {return exec_->detection_gap().active();}, [&] {
      feed.tick(false);
      exec_->update(now());
      EXPECT_FALSE(exec_->supervise(now()).has_value());
    }));
  exec_->set_steps({look()});
  // 검출 없이 1.5 s: monitor_fault_s 1 s 를 넘어도 검출은 detection_fault_s (20 s) 라 FAULT 아님, LOOK 안 보냄
  const auto t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
  while (std::chrono::steady_clock::now() < t_end) {
    feed.tick(false);
    exec_->update(now());
    const auto f = exec_->supervise(now());
    ASSERT_FALSE(f.has_value()) << f->code << " " << f->detail;
    exec_->step(now());
    spin();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(exec_->detection_gap().active());
  EXPECT_TRUE(arm_srv_->goals.empty());
  const auto w = exec_->wait_reasons();
  EXPECT_NE(std::find(w.begin(), w.end(), "DETECTION_LOST"), w.end());
  // 검출이 돌아옴 → 1 s 회복 뒤 LOOK
  ASSERT_TRUE(spin_until([&] {return !arm_srv_->goals.empty();}, [&] {
      feed.tick(true);
      exec_->update(now());
      EXPECT_FALSE(exec_->supervise(now()).has_value());
      exec_->step(now());
    }));
  EXPECT_FALSE(exec_->detection_gap().active());
  EXPECT_EQ(exec_->detection_gap().count(), 1);
  EXPECT_GT(exec_->detection_gap().ended() - exec_->detection_gap().started(), 1.0);
  EXPECT_TRUE(exec_->detection_gap().overlaps(exec_->detection_gap().started()));
  EXPECT_FALSE(exec_->detection_gap().overlaps(now() + 1.0));
}

// 안 돌아오면 detection_fault_s 뒤 DETECTION_LOST FAULT
TEST_F(ExecutorActions, DetectionGapFaultsAfterDetectionFaultS)
{
  start("", 30.0, {rclcpp::Parameter("require_detections", true), rclcpp::Parameter("detection_frames", std::vector<std::string>{"cam"}),
      rclcpp::Parameter("detection_timeout_s", 0.2), rclcpp::Parameter("wrist_on_grace_s", 0.1), rclcpp::Parameter("detection_fault_s", 0.8)},
    {"/odom:=" + ns_for_next() + "/odom"});
  PercFeed feed(server_, ns_);
  feed.warm([&] {spin();});
  const auto t0 = std::chrono::steady_clock::now();   // 끊김 시작 전 (FAULT 는 끊김 시작부터 detection_fault_s)
  ASSERT_TRUE(spin_until([&] {return exec_->detection_gap().active();}, [&] {
      feed.tick(false);
      exec_->update(now());
      exec_->supervise(now());
    }));
  exec_->set_steps({look()});   // 팔 작업이 없어야 바로 FAULT (팔 작업 · 쥔 퍽이면 sensor_fault_hold_s 만큼 미룸)
  std::optional<ExecEvent> f;
  ASSERT_TRUE(spin_until([&] {return f.has_value();}, [&] {
      feed.tick(false);
      exec_->update(now());
      f = exec_->supervise(now());
      if (!f) {exec_->step(now());}
    }));
  EXPECT_EQ(f->code, "DETECTION_LOST");
  EXPECT_GT(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 0.8);
  EXPECT_TRUE(arm_srv_->goals.empty());
}

// 정차점 보정표: 정밀 접근 중 받은 잰 정차점 → 그 접근이 실패해 재시도하면 Nav2 · 정밀 접근 목표 둘 다 잰 정차점 기준
//   (재시도가 박스를 못 재면 dice_map 정차점에 서서 실제 박스와 어긋남 → 먼 퍽이 안 닿음)
TEST_F(ExecutorActions, DockRetryUsesMeasuredStop)
{
  const std::string topic = "/exec_test_dock_box_" + std::to_string(::getpid());   // start() 전이라 ns_ 아직 없음
  start("", 30.0, {{"dock_box_topic", topic}, {"dock_handoff_m", 0.0}, {"nav_retries", 1}});
  FakeServer<Nav> dock_srv(server_, ns_ + "/dock_to_pose");
  auto pub = server_->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
  Step s;
  s.type = StepType::Navigate;
  s.target = Target::Fixed;
  s.pose = Pose2D{1.0, 0.0, 0.0};
  s.dock = true;
  s.name = "nav:B3";
  exec_->set_steps({s});
  const auto tick = [&] {exec_->step(now());};
  const auto finish = [&](FakeServer<Nav> & srv, std::size_t i, bool ok) {
      ASSERT_TRUE(spin_until([&] {return srv.goals.size() > i;}, tick));
      auto res = std::make_shared<Nav::Result>();
      if (ok) {srv.goals[i]->succeed(res);} else {srv.goals[i]->abort(res);}
    };
  finish(*nav_srv_, 0, true);                       // 1 단계 도착 → 정밀 접근
  ASSERT_TRUE(spin_until([&] {return dock_srv.goals.size() > 0;}, tick));
  EXPECT_NEAR(dock_srv.goals[0]->get_goal()->pose.pose.position.x, 1.0, 1e-6);
  geometry_msgs::msg::PoseStamped m;                // 잰 정차점 (map): 5 cm 멀고 4 cm 옆 · 0.1 rad
  m.header.frame_id = "map";
  m.pose.position.x = 1.05;
  m.pose.position.y = 0.04;
  m.pose.orientation.z = std::sin(0.05);
  m.pose.orientation.w = std::cos(0.05);
  ASSERT_TRUE(spin_until([&] {return pub->get_subscription_count() > 0;}, tick));
  for (int i = 0; i < 20; ++i) {
    pub->publish(m);
    spin();
    tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  finish(dock_srv, 0, false);                       // 정밀 접근 실패 → 재시도
  ASSERT_TRUE(spin_until([&] {return nav_srv_->goals.size() > 1;}, tick));
  const auto & g1 = nav_srv_->goals[1]->get_goal()->pose.pose.position;
  EXPECT_NEAR(g1.x, 1.05 - 0.025 * std::cos(0.1), 1e-3);   // Nav2 목표 = 잰 정차점 뒤 dock_nav_back_m
  EXPECT_NEAR(g1.y, 0.04 - 0.025 * std::sin(0.1), 1e-3);
  Pose2D fx;
  ASSERT_TRUE(exec_->dock_fix(s.pose, fx));
  EXPECT_NEAR(fx.yaw, 0.1, 1e-3);
  finish(*nav_srv_, 1, true);
  ASSERT_TRUE(spin_until([&] {return dock_srv.goals.size() > 1;}, tick));
  EXPECT_NEAR(dock_srv.goals[1]->get_goal()->pose.pose.position.x, 1.05, 1e-3);   // 정밀 접근 목표 = 잰 정차점
  // 다른 정차점 (6 cm 넘게 먼) 은 안 씀, 지우면 없음
  EXPECT_FALSE(exec_->dock_fix(Pose2D{1.06, 0.0, 0.0}, fx));
  exec_->clear_dock_fix();
  EXPECT_FALSE(exec_->dock_fix(s.pose, fx));
}
