"""
planning_master.py — 판단 마스터: 미션 상태머신 ("지금 무슨 일 중인가")
==========================================================================
인지 결과 · odom 을 **한 곳에서** 받아 다음 할 일을 정한다. 미션 플로우차트를 코드로 옮긴 것 (그림: docs/flowchart.png).

  INIT → SEARCH_QR → READ_QR → SEARCH_PUCK → APPROACH_PUCK → PICK → GO_TO_SHELF → PLACE
           ↑                                                                        │
           └──────────────────── 퍽 남음 ◀───────────────────────────────────────────┘
                                 다 옮김 → DONE
  어느 상태에서든: 제어 끊김 → FAULT (멈춤),  신호등 빨강 · 주황 → 그 자리에서 대기
  TODO 동적 장애물 (사람이 들어오거나 원통형 물체가 굴러옴): 정지 · 대기는 Nav2 collision_monitor 가 먼저 함.
       여기서는 오래 막히면 경로 다시 요청 · 너무 오래면 다른 목표부터 (Nav2 연결 때)
  TODO 제한 시간: 미션 시작부터 시간을 재서 남은 시간에 따라 "남은 퍽 포기하고 마무리" 같은 판단 (몇 분인지 미정)

받는 것  : /perception/qr (QrInstruction), /perception/detections, /perception/signals (DetectionArray)
           ← perception_master
           /odom (제어가 살아 있나 확인용) ← swerve_controller (ros2_control 안)
           /control/motion_mode (지금 누구 차례 — 상태 표시용) ← control_master
보내는 것: /mission/status (MissionStatus, 2 Hz)
부르는 것: /arm_task (ArmTask 액션 → control_master)
           TODO: Nav2 navigate_to_pose 액션 (이동 → Nav2 → /cmd_vel → swerve_controller)

"기본안 먼저": 지금은 퍽을 **하나씩** 옮기는 가장 단순한 흐름 (D9).
여러 개 싣고 다니기로 정해지면 PICK 다음 분기만 바꾸면 됨.

전력 나눠 쓰기 (D15) — 이 노드는 신경 쓸 필요 없음
  "바퀴와 팔을 동시에 안 움직이기" 는 control_master(차례) + swerve_controller(바퀴 차례일 때만 움직임) 가 지킨다.
  여기서는 PICK/PLACE 때 /arm_task 만 보내면, control_master 가 바퀴를 세우고 → 몸통이 멈춘 걸 확인하고 → 팔을 움직인다.
  단 PICK/PLACE 전에 Nav2 이동은 끝나 있어야 함 (아니면 Nav2 가 계속 cmd_vel 을 보냄 — 무시되긴 하지만 낭비)

2026-10-16 검토 반영
  R2  팔 작업 수명은 action_session.ArmSession 한 객체가 관리 (수락 답이 도착했지만 아직 처리 전인 순간의 FAULT 도 취소)
  R4  매 tick 순서: ① 진행 중 팔 작업 감시(결과 · 시간 제한) ② 건강(제어 끊김 → FAULT) ③ 정지 정책(신호) ④ 새 미션 전이
      예전에는 신호 대기 때 맨 앞에서 return 해서 팔 작업의 수락 · 완료 시간 제한과 결과 처리까지 멈췄음
  QR  읽은 지시는 mission_store.MissionStore (같은 글자 중복 제거 · 보관 상한)

dry_run: true (기본값)
  로봇·카메라 없이 흐름만 보려고, 인지·이동 단계는 시간이 지나면 저절로 넘어간다.
  단 PICK/PLACE 는 진짜로 /arm_task 액션을 부른다 → 층 사이 연결이 실제로 되는지 확인됨.
"""

from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from nt_interfaces.action import ArmTask
from nt_planning.action_session import ArmSession
from nt_planning.mission_store import MissionStore
from nt_interfaces.msg import DetectionArray, MissionStatus, MotionMode, QrInstruction
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class State(Enum):
    INIT = "INIT"                     # 센서 · 모터 확인, 팔 접기
    SEARCH_QR = "SEARCH_QR"           # QR 찾아 돌아다니기
    READ_QR = "READ_QR"               # 멈춰서 QR 읽기
    SEARCH_PUCK = "SEARCH_PUCK"       # 지시받은 색 퍽 찾기
    APPROACH_PUCK = "APPROACH_PUCK"   # 퍽 앞까지 이동 · 정렬 (팔 닿는 거리 289 mm 안으로)
    PICK = "PICK"                     # 멈춘 뒤 팔로 집기 (주행 중 집기 금지 규칙)
    GO_TO_SHELF = "GO_TO_SHELF"       # 목표 선반으로 이동
    PLACE = "PLACE"                   # 놓기
    DONE = "DONE"
    FAULT = "FAULT"                   # 무엇이든 이상 → 멈춤


class PlanningMaster(Node):
    def __init__(self):
        super().__init__("planning_master")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("dry_run_step_s", 1.5)
        self.declare_parameter("dry_run_pucks", 2)
        self.declare_parameter("control_timeout_s", 1.0)
        self.declare_parameter("arm_retry_s", 15.0)          # 팔 작업이 거절되면 이 시간 동안 다시 보냄 (켤 때 JTC 준비 대기)
        self.declare_parameter("arm_accept_timeout_s", 5.0)  # 받아 줬다는 답이 이만큼 안 오면 FAULT
        self.declare_parameter("arm_result_timeout_s", 60.0) # 끝났다는 답이 이만큼 안 오면 취소 + FAULT
        self.declare_parameter("arm_cancel_give_up_s", 10.0) # 취소한 팔 작업의 결과를 이만큼 기다리고 포기 → FAULT 유지 (R8)
        self.declare_parameter("qr_max_items", 32)           # 보관할 QR 지시 상한 (같은 글자는 하나로)
        self.dry_run = self.get_parameter("dry_run").value
        self.step_s = self.get_parameter("dry_run_step_s").value
        self.control_timeout_s = self.get_parameter("control_timeout_s").value
        self.arm_retry_s = self.get_parameter("arm_retry_s").value
        self.arm_accept_timeout_s = self.get_parameter("arm_accept_timeout_s").value
        self.arm_result_timeout_s = self.get_parameter("arm_result_timeout_s").value

        # 미션 기억
        self.state = State.INIT
        self.entered_at = self.now_s()
        self.missions = MissionStore(max_items=self.get_parameter("qr_max_items").value)   # 읽은 QR 지시들
        self.pucks_done = 0
        self.pucks_total = self.get_parameter("dry_run_pucks").value if self.dry_run else 0
        self.signal_stop = False      # 신호등이 빨강 · 주황이면 True
        self.last_odom = None
        self.last_odom_stamp = None
        self.turn = "-"               # control_master 가 알려 주는 지금 차례 (표시용)
        # 팔 작업 한 건의 수명 (보냄 → 수락 → 실행 → 결과 · 취소) — action_session.py
        self.arm_session = ArmSession(self.get_logger(), retry_s=self.arm_retry_s,
                                      accept_timeout_s=self.arm_accept_timeout_s,
                                      result_timeout_s=self.arm_result_timeout_s,
                                      cancel_give_up_s=self.get_parameter("arm_cancel_give_up_s").value)
        self.detail = ""

        # 연결
        self.create_subscription(QrInstruction, "/perception/qr", self.on_qr, 10)
        self.create_subscription(DetectionArray, "/perception/detections", self.on_detections, 10)
        self.create_subscription(DetectionArray, "/perception/signals", self.on_signals, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.pub_status = self.create_publisher(MissionStatus, "/mission/status", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(MotionMode, "/control/motion_mode", self.on_turn, latched)
        self.arm = ActionClient(self, ArmTask, "/arm_task")

        self.create_timer(0.1, self.tick)            # 10 Hz 로 판단
        self.create_timer(0.5, self.publish_status)  # 2 Hz 로 상태 방송
        self.get_logger().info(f"planning_master 시작 (dry_run={self.dry_run})")

    # ------------------------------------------------------------------ 입력
    def on_qr(self, msg: QrInstruction):
        if self.missions.add(msg):
            self.get_logger().info(f"QR 지시 추가: {msg.puck_color} → {msg.target_shelf_id} (보관 {len(self.missions)})")

    def on_detections(self, msg: DetectionArray):
        pass   # TODO 지시받은 색의 퍽이 보이면 위치 저장 → SEARCH_PUCK 에서 사용

    def on_signals(self, msg: DetectionArray):
        # 신호등은 빨강 · 주황 · 초록. 주황도 대기 (주황 다음은 빨강이라 미리 서는 쪽이 안전 — "느려도 완수", 기본안)
        self.signal_stop = any(d.kind == "traffic_light" and d.label in ("red", "orange") for d in msg.detections)

    def on_odom(self, msg: Odometry):
        # 측정 시각(header.stamp)이 새로워졌을 때만 "제어가 살아 있음" 으로 침
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_odom_stamp is not None and stamp <= self.last_odom_stamp:
            return
        self.last_odom_stamp = stamp
        self.last_odom = self.now_s()

    def on_turn(self, msg: MotionMode):
        self.turn = msg.reason or str(msg.mode)

    # ------------------------------------------------------------------ 판단
    def tick(self):
        now = self.now_s()
        # ⓪ 취소한 팔 작업 뒷정리는 상태와 상관없이 (FAULT · DONE 이어도 결과가 올 때까지 책임)
        self.arm_session.service_orphans(now)
        # (재검증 R8) 취소 완료를 끝내 확인 못 한 팔 작업이 있으면 FAULT 유지 (팔이 멈췄는지 사람이 확인)
        if self.arm_session.unresolved_cancels and self.state != State.FAULT:
            self.go(State.FAULT, "팔 작업 취소 완료를 확인 못 함 → 정지 (팔 상태 직접 확인)")
            return
        if self.state in (State.DONE, State.FAULT):
            return
        # ① 건강: 제어가 끊기면 FAULT (go 가 진행 중 팔 작업을 취소)
        if self.control_lost():
            self.go(State.FAULT, "제어(/odom)가 끊김 → 정지")
            return
        # ② 이미 시작한 팔 작업은 신호와 상관없이 계속 감시 (수락 · 완료 시간 제한, 결과 처리) — R4
        #    팔은 제자리 동작이라 신호 규칙("이동")과 별개. 새 팔 작업은 아래 ③ 때문에 신호 중에는 시작 안 함
        if self.signal_stop and not self.arm_session.active:
            # ③ 정지 정책: 새 진전(이동 · 새 팔 작업)은 막음
            # TODO Nav2 연결 때: 진행 중 navigate_to_pose 취소 또는 일시정지 + control_master 에 주행 중지 요청
            self.detail = "신호 빨강 · 주황 → 대기"
            return
        # ④ 미션 전이
        handler = getattr(self, "do_" + self.state.name.lower())
        nxt = handler()
        if nxt is not None and nxt != self.state:
            self.go(nxt)

    # 상태마다 함수 하나. "다음 상태" 를 돌려주면 넘어가고, None 이면 그대로 머무름
    def do_init(self):
        # TODO 센서 · 모터 확인 (토픽이 들어오는지)
        # 팔을 접은 자세로 → 그다음부터 주행 (주행 중에는 팔이 항상 접혀 있음)
        return self.run_arm(ArmTask.Goal.HOME, State.SEARCH_QR)

    def do_search_qr(self):
        if self.dry_run:
            return State.READ_QR if self.waited(self.step_s) else None
        # TODO 탐색 경로 따라 Nav2 이동, QR 후보가 보이면 멈추고 READ_QR
        return None

    def do_read_qr(self):
        if self.dry_run:
            return State.SEARCH_PUCK if self.waited(self.step_s) else None
        # TODO 새 지시가 들어오면 pucks_total 갱신 후 다음으로 (지시 상태: missions.mark(msg, ACTIVE/DONE))
        return State.SEARCH_PUCK if self.missions.next_waiting() is not None else None

    def do_search_puck(self):
        if self.dry_run:
            return State.APPROACH_PUCK if self.waited(self.step_s) else None
        return None   # TODO

    def do_approach_puck(self):
        if self.dry_run:
            return State.PICK if self.waited(self.step_s) else None
        return None   # TODO Nav2 로 선반 앞까지 → 정밀 정렬 → 정지 확인

    def do_pick(self):
        # 바퀴를 세우고 팔을 움직이는 순서는 control_master 가 지킴 (D15)
        return self.run_arm(ArmTask.Goal.PICK, State.GO_TO_SHELF)

    def do_go_to_shelf(self):
        if self.dry_run:
            return State.PLACE if self.waited(self.step_s) else None
        return None   # TODO QR 이 말한 선반으로 Nav2 이동

    def do_place(self):
        nxt = self.run_arm(ArmTask.Goal.PLACE, None)
        if nxt is None:
            return None
        if nxt == State.FAULT:
            return State.FAULT
        self.pucks_done += 1
        return State.DONE if self.pucks_done >= self.pucks_total else State.SEARCH_PUCK

    # ------------------------------------------------------------------ 팔 부르기
    def run_arm(self, task: int, next_state):
        """ArmTask 액션을 진행. 끝나면 next_state(성공) / FAULT(실패). 진행 중이면 None.

        보내기 · 수락 · 결과 · 시간 제한 · 취소는 ArmSession 이 맡음 (action_session.py, 시험 test_action_session.py)
          - 거절이면 arm_retry_s 동안 1 s 마다 다시, 서버가 안 보여도 arm_retry_s 뒤 FAULT
          - 수락 답 arm_accept_timeout_s, 결과 arm_result_timeout_s 넘으면 취소 + FAULT
        """
        def send():
            goal = ArmTask.Goal(task=task, target=PoseStamped())   # TODO 실제 퍽·선반 위치
            goal.target.header.frame_id = "base_link"
            return self.arm.send_goal_async(goal)

        kind, text = self.arm_session.poll(self.now_s(), task, send, self.arm.server_is_ready())
        if kind == ArmSession.PENDING:
            self.detail = text
            return None
        if kind == ArmSession.OK:
            return next_state or self.state
        self.detail = text
        return State.FAULT

    # ------------------------------------------------------------------ 도우미
    def go(self, nxt: State, detail: str = ""):
        if nxt == State.FAULT:
            # 멈춤 상태로 가면서 팔 작업을 계속 두지 않음 — 수락 답 대기 · 도착 · 실행 중 어느 단계든 (R2)
            self.arm_session.cancel("FAULT", self.now_s())
        self.get_logger().info(f"{self.state.value} → {nxt.value} {detail}")
        self.state = nxt
        self.entered_at = self.now_s()
        self.detail = detail

    def waited(self, seconds: float) -> bool:
        return self.now_s() - self.entered_at >= seconds

    def control_lost(self) -> bool:
        # 한 번이라도 받은 뒤에만 판단 (켜자마자 FAULT 방지)
        return self.last_odom is not None and self.now_s() - self.last_odom > self.control_timeout_s

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_status(self):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state.value
        extra = f" (취소 확인 대기 {self.arm_session.pending_cancels})" if self.arm_session.pending_cancels else ""
        if self.arm_session.unresolved_cancels:
            extra += f" (취소 확인 실패 {self.arm_session.unresolved_cancels})"
        msg.detail = f"[{self.turn}] {self.detail}{extra}"
        msg.pucks_done = self.pucks_done
        msg.pucks_total = self.pucks_total
        msg.fault = self.state == State.FAULT
        self.pub_status.publish(msg)


def main():
    rclpy.init()
    node = PlanningMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
