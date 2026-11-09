#!/usr/bin/env python3
"""학습한 지역 계획기 정책을 ROS 로 돌리는 노드 (MPPI · bt_navigator 대신). 실물 = navigation nav2.launch.py local_planner:=rl 이 띄움 (주행 · 정밀 접근 두 개),
Isaac 시험 = sim/robot_sim/rl/tasks/local_planner/deploy/run_isaac_dock.sh (같은 파일 링크). 학습 코드 = sim/robot_sim/rl/tasks/

받는 것: navigate_to_pose 액션 (목표), /odom (몸 속도), /joint_states (조향 위치 · 바퀴 속도), /scan (navigation scan_merger: 앞 · 뒤 합친 360°, 몸 상자 ±0.17 × ±0.19 안 점 버림), TF map → base_footprint (--goal_frame odom 이면 odom → base_footprint + 목표 받을 때 한 번 map → odom)
  /scan 이 scan_stale_s 넘게 안 오면 (라이다 한쪽 끊기면 scan_merger 가 안 냄) 0 속도 내고 기다림
부르는 것: compute_path_to_pose (Nav2 planner_server, 1 s 마다 다시)
보내는 것: cmd_vel_nav (TwistStamped, 20 Hz) → collision_monitor → /cmd_vel
관측은 학습 환경 (../env.py _compute_obs) 과 같은 순서 · 정규화. 성공 = 2 cm · 2° 안에서 0.3 s 멈춤.
목표 좌표계 (--goal_frame): map (기본) = 매 주기 map 기준 자세로 판정. odom = 목표를 받는 순간 map → odom 으로 한 번 바꿔 고정, 그 뒤 자세 · 판정 · 경로는 odom 기준
  → 접근 중 slam_toolbox 가 map → odom 을 고쳐도 (루프 클로저 등 1–2 cm 튐) 목표가 안 따라 튐. 정밀 접근 (dock_to_pose, 0.8 m 안 · 수 초) 용. 짧은 거리라 odom 미끄러짐은 작음
박스 맞춤 (--box_track, 정밀 접근 노드): 목표 = 지도 정차점 (박스를 봄, 박스 중심에서 --box_dist) → 스캔마다 박스 앞면을 맞춰 (box_tracker.py)
  목표를 '잰 박스 중심에서 box_dist, 정면' 으로 옮김. SLAM 지도 때와 시연 때 박스 자리가 달라도 원하는 거리에 섬. 경로 = 지금 자리 → 목표 직선 (학습과 같음)
  직선 경로를 몸 + 1 cm 가 쓸고 갈 때 박스 아닌 점 (벽) 이 연속 --dock_block_n 스캔 걸리면 멈추고 실패 (실물 B1 왼쪽 벽 충돌)
출발 자리 영역 끄기 (--near_disable_polygons, 주행 노드): 빠져나오기 경로를 썼거나 PolygonStop 이 지금 막을 때만 끔, 끈 동안 병진 --near_vmax
경로: 목표가 지도 장애물이면 (박스를 지도에 넣은 매핑) 목표를 보는 방향 반대로 물려 계획 + 직선 (--goal_back_m)
정책: --policy <exported/policy_N.pt> (TorchScript, torch 필요) 또는 <exported/policy_N.npz> (numpy, torch 없음 = Jetson). 둘 다 export.py 가 만듦
그림자 모드 (--shadow): 실물 시험용. navigate_to_pose 를 받지 않음 (MPPI · bt_navigator 가 주행), 대신 /plan (planner_server 경로, 끝 = 목표)
  을 받아 같은 목표로 정책 명령을 계산 → --shadow_topic (기본 cmd_vel_rl_shadow) 에만 냄 (collision_monitor · /cmd_vel 로 안 감).
  --log_csv 면 매 주기 시각 · 자세 · 정책 명령 · MPPI 명령 (/cmd_vel_nav) · 목표 거리 · 최소 스캔 기록. 정책 last_cmd 는 자기 명령이라 MPPI 주행과는 열린 고리 비교
실행: 시스템 Lyrical 파이썬 (npz 정책) 또는 Isaac 파이썬 (pt 정책) + 시스템 Lyrical rclpy
  python3 rl/tasks/local_planner/deploy/local_planner_node.py --policy <exported/policy_N.npz> --cfg <run>/cfg.json [--shadow --log_csv rl_shadow.csv]
  PYTHONPATH=/opt/ros/lyrical/lib/python3.12/site-packages:<em·lark> ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/local_planner/deploy/local_planner_node.py --policy <exported/policy.pt> --cfg <run>/cfg.json
"""
import argparse
import json
import math
import sys
import threading
import time

import numpy as np
import yaml

import rclpy
import rclpy.executors
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState, LaserScan

from box_tracker import BoxTracker


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Path:
    """경로 (map 좌표 점) · 호 길이 · 창 안 투영 (swerve_follow.paths.PathBatch 와 같은 식, 한 개)"""

    def __init__(self, pts):
        self.p = np.asarray(pts, float)
        seg = np.linalg.norm(np.diff(self.p, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.L = float(self.s[-1])

    def point_at(self, s):
        s = np.clip(s, 0.0, self.L)
        return np.stack([np.interp(s, self.s, self.p[:, 0]), np.interp(s, self.s, self.p[:, 1])], -1)

    def project(self, xy, s_prev=None, back=0.1, fwd=0.5):
        p0, p1 = self.p[:-1], self.p[1:]
        d = p1 - p0
        L2 = np.maximum((d * d).sum(1), 1e-12)
        t = np.clip(((xy - p0) * d).sum(1) / L2, 0, 1)
        s = self.s[:-1] + t * np.sqrt(L2)
        if s_prev is not None:
            s = np.clip(s, s_prev - back, s_prev + fwd)
            t = np.clip((s - self.s[:-1]) / np.sqrt(L2), 0, 1)
        foot = p0 + d * t[:, None]
        dist = np.linalg.norm(xy - foot, axis=1)
        k = int(dist.argmin())
        return max(0.0, float(s[k])), float(dist[k])


class LocalPlanner(Node):
    def __init__(self, a):
        super().__init__(a.node_name, parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=a.sim_time)])
        cfg = json.loads(open(a.cfg).read())['env']
        self.c = cfg
        sw_path = cfg['swerve_yaml'] if not a.swerve_yaml else a.swerve_yaml              # 다른 PC (Jetson) 면 경로 다시 줌
        sw = yaml.safe_load(open(sw_path))
        self.steer_names, self.drive_names = sw['joints']['steer'], sw['joints']['drive']
        self.rmin, self.rmax = np.array(sw['steer_range_rad']['min']), np.array(sw['steer_range_rad']['max'])
        self.max_wheel = float(sw['hardware_max']['wheel_speed'])
        self.lim3 = np.array([cfg['vmax'], cfg['vmax'], cfg['wmax']])
        if a.policy.endswith('.npz'):
            from numpy_policy import NumpyPolicy
            self.act = NumpyPolicy(a.policy)
        else:
            import torch
            pol = torch.jit.load(a.policy, map_location='cpu').eval()

            def act(o):
                with torch.inference_mode():
                    return pol(torch.from_numpy(np.asarray(o, np.float32)).unsqueeze(0))[0].numpy()
            self.act = act
        self.tol_p, self.tol_y = a.tol_pos, math.radians(a.tol_yaw_deg)
        self.planner_id = a.planner_id
        self.goal_frame = a.goal_frame
        self.box_track, self.box_dist = a.box_track, a.box_dist
        self.straight_max = a.straight_max
        self.escape_m = a.escape_m
        self.escaped = False                                    # 이번 목표에서 빠져나오기 경로를 씀 (= 치명 영역 안 출발 = 붙어 섬)
        self.settle, self.min_lin, self.min_ang = not a.no_settle, a.min_cmd_lin, a.min_cmd_ang
        self.near_ok_s = a.near_ok_s
        # 정밀 접근 (정차 0.2575 = 차체 앞 끝 ↔ 박스 면 1 cm) 동안 collision_monitor 영역 끄기: PolygonStop ±0.18 · FootprintApproach 가
        #   박스에 걸려 마지막 몇 cm 를 못 감. 목표마다 끄고 끝나면 (성공 · 실패 · 취소) 다시 켬
        self.cm_polygons = [x for x in a.disable_polygons.split(',') if x]
        # 정밀 접근 정차 (박스 면 1–2 cm) 에서 출발할 때: PolygonStop 이 박스에 걸려 0 속도 → 그때만 영역 끄고 출발 자리에서 벗어나면 다시 켬
        #   끄는 조건 ('스캔 최소 < 0.30' 같은 거리 조건은 출발 구역 벽에도 걸려 영역 끈 채 출발함):
        #     빠져나오기 경로를 씀 (= 치명 영역 안 출발) 또는 PolygonStop 이 지금 막고 있음 (collision_monitor_state STOP · PolygonStop,
        #     또는 /scan 점이 PolygonStop + 1 cm 안). 박스 면은 scan_merger 몸 상자 경계라 보였다 말았다 함 → 벗어날 때까지 매 주기 봄
        #   켜는 조건: near_leave_m 멀어짐, 또는 near_clear_m 움직였고 PolygonStop + near_clear_pad 안에 점 없음
        #   끈 동안 병진 near_vmax 상한 (몸 상자 안은 /scan 에 안 보여 벽에 붙어도 모름)
        self.near_polygons = [x for x in a.near_disable_polygons.split(',') if x]
        self.near_leave, self.near_off = a.near_leave_m, False
        self.near_clear_m, self.near_clear_pad, self.near_vmax = a.near_clear_m, a.near_clear_pad, a.near_vmax
        self.stop_half = a.stop_half                            # collision_monitor PolygonStop 반 변 (config/nav2.yaml ±0.18)
        self.cm_state, self.cm_state_t = None, -1e9
        # PolygonStop 이 계속 막으면 시간 초과 (40–60 s) 까지 버티지 않고 실패로 끝냄 (실물 L3 · B4: 지도 장애물 목표 앞에서
        #   정책은 계속 밀고 PolygonStop 은 0 으로 막아 40 s 멈춤 → 그동안 물린 목표 · 빠져나오기가 바뀌며 앞뒤로 움직임)
        self.stop_abort_s, self.stop_since = a.stop_abort_s, None
        # 정밀 접근 직선 경로 검사 (--box_track): 지금 자리 → 목표 직선을 몸 (dock_body_half + 1 cm) 이 쓸고 갈 때 박스 아닌 점 (벽) 이 걸리면
        #   멈추고 실패로 끝냄 (planning 이 1 단계부터 다시 = Nav2 로 정차점 뒤까지 온 뒤 넘김). 실물 B1: 정밀 접근 중 왼쪽 벽 충돌
        self.dock_body = np.array([float(v) for v in a.dock_body_half.split(',')])
        self.dock_block_n = a.dock_block_n
        self.box_reject_n = a.box_reject_n
        # 목표가 지도 장애물 (박스를 지도에 넣고 매핑 = arena5) 이라 planner 가 경로를 못 내면: 목표를 보는 방향 반대로 0.05 m 씩
        #   goal_back_m 까지 물려 계획하고 끝에 '물린 점 → 목표' 직선을 붙임 (박스 정면으로 들어옴). planner tolerance 를 키우는 것 대신
        #   (tolerance 는 아무 방향 가까운 빈 칸으로 끝내 박스 옆 · 비스듬히 들어옴)
        self.goal_back_m, self.goal_back = a.goal_back_m, None
        self.cm_client = self.create_client(SetParameters, f'{a.cm_node}/set_parameters') if (self.cm_polygons or self.near_polygons) else None
        cb = ReentrantCallbackGroup()
        qs = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom = np.zeros(3)
        self.q = np.zeros(4)
        self.w = np.zeros(4)
        self.scan, self.scan_t = None, -1e9
        self.create_subscription(Odometry, '/odom', self.on_odom, 10, callback_group=cb)
        self.create_subscription(JointState, '/joint_states', self.on_js, 10, callback_group=cb)
        self.create_subscription(LaserScan, a.scan_topic, self.on_scan, qs, callback_group=cb)
        if self.near_polygons or self.stop_abort_s > 0:
            self.create_subscription(CollisionMonitorState, a.cm_state_topic, self.on_cm_state, 10, callback_group=cb)
        self.scan_stale_s = a.scan_stale_s
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.lock = threading.Lock()
        if a.shadow:
            self.pub = self.create_publisher(TwistStamped, a.shadow_topic, 10)
            self.sh = {'path': None, 'goal': None, 'plan_t': -1e9, 's': 0.0, 'hist': None, 'last': np.zeros(3), 'mppi': np.zeros(3)}
            self.create_subscription(PathMsg, a.plan_topic, self.on_plan, 10, callback_group=cb)
            self.create_subscription(TwistStamped, '/cmd_vel_nav', lambda m: self.sh.__setitem__(
                'mppi', np.array([m.twist.linear.x, m.twist.linear.y, m.twist.angular.z])), 10, callback_group=cb)
            self.csv = open(a.log_csv, 'w') if a.log_csv else None
            if self.csv:
                self.csv.write('t,x,y,yaw,rl_vx,rl_vy,rl_wz,mppi_vx,mppi_vy,mppi_wz,goal_dist,goal_dyaw,scan_min\n')
            dt = cfg['ctrl_dt'] * cfg['decimation']
            self.create_timer(dt, self.shadow_tick, callback_group=cb)
            mode = f'그림자 ({a.plan_topic} → {a.shadow_topic})'
        else:
            self.pub = self.create_publisher(TwistStamped, 'cmd_vel_nav', 10)
            self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose', callback_group=cb)
            self.server = ActionServer(self, NavigateToPose, a.action_name, self.execute, callback_group=cb,
                                       goal_callback=lambda g: GoalResponse.ACCEPT, cancel_callback=lambda g: CancelResponse.ACCEPT)
            mode = f'주행 ({a.action_name} → cmd_vel_nav)'
        self.get_logger().info(f'정책 {a.policy} 불러옴, {mode}, 한계 {self.lim3.tolist()}, 성공 {self.tol_p * 100:.0f} cm · {a.tol_yaw_deg}°')

    # ------------------------------------------------------------ 입력
    def on_odom(self, m):
        t = m.twist.twist
        self.odom = np.array([t.linear.x, t.linear.y, t.angular.z])

    def on_scan(self, m):
        self.scan, self.scan_t = m, self.now()

    def on_cm_state(self, m):
        stop = m.action_type == CollisionMonitorState.STOP
        if not stop:
            self.stop_since = None
        elif self.stop_since is None:
            self.stop_since = self.now()
        self.cm_state, self.cm_state_t = m, self.now()

    def body_points(self):
        """/scan 점 → base_footprint xy (몸 상자 안 점은 scan_merger 가 이미 버림). 없으면 None"""
        m = self.scan
        if m is None or self.now() - self.scan_t > self.scan_stale_s:
            return None
        r = np.asarray(m.ranges, float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        if m.header.frame_id not in ('base_footprint', ''):
            pts, _ = self.scan_points('base_footprint')
            return pts
        return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1)

    def stop_blocking(self):
        """PolygonStop 이 지금 막고 있는지: collision_monitor 상태 (STOP · PolygonStop) 또는 /scan 점이 PolygonStop + 1 cm 안"""
        st = self.cm_state
        if st is not None and st.action_type == CollisionMonitorState.STOP and st.polygon_name in self.near_polygons:
            return True
        pts = self.body_points()
        if pts is None or not len(pts):
            return False
        h = self.stop_half + 0.01
        return bool(((np.abs(pts[:, 0]) <= h) & (np.abs(pts[:, 1]) <= h)).any())

    def stop_zone_clear(self):
        pts = self.body_points()
        if pts is None:
            return False
        h = self.stop_half + self.near_clear_pad
        return not bool(((np.abs(pts[:, 0]) <= h) & (np.abs(pts[:, 1]) <= h)).any())

    def straight_blocked(self, pose, goal_xy, goal_yaw, tracker, pts):
        """정밀 접근 직선 (pose → 목표) 을 몸이 쓸고 갈 때 박스 아닌 점이 몸 + 1 cm 안에 들어오는지. pts = goal_frame xy.
        박스 점 = 잰 박스 앞면 평면 (로봇 쪽 +3 cm) 뒤 · 옆 반 변 + 8 cm 안 → 뺌 (마지막 정차는 면 1 cm 라 박스는 늘 걸림).
        (걸린 점 수, 처음 걸린 거리) 돌려줌"""
        if pts is None or not len(pts):
            return 0, None
        y = tracker.yaw0 + tracker.th
        n = -np.array([math.cos(y), math.sin(y)])               # 박스 → 로봇 쪽 법선
        t = np.array([-n[1], n[0]])
        rel = pts - (tracker.c + tracker.h * n)
        box = (rel @ n < 0.03) & (np.abs(rel @ t) < tracker.h + 0.08)
        P = pts[~box]
        L = float(np.linalg.norm(np.asarray(goal_xy) - pose[:2]))
        if not len(P) or L < 1e-3:
            return 0, None
        hx, hy = self.dock_body + 0.01
        dyaw = wrap(goal_yaw - pose[2])
        for k in np.linspace(0.0, 1.0, max(2, int(L / 0.02) + 1)):
            cx, cy = pose[:2] + k * (np.asarray(goal_xy) - pose[:2])
            yw = pose[2] + k * dyaw
            c, sn = math.cos(yw), math.sin(yw)
            dx, dy = P[:, 0] - cx, P[:, 1] - cy
            bx, by = c * dx + sn * dy, -sn * dx + c * dy
            hit = (np.abs(bx) <= hx) & (np.abs(by) <= hy)
            if hit.any():
                return int(hit.sum()), k * L
        return 0, None

    def on_js(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        if all(n in idx for n in self.steer_names + self.drive_names):
            self.q = np.array([m.position[idx[n]] for n in self.steer_names])
            self.w = np.array([m.velocity[idx[n]] for n in self.drive_names]) if m.velocity else np.zeros(4)

    def tf2d(self, target, source):
        """source → target 평면 변환 (x, y, yaw)"""
        t = self.tf.lookup_transform(target, source, rclpy.time.Time())
        return np.array([t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)])

    @staticmethod
    def apply2d(T, xy, yaw=None):
        c, s_ = math.cos(T[2]), math.sin(T[2])
        xy = np.asarray(xy, float)
        out = np.stack([T[0] + c * xy[..., 0] - s_ * xy[..., 1], T[1] + s_ * xy[..., 0] + c * xy[..., 1]], -1)
        return out if yaw is None else (out, wrap(yaw + T[2]))

    def pose(self, frame='map'):
        return self.tf2d(frame, 'base_footprint')

    def lidar(self):
        """/scan (scan_merger, 몸 상자 안 점은 이미 버림) → 몸 기준 10° 칸 최솟값 (빈 칸 = 최대). 학습 env 라이다와 같은 칸 · 같은 상자.
        스캔이 없거나 scan_stale_s 보다 오래되면 None"""
        m = self.scan
        if m is None or self.now() - self.scan_t > self.scan_stale_s:
            return None
        n, rmax = self.c['lidar_rays'], self.c['lidar_max']
        out = np.full(n, rmax)
        r = np.asarray(m.ranges, float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        if m.header.frame_id not in ('base_footprint', ''):            # scan_merger frame 을 바꿨을 때만
            try:
                t = self.tf.lookup_transform('base_footprint', m.header.frame_id, rclpy.time.Time())
            except Exception:
                return None
            yw = yaw_of(t.transform.rotation)
            x, y = (t.transform.translation.x + math.cos(yw) * x - math.sin(yw) * y,
                    t.transform.translation.y + math.sin(yw) * x + math.cos(yw) * y)
        bx, by = (float(v) for v in self.c.get('lidar_body_half', (0.17, 0.19)))
        keep = ~((np.abs(x) <= bx) & (np.abs(y) <= by))               # scan_merger 와 같은 몸 상자 (frame 이 다를 때 대비)
        x, y = x[keep], y[keep]
        k = np.round(np.arctan2(y, x) / (2 * math.pi / n)).astype(int) % n
        np.minimum.at(out, k, np.hypot(x, y))
        return out

    def scan_points(self, frame):
        """/scan 점 → frame xy (촬영 시각 TF, 없으면 최신). 몸 상자 안 점 뺌. (점, 스캔 시각) 또는 (None, None)"""
        m = self.scan
        if m is None:
            return None, None
        r = np.asarray(m.ranges, float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        try:
            t = self.tf.lookup_transform(frame, m.header.frame_id or 'base_footprint', rclpy.time.Time.from_msg(m.header.stamp))
        except Exception:
            try:
                t = self.tf.lookup_transform(frame, m.header.frame_id or 'base_footprint', rclpy.time.Time())
            except Exception:
                return None, None
        T = np.array([t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)])
        return self.apply2d(T, np.stack([x, y], 1)), self.scan_t

    # ------------------------------------------------------------ 경로
    def plan(self, goal_ps):
        """planner_server 경로 (map). 실패하면 빠져나오기: 정밀 접근 정차 (박스 면 1 cm) 는 costmap 치명 영역 안이라 지금 자리에서는
        경로가 안 나옴 → 몸 뒤 (보는 방향 반대) escape_m 이내 점에서 다시 계획해 '지금 자리 → 뒤 점 직선 + 그 경로' 로 씀.
        목표 쪽도 같음: 목표가 치명 영역이면 _plan_goal 이 목표를 물려 계획"""
        path = self._plan_goal(goal_ps, None)
        if path is not None or self.escape_m <= 0:
            return path
        try:
            x, y, yaw = self.pose('map')
        except Exception:
            return None
        for d in np.arange(0.05, self.escape_m + 1e-6, 0.05):
            st = PoseStamped()
            st.header.frame_id, st.header.stamp = 'map', self.get_clock().now().to_msg()
            st.pose.position.x, st.pose.position.y = x - d * math.cos(yaw), y - d * math.sin(yaw)
            st.pose.orientation.z, st.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path = self._plan_goal(goal_ps, st)
            if path is not None:
                self.escaped = True
                self.get_logger().info(f'지금 자리에서 경로 안 나옴 → 뒤 {d * 100:.0f} cm 로 빠져나와 감', throttle_duration_sec=2.0)
                return Path([(x, y)] + [tuple(q) for q in path.p])
        return None

    def _plan_goal(self, goal_ps, start):
        """목표 그대로 계획 → 실패하면 목표를 보는 방향 반대로 0.05 m 씩 goal_back_m 까지 물려 계획 + 끝에 목표 점 (직선).
        성공한 물림 거리는 이 목표 동안 먼저 쓰고, 안 되면 더 먼 쪽만 봄 (1 s 마다 다시 계획)"""
        if self.goal_back is None:
            path = self._plan_from(goal_ps, start)
            if path is not None or self.goal_back_m <= 0:
                return path
        g = goal_ps.pose
        gyaw = yaw_of(g.orientation)
        backs = list(np.arange(0.05, self.goal_back_m + 1e-6, 0.05))
        if self.goal_back is not None:
            # 정한 거리 → 더 먼 쪽만 (실물 L3: 1 s 마다 35 ↔ 45 cm 로 바뀌어 직선 끝점이 10 cm 씩 앞뒤로 움직임)
            backs = [b for b in backs if b >= self.goal_back - 1e-6]
        for b in backs:
            ps = PoseStamped()
            ps.header = goal_ps.header
            ps.pose.orientation = g.orientation
            ps.pose.position.x, ps.pose.position.y = g.position.x - b * math.cos(gyaw), g.position.y - b * math.sin(gyaw)
            path = self._plan_from(ps, start)
            if path is not None:
                if self.goal_back is None or abs(b - self.goal_back) > 1e-6:
                    self.get_logger().info(f'목표가 지도 장애물 → 목표 뒤 {b * 100:.0f} cm 까지 계획 + 직선', throttle_duration_sec=2.0)
                self.goal_back = float(b)
                return Path([tuple(q) for q in path.p] + [(g.position.x, g.position.y)])
        if self.goal_back is not None:                           # 물린 자리도 막힘 → 다음엔 처음부터
            self.goal_back = None
            return self._plan_from(goal_ps, start)
        return None

    def _plan_from(self, goal_ps, start):
        if not self.planner.wait_for_server(timeout_sec=2.0):
            return None
        g = ComputePathToPose.Goal()
        g.goal, g.planner_id = goal_ps, self.planner_id
        g.use_start = start is not None
        if start is not None:
            g.start = start
        ev = threading.Event()
        res = {}

        def done(f):
            res['h'] = f.result()
            ev.set()
        self.planner.send_goal_async(g).add_done_callback(done)
        if not ev.wait(3.0) or not res['h'].accepted:
            return None
        ev.clear()
        res['h'].get_result_async().add_done_callback(lambda f: (res.__setitem__('r', f.result()), ev.set()))
        if not ev.wait(3.0):
            return None
        poses = res['r'].result.path.poses
        if len(poses) < 2:
            return None
        pts = [(p.pose.position.x, p.pose.position.y) for p in poses]
        gp = goal_ps.pose.position
        pts.append((gp.x, gp.y))                                        # 끝 = 정확한 목표 (학습과 같음)
        return Path(pts)

    def plan_frame(self, goal_ps, goal_xy, goal_yaw):
        """goal_frame 기준 경로. map = plan() 그대로. odom = 고정한 odom 목표를 지금 map → odom 으로 map 목표로 바꿔 planner 에 보내고,
        받은 경로를 다시 odom 으로 (끝 = 고정 목표 그대로)"""
        if self.goal_frame == 'map':
            return self.plan(goal_ps)
        try:
            T_mo = self.tf2d('map', 'odom')
        except Exception:
            return None
        gxy, gyaw = self.apply2d(T_mo, goal_xy, goal_yaw)
        ps = PoseStamped()
        ps.header.frame_id, ps.header.stamp = 'map', self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(gxy[0]), float(gxy[1])
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        path = self.plan(ps)
        if path is None:
            return None
        pts = self.apply2d(self.inv2d(T_mo), path.p)                   # map → odom
        pts[-1] = goal_xy
        return Path([tuple(p) for p in pts])

    @staticmethod
    def inv2d(T):
        c, s_ = math.cos(T[2]), math.sin(T[2])
        return np.array([-(c * T[0] + s_ * T[1]), -(-s_ * T[0] + c * T[1]), -T[2]])

    def set_polygons(self, enabled: bool, names=None):
        """collision_monitor 의 <영역>.enabled 를 바꿈 (Nav2 collision_monitor 는 실행 중 바꾸기 지원). 실패하면 경고만"""
        names = self.cm_polygons if names is None else names
        if not names:
            return True
        if not self.cm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('collision_monitor set_parameters 없음 → 영역 못 바꿈')
            return False
        req = SetParameters.Request(parameters=[ParamMsg(name=f'{n}.enabled', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled))
                                                for n in names])
        ev = threading.Event()
        fut = self.cm_client.call_async(req)
        fut.add_done_callback(lambda f: ev.set())
        ok = ev.wait(2.0) and fut.result() is not None and all(r.successful for r in fut.result().results)
        self.get_logger().info(f'collision_monitor {",".join(names)} {"켬" if enabled else "끔"}' + ('' if ok else ' (실패)'))
        return ok

    def straight(self, goal_xy):
        """planner 실패 때 대체: 목표가 straight_max 안이면 지금 자리 → 목표 직선 경로.
        박스가 밀려 고친 목표가 옛 박스 자리 치명 영역 (지도는 아직 옛 박스) 에 들어가면 planner_server 가 경로를 못 냄 → 마지막 짧은 구간은 직선
        (local_dock 학습도 직선 경로). 멀면 None"""
        try:
            pose = self.pose(self.goal_frame)
        except Exception:
            return None
        d = float(np.linalg.norm(np.asarray(goal_xy) - pose[:2]))
        if d > self.straight_max or d < 1e-3:
            return None
        self.get_logger().warning(f'경로 못 받음 → 목표 {d * 100:.0f} cm 직선 경로로 대신', throttle_duration_sec=2.0)
        return Path([tuple(pose[:2]), tuple(goal_xy)])

    # ------------------------------------------------------------ 실행
    def obs(self, path, pose, s, goal_xy, goal_yaw, hist, last_cmd):
        c = self.c
        cs, sn = math.cos(pose[2]), math.sin(pose[2])
        body = lambda p: np.stack([cs * (p[..., 0] - pose[0]) + sn * (p[..., 1] - pose[1]), -sn * (p[..., 0] - pose[0]) + cs * (p[..., 1] - pose[1])], -1)  # noqa: E731
        pk = body(path.point_at(s + c['path_obs_spacing'] * np.arange(c['path_obs_points']))).reshape(-1)
        gb = body(np.asarray(goal_xy))
        dy = wrap(goal_yaw - pose[2])
        rem = path.L - s
        goal = np.array([gb[0] / 2, gb[1] / 2, math.sin(dy), math.cos(dy), min(rem / 3, 2.0)])
        scan = self.lidar()
        if scan is None:
            return None, hist
        lid = np.clip(scan, 0, c['lidar_obs_clip']) / c['lidar_obs_clip']
        rpos = (self.q - self.rmin) / (self.rmax - self.rmin) * 2 - 1
        frame = np.concatenate([np.sin(self.q), np.cos(self.q), rpos, self.w / self.max_wheel, self.odom / self.lim3, last_cmd / self.lim3])
        hist = [frame] * c['history'] if hist is None else [frame] + hist[:-1]
        return np.concatenate([pk, goal, lid, np.concatenate(hist)]).astype(np.float32), hist

    def command(self, o, goal_dist=None):
        """관측 → 몸 속도 명령 (학습 env _command 와 같은 한계: 병진 크기 ≤ vmax, 목표 근처 속도 상한, 벽 근처 회전 상한)"""
        a = np.clip(self.act(o), -1, 1)
        lin = a[:2] * self.c['vmax']
        vlim = self.c['vmax']
        if self.c.get('approach_limit', False) and goal_dist is not None:
            vlim = min(vlim, self.c['approach_v0'] + self.c['approach_k'] * goal_dist)
        if self.near_off and self.near_vmax > 0:      # 출발 자리 영역 끈 동안 (몸 옆 벽이 안 보임)
            vlim = min(vlim, self.near_vmax)
        n = np.linalg.norm(lin)
        if n > vlim:
            lin *= vlim / n
        wz = a[2] * self.c['wmax']
        if self.c.get('rot_limit', False):           # 벽 가까우면 회전 상한 (학습 env _command 와 같은 식, 라이다 최솟값)
            scan = self.lidar()
            dmin = float(np.min(scan)) if scan is not None else 0.0
            t = np.clip((dmin - self.c['rot_clear_lo']) / (self.c['rot_clear_hi'] - self.c['rot_clear_lo']), 0.0, 1.0)
            lim = self.c['rot_wz_lo'] + (self.c['wmax'] - self.c['rot_wz_lo']) * t
            wz = float(np.clip(wz, -lim, lim))
        return np.array([lin[0], lin[1], wz])

    # ------------------------------------------------------------ 그림자 모드
    def on_plan(self, m):
        if len(m.poses) < 2:
            return
        pts = [(q.pose.position.x, q.pose.position.y) for q in m.poses]
        g = m.poses[-1].pose
        goal = (np.array([g.position.x, g.position.y]), yaw_of(g.orientation))       # planner_server 는 끝 자세 = 목표 자세
        with self.lock:
            sh = self.sh
            new_goal = sh['goal'] is None or np.linalg.norm(sh['goal'][0] - goal[0]) > 0.05 or abs(wrap(sh['goal'][1] - goal[1])) > 0.05
            sh['path'], sh['goal'], sh['plan_t'] = Path(pts), goal, self.now()
            try:
                sh['s'], _ = sh['path'].project(self.pose()[:2])
            except Exception:
                sh['s'] = 0.0
            if new_goal:
                sh['hist'], sh['last'] = None, np.zeros(3)

    def shadow_tick(self):
        with self.lock:
            sh = self.sh
            if sh['path'] is None or self.now() - sh['plan_t'] > 3.0:          # bt_navigator 가 1 s 마다 /plan 냄 → 3 s 없으면 목표 없음
                return
            try:
                pose = self.pose()
            except Exception:
                return
            sh['s'], _ = sh['path'].project(pose[:2], sh['s'])
            o, sh['hist'] = self.obs(sh['path'], pose, sh['s'], sh['goal'][0], sh['goal'][1], sh['hist'], sh['last'])
            if o is None:
                return
            cmd = self.command(o, float(np.linalg.norm(sh['goal'][0] - pose[:2])))
            sh['last'] = cmd
            self.send(cmd)
            if self.csv:
                gd = float(np.linalg.norm(sh['goal'][0] - pose[:2]))
                gy = abs(wrap(sh['goal'][1] - pose[2]))
                sm = float(np.min(self.lidar())) if self.lidar() is not None else float('nan')
                self.csv.write(f'{self.now():.3f},{pose[0]:.4f},{pose[1]:.4f},{pose[2]:.4f},' + ','.join(f'{v:.4f}' for v in (*cmd, *sh['mppi']))
                               + f',{gd:.4f},{gy:.4f},{sm:.3f}\n')
                self.csv.flush()

    def now(self):
        """ROS 시계 [s] (--sim_time 이면 /clock = 시뮬 시간 → 시뮬이 실시간보다 느려도 제어 주기 20 Hz 가 시뮬 기준으로 맞음)"""
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, cmd):
        # 명령 사이 틈 기록: swerve_controller cmd_timeout_s 0.3 s 를 넘기면 바퀴가 섬 (실물 섰다 갔다 확인용)
        t = time.monotonic()
        last = getattr(self, '_last_send', None)
        if last is not None and 0.25 < t - last < 5.0 and np.linalg.norm(np.asarray(cmd, dtype=float)) > 1e-6:
            self.get_logger().warning(f'명령 틈 {t - last:.2f} s (0.3 s 넘으면 swerve_controller 가 멈춤)', throttle_duration_sec=2.0)
        self._last_send = t
        m = TwistStamped()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), 'base_footprint'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = (float(v) for v in cmd)
        self.pub.publish(m)

    def log_rotation(self):
        """목표 하나 끝날 때 회전량 기록: odom 적분 ∫|ω|dt · 필요한 하한 |출발 yaw → 목표 yaw| · 남는 회전 · 명령 반전 횟수"""
        r = self.rot
        if r is None:
            return
        try:
            net = abs(wrap(float(self.pose(r['fr'])[2]) - r['yaw0']))
        except Exception:
            net = float('nan')
        self.get_logger().info(f'회전: odom ∫|ω| {math.degrees(r["odom"]):.0f}° · 필요 {math.degrees(r["need"]):.0f}° · 실제 순 {math.degrees(net):.0f}° '
                               f'· 남는 회전 {math.degrees(max(r["odom"] - r["need"], 0.0)):.0f}° · 명령 반전 {r["rev"]} 번')
        self.rot = None

    def execute(self, gh):
        """목표 하나 실행. --disable_polygons 면 이 목표 동안만 collision_monitor 영역을 끔 (끝나면 반드시 다시 켬)
        주행 노드 (--near_disable_polygons): 출발 전에 그 영역을 켬으로 확인 → 못 켜면 주행 안 하고 실패
          (정밀 접근 노드가 영역을 끈 뒤 다시 못 켰으면 비상 정지 없이 달리게 되므로)"""
        if not self.cm_polygons and self.near_polygons and not self.set_polygons(True, self.near_polygons):
            self.get_logger().error(f'collision_monitor {",".join(self.near_polygons)} 켬을 확인 못 함 → 주행 안 함 (실패)')
            gh.abort()
            return NavigateToPose.Result()
        if self.cm_polygons and not self.set_polygons(False):
            self.get_logger().warning('영역을 못 꺼서 그대로 진행 (정차 직전 비상 정지가 걸릴 수 있음)')
        self.rot = None
        try:
            return self._execute(gh)
        finally:
            self.log_rotation()
            if self.cm_polygons and not any(self.set_polygons(True) for _ in range(3)):
                self.get_logger().error(f'collision_monitor {",".join(self.cm_polygons)} 다시 켜기 3 번 실패 '
                                        '→ 영역이 꺼진 채 (다음 주행은 rl_nav 가 출발 전에 막음)')
            if self.near_off:
                self.set_polygons(True, self.near_polygons)
                self.near_off = False

    def _execute(self, gh):
        with self.lock:                                                  # 목표 하나씩
            goal_ps = gh.request.pose
            goal_xy = np.array([goal_ps.pose.position.x, goal_ps.pose.position.y])
            goal_yaw = yaw_of(goal_ps.pose.orientation)
            self.escaped = False
            self.goal_back = None
            fr = self.goal_frame
            if fr == 'odom':                                             # 목표를 지금 map → odom 으로 한 번 바꿔 고정
                src = goal_ps.header.frame_id or 'map'
                try:
                    goal_xy, goal_yaw = self.apply2d(self.tf2d('odom', src), goal_xy, goal_yaw)
                except Exception as e:
                    self.get_logger().warning(f'{src} → odom 변환 없음 ({e})')
                    gh.abort()
                    return NavigateToPose.Result()
                self.get_logger().info(f'목표 odom 고정 ({goal_xy[0]:.3f}, {goal_xy[1]:.3f}, {math.degrees(goal_yaw):.1f}°)')
            tracker, scan_seen, n_block, n_off = None, -1.0, 0, 0
            if self.box_track:                                           # 박스 맞춤: 경로 = 직선 (학습과 같음), 목표는 스캔마다 고침
                tracker = BoxTracker(goal_xy, goal_yaw, box_dist=self.box_dist)
                path = Path([tuple(self.pose(fr)[:2]), tuple(goal_xy)])
                start_dist = float(np.linalg.norm(goal_xy - self.pose(fr)[:2]))
            else:
                path = self.plan_frame(goal_ps, goal_xy, goal_yaw)
            if path is None:
                path = self.straight(goal_xy)
            if path is None:
                self.get_logger().warning('경로 못 받음')
                gh.abort()
                return NavigateToPose.Result()
            s, hist, last_cmd, hold = 0.0, None, np.zeros(3), 0.0
            plan_job = None                                              # 따로 도는 경로 계획 (1 s 마다)
            near_checked = not self.near_polygons
            near_xy = None
            settled = False
            near_t = 0.0
            s, _ = path.project(self.pose(fr)[:2])
            yaw0 = float(self.pose(fr)[2])
            # 회전 기록 (목표 하나): odom 각속도 적분 · wz 명령 부호 반전 · 필요한 회전 하한 (출발 yaw ↔ 목표 yaw)
            self.rot = {'yaw0': yaw0, 'need': abs(wrap(goal_yaw - yaw0)), 'odom': 0.0, 'rev': 0, 'last_wz': 0.0, 'fr': fr}
            t0 = t_plan = self.now()
            T = 2 * path.L / self.c['vmax'] + 30.0
            dt = self.c['ctrl_dt'] * self.c['decimation']
            nxt = self.now()
            while rclpy.ok():
                if gh.is_cancel_requested:
                    self.send(np.zeros(3))
                    gh.canceled()
                    return NavigateToPose.Result()
                if tracker is not None and self.scan_t > scan_seen:      # 새 스캔 → 박스 앞면 맞춤 → 목표 고침
                    scan_seen = self.scan_t
                    pts, _ = self.scan_points(fr)
                    if pts is not None and tracker.update(pts, self.pose(fr)[:2]):
                        g_new, y_new = tracker.goal()
                        if np.linalg.norm(g_new - goal_xy) > 0.003 or abs(wrap(y_new - goal_yaw)) > math.radians(0.2):
                            goal_xy, goal_yaw = g_new, y_new
                            path = Path([tuple(self.pose(fr)[:2]), tuple(goal_xy)])
                            s = 0.0
                        dc = tracker.c - tracker.c0
                        self.get_logger().info(f'박스 맞춤: 지도 대비 ({dc[0] * 100:+.1f}, {dc[1] * 100:+.1f}) cm · {math.degrees(tracker.th):+.1f}°'
                                               f' (측정 {tracker.n_ok} 번)', throttle_duration_sec=1.0)
                    elif tracker.last_why != 'freeze':
                        # 버린 이유 기록 (실물: dock3 측정 0 번인데 이유가 안 남아 원인을 못 찾음)
                        self.get_logger().info(f'박스 맞춤 버림: {tracker.last_why}', throttle_duration_sec=1.0)
                        # 박스는 보이는데 지도 자리 · 방향과 다름 (기울기 · 지도에서 N cm) 이 측정 한 번 없이 계속 → 지도 목표로 그냥 가지 않음
                        #   (실물: 19:06 dock2 가 B1 박스를 쳐서 박스가 돌아간 것으로 봄 → 19:25 측정 +13°, 20:04 측정 0 번 · 지도 목표로 가다 충돌)
                        # 실물: 박스가 아예 안 보임 (점 0 · B6 · L2) · 벽만 보임 (면 길이 · 면 밖 이어짐 · L1) 도 측정 0 번이면 같게 셈
                        #   (기울기 · 지도 거리만 세면 박스 없는 지도 정차점에 서서 '도착')
                        n_off += 1
                        if self.box_reject_n > 0 and tracker.n_ok == 0 and n_off >= self.box_reject_n:
                            self.send(np.zeros(3))
                            gh.abort()
                            self.get_logger().warning(f'박스가 지도 자리에 없음 ({tracker.last_why}, 연속 {n_off} 스캔, 측정 0 번) → 멈추고 실패. '
                                                      '박스 자리 · 방향 · 지도 정합 · dice_map 정차점 확인')
                            return NavigateToPose.Result()
                    elif tracker.n_ok == 0 and self.box_reject_n > 0 and start_dist > tracker.freeze + 0.05:
                        # 한 번도 못 잰 채 목표 freeze_dist 안까지 옴 (그 뒤는 몸 상자 · range_min 안이라 못 잼) → 박스 없는 자리에 서지 않음.
                        #   출발부터 freeze 근처면 (Nav2 가 정차점까지 다 온 뒤 넘김) 잴 기회가 없었으니 그대로 감
                        self.send(np.zeros(3))
                        gh.abort()
                        self.get_logger().warning(f'박스를 한 번도 못 잼 (출발 {start_dist * 100:.0f} cm) → 멈추고 실패. dice_map 정차점 확인')
                        return NavigateToPose.Result()
                    if self.dock_block_n > 0 and pts is not None:      # 직선 경로에 벽 → 연속 dock_block_n 스캔이면 멈추고 실패
                        nb, at = self.straight_blocked(self.pose(fr), goal_xy, goal_yaw, tracker, pts)
                        n_block = n_block + 1 if nb else 0
                        if n_block >= self.dock_block_n:
                            self.send(np.zeros(3))
                            gh.abort()
                            self.get_logger().warning(f'정밀 접근 직선 경로에 박스 아닌 점 {nb} 개 ({at * 100:.0f} cm 앞) → 멈추고 실패 '
                                                      f'(목표 {float(np.linalg.norm(goal_xy - self.pose(fr)[:2])) * 100:.0f} cm)')
                            return NavigateToPose.Result()
                # 1 s 마다 경로 다시 (bt_navigator 와 같은 주기). 계획은 따로 스레드에서 → 제어 주기는 계속 명령을 냄
                #   (실물: 같은 스레드에서 계획을 기다리는 동안 명령이 끊겨 swerve_controller cmd_vel 0.3 s 시간 초과로 섬 → 주행 중 섰다 갔다 의심)
                if tracker is None and plan_job is None and self.now() - t_plan > 1.0:
                    t_plan = self.now()
                    plan_job = {'done': False, 'path': None, 'L': path.L, 't0': time.monotonic()}

                    def _replan(job=plan_job, gp=goal_ps, gxy=goal_xy.copy(), gy=goal_yaw):
                        try:
                            np_ = self.plan_frame(gp, gxy, gy)
                            if np_ is None and job['L'] < self.straight_max:      # 짧은 구간 직선 대체 중이면 계속 직선 (현재 자리에서 다시)
                                np_ = self.straight(gxy)
                            job['path'] = np_
                        except Exception as e:
                            self.get_logger().warning(f'경로 다시 계획 실패: {e}', throttle_duration_sec=2.0)
                        finally:
                            job['dt'] = time.monotonic() - job['t0']
                            job['done'] = True
                    threading.Thread(target=_replan, daemon=True).start()
                if plan_job is not None and plan_job['done']:
                    if plan_job['dt'] > 0.3:
                        self.get_logger().info(f'경로 계획 {plan_job["dt"]:.2f} s 걸림 (제어는 계속)', throttle_duration_sec=5.0)
                    if plan_job['path'] is not None:
                        path = plan_job['path']
                        s, _ = path.project(self.pose(fr)[:2])
                    plan_job = None
                pose = self.pose(fr)
                s, _ = path.project(pose[:2], s)
                o, hist = self.obs(path, pose, s, goal_xy, goal_yaw, hist, last_cmd)
                if o is None:                                            # 스캔 끊김 → 멈추고 기다림 (시간 초과는 그대로 셈)
                    self.send(np.zeros(3))
                    last_cmd = np.zeros(3)
                    self.get_logger().warning(f'스캔 없음 ({self.scan_stale_s} s 넘음) → 정지', throttle_duration_sec=2.0)
                    nxt = self.now() + dt
                    while self.now() < nxt and rclpy.ok():
                        time.sleep(0.002)
                    if self.now() - t0 > T:
                        gh.abort()
                        return NavigateToPose.Result()
                    continue
                if not near_checked:                                      # 붙어 선 자리에서 출발: 영역 끄기 → near_leave_m 멀어지면 켜기
                    if near_xy is None:
                        near_xy = pose[:2].copy()
                    moved = float(np.linalg.norm(pose[:2] - near_xy))
                    cleared = self.near_off and moved > self.near_clear_m and self.stop_zone_clear()
                    if moved > self.near_leave or cleared:               # 출발 자리 벗어남 · 정지 구역 비었음 → 끝 (껐으면 켬)
                        near_checked = True
                        if self.near_off:
                            self.set_polygons(True, self.near_polygons)
                            self.near_off = False
                            self.get_logger().info(f'출발 자리 {moved * 100:.0f} cm 벗어남' + (' · 정지 구역 빔' if cleared else '') + ' → 영역 켬')
                    elif not self.near_off:
                        # 박스 면 (몸 중심 0.164–0.17 m) 은 scan_merger 몸 상자 경계라 스캔마다 보였다 말았다 함 → 한 번만 보면 놓침
                        #   (Isaac START: 놓친 뒤 PolygonStop 이 면 점을 잡아 150 s 멈춤) → 벗어날 때까지 매 주기 보고, 빠져나오기 경로를 썼으면 바로 끔
                        #   벽이 가깝기만 한 자리 (출발 구역) 는 PolygonStop 이 안 막으니 안 끔 (실물)
                        blocking = self.stop_blocking()
                        if (blocking or self.escaped) and self.set_polygons(False, self.near_polygons):
                            self.near_off = True
                            why = '빠져나오기 경로' if self.escaped else 'PolygonStop 막힘'
                            self.get_logger().info(f'출발 자리 {why} → {self.near_leave * 100:.0f} cm 벗어나거나 정지 구역 빌 때까지 영역 끔 '
                                                   f'(병진 ≤ {self.near_vmax:.2f} m/s)')
                dist = float(np.linalg.norm(goal_xy - pose[:2]))
                dy = abs(wrap(goal_yaw - pose[2]))
                # 도착 범위 안이면 0 명령으로 붙잡음 (바퀴 안 흔듦): 정책은 목표 근처에서 ±0.01 m/s · 회전 명령을 번갈아 내고,
                #   스워브는 명령 방향마다 바퀴를 다시 돌림 (회전 명령이면 바퀴가 회전 자세로) → 정차 뒤 바퀴만 흔들림 (Isaac B6 15 s).
                #   0 명령이면 컨트롤러가 바퀴 각을 그대로 둠. 범위 2 배 밖으로 밀리면 다시 정책
                if self.settle:
                    if dist < self.tol_p and dy < self.tol_y:
                        settled = True
                    elif dist > 2 * self.tol_p or dy > 2 * self.tol_y:
                        settled = False
                last_cmd = np.zeros(3) if settled else self.command(o, dist)
                if (self.min_lin > 0 or self.min_ang > 0) and np.linalg.norm(last_cmd[:2]) < self.min_lin and abs(last_cmd[2]) < self.min_ang:
                    last_cmd = np.zeros(3)                               # 아주 작은 명령 = 0 (바퀴 방향 안 바꿈)
                self.send(last_cmd)
                self.rot['odom'] += abs(float(self.odom[2])) * dt
                wz = float(last_cmd[2])
                if abs(wz) >= 0.05:
                    if self.rot['last_wz'] * wz < 0:
                        self.rot['rev'] += 1
                    self.rot['last_wz'] = wz
                still = np.linalg.norm(self.odom[:2]) < self.c['goal_lin_vel'] and abs(self.odom[2]) < self.c['goal_ang_vel']
                hold = hold + dt if (dist < self.tol_p and dy < self.tol_y and still) else 0.0
                # 거의 다 옴: 범위 2 배 안에 near_ok_s 머물면 성공으로 끝냄 (Isaac: dock 이 1.5 cm 경계 밖 1.53 cm 에서
                #   30 s 미세 조정하다 시간 초과 → planning 은 이동 실패로 처음부터 다시 함)
                near_t = near_t + dt if (dist < 2 * self.tol_p and dy < 2 * self.tol_y) else 0.0
                if self.near_ok_s > 0 and near_t >= self.near_ok_s and hold < self.c['goal_hold_s']:
                    self.send(np.zeros(3))
                    gh.succeed()
                    self.get_logger().warning(f'범위 2 배 안 {self.near_ok_s:.0f} s → 성공으로 끝냄, 오차 {dist * 100:.1f} cm · {math.degrees(dy):.1f}°')
                    return NavigateToPose.Result()
                fb = NavigateToPose.Feedback()
                fb.distance_remaining = float(path.L - s)
                gh.publish_feedback(fb)
                if hold >= self.c['goal_hold_s']:
                    self.send(np.zeros(3))
                    gh.succeed()
                    extra = '' if tracker is None else (f', 박스 지도 대비 {np.linalg.norm(tracker.c - tracker.c0) * 100:.1f} cm · {math.degrees(tracker.th):+.1f}° (측정 {tracker.n_ok} 번'
                                                        + (f', 마지막 이유 {tracker.last_why})' if tracker.n_ok == 0 else ')'))
                    self.get_logger().info(f'도착 {self.now() - t0:.1f} s, 오차 {dist * 100:.1f} cm · {math.degrees(dy):.1f}°' + extra)
                    return NavigateToPose.Result()
                if self.now() - t0 > T:
                    self.send(np.zeros(3))
                    gh.abort()
                    self.get_logger().warning('시간 초과')
                    return NavigateToPose.Result()
                if (self.stop_abort_s > 0 and not self.cm_polygons and not self.near_off and self.stop_since is not None
                        and self.now() - max(self.stop_since, t0) > self.stop_abort_s):
                    # 영역을 끈 목표 (정밀 접근 · 출발 자리) 는 해당 없음. 사람이 막고 서 있어도 끝남 → planning 이 이동 단계를 다시 함
                    self.send(np.zeros(3))
                    gh.abort()
                    self.get_logger().warning(f'PolygonStop 이 {self.stop_abort_s:.0f} s 넘게 막음 → 멈추고 실패 '
                                              f'(목표 {dist * 100:.0f} cm{f", 목표 뒤 {self.goal_back * 100:.0f} cm 물려 계획 중" if self.goal_back else ""})')
                    return NavigateToPose.Result()
                nxt += dt
                while self.now() < nxt and rclpy.ok():                    # 시뮬 시간으로 다음 주기까지
                    time.sleep(0.002)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--node_name', default='rl_local_planner', help='노드 이름 (주행 · 접근 노드 둘을 같이 띄울 때 다르게)')
    p.add_argument('--policy', required=True)
    p.add_argument('--cfg', required=True)
    p.add_argument('--tol_pos', type=float, default=0.02)
    p.add_argument('--tol_yaw_deg', type=float, default=2.0)
    p.add_argument('--sim_time', action='store_true')
    p.add_argument('--scan_topic', default='/scan', help='scan_merger 출력 (앞 · 뒤 합침 · 몸 상자 거름)')
    p.add_argument('--scan_stale_s', type=float, default=0.5)
    p.add_argument('--disable_polygons', default='', help='목표 동안 끌 collision_monitor 영역 (쉼표). 정밀 접근 노드 = PolygonStop,FootprintApproach')
    p.add_argument('--cm_node', default='/collision_monitor')
    p.add_argument('--near_disable_polygons', default='', help='출발 때 PolygonStop 이 막거나 빠져나오기 경로면 벗어날 때까지 끌 영역 (주행 노드 = PolygonStop,FootprintApproach: 정밀 접근 정차에서 떠날 때)')
    p.add_argument('--near_leave_m', type=float, default=0.25, help='출발 자리에서 이만큼 멀어지면 영역 다시 켬 [m]')
    p.add_argument('--near_clear_m', type=float, default=0.05, help='이만큼 움직였고 정지 구역 + --near_clear_pad 안에 점이 없으면 영역 다시 켬 [m]')
    p.add_argument('--near_clear_pad', type=float, default=0.03, help='[m]')
    p.add_argument('--near_vmax', type=float, default=0.10, help='출발 자리 영역 끈 동안 병진 상한 [m/s] (0 = 안 막음)')
    p.add_argument('--stop_half', type=float, default=0.18, help='collision_monitor PolygonStop 반 변 [m] (config/nav2.yaml)')
    p.add_argument('--cm_state_topic', default='/collision_monitor_state')
    p.add_argument('--stop_abort_s', type=float, default=8.0, help='collision_monitor 가 STOP 을 이만큼 이어 내면 실패로 끝냄 [s] (0 = 끔, 영역 끈 목표는 해당 없음)')
    p.add_argument('--dock_body_half', default='0.145,0.153', help='정밀 접근 직선 경로 검사 몸 반 크기 x,y [m] (차체 앞 끝 · 바퀴)')
    p.add_argument('--dock_block_n', type=int, default=3, help='정밀 접근 직선 경로가 연속 이만큼 스캔 막히면 멈추고 실패 (0 = 검사 안 함, --box_track 때만)')
    p.add_argument('--box_reject_n', type=int, default=10, help='박스 맞춤이 측정 0 번인 채 기울기 · 지도 거리 한계로 연속 이만큼 스캔 버려지면 멈추고 실패 (0 = 안 함, 약 1 s)')
    p.add_argument('--goal_back_m', type=float, default=0.45, help='목표가 지도 장애물이라 경로 실패면 목표를 이 거리까지 물려 계획 + 직선 [m] (0 = 안 함)')
    p.add_argument('--straight_max', type=float, default=0.6, help='planner 실패 때 이 거리 안이면 직선 경로로 대신 [m] (0 = 안 함)')
    p.add_argument('--goal_frame', default='map', choices=('map', 'odom'),
                   help='odom = 목표를 받을 때 odom 으로 고정 (접근 중 slam_toolbox map 보정에 안 흔들림). 정밀 접근 노드에 씀')
    p.add_argument('--near_ok_s', type=float, default=3.0, help='도착 범위 2 배 안에 이만큼 머물면 성공으로 끝냄 [s] (0 = 끔)')
    p.add_argument('--no_settle', action='store_true', help='도착 범위 (--tol_pos · --tol_yaw_deg) 안에서 0 명령으로 붙잡기 끄기')
    p.add_argument('--min_cmd_lin', type=float, default=0.0, help='병진 명령이 이보다 작고 회전도 --min_cmd_ang 보다 작으면 0 [m/s]. 기본 끔: '
                   'Isaac B1 = yaw 2.1° (범위 2° 밖) 에서 작은 회전 명령이 다 0 이 돼 28 s 멈춤')
    p.add_argument('--min_cmd_ang', type=float, default=0.0, help='[rad/s]')
    p.add_argument('--escape_m', type=float, default=0.20, help='지금 자리에서 경로 실패 (박스 면 1 cm 정차 = 치명 영역 안) 면 몸 뒤 이 거리까지 0.05 m 씩 물러난 점에서 다시 계획 (0 = 안 함)')
    p.add_argument('--box_track', action='store_true', help='정밀 접근: 스캔으로 박스 앞면을 재서 목표를 박스 기준으로 고침 (box_tracker.py)')
    p.add_argument('--box_dist', type=float, default=0.2575, help='목표 = 박스 중심에서 이 거리 (dice_map 정차 거리와 같게)')
    p.add_argument('--action_name', default='navigate_to_pose', help='받을 액션 이름 (정밀 접근 정책 = dock_to_pose 로 해서 MPPI · nf 와 안 겹치게)')
    p.add_argument('--shadow', action='store_true', help='그림자 모드: /plan 받아 정책 명령을 --shadow_topic 에만 냄 (주행 안 함)')
    p.add_argument('--plan_topic', default='/plan')
    p.add_argument('--shadow_topic', default='cmd_vel_rl_shadow')
    p.add_argument('--log_csv', default='', help='그림자 모드 기록 csv')
    p.add_argument('--swerve_yaml', default='', help='cfg.json 의 swerve_yaml 경로가 이 PC 에 없을 때 (Jetson)')
    p.add_argument('--planner_id', default='', help='planner_server 플러그인 (기본 "" = 첫 번째 = GridBased NavFn). Smac2D 는 params 에 local_planner_nav2/smac2d_planners.yaml 필요')
    a, ros_args = p.parse_known_args()                  # launch 의 Node 가 붙이는 --ros-args ... 는 rclpy 로
    rclpy.init(args=[sys.argv[0]] + ros_args)
    n = LocalPlanner(a)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():                     # Ctrl+C 로 context 가 먼저 닫히면 wait set 오류가 남 (실물) → 종료 중이면 무시
            raise
    finally:
        if getattr(n, 'csv', None):
            n.csv.close()


if __name__ == '__main__':
    main()
