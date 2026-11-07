#!/usr/bin/env python3
"""규칙 정밀 접근 노드 — dock_to_pose (NavigateToPose) 를 받아 박스 · 장소 정차점에 붙음. 팀 RL dock3 (local_planner_node.py) 자리에 씀.

  nav2.launch.py dock:=simple  (기본 dock:=rl = 팀 RL)
받는 것: dock_to_pose 목표 (보통 map, 박스를 봄), /scan (scan_merger, 박스 앞면 찾기 · 추적), /lidar_front/scan · /lidar_rear/scan (몸 상자 안 지운 원래 점, 안전),
         TF odom → base_footprint, map → odom (목표를 한 번 odom 으로)
보내는 것: cmd_vel_nav (TwistStamped, 20 Hz) → collision_monitor → cmd_vel_hold → /cmd_vel. 끝나면 0 몇 번
동작: simple_dock_core.DockCore (제자리로 돌아 박스를 보고 → 곧게 다가감, 최고 --v_max). 이 목표 동안 collision_monitor 영역을 끄고
      (박스 면 1 cm 앞이라 PolygonStop 이 걸림) 대신 라이다 원래 점으로 몸 둘레 --margin 안 물체를 직접 봄 → 있으면 멈추고 실패 (abort)
끝나면 (성공 · 실패 · 취소 · 시간 초과 모두) 영역을 다시 켬
"""
import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from simple_dock_core import DockCore


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def scan_pts(m, max_r=1.5):
    r = np.asarray(m.ranges, float)
    a = m.angle_min + m.angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > m.range_min) & (r < min(m.range_max, max_r))
    return np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]


def apply(T, P):
    """T (x, y, yaw) ∘ P (N,2)"""
    c, s = math.cos(T[2]), math.sin(T[2])
    return np.c_[c * P[:, 0] - s * P[:, 1] + T[0], s * P[:, 0] + c * P[:, 1] + T[1]]


class SimpleDock(Node):
    def __init__(self, a):
        super().__init__(a.node_name, parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=a.sim_time)])
        self.a = a
        cb = ReentrantCallbackGroup()
        self.tf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.tf, self)
        self.scan = None
        self.raw = {}
        self.lock = threading.Lock()
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data, callback_group=cb)
        for t in a.raw_topics.split(','):
            if t:
                self.create_subscription(LaserScan, t, lambda m, t=t: self.on_raw(t, m), qos_profile_sensor_data, callback_group=cb)
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_nav', 10)
        self.polys = [p for p in a.disable_polygons.split(',') if p]
        self.cm = self.create_client(SetParameters, f'{a.cm_node}/set_parameters', callback_group=cb) if self.polys else None
        self.busy = False
        self.server = ActionServer(self, NavigateToPose, a.action_name, self.execute, callback_group=cb,
                                   goal_callback=self.on_goal, cancel_callback=lambda g: CancelResponse.ACCEPT)
        self.get_logger().info(f'규칙 정밀 접근: {a.action_name} · 최고 {a.v_max} m/s · 몸 둘레 {a.margin * 100:.0f} cm 안 물체면 멈춤 · '
                               f'박스 맞춤 {f"켬 (찾기 ±{a.search * 100:.0f} cm)" if a.box_track else "끔"} · 끝 {a.tol_pos * 100:.1f} cm {a.tol_yaw_deg}° (안 줄면 {a.tol_ok * 100:.1f} cm)')

    # ---------------------------------------------------------------- 입력
    def on_scan(self, m):
        with self.lock:
            self.scan = m

    def on_raw(self, t, m):
        with self.lock:
            self.raw[t] = m

    def tf2d(self, target, source, stamp=None):
        t = self.tf.lookup_transform(target, source, stamp or rclpy.time.Time())
        return np.array([t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)])

    def raw_body(self):
        """라이다 원래 점 (몸 좌표). 0.3 s 넘게 오래된 것은 안 씀"""
        out = []
        now = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            raws = list(self.raw.values())
        for m in raws:
            age = now - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            if age > 0.3:
                continue
            try:
                T = self.tf2d('base_footprint', m.header.frame_id)
            except Exception:
                continue
            P = scan_pts(m, 1.0)
            if len(P):
                out.append(apply(T, P))
        return np.vstack(out) if out else np.zeros((0, 2)), len(out)

    def scan_odom(self):
        with self.lock:
            m = self.scan
        if m is None:
            return np.zeros((0, 2))
        try:
            T = self.tf2d('odom', m.header.frame_id or 'base_footprint')
        except Exception:
            return np.zeros((0, 2))
        P = scan_pts(m, 1.5)
        return apply(T, P) if len(P) else P

    # ---------------------------------------------------------------- 출력
    def send(self, v):
        m = TwistStamped()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), 'base_footprint'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = (float(x) for x in v)
        self.pub.publish(m)

    def stop(self):
        for _ in range(5):
            self.send((0.0, 0.0, 0.0))
            time.sleep(0.02)

    def set_polygons(self, enabled):
        if not self.polys:
            return True
        if not self.cm.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('collision_monitor set_parameters 없음 → 영역 못 바꿈')
            return False
        req = SetParameters.Request(parameters=[ParamMsg(name=f'{n}.enabled', value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=enabled)) for n in self.polys])
        ev = threading.Event()
        fut = self.cm.call_async(req)
        fut.add_done_callback(lambda f: ev.set())
        ok = ev.wait(2.0) and fut.result() is not None and all(r.successful for r in fut.result().results)
        self.get_logger().info(f'collision_monitor {",".join(self.polys)} {"켬" if enabled else "끔"}' + ('' if ok else ' (실패)'))
        return ok

    # ---------------------------------------------------------------- 실행
    def on_goal(self, g):
        if self.busy:
            self.get_logger().warning('다른 목표 실행 중 → 거절')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, gh):
        self.busy = True
        res = NavigateToPose.Result()
        try:
            ps = gh.request.pose
            fr = ps.header.frame_id or 'map'
            try:
                T = self.tf2d('odom', fr)
            except Exception as e:
                self.get_logger().error(f'TF {fr} → odom 없음: {e}')
                gh.abort()
                return res
            g = apply(T, np.array([[ps.pose.position.x, ps.pose.position.y]]))[0]
            gyaw = yaw_of(ps.pose.orientation) + T[2]
            core = DockCore(g, gyaw, box_track=self.a.box_track, box_dist=self.a.box_dist, v_max=self.a.v_max, w_max=self.a.w_max, margin=self.a.margin,
                            tol_pos=self.a.tol_pos, tol_yaw_deg=self.a.tol_yaw_deg, tol_ok=self.a.tol_ok, search=self.a.search)
            self.get_logger().info(f'정밀 접근 목표 odom ({g[0]:+.3f}, {g[1]:+.3f}, {math.degrees(gyaw):+.1f}°)')
            if not self.set_polygons(False):
                self.get_logger().warning('영역을 못 꺼서 그대로 진행 (박스 앞에서 PolygonStop 이 세울 수 있음)')
            t0 = time.time()
            last_log = 0.0
            seen = None
            period = 1.0 / self.a.rate
            while rclpy.ok():
                if gh.is_cancel_requested:
                    self.stop()
                    gh.canceled()
                    self.get_logger().info('취소됨')
                    return res
                if time.time() - t0 > self.a.timeout:
                    self.stop()
                    self.get_logger().warning(f'{self.a.timeout:.0f} s 넘음 → 실패')
                    gh.abort()
                    return res
                try:
                    p = self.tf2d('odom', 'base_footprint')
                except Exception:
                    self.send((0.0, 0.0, 0.0))
                    time.sleep(period)
                    continue
                raw, n_raw = self.raw_body()
                if n_raw == 0 and self.a.require_raw:
                    self.stop()
                    self.get_logger().error('라이다 원래 점 (안전 검사용) 이 안 옴 → 실패')
                    gh.abort()
                    return res
                with self.lock:
                    m = self.scan
                if m is not None and m is not seen:      # 새 스캔마다 한 번 (주기 20 Hz > 스캔 10 Hz → 같은 스캔 두 번 넣지 않음)
                    seen = m
                    core.update_box(self.scan_odom(), p[:2])
                cmd, st = core.step(p, raw, time.time() - t0)
                if st == 'fail':
                    self.stop()
                    self.get_logger().error(f'멈춤: {core.why}')
                    gh.abort()
                    return res
                if st == 'done':
                    self.stop()
                    self.get_logger().info(core.why + (f' · 박스 맞춤 {core.tr.n_ok} 번 ({core.tr.last_why})' if core.tr else ''))
                    gh.succeed()
                    return res
                self.send(cmd)
                fb = NavigateToPose.Feedback()
                gxy, _ = core.target()
                fb.distance_remaining = float(np.hypot(*(gxy - p[:2])))
                gh.publish_feedback(fb)
                if time.time() - last_log > 1.0:
                    last_log = time.time()
                    self.get_logger().info(f'{core.mode} · 남은 {fb.distance_remaining * 100:.1f} cm · 명령 ({cmd[0]:+.3f}, {cmd[1]:+.3f}, {cmd[2]:+.2f})'
                                           + (f' · 박스 {core.tr.last_why}' if core.tr else ''))
                time.sleep(period)
            return res
        finally:
            self.set_polygons(True)
            self.busy = False


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--node_name', default='simple_dock')
    p.add_argument('--action_name', default='dock_to_pose')
    p.add_argument('--v_max', type=float, default=0.15, help='최고 속도 [m/s] (11-07 실물: 0.05 는 너무 느림)')
    p.add_argument('--w_max', type=float, default=0.3, help='제자리 회전 최고 [rad/s]')
    p.add_argument('--margin', type=float, default=0.02, help='몸 둘레 이만큼 + 다가가는 속도 × 0.25 s 안에 박스가 아닌 물체면 그쪽으로 안 감 [m]')
    p.add_argument('--tol_pos', type=float, default=0.005, help='여기까지 오면 끝 [m]')
    p.add_argument('--tol_ok', type=float, default=0.015, help='이 안에서 1.5 s 동안 더 안 줄면 끝 [m] (launch dock_tol)')
    p.add_argument('--tol_yaw_deg', type=float, default=2.0)
    p.add_argument('--timeout', type=float, default=40.0)
    p.add_argument('--rate', type=float, default=20.0)
    p.add_argument('--box_dist', type=float, default=0.2575, help='목표 = 박스 중심에서 이 거리 (dice_map 정차 거리)')
    p.add_argument('--sim_time', action='store_true')
    p.add_argument('--no_box_track', dest='box_track', action='store_false', help='라이다 박스 맞춤 끔 (지도 정차점 그대로)')
    p.add_argument('--search', type=float, default=0.25, help='처음 박스 앞면을 찾는 범위 = 지도 박스 자리 ± [m] (사람이 놓은 박스 · 지도 각도 오차)')
    p.add_argument('--raw_topics', default='/lidar_front/scan,/lidar_rear/scan')
    p.add_argument('--no_require_raw', dest='require_raw', action='store_false', help='라이다 원래 점이 없어도 진행 (시뮬)')
    p.add_argument('--disable_polygons', default='PolygonStop,FootprintApproach')
    p.add_argument('--cm_node', default='/collision_monitor')
    a, ros_args = p.parse_known_args()                  # launch 의 Node 가 붙이는 --ros-args ... 는 rclpy 로
    rclpy.init(args=[sys.argv[0]] + ros_args)
    n = SimpleDock(a)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            n.stop()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
