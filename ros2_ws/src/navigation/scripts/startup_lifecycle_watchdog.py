#!/usr/bin/env python3
"""AMCL→지도·TF→Nav2 시작과 lifecycle만 최대3회 복구함.
받음: /scan, /map, TF, lifecycle 상태/응답. 보냄: lifecycle 서비스, 상태 JSON.
/mission/hold·cmd_vel·mission/start·하드웨어 재시작은 다루지 않음.
"""
import json
import math
import time

import rclpy
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from lifecycle_recovery_core import LifecycleRecovery, Observation, Snapshot


class StartupLifecycleWatchdog(Node):
    def __init__(self):
        super().__init__('startup_lifecycle_watchdog')
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.manage_localization = bool(param('manage_localization', True))
        self.scan_topic = param('scan_topic', '/scan')
        self.map_topic = param('map_topic', '/map')
        self.base_frame = param('base_frame', 'base_footprint')
        self.odom_frame = param('odom_frame', 'odom')
        self.map_frame = param('map_frame', 'map')
        self.data_timeout = float(param('data_timeout_s', 3.0))
        self.poll_s = float(param('poll_s', 0.5))
        self.probe_timeout = float(param('probe_timeout_s', 5.0))
        options = dict(max_recoveries=int(param('max_recoveries', 3)),
                       command_timeout=float(param('manager_timeout_s', 90.0)),
                       node_timeout=float(param('node_timeout_s', 20.0)),
                       drain_timeout=float(param('drain_timeout_s', 120.0)),
                       verify_timeout=float(param('verify_timeout_s', 45.0)),
                       stable_s=float(param('bond_stable_s', 16.0)),
                       native_reconnect_s=float(param('native_reconnect_wait_s', 11.0)),
                       cleanup_timeout=float(param('cleanup_timeout_s', 90.0)))
        loc_nodes = list(param('localization_nodes', ['map_server', 'amcl']))
        nav_nodes = list(param('navigation_nodes', ['planner_server', 'collision_monitor']))
        loc_manager = param('localization_manager', '/lifecycle_manager_localization')
        nav_manager = param('navigation_manager', '/lifecycle_manager_navigation')
        self.groups = {}
        if self.manage_localization:
            self.groups['localization'] = LifecycleRecovery(loc_manager, loc_nodes, **options)
        self.groups['navigation'] = LifecycleRecovery(nav_manager, nav_nodes, **options)
        self.service_clients, self.snapshots = {}, {}
        for group, core in self.groups.items():
            self.service_clients[(group, 'manager')] = self.create_client(ManageLifecycleNodes, core.name + '/manage_nodes')
            self.service_clients[(group, 'active')] = self.create_client(Trigger, core.name + '/is_active')
            for name in core.nodes:
                self.service_clients[(group, 'state', name)] = self.create_client(GetState, '/' + name.strip('/') + '/get_state')
                self.service_clients[(group, 'change', name)] = self.create_client(ChangeState, '/' + name.strip('/') + '/change_state')
            self.snapshots[group] = Snapshot()
        self.operations, self.polls, self.last_poll = {}, {}, {}
        self.scan_received = None
        self.have_map = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, latched)
        self.status_pub = self.create_publisher(String, '/startup/lifecycle_status', latched)
        self.last_status = None
        self.last_publish = -1.0
        self.finished = False
        # sim /clock이 멈춰도 서비스 timeout과 상태 보고는 단조 시계로 돌아감.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.tick, clock=self.steady_clock)
        self.get_logger().info('시작 lifecycle 감시: 센서·odom → localization → 지도·map TF → navigation; 해당 manager만 최대3회 복구')

    def destroy_node(self):
        self.timer.cancel()
        for key in list(self.polls):
            self._drop_poll(key)
        for future, _, client in self.operations.values():
            client.remove_pending_request(future)
            future.cancel()
        self.operations.clear()
        return super().destroy_node()

    def on_scan(self, msg):
        if msg.ranges and any(math.isfinite(x) and msg.range_min <= x <= msg.range_max for x in msg.ranges):
            self.scan_received = time.monotonic()

    def on_map(self, msg):
        self.have_map = msg.info.width > 0 and msg.info.height > 0 and msg.info.resolution > 0 and len(msg.data) == msg.info.width * msg.info.height

    def fresh_tf(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
        return now > 0 and stamp > 0 and -2.0 <= now - stamp <= self.data_timeout

    def _drop_poll(self, key):
        future, _, _ = self.polls.pop(key)
        client = self.service_clients[key]
        client.remove_pending_request(future)
        future.cancel()

    def _poll_states(self, group, now):
        core = self.groups[group]
        r = core.pending
        keys = [(group, 'active')] + [(group, 'state', n) for n in core.nodes]
        if r:
            if r.timed_out_at is None:
                return  # manager callback에 상태 조회를 줄줄이 쌓지 않음.
            keys = [(group, 'active')] if r.kind == 'manager' else [(group, 'state', r.target)]
        for key in keys:
            if key in self.polls or now - self.last_poll.get(key, -1e10) < self.poll_s:
                continue
            client = self.service_clients[key]
            if not client.service_is_ready():
                continue
            request = Trigger.Request() if key[1] == 'active' else GetState.Request()
            try:
                self.polls[key] = (client.call_async(request), now, bool(r))
                self.last_poll[key] = now
            except Exception as exc:
                core.last_error = '상태 조회 요청 실패: ' + str(exc)

    def _receive(self, now):
        for group, (future, token, client) in list(self.operations.items()):
            core = self.groups[group]
            if future.done():
                try:
                    response = future.result()
                    success = response is not None and bool(response.success)
                    detail = '' if success else 'lifecycle 전환 응답 false'
                except Exception as exc:
                    success, detail = False, str(exc)
                core.complete(token, success, now, detail)
                self.operations.pop(group)
            elif core.pending is None or core.pending.token != token:
                # timeout 뒤 상태 조회 장벽으로 끝난 이전 요청은 결과를 재적용하지 않음.
                client.remove_pending_request(future)
                future.cancel()
                self.operations.pop(group)
        for key, (future, sent, barrier) in list(self.polls.items()):
            if future.done():
                try:
                    response = future.result()
                    if response is not None:
                        value = bool(response.success) if key[1] == 'active' else int(response.current_state.id)
                        obs = Observation(value, sent, now)
                        if key[1] == 'active':
                            self.snapshots[key[0]].active = obs
                        else:
                            self.snapshots[key[0]].states[key[2]] = obs
                except Exception as exc:
                    self.groups[key[0]].last_error = '상태 조회 응답 실패: ' + str(exc)
                self.polls.pop(key)
            elif not barrier and now - sent > self.probe_timeout:
                self._drop_poll(key)
            # timeout 장벽 조회는 응답/전체 drain기한까지 하나만 유지함.

    def _send(self, group, request):
        key = (group, 'manager') if request.kind == 'manager' else (group, 'change', request.target)
        client = self.service_clients[key]
        if request.kind == 'manager':
            msg = ManageLifecycleNodes.Request()
            msg.command = request.command
        else:
            msg = ChangeState.Request()
            msg.transition.id = request.command
        try:
            future = client.call_async(msg)
            self.operations[group] = (future, request.token, client)
            self.get_logger().info(f'{group}: {request.purpose} 대상={request.target} 명령={request.command} 복구={self.groups[group].recoveries}/3')
        except Exception as exc:
            self.groups[group].complete(request.token, False, time.monotonic(), str(exc))

    def tick(self):
        now = time.monotonic()
        self._receive(now)
        scan_ok = self.scan_received is not None and now - self.scan_received <= self.data_timeout
        odom_ok = self.fresh_tf(self.odom_frame, self.base_frame)
        map_tf_ok = self.fresh_tf(self.map_frame, self.odom_frame)
        local_ready = not self.manage_localization or self.groups['localization'].phase == 'ready'
        for group, core in self.groups.items():
            s = self.snapshots[group]
            needs = []
            if not scan_ok:
                needs.append(self.scan_topic)
            if not odom_ok:
                needs.append(self.odom_frame + '→' + self.base_frame + ' TF')
            if group == 'navigation':
                if not local_ready:
                    needs.append('localization ACTIVE 안정')
                if not self.have_map:
                    needs.append(self.map_topic)
                if not map_tf_ok:
                    needs.append(self.map_frame + '→' + self.odom_frame + ' TF')
            s.inputs_ready = not needs
            s.input_reason = '기다림: ' + ', '.join(needs) if needs else '입력 준비됨'
            s.services_ready = all(client.service_is_ready() for key, client in self.service_clients.items() if key[0] == group)
            if not self.finished:
                core.recheck_before_handoff(now, s)
            request = core.tick(now, s)
            if request is not None:
                self._send(group, request)
            self._poll_states(group, now)
        states = {g: core.status() for g, core in self.groups.items()}
        ready = all(core.phase == 'ready' and core.healthy(self.snapshots[g], now) and self.snapshots[g].inputs_ready
                    for g, core in self.groups.items())
        self.finished = self.finished or ready
        status = {'phase': 'ready' if ready else 'monitoring' if self.finished else 'exhausted' if any(x.phase == 'exhausted' for x in self.groups.values()) else 'starting',
                  'ready': ready, 'startup_completed': self.finished,
                  'localization_ready': not self.manage_localization or (self.groups['localization'].phase == 'ready' and
                      self.groups['localization'].healthy(self.snapshots['localization'], now)),
                  'navigation_ready': self.groups['navigation'].phase == 'ready' and self.groups['navigation'].healthy(self.snapshots['navigation'], now),
                  'recovery_attempts': {g: x.recoveries for g, x in self.groups.items()},
                  'pending': {g: x['pending'] for g, x in states.items()},
                  'last_error': {g: x.last_error for g, x in self.groups.items()},
                  'inputs': {'scan': scan_ok, 'odom_tf': odom_ok, 'map': self.have_map, 'map_odom_tf': map_tf_ok},
                  'groups': states, 'monitor_only_after_startup': self.finished}
        encoded = json.dumps(status, ensure_ascii=False, sort_keys=True)
        if encoded != self.last_status or now - self.last_publish >= 1.0:
            self.status_pub.publish(String(data=encoded))
            self.last_publish = now
        if encoded != self.last_status:
            self.get_logger().info('lifecycle 시작 상태: ' + encoded)
            self.last_status = encoded


def main(args=None):
    rclpy.init(args=args)
    node = StartupLifecycleWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
