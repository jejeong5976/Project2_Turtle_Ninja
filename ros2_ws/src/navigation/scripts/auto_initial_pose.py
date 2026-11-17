#!/usr/bin/env python3
"""AMCL 초기 자세 자동 맞춤. 위치 메시지만 발행하며 주행/FAULT 상태는 바꾸지 않음."""
import os
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

from concurrent.futures import ProcessPoolExecutor
from dataclasses import fields
import json
import math
import multiprocessing
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from interfaces.msg import MissionStatus
from tf2_ros import Buffer, TransformListener, TransformException

from pose_match_core import (MatchConfig, decide_publication, pose_changed,
                             prepare_points, run_match_job, scan_points)


def yaw(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(values).all() or np.linalg.norm(values) < 1e-6:
        raise ValueError("잘못된 회전값")
    x, y, z, w = values / np.linalg.norm(values)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pose2(p):
    out = (float(p.position.x), float(p.position.y), yaw(p.orientation))
    if not np.isfinite(out).all():
        raise ValueError("잘못된 위치값")
    return out


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__("auto_initial_pose")
        # 탐색 기본값의 유일한 원본은 ROS 독립 MatchConfig임.
        defaults = MatchConfig()
        self.cfg = MatchConfig(**{f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
                                  for f in fields(MatchConfig)})
        self.cfg.validate()
        self.seed = (float(self.declare_parameter("init_x", 0.0).value),
                     float(self.declare_parameter("init_y", 0.0).value),
                     math.radians(float(self.declare_parameter("init_yaw_deg", 0.0).value)))
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.retry_s = float(self.declare_parameter("retry_s", 3.0).value)
        self.scan_max_age = float(self.declare_parameter("scan_max_age", 0.5).value)
        self.odom_max_age = float(self.declare_parameter("odom_max_age", 0.5).value)
        self.tf_max_age = float(self.declare_parameter("tf_max_age", 1.0).value)
        self.still_v = float(self.declare_parameter("still_v", 0.01).value)
        self.still_w = float(self.declare_parameter("still_w", 0.02).value)
        # 기존 amcl_settle의 초기 공분산과 같음. 팔/주행 보정값이 아님.
        self.init_std_xy = float(self.declare_parameter("init_std_xy", 0.05).value)
        self.init_std_yaw = math.radians(float(self.declare_parameter("init_std_yaw_deg", 3.0).value))
        # 시작 보조: AMCL 이 켤 때 initial_pose 를 버리면 (TF 가 늦게 뜸) map→odom 이 안 나오고, 이 노드도 map→odom 을 기다려
        #   서로 막힘 (planner_server 가 base_link→map 을 못 받음). map→odom 이 seed_after_s 넘게 없고
        #   지도 · 정지 odom 이 있으면 launch 시작 자세 (init_x · init_y · init_yaw_deg) 를 퍼짐과 함께 /initialpose 로 보냄
        #   (seed_every_s 마다, seed_max 번까지). map→odom 이 나오면 그 뒤는 원래 스캔 맞춤. 0 = 끔
        self.seed_after_s = float(self.declare_parameter("seed_after_s", 3.0).value)
        self.seed_every_s = float(self.declare_parameter("seed_every_s", 3.0).value)
        self.seed_max = int(self.declare_parameter("seed_max", 3).value)
        positive = (self.retry_s, self.scan_max_age, self.odom_max_age,
                    self.tf_max_age, self.still_v, self.still_w, self.init_std_xy, self.init_std_yaw)
        if not all(math.isfinite(v) and v > 0 for v in positive) or not np.isfinite(self.seed).all():
            raise ValueError("시간/속도/초기 자세 파라미터 확인 필요")
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.status_pub = self.create_publisher(String, "/startup/pose_match_status", transient)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.map_data = None
        self.map_revision = 0
        self.model = None
        self.model_revision = -1
        self.scan = None
        self.scan_received = 0.0
        self.odom_pose = None
        self.odom_received = 0.0
        self.odom_stamp = 0.0
        self.still = False
        self.motion_epoch = 0
        self.last_motion_stamp = -math.inf
        self.health_low = False
        self.need_match = True
        self.first_search = True
        self.future = None
        self.job = None
        self.attempt = 0
        self.published = 0
        self.next_attempt = 0.0
        self.no_map_odom_since = None   # map→odom 없음이 처음 보인 시각 (monotonic)
        self.seed_sent = 0
        self.seed_last = -math.inf
        self.last_status_key = None
        self.last_result = {}
        self.pool = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context("spawn"))
        self.create_subscription(OccupancyGrid, "/map", self.on_map, transient)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(DiagnosticStatus, "/localization_health", self.on_health, 10)
        # 미션이 STANDBY 일 때만 맞춤 (RUNNING 중 /initialpose 로 map→odom 이 움직이지 않게). 상태를 아직 못 받았으면 STANDBY 로 봄
        self.mission_state = ""
        self.create_subscription(MissionStatus, "/mission/status", self.on_mission_status, 10)
        self.create_timer(0.25, self.tick)
        self.get_logger().info("자동 초기 위치: 스캔/지도 탐색 · 정지 때 재시도 · /startup/pose_match_status")
        self.status("waiting_inputs", False, "map_scan_odom_tf")

    def on_map(self, msg):
        try:
            origin = (*pose2(msg.info.origin)[:2], yaw(msg.info.origin.orientation))
            data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
            if not data.size or msg.info.resolution <= 0:
                raise ValueError("빈 지도/해상도")
            meta = (float(msg.info.resolution), origin, msg.header.frame_id.lstrip('/') or "map")
            if self.map_data is not None and meta == self.map_data[1:] and np.array_equal(data, self.map_data[0]):
                return
            self.map_data = (data.copy(), *meta)
            self.map_revision += 1
            self.need_match = True
            self.model = None
        except (ValueError, TypeError) as exc:
            self.map_data = None
            self.model = None
            self.map_revision += 1
            self.need_match = True
            self.get_logger().warning(f"지도 다시 기다림: {exc}")

    def on_scan(self, msg):
        self.scan = msg
        self.scan_received = time.monotonic()

    def on_odom(self, msg):
        try:
            p = pose2(msg.pose.pose)
            t = msg.twist.twist
            velocity = (t.linear.x, t.linear.y, t.angular.z)
            stationary = np.isfinite(velocity).all() and math.hypot(*velocity[:2]) < self.still_v and abs(velocity[2]) < self.still_w
            # 계산 시작/끝만 비교하면 움직였다 돌아온 경우를 놓치므로 중간 이동도 기록함.
            if not stationary or (self.job is not None and pose_changed(self.job['odom_pose'], p)):
                self.motion_epoch += 1
                self.last_motion_stamp = stamp_seconds(msg.header.stamp)
            self.still = bool(stationary)
            self.odom_pose = p
            self.odom_received = time.monotonic()
            self.odom_stamp = stamp_seconds(msg.header.stamp)
        except (ValueError, TypeError):
            self.still = False
            self.motion_epoch += 1
            self.odom_received = 0.0

    def on_health(self, msg):
        for item in msg.values:
            if item.key == "match":
                try:
                    score = float(item.value)
                    self.health_low = math.isfinite(score) and score < self.cfg.min_score
                    if self.health_low:
                        self.need_match = True
                except ValueError:
                    pass
                break

    def current_inputs(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds / 1e9
        if self.map_data is None or self.scan is None or self.odom_pose is None:
            raise ValueError("map_scan_odom_wait")
        if now - self.scan_received > self.scan_max_age or now - self.odom_received > self.odom_max_age:
            raise ValueError("scan_odom_stale")
        age = ros_now - stamp_seconds(self.scan.header.stamp)
        odom_age = ros_now - self.odom_stamp
        if not (-0.2 <= age <= self.scan_max_age and -0.2 <= odom_age <= self.odom_max_age):
            raise ValueError("scan_odom_stamp_stale")
        msg = self.scan
        if stamp_seconds(msg.header.stamp) < self.last_motion_stamp:
            raise ValueError("scan_before_stationary")
        points = scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                             max(0.25, msg.range_min), min(4.0, msg.range_max))
        frame = msg.header.frame_id.lstrip('/')
        if not frame:
            raise ValueError("scan_frame_empty")
        if frame != self.base_frame:
            tr = self.tf.lookup_transform(self.base_frame, frame, Time.from_msg(msg.header.stamp))
            a = yaw(tr.transform.rotation)
            c, s = math.cos(a), math.sin(a)
            points = points @ np.array([[c, s], [-s, c]]) + [tr.transform.translation.x, tr.transform.translation.y]
        points = prepare_points(points, self.cfg)
        map_frame = self.map_data[3]
        # 합성 map→base의 stamp만 보지 않고 AMCL 소유 map→odom도 신선한지 확인함.
        mo = self.tf.lookup_transform(map_frame, self.odom_frame, Time())
        if ros_now - stamp_seconds(mo.header.stamp) > self.tf_max_age:
            raise ValueError("map_odom_stale")
        tr = self.tf.lookup_transform(map_frame, self.base_frame, Time())
        if ros_now - stamp_seconds(tr.header.stamp) > self.tf_max_age:
            raise ValueError("base_tf_stale")
        current = (tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation))
        if not np.isfinite(current).all():
            raise ValueError("pose_nonfinite")
        return points, current

    def status(self, state, ready, reason, **extra):
        data = {"state": state, "ready": bool(ready), "reason": reason,
                "score": None, "current_score": None, "elapsed_s": None,
                "attempt": self.attempt, "published": self.published}
        for k in ("score", "current_score", "elapsed_s"):
            if k in self.last_result:
                data[k] = self.last_result[k]
        data.update(extra)
        self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False, allow_nan=False)))
        key = (state, reason, self.attempt, self.published)
        if key != self.last_status_key:
            self.get_logger().info("초기 위치: " + json.dumps(data, ensure_ascii=False, allow_nan=False))
            self.last_status_key = key

    def on_mission_status(self, msg):
        self.mission_state = msg.state

    def tick(self):
        now = time.monotonic()
        if self.mission_state not in ("", "STANDBY"):
            if self.future is not None:
                self.motion_epoch += 1   # 진행 중 계산 결과는 버림
            self.status("paused_mission", True, "mission_" + self.mission_state)
            return
        try:
            points, current = self.current_inputs()
        except (ValueError, TransformException) as exc:
            # 진행 중 계산은 받아 두되 입력이 다시 신선해지기 전에는 발행하지 않음.
            if self.future is not None:
                self.motion_epoch += 1
            if self.maybe_seed(now):
                return
            self.status("waiting_inputs", False, str(exc))
            return
        self.no_map_odom_since = None
        ready = False
        current_score = None
        if self.model is not None and self.model_revision == self.map_revision:
            current_score = self.model.score(points, current, self.cfg.wall_distance)
            ready = current_score >= self.cfg.min_score
        if self.future is not None:
            if not self.future.done():
                self.status("matching", ready, "worker", current_score=current_score)
                return
            job, future = self.job, self.future
            self.job = self.future = None
            self.next_attempt = now + self.retry_s
            try:
                result, model = future.result()
            except Exception as exc:  # 자식 오류가 ROS 실행기를 끝내지 않고 다음 시도로 이어짐
                self.pool.shutdown(wait=False, cancel_futures=True)
                self.pool = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context("spawn"))
                self.need_match = True
                self.status("retrying", ready, "worker_error: " + str(exc), current_score=current_score)
                return
            if job['map_revision'] == self.map_revision:
                self.model, self.model_revision = model, self.map_revision
            if (job['map_revision'] != self.map_revision or job['epoch'] != self.motion_epoch or
                    not self.still or pose_changed(job['odom_pose'], self.odom_pose)):
                self.need_match = True
                self.status("retrying", ready, "moved_or_map_changed", current_score=current_score)
                return
            # 계산 완료 뒤 받은 현재 스캔/현재 TF로 다시 비교함.
            decision = decide_publication(result, model, points, current, self.cfg)
            self.last_result = {**result, **decision}
            ready = decision['current_score'] >= self.cfg.min_score
            self.need_match = decision['retry']
            if decision['publish']:
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = self.map_data[3]
                msg.header.stamp = self.get_clock().now().to_msg()
                px, py, pa = result['pose']
                msg.pose.pose.position.x, msg.pose.pose.position.y = float(px), float(py)
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(pa / 2), math.cos(pa / 2)
                msg.pose.covariance[0] = msg.pose.covariance[7] = self.init_std_xy ** 2
                msg.pose.covariance[35] = self.init_std_yaw ** 2
                self.pub.publish(msg)
                self.published += 1
                # 발행 성공과 위치 수렴은 다름. 새 TF의 현재 점수를 다시 확인함.
                self.need_match = True
                self.status("published", ready, "await_current_pose")
            else:
                self.status("aligned" if ready else "retrying", ready, decision['reason'])
            return
        if ready:
            self.need_match = False
            self.first_search = False
            self.status("aligned", True, "current_scan", current_score=current_score)
            return
        if self.model is not None and not ready:
            self.need_match = True
        if not self.still:
            self.status("deferred_motion", False, "wait_stationary", current_score=current_score)
            return
        if not self.need_match or now < self.next_attempt:
            self.status("retrying", False, "retry_delay", current_score=current_score)
            return
        center = self.seed if self.first_search else current
        self.first_search = False
        self.attempt += 1
        self.job = {'epoch': self.motion_epoch, 'map_revision': self.map_revision, 'odom_pose': self.odom_pose}
        grid, resolution, origin, _ = self.map_data
        try:
            self.future = self.pool.submit(run_match_job, grid, resolution, origin, points,
                                           center, current, self.cfg)
        except RuntimeError as exc:
            self.job = None
            self.pool.shutdown(wait=False, cancel_futures=True)
            self.pool = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context("spawn"))
            self.next_attempt = now + self.retry_s
            self.status("retrying", False, "submit_error: " + str(exc), current_score=current_score)
            return
        self.status("matching", False, "worker", current_score=current_score)

    def map_odom_ok(self):
        """AMCL 소유 map→odom 이 있고 신선한가 (지도 프레임 기준)"""
        if self.map_data is None:
            return False
        try:
            mo = self.tf.lookup_transform(self.map_data[3], self.odom_frame, Time())
        except TransformException:
            return False
        return self.get_clock().now().nanoseconds / 1e9 - stamp_seconds(mo.header.stamp) <= self.tf_max_age

    def maybe_seed(self, now):
        """map→odom 이 계속 없으면 launch 시작 자세로 /initialpose (시작 보조). 보냈으면 True"""
        if self.seed_after_s <= 0 or self.seed_max <= 0 or self.map_data is None:
            return False
        if self.map_odom_ok():
            self.no_map_odom_since = None
            return False
        if self.no_map_odom_since is None:
            self.no_map_odom_since = now
        odom_fresh = self.odom_pose is not None and now - self.odom_received <= self.odom_max_age
        if (now - self.no_map_odom_since < self.seed_after_s or not odom_fresh or not self.still or
                self.seed_sent >= self.seed_max or now - self.seed_last < self.seed_every_s):
            return False
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_data[3]
        msg.header.stamp = self.get_clock().now().to_msg()
        px, py, pa = self.seed
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(px), float(py)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(pa / 2), math.cos(pa / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = self.init_std_xy ** 2
        msg.pose.covariance[35] = self.init_std_yaw ** 2
        self.pub.publish(msg)
        self.seed_sent += 1
        self.seed_last = now
        self.get_logger().warning(
            f"map→odom {now - self.no_map_odom_since:.1f} s 없음 (AMCL 이 시작 자세를 못 받음) → launch 시작 자세 "
            f"({px:.3f}, {py:.3f}, {math.degrees(pa):.1f}°) 를 /initialpose 로 보냄 {self.seed_sent}/{self.seed_max}")
        self.status("seed_initial_pose", False, "map_odom_missing", seed_sent=self.seed_sent)
        return True

    def destroy_node(self):
        self.pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutoInitialPose()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
