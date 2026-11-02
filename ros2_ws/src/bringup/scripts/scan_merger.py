#!/usr/bin/env python3
"""앞 · 뒤 라이다 → 몸 중심 360° 스캔 하나 (2026-10-31). slam_toolbox · AMCL 은 스캔을 하나만 받음.

받기: scans 의 LaserScan 들 (기본 /lidar_front/scan · /lidar_rear/scan, SensorDataQoS)
내기: out (기본 /scan), frame_id = frame (기본 base_footprint), bins 칸 (기본 720 = 0.5°) — 칸마다 가장 가까운 점
  각 스캔 점을 TF(고정 장착, 처음 한 번 조회) 로 frame 에 옮기고, 몸 상자(body_half) 안 점(자기 바퀴 · 기둥 · 다른 라이다)은 버림.
  첫 번째 스캔이 올 때마다 냄 (시각 = 그 스캔 시각). 다른 스캔은 그 시각에서 max_age 안일 때만 섞음.
  첫 번째가 0.5 s 넘게 안 오면 다른 스캔이 올 때 그것만으로 냄 (한쪽 라이다가 죽어도 지도 · 위치 추정이 멈추지 않게).
두 라이다 시각 차(최대 max_age) 동안 로봇이 움직인 만큼은 보정하지 않음 — 0.10 m/s · 0.1 s = 1 cm (지도 칸 5 cm 보다 작음).
"""
import math

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        self.scans = self.declare_parameter('scans', ['/lidar_front/scan', '/lidar_rear/scan']).value
        self.frame = self.declare_parameter('frame', 'base_footprint').value
        self.bins = int(self.declare_parameter('bins', 720).value)
        self.rmin = float(self.declare_parameter('range_min', 0.05).value)
        self.rmax = float(self.declare_parameter('range_max', 8.0).value)
        self.half = [float(v) for v in self.declare_parameter('body_half', [0.17, 0.17]).value]   # 몸 상자 반폭 x · y [m]
        self.max_age = float(self.declare_parameter('max_age', 0.15).value)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)
        self.mount = {}                       # frame_id → (x, y, yaw) — 장착은 고정이라 한 번만
        self.latest = {}                      # 토픽 → (시각 s, 점 N×2)
        self.pub = self.create_publisher(LaserScan, self.declare_parameter('out', '/scan').value, qos_profile_sensor_data)
        for i, t in enumerate(self.scans):
            self.create_subscription(LaserScan, t, lambda m, t=t, i=i: self.on_scan(m, t, i == 0), qos_profile_sensor_data)
        self.get_logger().info(f'스캔 합치기: {self.scans} → {self.frame} {self.bins} 칸, 몸 상자 ±{self.half}')

    def points(self, m):
        f = m.header.frame_id
        if f not in self.mount:
            try:
                tr = self.tfb.lookup_transform(self.frame, f, Time()).transform
            except Exception as e:                                   # URDF 가 아직 안 떴으면 다음 스캔에서 다시
                self.get_logger().warning(f'TF {self.frame} ← {f} 없음: {e}', throttle_duration_sec=5.0)
                return None
            self.mount[f] = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        tx, ty, tyaw = self.mount[f]
        r = np.asarray(m.ranges, np.float32)
        a = m.angle_min + m.angle_increment * np.arange(len(r), dtype=np.float32) + tyaw
        ok = np.isfinite(r) & (r >= max(m.range_min, self.rmin)) & (r <= min(m.range_max, self.rmax))
        p = np.c_[tx + r[ok] * np.cos(a[ok]), ty + r[ok] * np.sin(a[ok])]
        return p[(np.abs(p[:, 0]) > self.half[0]) | (np.abs(p[:, 1]) > self.half[1])]

    def on_scan(self, m, topic, primary):
        p = self.points(m)
        if p is None:
            return
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        self.latest[topic] = (t, p)
        first = self.latest.get(self.scans[0])
        if not primary and first is not None and t - first[0] < 0.5:
            return                                                    # 첫 번째 스캔이 살아 있으면 그쪽 도착에 맞춰 냄
        parts = [q for tt, q in self.latest.values() if abs(tt - t) <= self.max_age]
        pts = np.concatenate(parts) if parts else p
        out = LaserScan()
        out.header.stamp, out.header.frame_id = m.header.stamp, self.frame
        out.angle_min, out.angle_increment = -math.pi, 2 * math.pi / self.bins
        out.angle_max = out.angle_min + out.angle_increment * (self.bins - 1)
        out.scan_time, out.time_increment = m.scan_time, 0.0
        out.range_min, out.range_max = self.rmin, self.rmax
        ranges = np.full(self.bins, np.inf, np.float32)
        if len(pts):
            k = ((np.arctan2(pts[:, 1], pts[:, 0]) + math.pi) / out.angle_increment).astype(int) % self.bins
            np.minimum.at(ranges, k, np.hypot(pts[:, 0], pts[:, 1]).astype(np.float32))
        out.ranges = ranges.tolist()
        self.pub.publish(out)


def main():
    rclpy.init()
    n = ScanMerger()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
