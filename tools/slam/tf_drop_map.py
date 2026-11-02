#!/usr/bin/env python3
"""bag 재생용 (2026-10-31): /tf_bag(기록된 /tf) 에서 map→odom 만 빼고 /tf 로 다시 냄 → 새 slam_toolbox 가 map→odom 의 유일한 주인.
  ros2 bag play <bag> --clock --remap /tf:=/tf_bag … 와 같이 (use_sim_time true)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage


def main():
    rclpy.init()
    n = Node('tf_drop_map', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    pub = n.create_publisher(TFMessage, '/tf', QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
    # bag 재생은 기록 때 QoS(record_qos.yaml) 로 냄 — /tf 가 best effort 라 구독도 best effort (reliable 구독이면 하나도 못 받음, 10-31)
    q = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

    def cb(m):
        keep = [t for t in m.transforms if not (t.header.frame_id == 'map' and t.child_frame_id == 'odom')]
        if keep:
            pub.publish(TFMessage(transforms=keep))
    n.create_subscription(TFMessage, '/tf_bag', cb, q)
    rclpy.spin(n)


if __name__ == '__main__':
    main()
