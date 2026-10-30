#!/usr/bin/env python3
"""가짜 perception 신호등: 시뮬 정답 상태 -> interfaces/TrafficLight (planning 신호등 기록기 시험용, 실물에서는 perception 이 냄)

받기: /sim/traffic_light_truth (std_msgs/String 'R' · 'Y' · 'G' · 'OFF', sim_nav_bridge.py)
보내기: /perception/traffic_light (TrafficLight, --hz 주기, 기본 5 Hz = 약속된 perception 주기)
잡음 (선택): --ambiguous P  켜진 동안 P 확률로 UNKNOWN + visible (색 애매)
             --drop P       P 확률로 UNKNOWN + not visible (못 찾음)
             --latency S    S 초 늦게 반영 (영상 처리 지연)
실행 (navigation · interfaces 빌드한 워크스페이스 source 뒤):
  ROS_DOMAIN_ID=17 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 scripts/fake_traffic_light.py --ambiguous 0.05
"""
import argparse
import collections
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import TrafficLight

p = argparse.ArgumentParser()
p.add_argument('--hz', type=float, default=5.0)
p.add_argument('--ambiguous', type=float, default=0.0)
p.add_argument('--drop', type=float, default=0.0)
p.add_argument('--latency', type=float, default=0.15)
p.add_argument('--seed', type=int, default=0)
a = p.parse_args()
random.seed(a.seed)
STATE = {'R': TrafficLight.RED, 'Y': TrafficLight.YELLOW, 'G': TrafficLight.GREEN, 'OFF': TrafficLight.OFF}

rclpy.init()
n = Node('fake_traffic_light')
hist = collections.deque(maxlen=200)   # (시각, 상태)
n.create_subscription(String, '/sim/traffic_light_truth', lambda m: hist.append((n.get_clock().now().nanoseconds * 1e-9, m.data)), 10)
pub = n.create_publisher(TrafficLight, '/perception/traffic_light', 10)


def tick():
    if not hist:
        return
    t = n.get_clock().now().nanoseconds * 1e-9 - a.latency
    cur = next((s for ts, s in reversed(hist) if ts <= t), hist[0][1])
    m = TrafficLight()
    m.header.stamp = n.get_clock().now().to_msg()
    m.header.frame_id = 'birdeye_camera_optical_frame'
    r = random.random()
    if r < a.drop:
        m.state, m.visible = TrafficLight.UNKNOWN, False
    elif cur != 'OFF' and r < a.drop + a.ambiguous:
        m.state, m.visible = TrafficLight.UNKNOWN, True
    else:
        m.state, m.visible = STATE[cur], True
    pub.publish(m)


n.create_timer(1.0 / a.hz, tick)
rclpy.spin(n)
