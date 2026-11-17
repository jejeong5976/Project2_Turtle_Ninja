"""위치 보정 노드의 신선도·이동 취소·자동 재시도 회귀. 장비/모터 없이 실행함."""
from concurrent.futures import Future
import math
import os
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock

os.environ.setdefault('ROS_DOMAIN_ID', '187')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from auto_initial_pose import AutoInitialPose
from pose_match_core import match
from test_auto_initial_pose import fixture, raycast


class RuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.get_default_context().init(args=[], domain_id=187, initialize_logging=False)
        cls.model = fixture()
        cls.truth = [.15, -.15, .08]
        cls.current = [.45, -.15, .08]
        cls.points = raycast(cls.model, cls.truth)
        cls.result = match(cls.model, cls.points, cls.current, cls.current)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = AutoInitialPose()
        self.node.pool.shutdown(wait=False)
        self.node.pool = Mock()
        self.node.pub = Mock()
        self.node.status = Mock()
        self.node.map_data = (self.model.grid, self.model.resolution, self.model.origin, 'map')
        self.node.map_revision = 1
        self.node.odom_pose = (0, 0, 0)
        self.node.still = True
        self.node.current_inputs = Mock(return_value=(self.points, self.current))

    def tearDown(self):
        self.node.destroy_node()

    def complete_job(self, epoch=0, map_revision=1):
        self.node.job = {'epoch': epoch, 'map_revision': map_revision, 'odom_pose': (0, 0, 0)}
        self.node.future = Future()
        self.node.future.set_result((self.result, self.model))

    def test_stationary_improvement_publishes_but_not_ready_yet(self):
        self.complete_job()
        self.node.tick()
        self.node.pub.publish.assert_called_once()
        args = self.node.status.call_args.args
        self.assertEqual(args[:2], ('published', False))
        self.node.current_inputs.return_value = (self.points, self.truth)
        self.node.tick()
        self.assertEqual(self.node.status.call_args.args[:2], ('aligned', True))

    def test_motion_during_worker_discards_even_after_returning(self):
        self.complete_job()
        self.node.motion_epoch = 1
        self.node.tick()
        self.node.pub.publish.assert_not_called()
        self.assertEqual(self.node.status.call_args.args[2], 'moved_or_map_changed')
        self.assertTrue(self.node.need_match)

    def test_map_change_discards(self):
        self.complete_job(map_revision=0)
        self.node.tick()
        self.node.pub.publish.assert_not_called()
        self.assertEqual(self.node.status.call_args.args[2], 'moved_or_map_changed')

    def test_stale_inputs_cancel_then_retry(self):
        self.complete_job()
        self.node.current_inputs.side_effect = ValueError('scan_odom_stale')
        self.node.tick()
        self.node.pub.publish.assert_not_called()
        self.assertGreater(self.node.motion_epoch, 0)
        self.node.current_inputs.side_effect = None
        self.node.tick()
        self.node.pub.publish.assert_not_called()
        self.assertTrue(self.node.need_match)

    def test_moving_defers_low_health_without_worker_submission(self):
        self.node.still = False
        self.node.health_low = self.node.need_match = True
        self.node.tick()
        self.node.pool.submit.assert_not_called()
        self.node.pub.publish.assert_not_called()
        self.assertEqual(self.node.status.call_args.args[0], 'deferred_motion')
        self.node.still = True
        self.node.tick()
        self.node.pool.submit.assert_called_once()

    def test_current_tf_already_correct_does_not_republish(self):
        self.complete_job()
        self.node.current_inputs.return_value = (self.points, self.truth)
        self.node.tick()
        self.node.pub.publish.assert_not_called()
        self.assertEqual(self.node.status.call_args.args[:2], ('aligned', True))

    def seed_setup(self):
        self.node.current_inputs.side_effect = ValueError('map_odom_stale')
        self.node.map_odom_ok = Mock(return_value=False)
        self.node.odom_received = time.monotonic()

    def test_seed_after_map_odom_missing(self):
        # AMCL 이 시작 자세를 버려 map→odom 이 없음 → seed_after_s 전에는 안 보냄, 뒤에는 launch 시작 자세를 /initialpose 로
        self.seed_setup()
        self.node.seed = (1.0, -2.0, math.radians(90.0))
        self.node.tick()
        self.node.pub.publish.assert_not_called()
        self.node.no_map_odom_since -= self.node.seed_after_s + 0.1
        self.node.odom_received = time.monotonic()
        self.node.tick()
        self.node.pub.publish.assert_called_once()
        msg = self.node.pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.pose.pose.position.x, 1.0)
        self.assertAlmostEqual(msg.pose.pose.position.y, -2.0)
        self.assertAlmostEqual(msg.pose.pose.orientation.z, math.sin(math.radians(45.0)))
        self.assertGreater(msg.pose.covariance[0], 0.0)
        self.assertEqual(self.node.status.call_args.args[0], 'seed_initial_pose')
        # seed_every_s 안에는 다시 안 보냄
        self.node.tick()
        self.assertEqual(self.node.pub.publish.call_count, 1)

    def test_seed_limited_and_needs_stationary(self):
        self.seed_setup()
        self.node.no_map_odom_since = time.monotonic() - 100.0
        self.node.still = False
        self.node.tick()
        self.node.pub.publish.assert_not_called()
        self.node.still = True
        for _ in range(self.node.seed_max + 2):
            self.node.seed_last = -math.inf
            self.node.odom_received = time.monotonic()
            self.node.tick()
        self.assertEqual(self.node.pub.publish.call_count, self.node.seed_max)

    def test_no_seed_when_map_odom_present(self):
        self.seed_setup()
        self.node.map_odom_ok.return_value = True
        self.node.no_map_odom_since = time.monotonic() - 100.0
        self.node.tick()
        self.node.pub.publish.assert_not_called()
        self.assertIsNone(self.node.no_map_odom_since)

    def test_odom_velocity_marks_motion_epoch(self):
        m = Odometry()
        m.pose.pose.orientation.w = 1.
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.twist.twist.linear.x = .1
        self.node.on_odom(m)
        self.assertFalse(self.node.still)
        self.assertEqual(self.node.motion_epoch, 1)
        m.twist.twist.linear.x = 0.
        self.node.on_odom(m)
        self.assertTrue(self.node.still)
        self.assertEqual(self.node.motion_epoch, 1)

    def test_scan_before_motion_not_used(self):
        # 실제 current_inputs를 직접 불러 지난 scan 판정이 TF 참조보다 앞서는지 확인함.
        self.node.scan = LaserScan()
        self.node.scan.header.stamp = self.node.get_clock().now().to_msg()
        self.node.scan_received = self.node.odom_received = time.monotonic()
        stamp = self.node.scan.header.stamp
        self.node.odom_stamp = stamp.sec + stamp.nanosec * 1e-9
        self.node.last_motion_stamp = self.node.odom_stamp + .01
        with self.assertRaisesRegex(ValueError, 'scan_before_stationary'):
            AutoInitialPose.current_inputs(self.node)


if __name__ == '__main__':
    unittest.main(verbosity=2)
