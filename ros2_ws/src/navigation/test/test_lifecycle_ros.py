"""선택 ROS 통합 시험임. RUN_LIFECYCLE_ROS_TESTS=1, 별도 ROS_DOMAIN_ID에서만 실행함.
가짜 manager의 STARTUP 실패/부분ACTIVE RESET실패→노드별cleanup→재시작을 통신까지 확인함.
"""
import json
import os
import sys
import time
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


@unittest.skipUnless(os.environ.get('RUN_LIFECYCLE_ROS_TESTS') == '1', '고립 ROS domain에서 선택 실행')
class LifecycleRosTests(unittest.TestCase):
    def test_partial_active_reset_failure_and_localization_before_navigation(self):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from lifecycle_msgs.srv import ChangeState, GetState
        from nav2_msgs.srv import ManageLifecycleNodes
        from nav_msgs.msg import OccupancyGrid
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from std_srvs.srv import Trigger
        from tf2_ros import TransformBroadcaster
        from startup_lifecycle_watchdog import StartupLifecycleWatchdog
        args = ['--ros-args']
        overrides = {'localization_manager': '/test_lm_loc', 'navigation_manager': '/test_lm_nav',
                     'localization_nodes': '[test_map_server,test_amcl]', 'navigation_nodes': '[test_planner,test_monitor]',
                     'scan_topic': '/test_scan', 'map_topic': '/test_map', 'base_frame': 'test_base',
                     'odom_frame': 'test_odom', 'map_frame': 'test_map', 'poll_s': '0.05',
                     'bond_stable_s': '0.1', 'verify_timeout_s': '4.0', 'manager_timeout_s': '2.0'}
        for key, value in overrides.items():
            args.extend(['-p', key + ':=' + value])
        rclpy.init(args=args)
        fake = Node('fake_lifecycle_system')
        groups = {'loc': ['test_map_server', 'test_amcl'], 'nav': ['test_planner', 'test_monitor']}
        states = {n: 1 for ns in groups.values() for n in ns}
        active = {'loc': False, 'nav': False}
        starts = {'loc': 0, 'nav': 0}
        events = []
        handles = []
        def manage(group):
            def callback(req, res):
                events.append((group, 'manager', int(req.command)))
                if req.command == 0:
                    starts[group] += 1
                    if group == 'loc' and starts[group] == 1:
                        states['test_map_server'] = 3
                        states['test_amcl'] = 2
                        res.success = False
                    else:
                        for n in groups[group]:
                            states[n] = 3
                        active[group] = True
                        res.success = True
                elif req.command == 3:
                    # Nav2 reset과 같이 inactive 노드를 deactivate하다 중단하는 장면임.
                    active[group] = False
                    res.success = False
                return res
            return callback
        def get_active(group):
            def callback(req, res):
                res.success = active[group]
                return res
            return callback
        def get_state(name):
            def callback(req, res):
                res.current_state.id = states[name]
                return res
            return callback
        def change_state(name):
            def callback(req, res):
                value = int(req.transition.id)
                events.append((name, 'node', value))
                res.success = (states[name], value) in ((3, 4), (2, 2))
                if res.success:
                    states[name] = 2 if value == 4 else 1
                return res
            return callback
        for g in groups:
            handles += [fake.create_service(ManageLifecycleNodes, '/test_lm_' + g + '/manage_nodes', manage(g)),
                        fake.create_service(Trigger, '/test_lm_' + g + '/is_active', get_active(g))]
            for n in groups[g]:
                handles += [fake.create_service(GetState, '/' + n + '/get_state', get_state(n)),
                            fake.create_service(ChangeState, '/' + n + '/change_state', change_state(n))]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        mp = fake.create_publisher(OccupancyGrid, '/test_map', latched)
        sp = fake.create_publisher(LaserScan, '/test_scan', qos_profile_sensor_data)
        broadcaster = TransformBroadcaster(fake)
        def publish():
            stamp = fake.get_clock().now().to_msg()
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'test_base'
            scan.angle_min, scan.angle_max, scan.angle_increment = -.5, .5, .5
            scan.range_min, scan.range_max, scan.ranges = .1, 5.0, [1., 1., 1.]
            sp.publish(scan)
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'test_odom', 'test_base'
            tf.transform.rotation.w = 1.0
            broadcaster.sendTransform(tf)
            if states['test_map_server'] == 3:
                m = OccupancyGrid(); m.header.frame_id = 'test_map'; m.header.stamp = stamp
                m.info.width, m.info.height, m.info.resolution = 2, 2, .1
                m.info.origin.orientation.w = 1.; m.data = [0, 100, 0, 0]
                mp.publish(m)
            if active['loc']:
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'test_map', 'test_odom'
                tf.transform.rotation.w = 1.0
                broadcaster.sendTransform(tf)
        timer = fake.create_timer(.05, publish)
        watchdog = StartupLifecycleWatchdog()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(fake); executor.add_node(watchdog)
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline and not (watchdog.last_status and json.loads(watchdog.last_status)['ready']):
                executor.spin_once(timeout_sec=.05)
            self.assertTrue(watchdog.finished, watchdog.last_status)
            self.assertEqual(starts, {'loc': 2, 'nav': 1})
            self.assertEqual(watchdog.groups['localization'].recoveries, 1)
            self.assertEqual(watchdog.groups['navigation'].recoveries, 0)
            self.assertIn(('test_amcl', 'node', 2), events)
            self.assertIn(('test_map_server', 'node', 4), events)
            self.assertIn(('test_map_server', 'node', 2), events)
            self.assertGreater(events.index(('nav', 'manager', 0)), max(i for i, e in enumerate(events) if e == ('loc', 'manager', 0)))
            status = json.loads(watchdog.last_status)
            self.assertTrue(status['ready'])
            self.assertTrue(all(status['inputs'].values()))
            before = list(events)
            active['nav'] = False
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.05)
            status = json.loads(watchdog.last_status)
            self.assertFalse(status['ready'])
            self.assertTrue(status['startup_completed'])
            self.assertEqual(status['phase'], 'monitoring')
            self.assertEqual(events, before)  # 시작 완료 뒤 manager를 새로 reset하지 않음.
            print('FAKE_LIFECYCLE_EVENTS=' + json.dumps(events))
        finally:
            fake.destroy_timer(timer)
            executor.shutdown()
            watchdog.destroy_node(); fake.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
